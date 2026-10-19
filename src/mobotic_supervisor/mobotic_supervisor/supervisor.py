import copy
import math
import uuid

import rclpy
from geometry_msgs.msg import TwistStamped
from mobotic_interfaces.msg import (
    BatterySystemState,
    OperatorState,
    SafetyIOState,
    SafetyState,
    VehicleModeRequest,
    VehicleModeState,
    WheelModuleOperationCommand,
    WheelModuleCommand,
    WheelModuleCommandArray,
    ComponentHealthArray,
    WheelModuleStatusArray,
)
from mobotic_interfaces.srv import SetVehicleMode
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.time import Time
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger

from .control_math import clamp, clamp_planar, slew, steering_position_representable


class MoboticSupervisor(Node):
    def __init__(self):
        super().__init__('mobotic_supervisor')

        self.declare_parameter('initial_mode', 'manual')
        self.declare_parameter('startup_grace', 10.0)
        self.declare_parameter('reset_timeout', 5.0)
        self.declare_parameter('required_health_sources', Parameter.Type.STRING_ARRAY)
        self.declare_parameter('health_timeout', 0.3)
        self.declare_parameter('require_operator_interlock', False)
        # Required topology is supplied by mobotic_config, not duplicated here.
        self.declare_parameter('expected_module_names', Parameter.Type.STRING_ARRAY)
        self.declare_parameter(
            'expected_steering_joint_names',
            Parameter.Type.STRING_ARRAY,
        )
        self.declare_parameter(
            'expected_traction_joint_names',
            Parameter.Type.STRING_ARRAY,
        )
        self.declare_parameter('direct_traction_control_mode', 'velocity')
        self.declare_parameter('direct_steering_encoder_resolutions', Parameter.Type.DOUBLE_ARRAY)
        self.declare_parameter('control_period', 0.02)
        self.declare_parameter('mode_transition_stop_duration', 0.25)
        self.declare_parameter('mode_transition_timeout', 10.0)
        self.declare_parameter('velocity_feedback_timeout', 0.3)
        self.declare_parameter('standstill_linear_velocity', 0.02)
        self.declare_parameter('standstill_angular_velocity', 0.02)
        self.declare_parameter('standstill_joint_velocity', 0.1)
        self.declare_parameter('manual_timeout', 0.25)
        self.declare_parameter('autonomy_timeout', 0.25)
        self.declare_parameter('safety_timeout', 0.3)
        self.declare_parameter('battery_timeout', 0.6)
        self.declare_parameter('wheel_status_timeout', 0.3)
        self.declare_parameter('minimum_battery_percentage', 0.2)
        self.declare_parameter('max_velocity_linear', 1.25)
        self.declare_parameter('max_velocity_angular', 1.51)
        self.declare_parameter('max_acceleration_linear', 0.5)
        self.declare_parameter('max_acceleration_angular', 0.52)
        self.declare_parameter('warning_speed_scale', 0.3)
        self.declare_parameter('override_speed_scale', 0.2)
        self.declare_parameter('max_direct_steering_velocity', 4.32)
        self.declare_parameter('max_direct_traction_velocity', 11.12)
        self.declare_parameter('max_direct_traction_current', 4.0)
        self.declare_parameter('output_frame_id', 'base_link')

        self.expected_module_names = list(self.get_parameter('expected_module_names').value)
        self.expected_steering_joint_names = list(
            self.get_parameter('expected_steering_joint_names').value
        )
        self.expected_traction_joint_names = list(
            self.get_parameter('expected_traction_joint_names').value
        )
        self.direct_steering_encoder_resolutions = list(
            self.get_parameter('direct_steering_encoder_resolutions').value
        )
        direct_traction_mode = str(
            self.get_parameter('direct_traction_control_mode').value
        )
        if direct_traction_mode != 'velocity':
            raise ValueError(
                'only traction velocity commands are supported'
            )
        self.direct_traction_control_mode = direct_traction_mode
        self.control_period = float(self.get_parameter('control_period').value)
        self.transition_duration = float(
            self.get_parameter('mode_transition_stop_duration').value
        )
        self.transition_timeout = float(
            self.get_parameter('mode_transition_timeout').value
        )
        self.velocity_feedback_timeout = float(
            self.get_parameter('velocity_feedback_timeout').value
        )
        self.standstill_linear_velocity = float(
            self.get_parameter('standstill_linear_velocity').value
        )
        self.standstill_angular_velocity = float(
            self.get_parameter('standstill_angular_velocity').value
        )
        self.standstill_joint_velocity = float(
            self.get_parameter('standstill_joint_velocity').value
        )
        self.manual_timeout = float(self.get_parameter('manual_timeout').value)
        self.autonomy_timeout = float(self.get_parameter('autonomy_timeout').value)
        self.safety_timeout = float(self.get_parameter('safety_timeout').value)
        self.battery_timeout = float(self.get_parameter('battery_timeout').value)
        self.wheel_status_timeout = float(
            self.get_parameter('wheel_status_timeout').value
        )
        self.minimum_battery_percentage = float(
            self.get_parameter('minimum_battery_percentage').value
        )
        self.max_velocity_linear = float(self.get_parameter('max_velocity_linear').value)
        self.max_velocity_angular = float(self.get_parameter('max_velocity_angular').value)
        self.max_acceleration_linear = float(
            self.get_parameter('max_acceleration_linear').value
        )
        self.max_acceleration_angular = float(
            self.get_parameter('max_acceleration_angular').value
        )
        self.warning_speed_scale = float(self.get_parameter('warning_speed_scale').value)
        self.override_speed_scale = float(self.get_parameter('override_speed_scale').value)
        self.max_direct_steering_velocity = float(
            self.get_parameter('max_direct_steering_velocity').value
        )
        self.max_direct_traction_velocity = float(
            self.get_parameter('max_direct_traction_velocity').value
        )
        self.max_direct_traction_current = float(
            self.get_parameter('max_direct_traction_current').value
        )
        self.output_frame_id = str(self.get_parameter('output_frame_id').value)

        self._validate_parameters()
        initial_mode = self._mode_from_name(str(self.get_parameter('initial_mode').value))

        self.selected_mode = initial_mode
        self.started_at = self.get_clock().now()
        self.startup_grace = float(self.get_parameter('startup_grace').value)
        self.reset_timeout = float(self.get_parameter('reset_timeout').value)
        if not all(math.isfinite(v) and v > 0 for v in (self.startup_grace, self.reset_timeout)):
            raise ValueError('startup/reset timeouts must be positive and finite')
        self.supervisor_instance = str(uuid.uuid4())
        self.authority_generation = 1
        self.fault_codes = set()
        self.reset_id = 0
        self.reset_started_at = None
        self.reset_result = ''
        self.standstill_since = None
        self.velocity_ledger = {}
        self.component_health = {}
        self.required_health_sources = list(self.get_parameter('required_health_sources').value)
        self.health_timeout = float(self.get_parameter('health_timeout').value)
        self.require_operator_interlock = bool(self.get_parameter('require_operator_interlock').value)
        self.last_final = {}
        self.requested_mode = initial_mode
        self.transition_in_progress = False
        self.transition_started_at = None
        self.transition_from_direct = False
        self.transition_standstill_since = None
        self.transition_timed_out = False
        self.command_accept_after = None
        self.motion_inputs_ready = False

        self.manual_command = None
        self.manual_command_received = None
        self.manual_state = None
        self.manual_state_received = None
        self.autonomy_command = None
        self.autonomy_command_received = None
        self.autonomy_joint_command = None
        self.autonomy_joint_received = None
        self.safety_state = None
        self.safety_received = None
        self.safety_io_state = None
        self.safety_io_received = None
        self.battery_state = None
        self.battery_received = None
        self.wheel_status = None
        self.wheel_status_received = None
        self.joint_state = None
        self.joint_state_received = None
        self.measured_velocity = None
        self.measured_velocity_received = None

        self.last_output = [0.0, 0.0, 0.0]
        self.last_control_time = self.get_clock().now()

        self.cmd_vel_publisher = self.create_publisher(TwistStamped, 'vehicle/cmd_vel', 1)
        self.final_command_publisher = self.create_publisher(
            WheelModuleCommandArray, 'wheel_modules/command', 1
        )
        self.mode_state_publisher = self.create_publisher(
            VehicleModeState, 'vehicle/mode', 1
        )
        self.module_command_publisher = self.create_publisher(
            WheelModuleOperationCommand, 'wheel_modules/operation_command', 1
        )

        self.input_subscriptions = [
            self.create_subscription(
                TwistStamped, 'operator/cmd_vel', self._manual_command_callback, 1
            ),
            self.create_subscription(
                OperatorState, 'operator/state', self._manual_state_callback, 1
            ),
            self.create_subscription(
                TwistStamped,
                'autonomy/cmd_vel',
                self._autonomy_command_callback,
                1,
            ),
            self.create_subscription(
                JointState,
                'autonomy/joint_setpoints',
                self._autonomy_joint_callback,
                1,
            ),
            self.create_subscription(
                SafetyState, 'safety/state', self._safety_callback, 1
            ),
            self.create_subscription(
                SafetyIOState,
                'safety/io_state',
                self._safety_io_callback,
                1,
            ),
            self.create_subscription(
                BatterySystemState,
                'battery/system_state',
                self._battery_callback,
                1,
            ),
            self.create_subscription(
                WheelModuleStatusArray,
                'wheel_modules/status',
                self._wheel_status_callback,
                1,
            ),
            self.create_subscription(
                JointState, 'joint_states', self._joint_state_callback, 1
            ),
            self.create_subscription(
                TwistStamped, 'vehicle/measured_velocity', self._velocity_feedback_callback, 1
            ),
            self.create_subscription(
                VehicleModeRequest,
                'operator/mode_request',
                self._mode_request_callback,
                1,
            ),
        ]
        self.mode_service = self.create_service(
            SetVehicleMode, 'vehicle/set_mode', self._set_mode_service
        )
        self.kinematics_subscription = self.create_subscription(
            JointState, 'kinematics/joint_setpoints', self._kinematics_candidate_callback, 1)
        self.reset_service = self.create_service(
            Trigger, 'wheel_modules/clear_errors', self._reset_errors_service)
        self.health_subscription = self.create_subscription(
            ComponentHealthArray, 'internal/component_health', self._health_callback, 1)
        self.timer = self.create_timer(self.control_period, self._control)

        self.get_logger().info(
            f'Supervisor initialized in {self._mode_name(self.selected_mode)} mode.'
        )

    def _validate_parameters(self):
        if not self.output_frame_id or self.output_frame_id != self.output_frame_id.strip():
            raise ValueError(
                'output_frame_id must be a non-empty frame without surrounding whitespace'
            )
        positive_values = [
            self.control_period,
            self.transition_duration,
            self.transition_timeout,
            self.velocity_feedback_timeout,
            self.standstill_linear_velocity,
            self.standstill_angular_velocity,
            self.standstill_joint_velocity,
            self.manual_timeout,
            self.autonomy_timeout,
            self.safety_timeout,
            self.battery_timeout,
            self.wheel_status_timeout,
            self.max_velocity_linear,
            self.max_velocity_angular,
            self.max_acceleration_linear,
            self.max_acceleration_angular,
            self.max_direct_steering_velocity,
            self.max_direct_traction_velocity,
            self.max_direct_traction_current,
        ]
        if any(not math.isfinite(value) or value <= 0.0 for value in positive_values):
            raise ValueError('supervisor periods and limits must be finite and positive')
        if self.transition_timeout <= self.transition_duration:
            raise ValueError('mode_transition_timeout must exceed the standstill hold duration')
        if not 0.0 <= self.minimum_battery_percentage <= 1.0:
            raise ValueError('minimum_battery_percentage must be in [0, 1]')
        if not 0.0 < self.warning_speed_scale <= 1.0:
            raise ValueError('warning_speed_scale must be in (0, 1]')
        if not 0.0 < self.override_speed_scale <= 1.0:
            raise ValueError('override_speed_scale must be in (0, 1]')
        if (
            not self.expected_module_names
            or len(set(self.expected_module_names)) != len(self.expected_module_names)
            or any(not name for name in self.expected_module_names)
        ):
            raise ValueError('expected_module_names must be non-empty and unique')
        module_count = len(self.expected_module_names)
        joint_names = (
            self.expected_steering_joint_names
            + self.expected_traction_joint_names
        )
        if (
            len(self.expected_steering_joint_names) != module_count
            or len(self.expected_traction_joint_names) != module_count
            or len(set(joint_names)) != len(joint_names)
            or any(not name for name in joint_names)
        ):
            raise ValueError(
                'steering and traction joint names must uniquely map every module'
            )
        if (
            len(self.direct_steering_encoder_resolutions) != module_count
            or any(not math.isfinite(value) or value <= 0.0
                   for value in self.direct_steering_encoder_resolutions)
        ):
            raise ValueError(
                'direct_steering_encoder_resolutions must provide a positive finite resolution for every steering joint'
            )

    @staticmethod
    def _mode_from_name(name):
        modes = {
            'manual': VehicleModeState.MANUAL,
            'autonomous': VehicleModeState.AUTONOMOUS,
            'kinematicoff': VehicleModeState.KINEMATICOFF,
            'auto_velocity': VehicleModeState.AUTO_VELOCITY,
            'auto_direct': VehicleModeState.AUTO_DIRECT,
        }
        if name not in modes:
            raise ValueError('initial_mode must be manual, auto_velocity, or auto_direct')
        return modes[name]

    @staticmethod
    def _mode_name(mode):
        names = {
            VehicleModeState.MANUAL: 'manual',
            VehicleModeState.AUTO_VELOCITY: 'auto_velocity',
            VehicleModeState.AUTO_DIRECT: 'auto_direct',
            VehicleModeState.UNKNOWN: 'unknown',
        }
        return names.get(mode, 'invalid')

    @staticmethod
    def _finite_twist(msg):
        return all(
            math.isfinite(value)
            for value in (msg.twist.linear.x, msg.twist.linear.y, msg.twist.angular.z)
        )

    def _velocity_frame_valid(self, msg):
        # Candidates are already body-frame velocities, never TF-transformed
        # here. Relabeling a different/unknown frame would change their meaning.
        return msg is not None and msg.header.frame_id == self.output_frame_id

    def _stamp_valid(self, stamp, timeout, now=None):
        if hasattr(stamp, 'sec') and (stamp.sec < 0 or not 0 <= stamp.nanosec < 1000000000):
            return False
        try:
            message_time = Time.from_msg(stamp)
        except (TypeError, ValueError, OverflowError):
            return False
        if message_time.nanoseconds == 0:
            return False
        if now is None:
            now = self.get_clock().now()
        age = (now - message_time).nanoseconds
        return (
            -Duration(seconds=0.1).nanoseconds
            <= age <= Duration(seconds=timeout).nanoseconds
        )

    def _manual_command_callback(self, msg):
        self._refresh_motion_readiness(self.get_clock().now())
        if not self._velocity_frame_valid(msg):
            self.manual_command = None
            self.manual_command_received = None
            self.get_logger().warning(
                f'Rejected manual velocity frame {msg.header.frame_id!r}; '
                f'expected {self.output_frame_id!r}.'
            )
            return
        if self._finite_twist(msg) and self._command_stamp_valid(msg.header.stamp, self.manual_timeout):
            self.manual_command = msg
            self.manual_command_received = self.get_clock().now()
        else:
            self.get_logger().warning('Rejected invalid or stale manual velocity command.')

    def _manual_state_callback(self, msg):
        if self._stamp_valid(msg.header.stamp, self.manual_timeout):
            self.manual_state = msg
            self.manual_state_received = self.get_clock().now()

    def _autonomy_command_callback(self, msg):
        self._refresh_motion_readiness(self.get_clock().now())
        if not self._velocity_frame_valid(msg):
            self.autonomy_command = None
            self.autonomy_command_received = None
            self.get_logger().warning(
                f'Rejected autonomous velocity frame {msg.header.frame_id!r}; '
                f'expected {self.output_frame_id!r}.'
            )
            return
        if self._finite_twist(msg) and self._command_stamp_valid(msg.header.stamp, self.autonomy_timeout):
            self.autonomy_command = msg
            self.autonomy_command_received = self.get_clock().now()
        else:
            self.get_logger().warning('Rejected invalid or stale autonomous velocity command.')

    def _autonomy_joint_callback(self, msg):
        self._refresh_motion_readiness(self.get_clock().now())
        if self._joint_command_valid(msg):
            self.autonomy_joint_command = msg
            self.autonomy_joint_received = self.get_clock().now()
        else:
            self.autonomy_joint_command = None
            self.autonomy_joint_received = None
            self.get_logger().warning('Rejected invalid autonomous joint setpoints.')

    def _safety_callback(self, msg):
        if self._stamp_valid(msg.header.stamp, self.safety_timeout):
            self.safety_state = msg
            self.safety_received = self.get_clock().now()
        self._refresh_motion_readiness(self.get_clock().now())

    def _safety_io_callback(self, msg):
        if self._stamp_valid(msg.header.stamp, self.safety_timeout):
            self.safety_io_state = msg
            self.safety_io_received = self.get_clock().now()
        self._refresh_motion_readiness(self.get_clock().now())

    def _battery_callback(self, msg):
        if self._stamp_valid(msg.header.stamp, self.battery_timeout):
            self.battery_state = msg
            self.battery_received = self.get_clock().now()
        self._refresh_motion_readiness(self.get_clock().now())

    def _wheel_status_callback(self, msg):
        if self._stamp_valid(msg.header.stamp, self.wheel_status_timeout):
            self.wheel_status = msg
            self.wheel_status_received = self.get_clock().now()
        self._refresh_motion_readiness(self.get_clock().now())

    def _joint_state_callback(self, msg):
        if not self._joint_feedback_valid(msg):
            self.joint_state = None
            self.joint_state_received = None
            self.transition_standstill_since = None
            self._refresh_motion_readiness(self.get_clock().now())
            return
        self.joint_state = msg
        self.joint_state_received = self.get_clock().now()
        if any(abs(value) > self.standstill_joint_velocity for value in msg.velocity):
            self.transition_standstill_since = None
        self._refresh_motion_readiness(self.get_clock().now())

    def _joint_feedback_valid(self, msg):
        if not self._stamp_valid(msg.header.stamp, self.wheel_status_timeout):
            return False
        expected_names = set(
            self.expected_steering_joint_names
            + self.expected_traction_joint_names
        )
        if len(msg.name) != len(expected_names) or set(msg.name) != expected_names:
            return False
        if len(msg.position) != len(msg.name) or len(msg.velocity) != len(msg.name):
            return False
        if not all(math.isfinite(value) for value in msg.position + msg.velocity):
            return False
        return self._steering_positions_valid(msg)

    def _steering_positions_valid(self, msg):
        if len(self.direct_steering_encoder_resolutions) != len(self.expected_steering_joint_names):
            return False
        indices = {name: index for index, name in enumerate(msg.name)}
        return all(
            steering_position_representable(msg.position[indices[name]], resolution)
            for name, resolution in zip(
                self.expected_steering_joint_names, self.direct_steering_encoder_resolutions
            )
        )

    def _velocity_feedback_callback(self, msg):
        if (
            not self._finite_twist(msg)
            or msg.header.frame_id != self.output_frame_id
            or not self._stamp_valid(msg.header.stamp, self.velocity_feedback_timeout)
        ):
            self.measured_velocity = None
            self.measured_velocity_received = None
            self.transition_standstill_since = None
            return
        self.measured_velocity = msg
        self.measured_velocity_received = self.get_clock().now()
        if (
            math.hypot(msg.twist.linear.x, msg.twist.linear.y) > self.standstill_linear_velocity
            or abs(msg.twist.angular.z) > self.standstill_angular_velocity
        ):
            self.transition_standstill_since = None

    def _command_stamp_valid(self, stamp, timeout):
        # A future-dated command must not cross a later recovery boundary.
        return self.motion_inputs_ready and self._stamp_valid(stamp, timeout) and (
            Time.from_msg(stamp).nanoseconds <= self.get_clock().now().nanoseconds
        ) and (
            self.command_accept_after is None
            or Time.from_msg(stamp).nanoseconds >= self.command_accept_after.nanoseconds
        )

    def _mode_request_callback(self, msg):
        if msg.requested_mode == VehicleModeState.AUTO_DIRECT:
            self.get_logger().warning(
                'AUTO_DIRECT is terminal/service-only; use vehicle/set_mode.'
            )
            return
        if not self._stamp_valid(msg.header.stamp, 1.0):
            self.get_logger().warning('Rejected stale vehicle mode request.')
            return
        accepted, _, message = self._request_mode(msg.requested_mode)
        if not accepted:
            self.get_logger().warning(message)

    def _set_mode_service(self, request, response):
        accepted, transition_started, message = self._request_mode(request.requested_mode)
        response.accepted = accepted
        response.transition_started = transition_started
        response.current_mode = self.selected_mode
        response.message = message
        return response

    def _request_mode(self, requested_mode):
        if getattr(self, 'fault_codes', set()) or getattr(self, 'reset_started_at', None) is not None:
            return False, False, 'reset faults before selecting a mode'
        valid_modes = {
            VehicleModeState.MANUAL,
            VehicleModeState.AUTO_VELOCITY,
            VehicleModeState.AUTO_DIRECT,
        }
        if requested_mode not in valid_modes:
            return False, False, 'invalid vehicle mode'
        if self.transition_in_progress and not self.transition_timed_out:
            if requested_mode == self.requested_mode:
                return True, False, 'mode transition already in progress'
            return False, False, 'another mode transition is in progress'
        self.requested_mode = requested_mode
        self.transition_in_progress = True
        self._new_authority(preserve_stop=True)
        self.transition_started_at = self.get_clock().now()
        self.transition_from_direct = self.selected_mode == VehicleModeState.AUTO_DIRECT
        self.transition_standstill_since = None
        self.transition_timed_out = False
        self.command_accept_after = self.transition_started_at
        # Discard candidates immediately, preserving last_output only for the
        # controlled stop. Even reselecting the current mode verifies standstill.
        self.manual_command = self.manual_command_received = None
        self.autonomy_command = self.autonomy_command_received = None
        self.autonomy_joint_command = self.autonomy_joint_received = None
        self._publish_mode_state(self.transition_started_at, False,
                                 'mode transition started: stopping before source selection')
        self.get_logger().info(
            f'Mode transition requested: {self._mode_name(self.selected_mode)} -> '
            f'{self._mode_name(requested_mode)}'
        )
        return True, True, 'mode transition started'

    def _received_fresh(self, received_time, timeout, now):
        if received_time is None:
            return False
        age = (now - received_time).nanoseconds
        return 0 <= age <= Duration(seconds=timeout).nanoseconds

    def _message_fresh(self, message, received_time, timeout, now):
        # A recent delivery does not give an old sample another full lifetime.
        # Check both clocks against the same control-cycle time before use.
        return (
            message is not None
            and self._received_fresh(received_time, timeout, now)
            and self._stamp_valid(message.header.stamp, timeout, now)
        )

    def _enable_gate(self, now):
        if 'host' in getattr(self, 'required_health_sources', []):
            health, received = self.component_health.get('host', (None, None))
            if (not self._message_fresh(health, received, self.health_timeout, now)
                    or not health.monitoring_valid or health.fault_active):
                return False, 'host deployment fault or monitoring unavailable'
        # Drive enable depends only on safety and battery readiness. Requiring
        # enabled wheel feedback here would prevent disabled drives from starting.
        if not self._message_fresh(
            self.safety_state, self.safety_received, self.safety_timeout, now
        ):
            return False, 'safety state stale or missing'
        if not self._message_fresh(
            self.safety_io_state, self.safety_io_received, self.safety_timeout, now
        ):
            return False, 'safety I/O state stale or missing'
        if self.safety_io_state.emergency_stop_active:
            return False, 'emergency stop active in safety I/O state'
        if not self.safety_io_state.safety_enable_active:
            return False, 'safety enable inactive in safety I/O state'
        if self.safety_io_state.sto_active:
            return False, 'safe torque off active in safety I/O state'
        if not self.safety_state.communication_ok:
            return False, 'safety communication unavailable'
        if not self.safety_state.system_ready:
            return False, self.safety_state.status_message or 'safety system not ready'
        if (
            self.safety_state.emergency_stop_active
            or self.safety_state.sto_active
        ):
            return False, self.safety_state.status_message or 'safety stop active'
        if (
            self.safety_state.protective_stop_active
            and not self.safety_state.safety_override_active
        ):
            return False, self.safety_state.status_message or 'protective stop active'
        io_protective_stop = not (
            self.safety_io_state.front_left_protective_field_clear
            and self.safety_io_state.rear_right_protective_field_clear
            and self.safety_io_state.front_left_ossd_active
            and self.safety_io_state.rear_right_ossd_active
        )
        if io_protective_stop and not self.safety_state.safety_override_active:
            return False, 'protective stop active in safety I/O state'
        if not self.safety_state.motion_permitted:
            return False, self.safety_state.status_message or 'motion prohibited by safety system'

        if not self._message_fresh(
            self.battery_state, self.battery_received, self.battery_timeout, now
        ):
            return False, 'battery state stale or missing'
        if not self.battery_state.communication_ok:
            return False, self.battery_state.status_message or 'battery communication unavailable'
        if not self.battery_state.system_ready:
            return False, self.battery_state.status_message or 'battery system not ready'
        if not self.battery_state.high_voltage_connected:
            return False, 'battery high voltage is not connected'
        if (
            not math.isfinite(self.battery_state.minimum_percentage)
            or not 0.0 <= self.battery_state.minimum_percentage <= 1.0
            or self.battery_state.minimum_percentage < self.minimum_battery_percentage
        ):
            return False, 'battery state of charge invalid or below enable threshold'
        if getattr(self, 'fault_codes', set()) or getattr(self, 'reset_started_at', None) is not None:
            return False, 'system fault/reset inhibits enable'
        operator = getattr(self, 'manual_state', None)
        if getattr(self, 'require_operator_interlock', False) and (
                not self._message_fresh(operator, self.manual_state_received, self.manual_timeout, now)
                or not operator.connected or not operator.estop_state_known):
            return False, 'required physical operator interlock unavailable'
        if (operator is not None and getattr(operator, 'estop_state_known', False)
                and operator.estop_pressed):
            return False, 'physical operator E-stop pressed'
        if self.wheel_status is not None:
            if not self._message_fresh(self.wheel_status, self.wheel_status_received, self.wheel_status_timeout, now):
                return False, 'driver status stale'
            if any(not getattr(m, 'initialized', True) or not getattr(m, 'position_valid', True)
                   or m.error_code != 0 for m in self.wheel_status.modules):
                return False, 'driver initialization/position/fault prevents enable'
        else:
            return False, 'waiting for passive driver status'
        return True, 'safety, battery and disabled-driver initialization ready'

    def _base_gate(self, now):
        enable_ready, reason = self._enable_gate(now)
        if not enable_ready:
            return False, reason

        if not self._message_fresh(
            self.wheel_status, self.wheel_status_received, self.wheel_status_timeout, now
        ):
            return False, 'wheel-module status stale or missing'
        modules = {module.name: module for module in self.wheel_status.modules}
        if (
            len(self.wheel_status.modules) != len(self.expected_module_names)
            or set(modules) != set(self.expected_module_names)
        ):
            return False, 'wheel-module status does not match configured modules'
        for name in self.expected_module_names:
            module = modules[name]
            if not module.feedback_fresh:
                return False, f'wheel module {name} feedback stale or missing'
            if not module.enabled:
                return False, f'wheel module {name} is disabled'
            if module.error_code != 0:
                return False, f'wheel module {name} reports error {module.error_code}'
        return True, 'hardware ready'

    def _motion_hardware_gate(self, now):
        for name in getattr(self, 'required_health_sources', []):
            message, received = self.component_health.get(name, (None, None))
            if not self._message_fresh(message, received, self.health_timeout, now) or not message.monitoring_valid or message.fault_active:
                return False, 'required component health unavailable: ' + name
        base_ready, reason = self._base_gate(now)
        if not base_ready:
            return False, reason

        if self.selected_mode == VehicleModeState.AUTO_DIRECT:
            if (
                self.safety_state.warning_field_active
                or self._io_warning_active()
                or self.safety_state.safety_override_active
            ):
                return False, 'autonomous direct mode inhibited by reduced-speed safety state'
            if not self._message_fresh(
                self.joint_state, self.joint_state_received, self.wheel_status_timeout, now
            ):
                return False, 'joint-state feedback stale or missing'
        return True, 'motion hardware ready'

    def _invalidate_motion_commands(self, now):
        self._new_authority()
        # Preserve a later mode-switch/recovery boundary across a clock reversal.
        if (
            self.command_accept_after is None
            or now.nanoseconds > self.command_accept_after.nanoseconds
        ):
            self.command_accept_after = now
        self.manual_command = None
        self.manual_command_received = None
        self.autonomy_command = None
        self.autonomy_command_received = None
        self.autonomy_joint_command = None
        self.autonomy_joint_received = None
        self.last_output = [0.0, 0.0, 0.0]
        self.last_control_time = now

    def _refresh_motion_readiness(self, now):
        ready, reason = self._motion_hardware_gate(now)
        if ready != self.motion_inputs_ready:
            # Clear on loss AND recovery: input received during inhibition must
            # not become motion merely because we give it a fresh output stamp.
            self._invalidate_motion_commands(now)
        self.motion_inputs_ready = ready
        return ready, reason

    def _source_gate(self, now):
        hardware_ready, reason = self._motion_hardware_gate(now)
        if not hardware_ready:
            return False, reason

        if self.selected_mode == VehicleModeState.MANUAL:
            if not self._message_fresh(
                self.manual_state, self.manual_state_received, self.manual_timeout, now
            ):
                return False, 'manual-control state stale or missing'
            if not (
                self.manual_state.connected
                and self.manual_state.command_active
            ):
                return False, self.manual_state.status_message or 'manual command inactive'
            if not self._message_fresh(
                self.manual_command, self.manual_command_received, self.manual_timeout, now
            ):
                return False, 'manual velocity command stale or missing'
            if not self._velocity_frame_valid(self.manual_command):
                return False, 'manual velocity command frame mismatch'
            return True, 'manual command active'

        if self.selected_mode == VehicleModeState.AUTO_VELOCITY:
            if not self._message_fresh(
                self.autonomy_command, self.autonomy_command_received,
                self.autonomy_timeout, now
            ):
                return False, 'autonomous velocity command stale or missing'
            if not self._velocity_frame_valid(self.autonomy_command):
                return False, 'autonomous velocity command frame mismatch'
            return True, 'autonomous velocity command active'

        if self.selected_mode == VehicleModeState.AUTO_DIRECT:
            if not self._message_fresh(
                self.autonomy_joint_command, self.autonomy_joint_received,
                self.autonomy_timeout, now
            ):
                return False, 'autonomous joint setpoints stale or missing'
            return True, 'autonomous joint setpoints active'
        return False, 'no valid vehicle mode'

    def _joint_command_valid(self, msg):
        if not self._command_stamp_valid(msg.header.stamp, self.autonomy_timeout):
            return False
        expected_names = set(
            self.expected_steering_joint_names
            + self.expected_traction_joint_names
        )
        if len(msg.name) != len(expected_names):
            return False
        if len(set(msg.name)) != len(msg.name) or set(msg.name) != expected_names:
            return False
        if len(msg.position) != len(msg.name) or len(msg.velocity) != len(msg.name):
            return False
        if msg.effort:
            return False
        if not all(math.isfinite(value) for value in msg.position + msg.velocity):
            return False
        if msg.effort and not all(math.isfinite(value) for value in msg.effort):
            return False
        indices = {name: index for index, name in enumerate(msg.name)}
        if any(not 0 <= msg.velocity[indices[name]] <= self.max_direct_steering_velocity
               for name in self.expected_steering_joint_names):
            return False
        if any(abs(msg.velocity[indices[name]]) > self.max_direct_traction_velocity
               for name in self.expected_traction_joint_names):
            return False
        return self._steering_positions_valid(msg)

    def _io_warning_active(self):
        return self.safety_io_state is not None and not (
            self.safety_io_state.front_left_warning_field_clear
            and self.safety_io_state.rear_right_warning_field_clear
        )

    def _speed_scale(self):
        scale = 1.0
        if (
            self.safety_state is not None
            and self.safety_state.warning_field_active
        ) or self._io_warning_active():
            scale = min(scale, self.warning_speed_scale)
        if self.safety_state is not None and self.safety_state.safety_override_active:
            scale = min(scale, self.override_speed_scale)
        return scale

    def _publish_velocity(self, source, now, immediate_stop=False):
        output = TwistStamped()
        output.header.stamp = now.to_msg()
        output.header.frame_id = self.output_frame_id

        if source is None or immediate_stop:
            self.last_output = [0.0, 0.0, 0.0]
            self.last_control_time = now
            self._publish_velocity_candidate(output, source, now)
            return

        scale = self._speed_scale()
        target_x, target_y = clamp_planar(
            source.twist.linear.x,
            source.twist.linear.y,
            self.max_velocity_linear * scale,
        )
        target_angular = clamp(
            source.twist.angular.z,
            -self.max_velocity_angular * scale,
            self.max_velocity_angular * scale,
        )
        dt = max((now - self.last_control_time).nanoseconds * 1.0e-9, 0.0)
        output_x = slew(
            target_x, self.last_output[0], self.max_acceleration_linear * scale * dt
        )
        output_y = slew(
            target_y, self.last_output[1], self.max_acceleration_linear * scale * dt
        )
        output.twist.linear.x, output.twist.linear.y = clamp_planar(
            output_x,
            output_y,
            self.max_velocity_linear * scale,
        )
        output.twist.angular.z = clamp(
            slew(
                target_angular,
                self.last_output[2],
                self.max_acceleration_angular * scale * dt,
            ),
            -self.max_velocity_angular * scale,
            self.max_velocity_angular * scale,
        )
        self.last_output = [
            output.twist.linear.x,
            output.twist.linear.y,
            output.twist.angular.z,
        ]
        self.last_control_time = now
        self._publish_velocity_candidate(output, source, now)

    def _publish_direct(self, source, now):
        output = copy.deepcopy(source)
        output.header.stamp = now.to_msg()
        indices = {name: index for index, name in enumerate(output.name)}
        for name in self.expected_steering_joint_names:
            index = indices[name]
            output.velocity[index] = clamp(
                abs(output.velocity[index]),
                0.0,
                self.max_direct_steering_velocity,
            )
        for name in self.expected_traction_joint_names:
            index = indices[name]
            if self.direct_traction_control_mode == 'velocity':
                output.velocity[index] = clamp(
                    output.velocity[index],
                    -self.max_direct_traction_velocity,
                    self.max_direct_traction_velocity,
                )
            else:
                output.effort[index] = clamp(
                    output.effort[index],
                    -self.max_direct_traction_current,
                    self.max_direct_traction_current,
                )
        output.effort = []
        self._emit_final(output, source.header.stamp, now, WheelModuleCommandArray.MOTION)

    def _publish_direct_stop(self, now):
        if not self._message_fresh(self.joint_state, self.joint_state_received, self.wheel_status_timeout, now):
            return
        output = JointState()
        output.header.stamp = now.to_msg()
        output.name = (
            self.expected_steering_joint_names
            + self.expected_traction_joint_names
        )
        feedback_positions = {}
        if self.joint_state is not None:
            feedback_positions = dict(zip(self.joint_state.name, self.joint_state.position))
        command_positions = {}
        if self.autonomy_joint_command is not None:
            command_positions = dict(
                zip(
                    self.autonomy_joint_command.name,
                    self.autonomy_joint_command.position,
                )
            )
        output.position = [
            feedback_positions.get(name, command_positions.get(name, 0.0))
            if name in self.expected_steering_joint_names
            else 0.0
            for name in output.name
        ]
        output.velocity = [0.0] * len(output.name)
        output.effort = []
        self._emit_final(output, now.to_msg(), now, WheelModuleCommandArray.HOLD)

    def _publish_mode_state(self, now, motion_permitted, status_message):
        state = VehicleModeState()
        state.header.stamp = now.to_msg()
        errors, safety = self._system_conditions(now)
        faults = getattr(self, 'fault_codes', set())
        state.current_mode = (VehicleModeState.ERROR if faults or errors else
                              VehicleModeState.SAFETY if safety else self.selected_mode)
        state.selected_mode = self.selected_mode
        state.requested_mode = self.requested_mode
        state.transition_in_progress = self.transition_in_progress
        state.motion_permitted = bool(motion_permitted and not faults and not errors and not safety
                                      and getattr(self, 'reset_started_at', None) is None)
        state.safety_stop_active = safety
        state.error_active = bool(faults or errors)
        state.reset_in_progress = getattr(self, 'reset_started_at', None) is not None
        state.reset_id = getattr(self, 'reset_id', 0)
        state.reset_result = getattr(self, 'reset_result', '')
        state.reason_codes = sorted(faults | set(errors))
        state.supervisor_instance = getattr(self, 'supervisor_instance', '')
        state.authority_generation = getattr(self, 'authority_generation', 0)
        module_names = [m.name for m in self.wheel_status.modules] if self.wheel_status is not None else []
        state.readiness_known = (self._message_fresh(self.wheel_status, self.wheel_status_received, self.wheel_status_timeout, now)
                                and len(module_names) == len(self.expected_module_names)
                                and set(module_names) == set(self.expected_module_names))
        state.drives_enabled = state.readiness_known and all(m.enabled for m in self.wheel_status.modules)
        state.ready_to_drive = state.readiness_known and all(
            getattr(m, 'initialized', True) and getattr(m, 'position_valid', True)
            and m.feedback_fresh and m.error_code == 0 for m in self.wheel_status.modules)
        monitors_known = all(self._message_fresh(message, received, timeout, now)
                             for message, received, timeout in (
                                 (self.safety_state, self.safety_received, self.safety_timeout),
                                 (self.safety_io_state, self.safety_io_received, self.safety_timeout),
                                 (self.battery_state, self.battery_received, self.battery_timeout)))
        if state.ready_to_drive and monitors_known:
            self.startup_complete = True
        if not getattr(self, 'startup_complete', False) and not state.error_active and not safety:
            state.current_mode = VehicleModeState.UNKNOWN
            state.motion_permitted = False
        state.status_message = status_message
        self.mode_state_publisher.publish(state)

    def _standstill_gate(self, now):
        for message, received, timeout, label in (
            (self.measured_velocity, self.measured_velocity_received,
             self.velocity_feedback_timeout, 'vehicle velocity'),
            (self.joint_state, self.joint_state_received,
             self.wheel_status_timeout, 'joint velocity'),
        ):
            if message is None or not self._received_fresh(received, timeout, now):
                return False, f'{label} feedback stale or missing'
            stamp_ns = Time.from_msg(message.header.stamp).nanoseconds
            age = (now.nanoseconds - stamp_ns) * 1.0e-9
            if not 0.0 <= age <= timeout:
                return False, f'{label} feedback timestamp invalid or stale'
            if stamp_ns < self.transition_started_at.nanoseconds:
                return False, f'waiting for {label} feedback after mode request'

        velocity = self.measured_velocity.twist
        if (
            math.hypot(velocity.linear.x, velocity.linear.y) > self.standstill_linear_velocity
            or abs(velocity.angular.z) > self.standstill_angular_velocity
        ):
            return False, 'vehicle is still moving'
        if any(abs(value) > self.standstill_joint_velocity for value in self.joint_state.velocity):
            return False, 'joints are still moving'
        return True, 'standstill confirmed'

    def _control_transition(self, now):
        elapsed = (now - self.transition_started_at).nanoseconds * 1.0e-9
        if elapsed >= self.transition_timeout:
            self.transition_timed_out = True
        hardware_ready, hardware_reason = self._base_gate(now)
        if self.transition_from_direct:
            # Autonomy owns its kinematics in this mode. Send only the direct
            # zero-velocity/current command, avoiding competing driver inputs.
            self._publish_direct_stop(now)
        else:
            # Ramp the existing supervised velocity to zero while hardware is
            # ready; safety/battery loss still commands an immediate stop.
            self._publish_velocity(
                TwistStamped(), now,
                immediate_stop=not hardware_ready or self.transition_timed_out,
            )

        if self.transition_timed_out:
            self.transition_standstill_since = None
            self._publish_mode_state(
                now, False, 'mode transition timed out: motion blocked; request mode again to retry'
            )
            return

        stationary, reason = self._standstill_gate(now)
        if not self.transition_from_direct and any(value != 0.0 for value in self.last_output):
            stationary, reason = False, 'ramping supervised velocity to zero'
        if not hardware_ready or not stationary:
            self.transition_standstill_since = None
            self._publish_mode_state(
                now, False, f'mode transition: {hardware_reason if not hardware_ready else reason}'
            )
            return

        if self.transition_standstill_since is None:
            self.transition_standstill_since = now
        held = (now - self.transition_standstill_since).nanoseconds * 1.0e-9
        latest_proof_ns = min(
            Time.from_msg(self.measured_velocity.header.stamp).nanoseconds,
            Time.from_msg(self.joint_state.header.stamp).nanoseconds,
        )
        proof_duration = (
            latest_proof_ns - self.transition_standstill_since.nanoseconds
        ) * 1.0e-9
        if held < self.transition_duration or proof_duration < self.transition_duration:
            self._publish_mode_state(now, False, 'mode transition: confirming sustained standstill')
            return

        self.selected_mode = self.requested_mode
        self.transition_in_progress = False
        self.transition_started_at = None
        self.transition_from_direct = False
        self.transition_standstill_since = None
        # Commands buffered before the switch must not move the new source.
        self._invalidate_motion_commands(now)
        self._publish_mode_state(now, False, 'mode transition complete: waiting for new command')
        self.get_logger().info(
            f'Mode transition complete: {self._mode_name(self.selected_mode)}'
        )

    def _control(self):
        now = self.get_clock().now()
        errors, _ = self._system_conditions(now)
        if not hasattr(self, 'fault_codes'):
            self.fault_codes = set()
        if errors:
            self.fault_codes.update(errors)
        self._observe_reset(now, errors)
        enable_ready, _ = self._enable_gate(now)
        module_command = WheelModuleOperationCommand()
        module_command.header.stamp = now.to_msg()
        module_command.enable = enable_ready
        module_command.supervisor_instance = getattr(self, 'supervisor_instance', '')
        module_command.authority_generation = getattr(self, 'authority_generation', 0)
        module_command.reset_id = self.reset_id if getattr(self, 'reset_started_at', None) is not None else 0
        self.module_command_publisher.publish(module_command)

        # Also observe expiry when no new hardware callbacks arrive.
        self._refresh_motion_readiness(now)

        if self.transition_in_progress:
            self._control_transition(now)
            return

        source_ready, reason = self._source_gate(now)
        if self.selected_mode == VehicleModeState.AUTO_DIRECT:
            if source_ready:
                self._publish_direct(self.autonomy_joint_command, now)
            else:
                self._publish_direct_stop(now)
            self._publish_mode_state(now, source_ready, reason)
            return

        source = None
        if source_ready and self.selected_mode == VehicleModeState.MANUAL:
            source = self.manual_command
        elif source_ready and self.selected_mode == VehicleModeState.AUTO_VELOCITY:
            source = self.autonomy_command
        self._publish_velocity(source, now, immediate_stop=not source_ready)
        self._publish_mode_state(now, source_ready, reason)

    def _new_authority(self, preserve_stop=False):
        self.authority_generation = getattr(self, 'authority_generation', 0) + 1
        self.velocity_ledger = {}
        if not preserve_stop:
            self.last_final = {}

    def _system_conditions(self, now):
        errors = []
        started = getattr(self, 'started_at', now)
        grace = (now - started).nanoseconds < int(getattr(self, 'startup_grace', 10.0) * 1e9)
        for message, received, timeout, code in (
                (self.safety_state, self.safety_received, self.safety_timeout, 'SAFETY_MONITOR_LOST'),
                (self.safety_io_state, self.safety_io_received, self.safety_timeout, 'SAFETY_IO_LOST'),
                (self.battery_state, self.battery_received, self.battery_timeout, 'BMS_MONITOR_LOST'),
                (self.wheel_status, self.wheel_status_received, self.wheel_status_timeout, 'DRIVER_MONITOR_LOST')):
            if not self._message_fresh(message, received, timeout, now) and not grace:
                errors.append(code)
        safety = self.safety_state
        io = self.safety_io_state
        safety_fresh = self._message_fresh(safety, self.safety_received, self.safety_timeout, now)
        confirmed_stop = bool(safety_fresh and safety.communication_ok and
            ((safety.emergency_stop_active and getattr(io, 'emergency_stop_state_known', False))
             or (safety.sto_active and getattr(io, 'sto_state_known', False))
             or (safety.protective_stop_active and not safety.safety_override_active
                 and getattr(io, 'scanner_state_known', False))
             or (getattr(io, 'safety_enable_state_known', False) and not io.safety_enable_active)))
        if safety_fresh and not safety.communication_ok and not grace:
            errors.append('SAFETY_MONITOR_LOST')
        operator = getattr(self, 'manual_state', None)
        if operator is not None and getattr(operator, 'estop_state_known', False) and operator.estop_pressed:
            # A known pressed E-stop remains inhibited even after telemetry expires.
            confirmed_stop = True
        if (self.selected_mode == VehicleModeState.MANUAL or getattr(self, 'require_operator_interlock', False)) and not grace:
            if not self._message_fresh(operator, self.manual_state_received, self.manual_timeout, now) or not operator.connected:
                errors.append('OPERATOR_LOST')
            elif getattr(self, 'require_operator_interlock', False) and not operator.estop_state_known:
                errors.append('OPERATOR_ESTOP_UNKNOWN')
        battery = self.battery_state
        if battery is not None and not battery.communication_ok and not grace:
            errors.append('BMS_MONITOR_LOST')
        if battery is not None and getattr(battery, 'faulted_batteries', []):
            errors.append('BMS_FAULT')
        if self.wheel_status is not None:
            names = [m.name for m in self.wheel_status.modules]
            if not grace and (len(names) != len(self.expected_module_names) or set(names) != set(self.expected_module_names)):
                errors.append('DRIVER_STATUS_INVALID')
            for module in self.wheel_status.modules:
                if module.error_code:
                    errors.append('DRIVE_FAULT:' + module.name)
                if not grace and (not module.feedback_fresh or not getattr(module, 'initialized', True)):
                    errors.append('DRIVE_FEEDBACK_LOST:' + module.name)
                if not grace and not getattr(module, 'position_valid', True):
                    errors.append('POSITION_CONTINUITY_UNKNOWN:' + module.name)
        if self.transition_timed_out and getattr(self, 'reset_started_at', None) is None:
            errors.append('MODE_TRANSITION_TIMEOUT')
        for name in getattr(self, 'required_health_sources', []):
            message, received = self.component_health.get(name, (None, None))
            if (name == 'host' or not grace) and (not self._message_fresh(message, received, self.health_timeout, now)
                    or not message.monitoring_valid or message.fault_active):
                errors.append('COMPONENT_HEALTH_LOST:' + name)
        return errors, confirmed_stop

    def _health_callback(self, array):
        now = self.get_clock().now()
        ids = [m.component_id for m in array.components]
        if len(ids) != len(set(ids)):
            self.component_health.clear()
            return
        for message in array.components:
            if message.component_id in self.required_health_sources:
                self.component_health[message.component_id] = (message, now)

    def _observe_reset(self, now, errors):
        proof_times_valid = (self.joint_state is not None and self.measured_velocity is not None and
                             all(0 < Time.from_msg(m.header.stamp).nanoseconds <= now.nanoseconds
                                 for m in (self.joint_state, self.measured_velocity)))
        stationary = (proof_times_valid and self._joint_feedback_valid(self.joint_state)
                      and self._finite_twist(self.measured_velocity)
                      and self.measured_velocity.header.frame_id == self.output_frame_id
                      and self._message_fresh(self.joint_state, self.joint_state_received, self.wheel_status_timeout, now)
                      and self._message_fresh(self.measured_velocity, self.measured_velocity_received, self.velocity_feedback_timeout, now)
                      and all(abs(v) <= self.standstill_joint_velocity for v in self.joint_state.velocity)
                      and math.hypot(self.measured_velocity.twist.linear.x, self.measured_velocity.twist.linear.y) <= self.standstill_linear_velocity
                      and abs(self.measured_velocity.twist.angular.z) <= self.standstill_angular_velocity)
        if not stationary:
            self.standstill_since = None
        elif getattr(self, 'standstill_since', None) is None:
            self.standstill_since = now
        started = getattr(self, 'reset_started_at', None)
        if started is None:
            return
        acknowledged = (self.wheel_status is not None and self._message_fresh(
            self.wheel_status, self.wheel_status_received, self.wheel_status_timeout, now)
            and Time.from_msg(self.wheel_status.header.stamp).nanoseconds > started.nanoseconds
            and all(m.reset_id == self.reset_id and m.reset_acknowledged for m in self.wheel_status.modules))
        if acknowledged and not errors and stationary:
            self.fault_codes.clear()
            self.reset_started_at = None
            self.reset_result = 'completed; fresh command and manual rearm required'
            self.transition_timed_out = self.transition_in_progress = False
            self._invalidate_motion_commands(now)
        elif (now - started).nanoseconds > int(self.reset_timeout * 1e9):
            self.reset_started_at = None
            self.reset_result = 'reset timeout'
            self.fault_codes.add('RESET_TIMEOUT')

    def _reset_errors_service(self, request, response):
        del request
        now = self.get_clock().now()
        self._observe_reset(now, self._system_conditions(now)[0])
        proof_ns = min(Time.from_msg(m.header.stamp).nanoseconds for m in
                       (self.joint_state, self.measured_velocity)) if self.standstill_since is not None else 0
        if self.reset_started_at is not None or (self.transition_in_progress and not self.transition_timed_out):
            response.success, response.message = False, 'reset/transition already active'
        elif self.standstill_since is None or proof_ns - self.standstill_since.nanoseconds < int(self.transition_duration * 1e9):
            response.success, response.message = False, 'fresh sustained measured standstill required'
        else:
            self.reset_id += 1
            self.transition_in_progress = False
            self.reset_started_at = now
            self.reset_result = 'accepted; recovery not yet verified'
            self._invalidate_motion_commands(now)
            response.success, response.message = True, self.reset_result
        return response

    def _publish_velocity_candidate(self, output, source, now):
        if hasattr(self, 'velocity_ledger'):
            purpose = (WheelModuleCommandArray.CONTROLLED_STOP if self.transition_in_progress else
                       WheelModuleCommandArray.MOTION if source is not None else WheelModuleCommandArray.HOLD)
            self.velocity_ledger[now.nanoseconds] = (self.authority_generation, purpose,
                source.header.stamp if source is not None and not self.transition_in_progress else now.to_msg())
            self.velocity_ledger = {k: v for k, v in self.velocity_ledger.items() if now.nanoseconds - k <= int(self.control_period * 10 * 1e9)}
        self.cmd_vel_publisher.publish(output)

    def _kinematics_candidate_callback(self, msg):
        now = self.get_clock().now()
        entry = self.velocity_ledger.pop(Time.from_msg(msg.header.stamp).nanoseconds, None)
        if entry is None or entry[0] != self.authority_generation or self.selected_mode == VehicleModeState.KINEMATICOFF:
            return
        if not self._joint_feedback_valid(msg) or msg.effort:
            return
        if entry[1] == WheelModuleCommandArray.MOTION and not self._source_gate(now)[0]:
            return
        self._emit_final(msg, entry[2], now, entry[1])

    def _emit_final(self, msg, source_stamp, now, purpose):
        if not hasattr(self, 'final_command_publisher'):
            return
        indices = {name: i for i, name in enumerate(msg.name)}
        if len(indices) != len(msg.name) or set(indices) != set(self.expected_steering_joint_names + self.expected_traction_joint_names):
            return
        output = WheelModuleCommandArray()
        output.header.stamp = now.to_msg()
        output.source_stamp = source_stamp
        output.supervisor_instance = self.supervisor_instance
        output.authority_generation = self.authority_generation
        output.selected_mode = self.selected_mode
        output.purpose = purpose
        feedback = dict(zip(self.joint_state.name, self.joint_state.position)) if self.joint_state is not None else {}
        for name, steering, traction in zip(self.expected_module_names, self.expected_steering_joint_names, self.expected_traction_joint_names):
            record = WheelModuleCommand()
            record.name = name
            record.steering_position = msg.position[indices[steering]]
            record.steering_profile_velocity = msg.velocity[indices[steering]]
            record.traction_velocity = msg.velocity[indices[traction]]
            if purpose != WheelModuleCommandArray.MOTION:
                if steering not in feedback:
                    return
                record.steering_position = feedback[steering]
                record.steering_profile_velocity = 0.0
                previous = self.last_final.get(name, 0.0)
                if purpose == WheelModuleCommandArray.HOLD or record.traction_velocity * previous < 0:
                    record.traction_velocity = 0.0
                else:
                    record.traction_velocity = math.copysign(min(abs(previous), abs(record.traction_velocity)), previous)
            output.modules.append(record)
        self.last_final = {m.name: m.traction_velocity for m in output.modules}
        self.final_command_publisher.publish(output)


def main(args=None):
    rclpy.init(args=args)
    node = MoboticSupervisor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
