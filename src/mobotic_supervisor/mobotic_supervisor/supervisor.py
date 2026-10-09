import copy
import math

import rclpy
from geometry_msgs.msg import TwistStamped
from mobotic_interfaces.msg import (
    BatterySystemState,
    ManualControlState,
    SafetyIOState,
    SafetyState,
    VehicleModeRequest,
    VehicleModeState,
    WheelModuleCommand,
    WheelModuleStatusArray,
)
from mobotic_interfaces.srv import SetVehicleMode
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.time import Time
from sensor_msgs.msg import JointState

from .control_math import clamp, clamp_planar, slew, steering_position_representable


class MoboticSupervisor(Node):
    def __init__(self):
        super().__init__('mobotic_supervisor')

        self.declare_parameter('initial_mode', 'manual')
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
        if direct_traction_mode not in ('velocity', 'current'):
            raise ValueError(
                'direct_traction_control_mode must be velocity or current'
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

        self.current_mode = initial_mode
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

        self.cmd_vel_publisher = self.create_publisher(TwistStamped, 'cmd_vel', 1)
        self.direct_command_publisher = self.create_publisher(
            JointState, 'supervisor/joint_setpoints', 1
        )
        self.mode_state_publisher = self.create_publisher(
            VehicleModeState, 'vehicle/mode_state', 1
        )
        self.module_command_publisher = self.create_publisher(
            WheelModuleCommand, 'wheel_modules/command', 1
        )

        self.input_subscriptions = [
            self.create_subscription(
                TwistStamped, 'manual/cmd_vel', self._manual_command_callback, 1
            ),
            self.create_subscription(
                ManualControlState, 'manual/state', self._manual_state_callback, 1
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
                TwistStamped, 'agv_vel', self._velocity_feedback_callback, 1
            ),
            self.create_subscription(
                VehicleModeRequest,
                'vehicle/mode_request',
                self._mode_request_callback,
                1,
            ),
        ]
        self.mode_service = self.create_service(
            SetVehicleMode, 'vehicle/set_mode', self._set_mode_service
        )
        self.timer = self.create_timer(self.control_period, self._control)

        self.get_logger().info(
            f'Supervisor initialized in {self._mode_name(self.current_mode)} mode.'
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
        message_time = Time.from_msg(stamp)
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
        return self.motion_inputs_ready and self._stamp_valid(stamp, timeout) and (
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
        response.current_mode = self.current_mode
        response.message = message
        return response

    def _request_mode(self, requested_mode):
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
        self.transition_started_at = self.get_clock().now()
        self.transition_from_direct = self.current_mode == VehicleModeState.AUTO_DIRECT
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
            f'Mode transition requested: {self._mode_name(self.current_mode)} -> '
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
        return True, 'safety and battery ready'

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
        base_ready, reason = self._base_gate(now)
        if not base_ready:
            return False, reason

        if self.current_mode == VehicleModeState.AUTO_DIRECT:
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
        if not ready or not self.motion_inputs_ready:
            # Clear on loss AND recovery: input received during inhibition must
            # not become motion merely because we give it a fresh output stamp.
            self._invalidate_motion_commands(now)
        self.motion_inputs_ready = ready
        return ready, reason

    def _source_gate(self, now):
        hardware_ready, reason = self._motion_hardware_gate(now)
        if not hardware_ready:
            return False, reason

        if self.current_mode == VehicleModeState.MANUAL:
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

        if self.current_mode == VehicleModeState.AUTO_VELOCITY:
            if not self._message_fresh(
                self.autonomy_command, self.autonomy_command_received,
                self.autonomy_timeout, now
            ):
                return False, 'autonomous velocity command stale or missing'
            if not self._velocity_frame_valid(self.autonomy_command):
                return False, 'autonomous velocity command frame mismatch'
            return True, 'autonomous velocity command active'

        if self.current_mode == VehicleModeState.AUTO_DIRECT:
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
        if len(msg.effort) not in (0, len(msg.name)):
            return False
        if not all(math.isfinite(value) for value in msg.position + msg.velocity):
            return False
        if msg.effort and not all(math.isfinite(value) for value in msg.effort):
            return False
        if self.direct_traction_control_mode == 'current' and not msg.effort:
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
            self.cmd_vel_publisher.publish(output)
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
        self.cmd_vel_publisher.publish(output)

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
        self.direct_command_publisher.publish(output)

    def _publish_direct_stop(self, now):
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
        output.effort = [0.0] * len(output.name)
        self.direct_command_publisher.publish(output)

    def _publish_mode_state(self, now, motion_permitted, status_message):
        state = VehicleModeState()
        state.header.stamp = now.to_msg()
        state.current_mode = self.current_mode
        state.requested_mode = self.requested_mode
        state.transition_in_progress = self.transition_in_progress
        state.motion_permitted = motion_permitted
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

        self.current_mode = self.requested_mode
        self.transition_in_progress = False
        self.transition_started_at = None
        self.transition_from_direct = False
        self.transition_standstill_since = None
        # Commands buffered before the switch must not move the new source.
        self._invalidate_motion_commands(now)
        self._publish_mode_state(now, False, 'mode transition complete: waiting for new command')
        self.get_logger().info(
            f'Mode transition complete: {self._mode_name(self.current_mode)}'
        )

    def _control(self):
        now = self.get_clock().now()
        enable_ready, _ = self._enable_gate(now)
        module_command = WheelModuleCommand()
        module_command.header.stamp = now.to_msg()
        module_command.enable = enable_ready
        module_command.clear_errors = False
        self.module_command_publisher.publish(module_command)

        # Also observe expiry when no new hardware callbacks arrive.
        self._refresh_motion_readiness(now)

        if self.transition_in_progress:
            self._control_transition(now)
            return

        source_ready, reason = self._source_gate(now)
        if self.current_mode == VehicleModeState.AUTO_DIRECT:
            if source_ready:
                self._publish_direct(self.autonomy_joint_command, now)
            else:
                self._publish_direct_stop(now)
            self._publish_mode_state(now, source_ready, reason)
            return

        source = None
        if source_ready and self.current_mode == VehicleModeState.MANUAL:
            source = self.manual_command
        elif source_ready and self.current_mode == VehicleModeState.AUTO_VELOCITY:
            source = self.autonomy_command
        self._publish_velocity(source, now, immediate_stop=not source_ready)
        self._publish_mode_state(now, source_ready, reason)


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
