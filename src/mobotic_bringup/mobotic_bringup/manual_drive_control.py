"""Explicit manual-only commissioning authority; independent hardware safety required.

Uses the driver's existing leases and reset protocol without changing watchdogs.
No supervisor, autonomy, ROS scanner policy or battery readiness gates.
"""
import copy
import json
from mobotic_kinematics.manual_steering import ManualSteering
import math
import time
import uuid

import rclpy
from geometry_msgs.msg import TwistStamped
from mobotic_interfaces.msg import (OperatorState, VehicleModeState, WheelModuleCommand,
    WheelModuleCommandArray, WheelModuleOperationCommand, WheelModuleStatusArray)
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger


def stamp_ns(stamp):
    if stamp.sec < 0 or not 0 <= stamp.nanosec < 1_000_000_000:
        return 0
    return stamp.sec * 1_000_000_000 + stamp.nanosec


class ManualDriveControl(Node):
    # This authority has no scanner-zone input, even with a faster robot profile.
    SPEED_LIMIT = 0.3

    def __init__(self):
        super().__init__('manual_drive_control')
        defaults = {
            'steering_geometry': '{}',
            'module_names': ['front_left'], 'steering_joint_names': ['front_left_steering'],
            'traction_joint_names': ['front_left_traction'], 'output_frame_id': 'base_link',
            'control_period': 0.02, 'manual_timeout': 0.25, 'feedback_timeout': 0.3,
            'candidate_timeout': 0.1, 'reset_timeout': 20.0,
            'control_gap_timeout': 0.3, 'clock_skew_tolerance': 0.04,
            'max_velocity_linear': 1.25, 'max_velocity_angular': 1.51,
            'max_steering_velocity': 1.0, 'max_traction_velocity': 1.25,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
            setattr(self, name, self.get_parameter(name).value)
        geometry = json.loads(self.steering_geometry)
        self.steering_policy = ManualSteering(geometry) if geometry else None
        if self.steering_policy:
            self.steering_policy.max_yaw = self.max_velocity_angular
        names = self.steering_joint_names + self.traction_joint_names
        if (not self.module_names or len(set(self.module_names)) != len(self.module_names)
                or len(self.steering_joint_names) != len(self.module_names)
                or len(self.traction_joint_names) != len(self.module_names)
                or len(set(names)) != len(names) or not all(names)):
            raise ValueError('complete, unique module and joint names required')
        if any(not math.isfinite(getattr(self, n)) or getattr(self, n) <= 0
               for n, v in defaults.items() if isinstance(v, float)):
            raise ValueError('limits and timeouts must be finite and positive')
        if self.control_period >= min(self.manual_timeout, self.candidate_timeout):
            raise ValueError('control period must be shorter than command lifetimes')
        if self.clock_skew_tolerance >= min(self.manual_timeout, self.feedback_timeout):
            raise ValueError('clock skew tolerance must be shorter than input lifetimes')
        if self.control_gap_timeout < self.candidate_timeout:
            raise ValueError('control gap timeout cannot be shorter than candidate timeout')
        self.instance, self.generation, self.reset_id = 'manual-' + uuid.uuid4().hex, 1, 0
        self.phase, self.reason = 'disabled', 'neutral controls and B2/enable_drives required'
        self.samples, self.ledger = {}, {}
        self.boundary = self.get_clock().now().nanoseconds
        self.recovery_started, self.standstill_since = None, None
        self.recovery_stamp, self.startup_seen = 0, False
        self.last_tick, self.last_ros_tick = time.monotonic(), self.boundary
        self.mode_pub = self.create_publisher(VehicleModeState, 'vehicle/mode', 1)
        self.operation_pub = self.create_publisher(WheelModuleOperationCommand, 'wheel_modules/operation_command', 1)
        self.command_pub = self.create_publisher(WheelModuleCommandArray, 'wheel_modules/command', 1)
        self.velocity_pub = self.create_publisher(TwistStamped, 'vehicle/cmd_vel', 1)
        self.inputs = [self.create_subscription(kind, topic,
            lambda msg, key=key: self._receive(key, msg), 1)
            for key, kind, topic in (
                ('operator', OperatorState, 'operator/state'),
                ('homing', OperatorState, 'operator/homing_state'),
                ('command', TwistStamped, 'operator/cmd_vel'),
                ('status', WheelModuleStatusArray, 'wheel_modules/status'),
                ('joints', JointState, 'joint_states'))]
        self.inputs.append(self.create_subscription(JointState, 'kinematics/joint_setpoints', self._candidate, 1))
        self.control_services = [self.create_service(Trigger, topic, callback) for topic, callback in (
            ('wheel_modules/enable_drives', self._enable),
            ('wheel_modules/clear_errors', self._enable),
            ('wheel_modules/disable_drives', self._disable_service))]
        self.timer = self.create_timer(self.control_period, self._tick)
        self.get_logger().warning('MANUAL-ONLY TEST: no supervisor/scanner/battery ROS gates. '
                                 'Independent hardware safety required. B2 enables; autonomy unavailable.')

    def _receive(self, key, msg):
        stamp = stamp_ns(msg.header.stamp)
        limit = self.manual_timeout if key in ('operator', 'command', 'homing') else self.feedback_timeout
        age = self.get_clock().now().nanoseconds - stamp
        if stamp <= 0 or not -self.clock_skew_tolerance * 1e9 <= age <= limit * 1e9:
            return
        previous = self.samples.get(key)
        if previous is not None and stamp <= stamp_ns(previous[0].header.stamp):
            return
        self.samples[key] = (msg, time.monotonic())

    def _fresh(self, key):
        entry = self.samples.get(key)
        if entry is None:
            return None
        msg, received = entry
        limit = self.manual_timeout if key in ('operator', 'command', 'homing') else self.feedback_timeout
        age = self.get_clock().now().nanoseconds - stamp_ns(msg.header.stamp)
        return msg if (-self.clock_skew_tolerance * 1e9 <= age <= limit * 1e9
                       and 0 <= time.monotonic() - received <= limit) else None

    def _operator(self, neutral=False):
        state = self._fresh('operator')
        homing = self._fresh('homing')
        if (self.phase == 'recovering' and state is not None and state.source_id == 'remdevice'
                and homing is not None and homing.source_id == 'remdevice_homing'
                and not state.estop_pressed and homing.estop_state_known):
            state = homing
        return bool(state is not None and state.connected and not state.estop_pressed
                    and (not neutral or state.controls_neutral))

    def _modules(self):
        msg = self._fresh('status')
        if (msg is None or len(msg.modules) != len(self.module_names)
                or {m.name for m in msg.modules} != set(self.module_names)):
            return None
        return msg.modules

    def _ready(self):
        modules = self._modules()
        return bool(modules and all(
            m.enabled and m.initialized and m.startup_complete and not m.startup_failed
            and not m.startup_pending and m.feedback_fresh and m.position_valid
            and not m.error_code and not (m.sto_state_known and m.sto_active)
            and all(d.enabled and d.initialized and d.status_known and d.fault_known
                    and not d.error_code and not d.error_register for d in (m.steering, m.traction))
            for m in modules))

    def _joint_valid(self, msg):
        expected = set(self.steering_joint_names + self.traction_joint_names)
        return (len(msg.name) == len(expected) and set(msg.name) == expected
                and len(msg.position) == len(msg.name) and len(msg.velocity) == len(msg.name)
                and not msg.effort and all(math.isfinite(v) for v in msg.position + msg.velocity))

    def _stationary(self):
        joints = self._fresh('joints')
        return bool(joints is not None and self._joint_valid(joints) and all(
            abs(v) <= (0.02 if n in self.traction_joint_names else 0.1)
            for n, v in zip(joints.name, joints.velocity)))

    def _disabled_feedback(self):
        modules = self._modules()
        if not modules or any(m.enabled or m.startup_pending for m in modules):
            return False
        bindings = dict(zip(self.module_names, zip(self.steering_joint_names, self.traction_joint_names)))
        now = self.get_clock().now().nanoseconds
        return all(d.name == name and d.status_known and d.device_state_known and not d.enabled
                   and stamp_ns(d.status_stamp) > 0
                   and 0 <= now - stamp_ns(d.status_stamp) <= self.feedback_timeout * 1e9
                   for m in modules for d, name in zip((m.steering, m.traction), bindings[m.name]))

    def _new_generation(self):
        if self.steering_policy:
            self.steering_policy.reset_alignment()
        self.generation += 1
        self.boundary = self.get_clock().now().nanoseconds
        self.ledger.clear()
        self.samples.pop('command', None)

    def _disable(self, reason):
        if self.phase != 'disabled':
            self._new_generation()
            self.get_logger().warning(reason + '; neutral controls and B2 required')
        self.phase, self.reason = 'disabled', reason
        self.recovery_started = self.standstill_since = None

    def _disable_service(self, request, response):
        self._disable('explicit manual disable')
        self._tick()
        response.success, response.message = True, 'disable requested; verify wheel_modules/status'
        return response

    def _enable(self, request, response):
        if self.count_publishers('vehicle/mode') > 1:
            response.success, response.message = False, 'another vehicle authority is publishing'
        elif self.phase != 'disabled':
            response.success, response.message = True, 'already enabled or recovering; unchanged'
        elif not self._operator(neutral=True):
            response.success, response.message = False, 'fresh connected operator with neutral controls required'
        elif not self._disabled_feedback():
            response.success, response.message = False, 'fresh native disabled feedback required for every drive'
        elif self._fresh('joints') is not None and not self._stationary():
            response.success, response.message = False, 'measured movement or invalid feedback prevents reset'
        else:
            self.reset_id += 1
            self.phase, self.reason = 'recovering', 'zero-target clear/initialize/enable in progress'
            self.recovery_started = time.monotonic()
            self.recovery_stamp = self.get_clock().now().nanoseconds
            self.standstill_since, self.startup_seen = None, False
            self._new_generation()
            response.success, response.message = True, 'accepted; wait for verified enabled standstill'
        return response

    def _motion_source(self):
        command, state = self._fresh('command'), self._fresh('operator')
        if (self.phase != 'enabled' or not self._ready() or not self._operator()
                or not state.command_active or command is None
                or stamp_ns(command.header.stamp) <= self.boundary
                or command.header.frame_id != self.output_frame_id):
            return None
        t = command.twist
        if not all(math.isfinite(v) for v in (t.linear.x, t.linear.y, t.linear.z,
                                             t.angular.x, t.angular.y, t.angular.z)):
            return None
        if t.linear.z != 0 or t.angular.x != 0 or t.angular.y != 0:
            return None
        return command

    def _tick(self):
        now, steady = self.get_clock().now(), time.monotonic()
        steady_dt = steady - self.last_tick
        ros_dt = (now.nanoseconds - self.last_ros_tick) * 1e-9
        if steady_dt > self.control_gap_timeout or steady_dt < 0:
            self._disable('sustained control-loop gap; re-enable required')
        elif (steady_dt > self.candidate_timeout or ros_dt <= 0
              or abs(ros_dt-steady_dt) > self.candidate_timeout):
            # Brief scheduling/clock disturbance: flush pending motion, retain
            # the drive-enable phase. Fresh physical inputs are still required;
            # the independent 100 ms driver watchdog remains unchanged.
            self.ledger.clear()
            self.samples.pop('command', None)
        self.last_tick, self.last_ros_tick = steady, now.nanoseconds
        if self.count_publishers('vehicle/mode') > 1:
            self._disable('another vehicle authority detected; stop the other stack')
        if self.phase != 'disabled' and not self._operator():
            self._disable('operator lost or STOP asserted')
        modules = self._modules()
        accepted = bool(self.phase == 'recovering' and modules
                        and stamp_ns(self._fresh('status').header.stamp) > self.recovery_stamp
                        and all(m.reset_id == self.reset_id for m in modules))
        if self.phase == 'recovering':
            if accepted and any(m.startup_pending for m in modules):
                self.startup_seen = True
            if steady - self.recovery_started >= self.reset_timeout:
                self._disable('drive recovery timed out')
            elif self.startup_seen and accepted and any(m.startup_failed for m in modules):
                self._disable('drive startup failed')
            elif accepted and self._ready() and all(m.reset_acknowledged for m in modules) and self._stationary():
                if self.standstill_since is None:
                    self.standstill_since = stamp_ns(self._fresh('joints').header.stamp)
                if stamp_ns(self._fresh('joints').header.stamp) - self.standstill_since >= 250_000_000:
                    self.phase, self.reason = 'enabled', 'manual-only ready; fresh neutral rearm required'
                    self._new_generation()
                    self.get_logger().info(self.reason)
            else:
                self.standstill_since = None
        if self.phase == 'enabled' and not self._ready():
            self._disable('drive feedback lost or faulted')
        source = self._motion_source()
        mode = VehicleModeState()
        mode.header.stamp = now.to_msg()
        mode.current_mode = mode.selected_mode = mode.requested_mode = VehicleModeState.MANUAL
        mode.supervisor_instance, mode.authority_generation = self.instance, self.generation
        mode.reset_id, mode.reset_in_progress = self.reset_id, self.phase == 'recovering'
        mode.motion_permitted = source is not None
        state = self._fresh('operator')
        if (state and state.controls_neutral and self.steering_policy and source is not None
                and stamp_ns(state.header.stamp) >= stamp_ns(source.header.stamp)
                and all(v == 0.0 for v in (source.twist.linear.x, source.twist.linear.y, source.twist.angular.z))):
            self.steering_policy.release_crab_boundary()
        mode.safety_stop_active = bool(state and state.estop_pressed)
        mode.readiness_known = modules is not None
        mode.drives_enabled = bool(modules and all(m.enabled for m in modules))
        mode.ready_to_drive = self.phase == 'enabled' and self._ready()
        mode.reason_codes = ['MANUAL_ONLY_TEST'] + ([] if mode.ready_to_drive else ['B2_ENABLE_REQUIRED'])
        mode.status_message = self.reason
        self.mode_pub.publish(mode)
        operation = WheelModuleOperationCommand()
        operation.header.stamp = now.to_msg()
        operation.supervisor_instance, operation.authority_generation = self.instance, self.generation
        operation.enable = self.phase == 'enabled' or (self.phase == 'recovering' and accepted)
        operation.reset_id = self.reset_id if self.phase == 'recovering' else 0
        self.operation_pub.publish(operation)
        velocity = TwistStamped()
        velocity.header.stamp, velocity.header.frame_id = now.to_msg(), self.output_frame_id
        if source is not None:
            velocity.twist = copy.deepcopy(source.twist)
            linear_limit = min(self.max_velocity_linear, self.SPEED_LIMIT)
            angular_limit = self.max_velocity_angular * linear_limit / self.max_velocity_linear
            # Keep yaw/forward curvature and crab direction under either cap.
            velocity.twist.linear.x, velocity.twist.linear.y, velocity.twist.angular.z = ManualSteering.cap_velocity(
                (velocity.twist.linear.x, velocity.twist.linear.y, velocity.twist.angular.z),
                linear_limit, angular_limit)
            if self.steering_policy:
                velocity.twist.linear.x, velocity.twist.linear.y, velocity.twist.angular.z = self.steering_policy.project(
                    (velocity.twist.linear.x, velocity.twist.linear.y, velocity.twist.angular.z))
            self.ledger[now.nanoseconds] = (self.generation, source.header.stamp, steady,
                (velocity.twist.linear.x, velocity.twist.linear.y, velocity.twist.angular.z))
        self.ledger = {k: v for k, v in self.ledger.items() if steady - v[2] <= self.candidate_timeout}
        self.velocity_pub.publish(velocity)

    def _candidate(self, msg):
        entry = self.ledger.pop(stamp_ns(msg.header.stamp), None)
        now = self.get_clock().now()
        source = self._motion_source()
        if (entry is None or entry[0] != self.generation or source is None
                or not 0 <= time.monotonic() - entry[2] <= self.candidate_timeout
                or not 0 <= now.nanoseconds - stamp_ns(entry[1]) <= self.manual_timeout * 1e9
                or not self._joint_valid(msg)):
            return
        home = False
        if self.steering_policy:
            feedback = self._fresh('joints')
            if feedback is None:
                return
            state = self._fresh('operator')
            stopped = all(v == 0.0 for v in (source.twist.linear.x, source.twist.linear.y, source.twist.angular.z))
            home = bool(stopped and self.steering_policy.return_home_on_neutral
                        and state and state.connected and state.command_active and state.controls_neutral
                        and stamp_ns(state.header.stamp) >= stamp_ns(source.header.stamp))
            try:
                msg = self.steering_policy.align(msg, feedback, (0., 0., 0.) if stopped else entry[3], neutral=home)
            except ValueError:
                return
        indices = {n: i for i, n in enumerate(msg.name)}
        output = WheelModuleCommandArray()
        output.header.stamp, output.source_stamp = now.to_msg(), entry[1]
        output.supervisor_instance, output.authority_generation = self.instance, self.generation
        output.selected_mode, output.purpose = VehicleModeState.MANUAL, WheelModuleCommandArray.MOTION
        for name, steering, traction in zip(self.module_names, self.steering_joint_names, self.traction_joint_names):
            record = WheelModuleCommand()
            record.name, record.steering_position = name, msg.position[indices[steering]]
            record.steering_profile_velocity = msg.velocity[indices[steering]]
            record.traction_velocity = msg.velocity[indices[traction]]
            steering_limit = max(self.max_steering_velocity, self.steering_policy.neutral_home_velocity) if home else self.max_steering_velocity
            if (not 0 <= record.steering_profile_velocity <= steering_limit
                    or abs(record.traction_velocity) > self.max_traction_velocity):
                return
            output.modules.append(record)
        # Rotation can produce wheel speeds above the body's planar speed.
        # Bound the final wheel command too, preserving relative wheel speeds.
        peak = max(abs(module.traction_velocity) for module in output.modules)
        scale = min(1.0, self.SPEED_LIMIT / peak) if peak else 1.0
        for module in output.modules:
            module.traction_velocity *= scale
        self.command_pub.publish(output)


def main(args=None):
    rclpy.init(args=args)
    node = ManualDriveControl()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
