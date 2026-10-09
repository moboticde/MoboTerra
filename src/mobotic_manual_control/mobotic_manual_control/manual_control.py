import math

import rclpy
from geometry_msgs.msg import TwistStamped
from mobotic_interfaces.msg import ManualControlState, VehicleModeRequest, VehicleModeState
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import Joy


class ManualControl(Node):
    def __init__(self):
        super().__init__('manual_control')

        # SDL standardized game_controller_node mapping, F310/F710 USB profile.
        # Face buttons 0..3 are deliberately unused.
        self.declare_parameter('axis_speed', 3)
        self.declare_parameter('axis_steer', 0)
        self.declare_parameter('axis_crab', 2)
        self.declare_parameter('button_deadman', -1)
        self.declare_parameter('buttons_boost', [9, 10])
        self.declare_parameter('button_mode_switch', 4)
        self.declare_parameter('mode_switch_type', 'toggle')
        self.declare_parameter('mode_switch_manual_value', 0)
        self.declare_parameter('mode_switch_auto_velocity_value', 1)
        self.declare_parameter('mode_state_timeout', 0.5)
        self.declare_parameter('max_velocity_linear', 1.25)
        self.declare_parameter('max_velocity_angular', 1.51)
        self.declare_parameter('max_acceleration_linear', 0.5)
        self.declare_parameter('max_acceleration_angular', 0.52)
        self.declare_parameter('scale_linear', 0.4)
        self.declare_parameter('scale_angular', 0.4)
        self.declare_parameter('joystick_timeout', 0.25)
        self.declare_parameter('publish_period', 0.02)
        self.declare_parameter('frame_id', 'base_link')

        self.axis_speed = int(self.get_parameter('axis_speed').value)
        self.axis_steer = int(self.get_parameter('axis_steer').value)
        self.axis_crab = int(self.get_parameter('axis_crab').value)
        self.button_deadman = int(self.get_parameter('button_deadman').value)
        self.buttons_boost = [int(index) for index in self.get_parameter('buttons_boost').value]
        self.button_mode_switch = int(self.get_parameter('button_mode_switch').value)
        self.mode_switch_type = str(self.get_parameter('mode_switch_type').value)
        self.mode_switch_manual_value = int(self.get_parameter('mode_switch_manual_value').value)
        self.mode_switch_auto_velocity_value = int(
            self.get_parameter('mode_switch_auto_velocity_value').value)
        self.mode_state_timeout = float(self.get_parameter('mode_state_timeout').value)
        self.max_velocity_linear = float(self.get_parameter('max_velocity_linear').value)
        self.max_velocity_angular = float(self.get_parameter('max_velocity_angular').value)
        self.max_acceleration_linear = float(self.get_parameter('max_acceleration_linear').value)
        self.max_acceleration_angular = float(self.get_parameter('max_acceleration_angular').value)
        self.scale_linear = float(self.get_parameter('scale_linear').value)
        self.scale_angular = float(self.get_parameter('scale_angular').value)
        self.joystick_timeout = float(self.get_parameter('joystick_timeout').value)
        self.publish_period = float(self.get_parameter('publish_period').value)
        self.frame_id = str(self.get_parameter('frame_id').value)

        self._validate_parameters()

        self.command_publisher = self.create_publisher(TwistStamped, 'manual/cmd_vel', 1)
        self.state_publisher = self.create_publisher(ManualControlState, 'manual/state', 1)
        self.mode_publisher = self.create_publisher(VehicleModeRequest, 'vehicle/mode_request', 1)
        self.joy_subscription = self.create_subscription(Joy, 'joy', self._joy_callback, 1)
        self.mode_subscription = self.create_subscription(
            VehicleModeState, 'vehicle/mode_state', self._vehicle_mode_callback, 1)
        self.timer = self.create_timer(self.publish_period, self._publish)

        self.last_joy_time = None
        self.last_update_time = self.get_clock().now()
        self.deadman_pressed = False
        self.turbo_active = False
        self.command = [0.0, 0.0, 0.0]
        self.warned_invalid_layout = False
        self.mode_switch_state = None
        self.last_mode_joy_stamp = None
        self.vehicle_mode_state = None
        self.vehicle_mode_received = None
        if self.button_mode_switch < 0:
            self.get_logger().warning('RP1 mode switch is unbound; verify the REMdevice receiver mapping.')

    def _validate_parameters(self):
        indices = [
            self.axis_speed,
            self.axis_steer,
            self.axis_crab,
            *self.buttons_boost,
        ]
        values = [
            self.max_velocity_linear,
            self.max_velocity_angular,
            self.max_acceleration_linear,
            self.max_acceleration_angular,
            self.scale_linear,
            self.scale_angular,
            self.joystick_timeout,
            self.publish_period,
        ]
        if any(index < 0 for index in indices):
            raise ValueError('joystick axis and button indices must be non-negative')
        if self.button_deadman < -1:
            raise ValueError('deadman index must be -1 (disabled) or non-negative')
        if not self.buttons_boost or len(set(self.buttons_boost)) != len(self.buttons_boost):
            raise ValueError('boost buttons must be a non-empty list of distinct indices')
        if self.button_deadman >= 0 and self.button_deadman in self.buttons_boost:
            raise ValueError('deadman and boost buttons cannot overlap')
        if self.button_mode_switch < -1:
            raise ValueError('RP1 button index must be -1 (unbound) or non-negative')
        if self.button_mode_switch >= 0 and self.button_mode_switch in (self.button_deadman, *self.buttons_boost):
            raise ValueError('RP1 cannot share a driving-control button')
        if {self.mode_switch_manual_value, self.mode_switch_auto_velocity_value} != {0, 1}:
            raise ValueError('RP1 Manual/Auto values must be distinct digital values 0 and 1')
        if self.mode_switch_type not in ('toggle', 'position'):
            raise ValueError('mode_switch_type must be toggle or position')
        if not math.isfinite(self.mode_state_timeout) or self.mode_state_timeout <= 0:
            raise ValueError('mode_state_timeout must be finite and positive')
        if any(not math.isfinite(value) for value in values):
            raise ValueError('manual-control numeric parameters must be finite')
        if min(values[:4]) <= 0.0 or self.joystick_timeout <= 0.0 or self.publish_period <= 0.0:
            raise ValueError('velocity, acceleration, timeout, and publish period must be positive')
        if not 0.0 <= self.scale_linear <= 1.0 or not 0.0 <= self.scale_angular <= 1.0:
            raise ValueError('non-turbo scales must be in [0, 1]')

    def _layout_valid(self, msg):
        axes = (self.axis_speed, self.axis_steer, self.axis_crab)
        button_count = max(self.button_deadman, *self.buttons_boost) + 1
        return (len(msg.axes) > max(axes) and len(msg.buttons) >= button_count
                and all(math.isfinite(msg.axes[index]) and abs(msg.axes[index]) <= 1.0 for index in axes))

    @staticmethod
    def _limit(value, previous, acceleration, dt):
        delta = max(acceleration * dt, 0.0)
        return min(max(value, previous - delta), previous + delta)

    def _manual_enabled(self):
        return self.button_deadman < 0 or self.deadman_pressed

    def _joystick_connected(self, now):
        if self.last_joy_time is None:
            return False
        age = (now - self.last_joy_time).nanoseconds
        return 0 <= age <= Duration(seconds=self.joystick_timeout).nanoseconds

    def _reset_command(self, now):
        self.command = [0.0, 0.0, 0.0]
        self.deadman_pressed = False
        self.turbo_active = False
        self.last_joy_time = None
        self.last_update_time = now
        self.mode_switch_state = None
        self.last_mode_joy_stamp = None

    def _request_joystick_mode(self, mode, now):
        # RP1 can request only modes 0/1; direct control is service-only.
        if mode not in (VehicleModeRequest.MANUAL, VehicleModeRequest.AUTO_VELOCITY):
            return False
        request = VehicleModeRequest()
        request.header.stamp = now.to_msg()
        request.header.frame_id = self.frame_id
        request.requested_mode = mode
        self.mode_publisher.publish(request)
        return True

    def _mode_switch_callback(self, msg, now):
        if self.button_mode_switch < 0 or self.button_mode_switch >= len(msg.buttons):
            self.mode_switch_state = None
            self.last_mode_joy_stamp = None
            return
        stamp = Time.from_msg(msg.header.stamp)
        age = (now - stamp).nanoseconds
        if stamp.nanoseconds <= 0 or not 0 <= age <= Duration(seconds=self.joystick_timeout).nanoseconds:
            self.mode_switch_state = None
            return
        if self.last_mode_joy_stamp is not None and stamp.nanoseconds <= self.last_mode_joy_stamp:
            return
        self.last_mode_joy_stamp = stamp.nanoseconds
        value = msg.buttons[self.button_mode_switch]
        if value not in (self.mode_switch_manual_value, self.mode_switch_auto_velocity_value):
            self.mode_switch_state = None
            return
        previous = self.mode_switch_state
        self.mode_switch_state = value
        # Prime startup/reconnect. Only a real position change requests a
        # mode; autorepeat must not override a terminal-selected AUTO_DIRECT.
        if self.mode_switch_type == 'toggle':
            if previous == 0 and value == 1 and self._vehicle_mode_fresh(now):
                state = self.vehicle_mode_state
                if not state.transition_in_progress:
                    mode = (VehicleModeRequest.AUTO_VELOCITY if state.current_mode == VehicleModeRequest.MANUAL
                            else VehicleModeRequest.MANUAL)
                    self._request_joystick_mode(mode, now)
        elif previous is not None and value != previous:
            mode = (VehicleModeRequest.MANUAL if value == self.mode_switch_manual_value
                    else VehicleModeRequest.AUTO_VELOCITY)
            self._request_joystick_mode(mode, now)

    def _vehicle_mode_callback(self, msg):
        now = self.get_clock().now()
        stamp = Time.from_msg(msg.header.stamp).nanoseconds
        if (msg.current_mode not in (0, 1, 2) or stamp <= 0
                or not 0 <= now.nanoseconds - stamp <= Duration(seconds=self.mode_state_timeout).nanoseconds):
            return
        if self.vehicle_mode_state is not None:
            previous = Time.from_msg(self.vehicle_mode_state.header.stamp).nanoseconds
            if stamp <= previous and self._vehicle_mode_fresh(now):
                return
        self.vehicle_mode_state = msg
        self.vehicle_mode_received = now

    def _vehicle_mode_fresh(self, now):
        if self.vehicle_mode_state is None or self.vehicle_mode_received is None:
            return False
        limit = Duration(seconds=self.mode_state_timeout).nanoseconds
        source_age = now.nanoseconds - Time.from_msg(self.vehicle_mode_state.header.stamp).nanoseconds
        receipt_age = (now - self.vehicle_mode_received).nanoseconds
        return 0 <= source_age <= limit and 0 <= receipt_age <= limit

    def _joy_callback(self, msg):
        now = self.get_clock().now()
        # Detect a gap before overwriting the last receipt time, even if the
        # publish timer has not run since disconnection. Never ramp from an old
        # target or integrate acceleration over the disconnected interval.
        if not self._joystick_connected(now):
            self._reset_command(now)
        if not self._layout_valid(msg):
            if not self.warned_invalid_layout:
                self.get_logger().error('Joy layout or configured axis values are invalid.')
                self.warned_invalid_layout = True
            self._reset_command(now)
            return

        self.warned_invalid_layout = False
        self.last_joy_time = now
        self._mode_switch_callback(msg, now)
        self.deadman_pressed = self.button_deadman >= 0 and bool(msg.buttons[self.button_deadman])
        self.turbo_active = any(msg.buttons[index] for index in self.buttons_boost)

        if not self._manual_enabled():
            self.command = [0.0, 0.0, 0.0]
            self.last_update_time = now
            return

        speed = msg.axes[self.axis_speed]
        steer = msg.axes[self.axis_steer]
        crab = msg.axes[self.axis_crab]
        linear_scale = 1.0 if self.turbo_active else self.scale_linear
        angular_scale = 1.0 if self.turbo_active else self.scale_angular

        requested = [
            self.max_velocity_linear * speed * linear_scale,
            self.max_velocity_linear * crab * linear_scale,
            self.max_velocity_angular * steer * speed * angular_scale,
        ]

        dt = max((now - self.last_update_time).nanoseconds * 1.0e-9, 0.0)
        self.command[0] = self._limit(
            requested[0], self.command[0], self.max_acceleration_linear, dt
        )
        self.command[1] = self._limit(
            requested[1], self.command[1], self.max_acceleration_linear, dt
        )
        self.command[2] = self._limit(
            requested[2], self.command[2], self.max_acceleration_angular, dt
        )
        self.last_update_time = now

    def _publish(self):
        now = self.get_clock().now()
        connected = self._joystick_connected(now)
        if not connected:
            self._reset_command(now)
        active = connected and self._manual_enabled()
        command = self.command if active else [0.0, 0.0, 0.0]

        twist = TwistStamped()
        twist.header.stamp = now.to_msg()
        twist.header.frame_id = self.frame_id
        twist.twist.linear.x = command[0]
        twist.twist.linear.y = command[1]
        twist.twist.angular.z = command[2]
        self.command_publisher.publish(twist)

        state = ManualControlState()
        state.header = twist.header
        state.connected = connected
        state.deadman_pressed = self.deadman_pressed if connected else False
        state.command_active = active
        state.turbo_active = self.turbo_active if connected else False
        if not connected:
            state.status_message = 'joystick unavailable or stale'
        elif not self._manual_enabled():
            state.status_message = 'deadman released'
        else:
            state.status_message = 'manual command active'
        self.state_publisher.publish(state)


def main(args=None):
    rclpy.init(args=args)
    node = ManualControl()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
