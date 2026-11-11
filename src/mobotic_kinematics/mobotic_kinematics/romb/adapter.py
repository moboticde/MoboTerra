"""Connect the original romb kinematics to supervised MoboTerra interfaces."""
import copy
import math
import time

import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.parameter import Parameter
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import JointState
from mobotic_interfaces.msg import VehicleModeState
from .coordinates import RombContract


def stamp_ns(stamp):
    if stamp.sec < 0 or not 0 <= stamp.nanosec < 1_000_000_000:
        return 0
    return stamp.sec * 1_000_000_000 + stamp.nanosec


class RombKinematicsAdapter(Node):
    def __init__(self):
        super().__init__('mobotic_kinematics')
        defaults = dict(module_names=[], steering_joint_names=[], traction_joint_names=[],
                        wheel_mount_angles=[], max_steering_velocity=.3,
                        max_traction_velocity=.3, minimum_linear_speed=.001,
                        cmd_vel_timeout=.5, joint_feedback_timeout=.3,
                        output_frame_id='base_link')
        for name, value in defaults.items():
            if name in ('module_names', 'steering_joint_names', 'traction_joint_names'):
                self.declare_parameter(name, Parameter.Type.STRING_ARRAY)
            elif name == 'wheel_mount_angles':
                self.declare_parameter(name, Parameter.Type.DOUBLE_ARRAY)
            else:
                self.declare_parameter(name, value)
        get = lambda name: self.get_parameter(name).value
        self.contract = RombContract(get('module_names'), get('steering_joint_names'),
            get('traction_joint_names'), get('wheel_mount_angles'), get('max_traction_velocity'),
            get('max_steering_velocity'), get('minimum_linear_speed'))
        self.feedback_timeout = float(get('joint_feedback_timeout'))
        self.command_timeout = min(.25, float(get('cmd_vel_timeout')))
        if not all(math.isfinite(t) and t > 0 for t in (self.feedback_timeout, self.command_timeout)):
            raise ValueError('Romb input deadlines must be positive and finite')
        self.frame = get('output_frame_id')
        self.feedback = None
        self.feedback_received = None
        self.last_feedback_stamp = self.last_command_stamp = 0
        self.last_output_stamp = self.last_measured_stamp = 0
        self.pending_commands = {}
        self.pending_feedback = {}
        self.authority = None
        self.command_publisher = self.create_publisher(JointState, 'kinematics/joint_setpoints', 1)
        self.measured_publisher = self.create_publisher(TwistStamped, 'vehicle/measured_velocity', 1)
        self.romb_command = self.create_publisher(TwistStamped, 'romb/kinematics/cmd_vel', 1)
        self.romb_feedback = self.create_publisher(JointState, 'romb/kinematics/joint_states', 1)
        self.subscriptions_kept = [
            self.create_subscription(JointState, 'joint_states', self._feedback, 1),
            self.create_subscription(TwistStamped, 'vehicle/cmd_vel', self._command, 1),
            self.create_subscription(JointState, 'romb/kinematics/raw_setpoints', self._setpoints, 1),
            self.create_subscription(TwistStamped, 'romb/kinematics/raw_velocity', self._measured, 1),
            self.create_subscription(VehicleModeState, 'vehicle/mode', self._authority, 1),
        ]
        self.steady_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.timer = self.create_timer(.05, self._expire, clock=self.steady_clock)
        self.get_logger().info('Original romb kinematics bridge: traction m/s, steering rad; controller mounting adaptation and existing supervisor gates retained')

    def _now(self):
        return self.get_clock().now().nanoseconds

    def _fresh(self, stamp, timeout):
        return stamp > 0 and -.1 <= (self._now() - stamp) * 1e-9 <= timeout

    def _feedback_fresh(self):
        return (self.feedback is not None and self.feedback_received is not None
                and self._fresh(stamp_ns(self.feedback.header.stamp), self.feedback_timeout)
                and 0 <= time.monotonic() - self.feedback_received <= self.feedback_timeout)

    def _expire(self):
        now = time.monotonic()
        for pending, timeout in ((self.pending_commands, self.command_timeout),
                                 (self.pending_feedback, self.feedback_timeout)):
            for stamp, received in list(pending.items()):
                if now - received > timeout:
                    pending.pop(stamp, None)
        if not self._feedback_fresh():
            self.feedback = None
            self.pending_commands.clear()
            self.pending_feedback.clear()
            self.contract.reset()

    def _authority(self, msg):
        if not self._fresh(stamp_ns(msg.header.stamp), .3):
            return
        authority = (msg.supervisor_instance, msg.authority_generation, msg.reset_id)
        if authority != self.authority:
            self.authority = authority
            self.pending_commands.clear()
            self.contract.reset()

    def _feedback(self, msg):
        stamp = stamp_ns(msg.header.stamp)
        if not self._fresh(stamp, self.feedback_timeout) or stamp <= self.last_feedback_stamp:
            return
        try:
            positions = self.contract.feedback_positions(msg.name, msg.position, msg.velocity)
        except ValueError:
            self.feedback = None
            self.pending_commands.clear()
            self.pending_feedback.clear()
            self.contract.reset()
            return
        self.last_feedback_stamp = stamp
        self.feedback = copy.deepcopy(msg)
        self.feedback_received = time.monotonic()
        self.pending_feedback[stamp] = self.feedback_received
        while len(self.pending_feedback) > 32:
            self.pending_feedback.pop(next(iter(self.pending_feedback)))
        converted = copy.deepcopy(msg)
        converted.position = positions
        converted.header.frame_id = self.frame
        self.romb_feedback.publish(converted)

    def _command(self, msg):
        stamp = stamp_ns(msg.header.stamp)
        values = (msg.twist.linear.x, msg.twist.linear.y, msg.twist.angular.z)
        if (not self._feedback_fresh() or not self._fresh(stamp, self.command_timeout)
                or stamp <= self.last_command_stamp or not all(math.isfinite(v) for v in values)):
            return
        self.last_command_stamp = stamp
        self.pending_commands[stamp] = time.monotonic()
        while len(self.pending_commands) > 32:
            self.pending_commands.pop(next(iter(self.pending_commands)))
        self.romb_command.publish(msg)

    def _setpoints(self, msg):
        stamp = stamp_ns(msg.header.stamp)
        received = self.pending_commands.pop(stamp, None)
        if (received is None or time.monotonic() - received > self.command_timeout
                or not self._fresh(stamp, self.command_timeout) or stamp <= self.last_output_stamp
                or not self._feedback_fresh()):
            return
        try:
            positions, speeds, profiles = self.contract.commands(msg.name, msg.position, msg.velocity,
                self.feedback.name, self.feedback.position)
        except (ValueError, KeyError, IndexError):
            return
        output = JointState()
        output.header = copy.deepcopy(msg.header)  # Preserve original command acquisition time.
        output.header.frame_id = self.frame
        output.name = self.contract.steering + self.contract.traction
        output.position = positions + [0.0] * len(speeds)
        output.velocity = profiles + speeds
        self.last_output_stamp = stamp
        self.command_publisher.publish(output)

    def _measured(self, msg):
        stamp = stamp_ns(msg.header.stamp)
        received = self.pending_feedback.pop(stamp, None)
        values = (msg.twist.linear.x, msg.twist.linear.y, msg.twist.angular.z)
        if (received is None or time.monotonic() - received > self.feedback_timeout
                or not self._fresh(stamp, self.feedback_timeout) or stamp <= self.last_measured_stamp
                or not self._feedback_fresh() or not all(math.isfinite(v) for v in values)):
            return
        output = copy.deepcopy(msg)
        output.header.frame_id = self.frame
        self.last_measured_stamp = stamp
        self.measured_publisher.publish(output)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = RombKinematicsAdapter()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
