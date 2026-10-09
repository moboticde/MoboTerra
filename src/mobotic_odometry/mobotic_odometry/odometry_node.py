"""ROS transport for measured body-velocity odometry; no motion authority."""
import math
import time

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import TransformStamped, TwistStamped
from nav_msgs.msg import Odometry
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from std_srvs.srv import Trigger
from tf2_ros import TransformBroadcaster

from .integration import PlanarOdometry


class MoboticOdometry(Node):
    def __init__(self):
        super().__init__('mobotic_odometry')
        defaults = {
            'odom_frame_id': 'odom', 'base_frame_id': 'base_link', 'publish_tf': True,
            'feedback_timeout': 0.3, 'max_integration_interval': 0.3,
            'initial_pose': [0.0, 0.0, 0.0], 'initial_pose_variances': [0.01, 0.01, 0.01],
            'twist_variances': [0.01, 0.01, 0.01],
            'process_variance_rates': [0.001, 0.001, 0.001],
            'gap_variance_rates': [1.0, 1.0, 1.0], 'unobserved_variance': 1e6,
            'max_linear_speed': 5.0, 'max_angular_speed': 5.0,
        }
        for key, value in defaults.items():
            self.declare_parameter(key, value)
        parameters = {key: self.get_parameter(key).value for key in defaults}
        self.odom_frame = parameters.pop('odom_frame_id')
        self.base_frame = parameters.pop('base_frame_id')
        publish_tf = parameters.pop('publish_tf')
        for frame in (self.odom_frame, self.base_frame):
            if not frame or frame.startswith('/') or any(character.isspace() for character in frame):
                raise ValueError('Frame IDs must be nonempty, without leading slash or whitespace')
        if self.odom_frame == self.base_frame:
            raise ValueError('Odometry and body frame IDs must differ')
        self.engine = PlanarOdometry(**parameters)
        self.publisher = self.create_publisher(Odometry, 'odometry', 10)
        self.diagnostic_publisher = self.create_publisher(DiagnosticArray, 'diagnostics', 10)
        self.broadcaster = TransformBroadcaster(self) if publish_tf else None
        self.subscription = self.create_subscription(TwistStamped, 'agv_vel', self._receive, 1)
        self.reset_service = self.create_service(Trigger, 'odometry/reset', self._reset)
        self.steady_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.health_timer = self.create_timer(0.2, self._diagnostics, clock=self.steady_clock)

    def _receive(self, msg):
        stamp = msg.header.stamp
        if stamp.sec < 0 or not 0 <= stamp.nanosec < 1_000_000_000:
            self.engine.invalidate('Malformed acquisition timestamp')
            return
        if msg.header.frame_id != self.base_frame:
            self.engine.invalidate('Measured velocity is not in configured base frame')
            return
        if any(value != 0.0 for value in (msg.twist.linear.z, msg.twist.angular.x, msg.twist.angular.y)):
            self.engine.invalidate('Non-planar measured velocity is unsupported')
            return
        stamp_ns = stamp.sec * 1_000_000_000 + stamp.nanosec
        estimate = self.engine.observe(
            stamp_ns, self.get_clock().now().nanoseconds, time.monotonic(),
            (msg.twist.linear.x, msg.twist.linear.y, msg.twist.angular.z),
        )
        if estimate is None:
            return
        output = Odometry()
        output.header.stamp = msg.header.stamp  # Acquisition time, never re-stamped receipt time.
        output.header.frame_id = self.odom_frame
        output.child_frame_id = self.base_frame
        output.pose.pose.position.x, output.pose.pose.position.y = estimate.pose[:2]
        output.pose.pose.orientation.z = math.sin(estimate.pose[2] * 0.5)
        output.pose.pose.orientation.w = math.cos(estimate.pose[2] * 0.5)
        output.pose.covariance = list(estimate.pose_covariance)
        output.twist.twist.linear.x, output.twist.twist.linear.y = estimate.twist[:2]
        output.twist.twist.angular.z = estimate.twist[2]
        output.twist.covariance = list(estimate.twist_covariance)
        self.publisher.publish(output)
        if self.broadcaster is not None:
            transform = TransformStamped()
            transform.header = output.header
            transform.child_frame_id = output.child_frame_id
            transform.transform.translation.x = output.pose.pose.position.x
            transform.transform.translation.y = output.pose.pose.position.y
            transform.transform.rotation = output.pose.pose.orientation
            self.broadcaster.sendTransform(transform)

    def _reset(self, request, response):
        del request
        fresh = self.engine.fresh(self.get_clock().now().nanoseconds, time.monotonic())
        stopped = fresh and math.hypot(*self.engine.last_twist[:2]) <= 0.02 and abs(self.engine.last_twist[2]) <= 0.02
        response.success = bool(stopped)
        if stopped:
            self.engine.reset()
            response.message = 'Reset to configured initial pose; waiting for a new feedback sample'
        else:
            response.message = 'Reset rejected: fresh measured standstill is required'
        return response

    def _diagnostics(self):
        array = DiagnosticArray()
        array.header.stamp = self.get_clock().now().to_msg()
        healthy = self.engine.fresh(self.get_clock().now().nanoseconds, time.monotonic())
        status = DiagnosticStatus()
        status.name = f'{self.get_fully_qualified_name()}/wheel_odometry'
        status.hardware_id = 'MoboTerra'
        status.level = DiagnosticStatus.STALE if not healthy else DiagnosticStatus.WARN if self.engine.pose_degraded else DiagnosticStatus.OK
        status.message = self.engine.reason
        status.values = [KeyValue(key='pose_degraded', value=str(self.engine.pose_degraded)),
                         KeyValue(key='last_acquisition_ns', value=str(self.engine.last_stamp_ns)),
                         KeyValue(key='tf_enabled', value=str(self.broadcaster is not None))]
        array.status = [status]
        self.diagnostic_publisher.publish(array)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = MoboticOdometry()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()
