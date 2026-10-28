#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import tf2_ros
import tf_transformations as tft  # The ROS 2 replacement for tf.transformations
import numpy as np
from collections import deque
import math

from geometry_msgs.msg import TwistStamped


class AGVTwistEstimator(Node):
    def __init__(self):
        super().__init__('agv_twist_estimator_robust_node')

        # --- Parameters (ROS 2 requires declaration) ---
        self.declare_parameter('world_frame', 'map')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('polyorder', 2)
        self.declare_parameter('window_length', 31)
        self.declare_parameter('update_rate', 50.0)
        self.declare_parameter('max_jump_distance', 0.5)

        self.world_frame = self.get_parameter('world_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.polyorder = self.get_parameter('polyorder').value
        self.full_window_length = self.get_parameter('window_length').value
        self.update_rate = self.get_parameter('update_rate').value
        self.max_jump_distance = self.get_parameter('max_jump_distance').value

        self.min_window_length = self.polyorder + 2

        # --- ROS 2 Comms ---
        self.twist_pub = self.create_publisher(TwistStamped, '/mobotic/twist_estimate', 10)

        # TF2 Buffer and Listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # --- Data Buffers ---
        self.timestamps = deque(maxlen=self.full_window_length)
        self.positions_x = deque(maxlen=self.full_window_length)
        self.positions_y = deque(maxlen=self.full_window_length)
        self.yaws_unwrapped = deque(maxlen=self.full_window_length)

        # Timer setup
        timer_period = 1.0 / self.update_rate
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.get_logger().info("AGV Twist estimator started (ROS 2 Jazzy).")
        self.get_logger().info(f"Window: {self.full_window_length} | Polyorder: {self.polyorder}")

    def reset_filter_state(self):
        self.get_logger().warn("Localization jump detected. Resetting filter state.")
        self.timestamps.clear()
        self.positions_x.clear()
        self.positions_y.clear()
        self.yaws_unwrapped.clear()

    def fit_and_differentiate(self, times, data_series):
        if len(times) < self.min_window_length:
            return None, None

        # Numerical Stability: Center the time data
        time_mean = np.mean(times)
        normalized_times = times - time_mean

        try:
            coeffs = np.polyfit(normalized_times, data_series, self.polyorder)
            deriv_coeffs = np.polyder(coeffs)

            last_normalized_time = normalized_times[-1]
            filtered_val = np.polyval(coeffs, last_normalized_time)
            filtered_deriv = np.polyval(deriv_coeffs, last_normalized_time)

            return filtered_val, filtered_deriv
        except (np.linalg.LinAlgError, ValueError):
            return None, None

    def timer_callback(self):
        try:
            # lookup_transform(target_frame, source_frame, time)
            transform = self.tf_buffer.lookup_transform(
                self.world_frame,
                self.base_frame,
                rclpy.time.Time()
            )
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            # Throttle logging in ROS 2
            self.get_logger().warn(f"Waiting for transform: {e}", throttle_duration_sec=5.0)
            return

        # Jump Detection
        curr_x = transform.transform.translation.x
        curr_y = transform.transform.translation.y
        if self.positions_x:
            dist = math.hypot(curr_x - self.positions_x[-1], curr_y - self.positions_y[-1])
            if dist > self.max_jump_distance:
                self.reset_filter_state()

        self.store_new_data(transform)

        if len(self.timestamps) < self.min_window_length:
            return

        times = np.array(self.timestamps)

        # Calculate World-frame Derivatives
        _, vx_world = self.fit_and_differentiate(times, list(self.positions_x))
        _, vy_world = self.fit_and_differentiate(times, list(self.positions_y))
        filtered_yaw, wz_base = self.fit_and_differentiate(times, list(self.yaws_unwrapped))

        if vx_world is None or filtered_yaw is None:
            return

        # Rotate World Velocity to Base Frame
        cos_y, sin_y = math.cos(filtered_yaw), math.sin(filtered_yaw)
        vx_base = vx_world * cos_y + vy_world * sin_y
        vy_base = -vx_world * sin_y + vy_world * cos_y

        self.publish_twist(transform.header.stamp, vx_base, vy_base, wz_base)

    def store_new_data(self, transform):
        # Convert ROS 2 Time msg to float seconds
        t = transform.header.stamp
        new_time = t.sec + t.nanosec * 1e-9

        if self.timestamps and new_time <= self.timestamps[-1]:
            return

        # Get Yaw using tf_transformations
        q = transform.transform.rotation
        quat_list = [q.x, q.y, q.z, q.w]
        _, _, yaw = tft.euler_from_quaternion(quat_list)

        # Unwrapping
        if self.yaws_unwrapped:
            # np.unwrap works on arrays; for single points we manual check or append
            yaws_list = list(self.yaws_unwrapped)
            yaws_list.append(yaw)
            new_unwrapped_yaw = np.unwrap(yaws_list)[-1]
        else:
            new_unwrapped_yaw = yaw

        self.timestamps.append(new_time)
        self.positions_x.append(transform.transform.translation.x)
        self.positions_y.append(transform.transform.translation.y)
        self.yaws_unwrapped.append(new_unwrapped_yaw)

    def publish_twist(self, stamp, vx, vy, wz):
        msg = TwistStamped()
        msg.header.stamp = stamp
        msg.header.frame_id = self.base_frame
        msg.twist.linear.x = float(vx)
        msg.twist.linear.y = float(vy)
        msg.twist.angular.z = float(wz)
        self.twist_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = AGVTwistEstimator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
