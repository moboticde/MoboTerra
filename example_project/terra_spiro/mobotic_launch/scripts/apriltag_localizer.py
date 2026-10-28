#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import tf2_ros
import numpy as np
from scipy.spatial.transform import Rotation as R

from apriltag_msgs.msg import AprilTagDetectionArray
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry


class ApriltagLocalizer(Node):
    def __init__(self):
        super().__init__('apriltag_localizer')

        # TF2 Setup
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # Subscription to AprilTag detections
        self.subscription = self.create_subscription(
            AprilTagDetectionArray,
            '/detections',
            self.detections_callback,
            10
        )

        self.odom_pub = self.create_publisher(Odometry, 'apriltag_odom', 10)

        self.get_logger().info("AprilTag Localizer Node Started.")

    def detections_callback(self, msg):
        if not msg.detections:
            return

        # 1. Determine the tag with the highest decision margin
        best_det = max(msg.detections, key=lambda d: d.decision_margin)
        # best localization
        print(best_det)
        tag_id = best_det.id
        print(tag_id)
        tag_frame = f'tag_{tag_id}_map'
        camera_frame = msg.header.frame_id

        time = msg.header.stamp

        try:
            # 2. Get static maps -> tagN transform
            t_map_tag = self.tf_buffer.lookup_transform(
                'maps',
                tag_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.05)
            )

            # 3. Get camera_optical_frame -> base_link transform (robot kinematics)
            t_cam_base = self.tf_buffer.lookup_transform(
                camera_frame,
                'base_link',
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.05)
            )

            # 4. Get camera_optical_frame -> tagN directly from the detection message
            t_cam_opt_tag = self.tf_buffer.lookup_transform(
                f'tag_{tag_id}',
                camera_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.05)
            )

            t_map_tag.transform.rotation.x = 0.0
            t_map_tag.transform.rotation.y = 1.0
            t_map_tag.transform.rotation.z = 0.0
            t_map_tag.transform.rotation.w = 0.0

            # 5. Convert everything to 4x4 Homogeneous Transformation Matrices
            H_map_tag = self._transform_msg_to_matrix(t_map_tag.transform)
            H_cam_base = self._transform_msg_to_matrix(t_cam_base.transform)
            H_cam_tag = self._transform_msg_to_matrix(t_cam_opt_tag.transform)

            # 6. Calculate maps -> base_link
            H_tag_cam = np.linalg.inv(H_cam_tag)
            H_map_base = H_map_tag @ H_cam_tag @ H_cam_base

            # 7. Convert back to TransformStamped and broadcast
            t_map_base = self._matrix_to_transform_msg(H_map_base, time, 'odom', 'base_link_improved_loc')
            self.tf_broadcaster.sendTransform(t_map_base)
            self._publish_odometry(H_map_base, time)

        except tf2_ros.TransformException as ex:
            self.get_logger().warn(f'Transform error: {ex}')

    # --- Helper Methods for Matrix Math ---

    def _transform_msg_to_matrix(self, t_msg):
        """Converts geometry_msgs/Transform to a 4x4 numpy matrix."""
        mat = np.eye(4)
        t = t_msg.translation
        q = t_msg.rotation
        mat[:3, :3] = R.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
        mat[:3, 3] = [t.x, t.y, t.z]
        return mat

    def _pose_msg_to_matrix(self, p_msg):
        """Converts geometry_msgs/Pose to a 4x4 numpy matrix."""
        mat = np.eye(4)
        t = p_msg.position
        q = p_msg.orientation
        mat[:3, :3] = R.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
        mat[:3, 3] = [t.x, t.y, t.z]
        return mat

    def _matrix_to_transform_msg(self, mat, time_stamp, frame_id, child_frame_id):
        """Converts a 4x4 numpy matrix into a geometry_msgs/TransformStamped."""
        t = TransformStamped()
        t.header.stamp = time_stamp
        t.header.frame_id = frame_id
        t.child_frame_id = child_frame_id

        t.transform.translation.x = float(mat[0, 3])
        t.transform.translation.y = float(mat[1, 3])
        t.transform.translation.z = float(mat[2, 3])

        q = R.from_matrix(mat[:3, :3]).as_quat() # Returns [x, y, z, w]
        t.transform.rotation.x = float(q[0])
        t.transform.rotation.y = float(q[1])
        t.transform.rotation.z = float(q[2])
        t.transform.rotation.w = float(q[3])

        return t

    def _publish_odometry(self, H_map_base, time_stamp):
        """Constructs and publishes a nav_msgs/Odometry message."""
        odom = Odometry()
        odom.header.stamp = time_stamp
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link_improved_loc'

        # Set Position
        odom.pose.pose.position.x = float(H_map_base[0, 3])
        odom.pose.pose.position.y = float(H_map_base[1, 3])
        odom.pose.pose.position.z = float(H_map_base[2, 3])

        # Set Orientation
        q = R.from_matrix(H_map_base[:3, :3]).as_quat()
        odom.pose.pose.orientation.x = float(q[0])
        odom.pose.pose.orientation.y = float(q[1])
        odom.pose.pose.orientation.z = float(q[2])
        odom.pose.pose.orientation.w = float(q[3])

        # Note: Twist (velocity) is implicitly zero in Python initialization.
        # Since AprilTags only give us position snapshots, we have no velocity data.

        # Optional: Set a basic covariance so Nav2 or EKF doesn't complain about 0 covariance
        # A 6x6 matrix represented as a 36-element array. Setting diagonal elements.
        covariance = np.zeros((6, 6))
        np.fill_diagonal(covariance, 0.05) # 0.05 variance for x, y, z, roll, pitch, yaw
        odom.pose.covariance = covariance.flatten().tolist()

        self.odom_pub.publish(odom)


def main(args=None):
    rclpy.init(args=args)
    node = ApriltagLocalizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()