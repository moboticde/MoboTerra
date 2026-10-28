#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import numpy as np
from scipy.spatial.transform import Rotation as R

from geometry_msgs.msg import PoseStamped
from std_srvs.srv import Empty
from tf2_ros import Buffer, TransformListener
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException

class RelativePoseCalculator(Node):
    def __init__(self):
        super().__init__('relative_pose_calculator')

        # 1. Declare and Fetch Parameters
        self.declare_parameter('global_frame', 'map')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('robot_frame', 'base_link')

        self.global_frame = self.get_parameter('global_frame').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.robot_frame = self.get_parameter('robot_frame').value

        # 2. Setup Publishers (No namespaces, PoseStamped messages)
        self.odom_pub = self.create_publisher(PoseStamped, 'relative_odom_pose', 1)
        self.loc_pub = self.create_publisher(PoseStamped, 'relative_localization_pose', 1)

        # 3. Setup Reset Service
        self.reset_srv = self.create_service(Empty, 'reset_relative_poses', self.reset_callback)

        # 4. Setup TF2 Buffer and Listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # State variables for the baselines
        self.needs_reset = True
        self.T_o_start_inv = np.eye(4)
        self.T_m_start_inv = np.eye(4)

        # 5. Setup Timer (Running at 50Hz / 0.02s)
        self.timer = self.create_timer(0.02, self.timer_callback)
        self.get_logger().info(f"Node started. Waiting for transforms to '{self.robot_frame}'...")

    def reset_callback(self, request, response):
        """Triggers the recalculation of the starting baseline."""
        self.needs_reset = True
        self.get_logger().info("Reset triggered. Capturing new baseline...")
        return response

    def transform_to_matrix(self, transform_msg):
        """Converts a geometry_msgs/Transform into a 4x4 numpy matrix."""
        tx = transform_msg.transform.translation.x
        ty = transform_msg.transform.translation.y
        tz = transform_msg.transform.translation.z

        qx = transform_msg.transform.rotation.x
        qy = transform_msg.transform.rotation.y
        qz = transform_msg.transform.rotation.z
        qw = transform_msg.transform.rotation.w

        mat = np.eye(4)
        # scipy Rotation uses [x, y, z, w] format
        mat[:3, :3] = R.from_quat([qx, qy, qz, qw]).as_matrix()
        mat[:3, 3] = [tx, ty, tz]

        return mat

    def matrix_to_pose(self, mat, frame_id, timestamp):
        """Converts a 4x4 numpy matrix back into a geometry_msgs/PoseStamped."""
        pose = PoseStamped()
        pose.header.stamp = timestamp
        pose.header.frame_id = frame_id

        pose.pose.position.x = mat[0, 3]
        pose.pose.position.y = mat[1, 3]
        pose.pose.position.z = mat[2, 3]

        # Convert rotation matrix back to quaternion [x, y, z, w]
        quat = R.from_matrix(mat[:3, :3]).as_quat()
        pose.pose.orientation.x = quat[0]
        pose.pose.orientation.y = quat[1]
        pose.pose.orientation.z = quat[2]
        pose.pose.orientation.w = quat[3]

        return pose

    def timer_callback(self):
        try:
            # Lookup the latest transforms
            now = rclpy.time.Time()
            t_odom = self.tf_buffer.lookup_transform(self.odom_frame, self.robot_frame, now)
            t_map = self.tf_buffer.lookup_transform(self.global_frame, self.robot_frame, now)

            # Convert raw transforms into 4x4 matrices
            T_o_current = self.transform_to_matrix(t_odom)
            T_m_current = self.transform_to_matrix(t_map)

            # Handle Reset / Initialization
            if self.needs_reset:
                self.T_o_start_inv = np.linalg.inv(T_o_current)
                self.T_m_start_inv = np.linalg.inv(T_m_current)
                self.needs_reset = False
                self.get_logger().info("New baseline mathematically zeroed. Publishing relative poses.")
                return

            # Calculate Relative Poses: P_relative = T_start_inverse * T_current
            P_odom = self.T_o_start_inv @ T_o_current
            P_map = self.T_m_start_inv @ T_m_current

            # Capture a single timestamp so both messages perfectly align in PlotJuggler
            sync_stamp = self.get_clock().now().to_msg()

            # Generate Messages tied to the global frame for RViz
            msg_odom = self.matrix_to_pose(P_odom, self.global_frame, sync_stamp)
            msg_map = self.matrix_to_pose(P_map, self.global_frame, sync_stamp)

            # Publish
            self.odom_pub.publish(msg_odom)
            self.loc_pub.publish(msg_map)

        except (LookupException, ConnectivityException, ExtrapolationException):
            # Silently pass TF errors, as they are common during startup/bag looping
            pass

def main(args=None):
    rclpy.init(args=args)
    node = RelativePoseCalculator()
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