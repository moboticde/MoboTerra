#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster

class OdomToTf(Node):
    def __init__(self):
        super().__init__('odom_to_tf')

        # 1. Initialize the dynamic TF broadcaster (for odometry)
        self.tf_broadcaster = TransformBroadcaster(self)

        # 2. Initialize the static TF broadcaster (for the laser)
        self.static_broadcaster = StaticTransformBroadcaster(self)

        # Publish the static laser transform immediately on startup
        self.publish_laser_transform()

        # Subscribe to the odometry topic
        self.subscription = self.create_subscription(
            Odometry,
            '/mobotic/odometry',
            self.odom_callback,
            1)

    def publish_laser_transform(self):
        t = TransformStamped()

        # Header configuration
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'base_link'    # The parent frame
        t.child_frame_id = 'front_scan'   # The child frame (update to match your scan's frame_id)

        # --- UPDATE THESE VALUES ---
        # Physical position of the laser relative to the center of the base_link (in meters)
        # Example: 10cm forward, 0cm left/right, 20cm up
        t.transform.translation.x = 1.05
        t.transform.translation.y = 0.661
        t.transform.translation.z = 0.2

        # Orientation of the laser (Quaternion)
        # Assuming the laser is mounted perfectly flat and facing perfectly forward:
        # [ 0, 0, -0.4871745, -0.8733046 ]
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = -0.4871745
        t.transform.rotation.w = -0.8733046

        # Broadcast the static transform
        self.static_broadcaster.sendTransform(t)
        self.get_logger().info('Published static transform: base_link -> laser_frame')

    def odom_callback(self, msg: Odometry):
        # Create a dynamic TransformStamped message
        t = TransformStamped()

        # Populate the header dynamically from the incoming odometry message
        t.header.stamp = msg.header.stamp
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'

        # Populate the translation
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z

        # Populate the rotation
        t.transform.rotation.x = msg.pose.pose.orientation.x
        t.transform.rotation.y = msg.pose.pose.orientation.y
        t.transform.rotation.z = msg.pose.pose.orientation.z
        t.transform.rotation.w = msg.pose.pose.orientation.w

        # Broadcast the dynamic transform
        self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = OdomToTf()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()