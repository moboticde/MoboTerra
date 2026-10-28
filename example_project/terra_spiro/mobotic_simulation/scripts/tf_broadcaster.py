#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray, TransformStamped
from tf2_ros import TransformBroadcaster

class GroundTruthTfBroadcaster(Node):
    def __init__(self):
        super().__init__('ground_truth_tf_broadcaster')
        # Subscribe to the topic bridged from Gazebo
        self.subscription = self.create_subscription(
            PoseArray,
            'pose',
            self.pose_callback,
            1)
        self.tf_broadcaster = TransformBroadcaster(self)

    def pose_callback(self, msg):
        if not msg.poses:
            return

        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'mobotic/base_link' # Or 'odom' depending on your tf tree

        # Assuming the robot is the first pose in the array
        t.transform.translation.x = msg.poses[0].position.x
        t.transform.translation.y = msg.poses[0].position.y
        t.transform.translation.z = msg.poses[0].position.z
        t.transform.rotation = msg.poses[0].orientation

        self.tf_broadcaster.sendTransform(t)

def main():
    rclpy.init()
    node = GroundTruthTfBroadcaster()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()