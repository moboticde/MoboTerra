#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

class TfToPosePublisher(Node):
    def __init__(self):
        super().__init__('tf_to_pose_publisher')
        
        # Define the frames
        self.target_frame = 'world'      # The fixed global frame
        self.source_frame = 'base_link'  # The moving robot frame
        
        # Initialize the transform buffer and listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Create a publisher for the PoseStamped message
        self.publisher = self.create_publisher(PoseStamped, '/pose_world_frame', 10)
        
        # Create a timer to look up and publish the transform at 20 Hz (0.05 seconds)
        self.timer = self.create_timer(0.05, self.timer_callback)
        self.get_logger().info(f"Publishing {self.target_frame} -> {self.source_frame} to /pose_world_frame")

    def timer_callback(self):
        try:
            # Look up the latest transform from world to base_link
            t = self.tf_buffer.lookup_transform(
                self.target_frame,
                self.source_frame,
                rclpy.time.Time() # Gets the latest available transform
            )
            
            # Create the PoseStamped message
            msg = PoseStamped()
            
            # Use the exact timestamp from the transform to ensure perfect sync
            msg.header.stamp = t.header.stamp 
            msg.header.frame_id = self.target_frame
            
            # Populate translation -> position
            msg.pose.position.x = t.transform.translation.x
            msg.pose.position.y = t.transform.translation.y
            msg.pose.position.z = t.transform.translation.z
            
            # Populate rotation -> orientation
            msg.pose.orientation = t.transform.rotation
            
            # Publish the message
            self.publisher.publish(msg)
            
        except TransformException as ex:
            # If the transform chain is broken or hasn't loaded yet, log it once in a while
            self.get_logger().debug(f'Could not transform {self.target_frame} to {self.source_frame}: {ex}')

def main(args=None):
    rclpy.init(args=args)
    node = TfToPosePublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
