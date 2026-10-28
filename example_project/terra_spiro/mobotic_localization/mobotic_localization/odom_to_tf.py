import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from tf2_ros import TransformBroadcaster


class OdomToTf(Node):
    def __init__(self):
        super().__init__("odom_to_tf")

        self.declare_parameter("odom_topic", "/mobotic/odometry")
        self.declare_parameter("parent_frame", "")
        self.declare_parameter("child_frame", "")

        odom_topic = self.get_parameter("odom_topic").get_parameter_value().string_value
        self.parent_frame_override = (
            self.get_parameter("parent_frame").get_parameter_value().string_value
        )
        self.child_frame_override = (
            self.get_parameter("child_frame").get_parameter_value().string_value
        )

        self.broadcaster = TransformBroadcaster(self)
        self.sub = self.create_subscription(Odometry, odom_topic, self.on_odom, 20)

        self.get_logger().info(f"Broadcasting TF from odometry on '{odom_topic}'")

    def on_odom(self, msg: Odometry):
        tf = TransformStamped()
        tf.header.stamp = msg.header.stamp
        tf.header.frame_id = self.parent_frame_override or msg.header.frame_id
        tf.child_frame_id = self.child_frame_override or msg.child_frame_id
        tf.transform.translation.x = msg.pose.pose.position.x
        tf.transform.translation.y = msg.pose.pose.position.y
        tf.transform.translation.z = msg.pose.pose.position.z
        tf.transform.rotation = msg.pose.pose.orientation
        self.broadcaster.sendTransform(tf)


def main():
    rclpy.init()
    node = OdomToTf()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
