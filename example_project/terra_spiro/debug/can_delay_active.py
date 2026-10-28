import rclpy
from rclpy.node import Node
from can_msgs.msg import Frame
import time

class CanLatencyMonitor(Node):
    def __init__(self):
        super().__init__('can_latency_monitor')

        # Configuration
        self.target_can_id = 0x123  # The ID we are looking for
        self.tx_timestamp = None

        # Publishers and Subscribers
        self.publisher = self.create_publisher(Frame, 'can_tx', 10)
        self.subscription = self.create_subscription(
            Frame,
            'can_rx',
            self.can_rx_callback,
            10)

        # Timer to send the message every 1 second
        self.timer = self.create_timer(1.0, self.send_can_frame)

        self.get_logger().info(f'Monitoring latency for CAN ID: {hex(self.target_can_id)}')

        self.msg = Frame()
        self.msg.id = self.target_can_id
        self.msg.dlc = 8
        self.msg.data = [1, 2, 3, 4, 5, 6, 7, 8]


    def send_can_frame(self):
        # Capture the time just before publishing
        self.tx_timestamp = self.get_clock().now()

        #Stamp the message
        self.msg.header.stamp = self.tx_timestamp.to_msg()

        self.publisher.publish(self.msg)

    def can_rx_callback(self, msg):
        if msg.id != self.target_can_id or not self.tx_timestamp:
            return

        rx_timestamp = self.get_clock().now()

        duration = rx_timestamp - self.tx_timestamp
        ms_latency = duration.nanoseconds / 1e6

        self.get_logger().info(f'Latency: {ms_latency:.3f} ms')

        self.tx_timestamp = None

def main(args=None):
    rclpy.init(args=args)
    node = CanLatencyMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()