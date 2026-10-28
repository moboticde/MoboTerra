import rclpy
from rclpy.node import Node
from can_msgs.msg import Frame
from collections import deque

class CanTrafficAnalyzer(Node):
    def __init__(self):
        super().__init__('can_traffic_analyzer')

        self.tracked_ids = {0x70A: deque(maxlen=1)}

        # Subscriber for Outgoing messages
        self.tx_sub = self.create_subscription(
            Frame,
            'can_tx',
            self.tx_callback,
            10)

        # Subscriber for Incoming messages
        self.rx_sub = self.create_subscription(
            Frame,
            'can_rx',
            self.rx_callback,
            10)

        self.get_logger().info(f"Analyzing latency for IDs: {[hex(i) for i in self.tracked_ids.keys()]}")

    def tx_callback(self, msg):
        if msg.id in self.tracked_ids:
            now = self.get_clock().now()
            self.tracked_ids[msg.id].append(now)

    def rx_callback(self, msg):
        if msg.id in self.tracked_ids and len(self.tracked_ids[msg.id]) > 0:
            rx_now = self.get_clock().now()

            tx_time = self.tracked_ids[msg.id].popleft()

            duration = rx_now - tx_time
            ms_latency = duration.nanoseconds / 1e6

            self.get_logger().info(
                f"ID: {hex(msg.id)} | Latency: {ms_latency:.3f} ms"
            )

def main(args=None):
    rclpy.init(args=args)
    node = CanTrafficAnalyzer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
