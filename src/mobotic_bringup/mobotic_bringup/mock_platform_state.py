import rclpy
from mobotic_interfaces.msg import BatterySystemState, ScannerSafetyState
from rclpy.node import Node


class MockPlatformState(Node):
    """Publish permissive non-hardware state for the virtual bringup only."""

    def __init__(self):
        super().__init__('mock_platform_state')
        self.declare_parameter('publish_period', 0.05)
        self.declare_parameter('frame_id', 'base_link')

        publish_period = float(self.get_parameter('publish_period').value)
        if publish_period <= 0.0:
            raise ValueError('publish_period must be positive')
        self.frame_id = str(self.get_parameter('frame_id').value)

        self.scanner_publishers = {
            location: self.create_publisher(
                ScannerSafetyState, f'scanner/{location}/safety_zone_status', 1)
            for location in ('front_left', 'rear_right')
        }
        self.battery_publisher = self.create_publisher(
            BatterySystemState, 'battery/system_state', 1
        )
        self.timer = self.create_timer(publish_period, self._publish)
        self.get_logger().warning(
            'Publishing MOCK scanner and battery readiness. '
            'This node must never be used with physical drives.'
        )

    def _publish(self):
        stamp = self.get_clock().now().to_msg()

        for location, publisher in self.scanner_publishers.items():
            scanner = ScannerSafetyState()
            scanner.header.stamp = stamp
            scanner.header.frame_id = self.frame_id
            scanner.scanner_id = location
            scanner.communication_ok = True
            scanner.protective_state_known = scanner.warning_state_known = True
            scanner.protective_field_clear = scanner.warning_field_clear = True
            scanner.source = 'virtual scanner fixture'
            publisher.publish(scanner)

        battery = BatterySystemState()
        battery.header.stamp = stamp
        battery.header.frame_id = self.frame_id
        battery.communication_ok = True
        battery.system_ready = True
        battery.high_voltage_connected = True
        battery.minimum_percentage = 1.0
        battery.status_message = 'virtual battery ready (mock state)'
        self.battery_publisher.publish(battery)


def main(args=None):
    rclpy.init(args=args)
    node = MockPlatformState()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
