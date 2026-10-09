import rclpy
from mobotic_interfaces.msg import BatterySystemState, SafetyIOState, SafetyState
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

        self.safety_publisher = self.create_publisher(
            SafetyState, 'safety/state', 1
        )
        self.safety_io_publisher = self.create_publisher(
            SafetyIOState, 'safety/io_state', 1
        )
        self.battery_publisher = self.create_publisher(
            BatterySystemState, 'battery/system_state', 1
        )
        self.timer = self.create_timer(publish_period, self._publish)
        self.get_logger().warning(
            'Publishing MOCK safety and battery readiness. '
            'This node must never be used with physical drives.'
        )

    def _publish(self):
        stamp = self.get_clock().now().to_msg()

        safety = SafetyState()
        safety.header.stamp = stamp
        safety.header.frame_id = self.frame_id
        safety.communication_ok = True
        safety.system_ready = True
        safety.motion_permitted = True
        safety.protective_stop_active = False
        safety.warning_field_active = False
        safety.emergency_stop_active = False
        safety.sto_active = False
        safety.safety_override_active = False
        safety.status_message = 'virtual platform ready (mock state)'
        self.safety_publisher.publish(safety)

        safety_io = SafetyIOState()
        safety_io.header.stamp = stamp
        safety_io.header.frame_id = self.frame_id
        safety_io.emergency_stop_active = False
        safety_io.safety_enable_active = True
        safety_io.sto_active = False
        safety_io.front_left_protective_field_clear = True
        safety_io.front_left_warning_field_clear = True
        safety_io.front_left_ossd_active = True
        safety_io.rear_right_protective_field_clear = True
        safety_io.rear_right_warning_field_clear = True
        safety_io.rear_right_ossd_active = True
        self.safety_io_publisher.publish(safety_io)

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
