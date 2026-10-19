"""Vendor-independent scanner field state from commissioned safety-controller I/O."""
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from mobotic_interfaces.msg import SafetyIOState, ScannerSafetyState
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue


class ScannerSafetyAdapter(Node):
    def __init__(self):
        super().__init__('scanner_safety_adapter')
        self.declare_parameter('status_timeout', 0.3)
        self.timeout = float(self.get_parameter('status_timeout').value)
        if self.timeout <= 0:
            raise ValueError('status timeout must be positive')
        self.state = None
        self.subscription = self.create_subscription(SafetyIOState, 'safety/io_state', self._receive, 1)
        self.publishers = {name: self.create_publisher(ScannerSafetyState, f'scanner/{name}/safety_zone_status', 1)
                           for name in ('front_left', 'rear_right')}
        self.diagnostics = {name: self.create_publisher(DiagnosticArray, f'scanner/{name}/diagnostics', 1)
                            for name in self.publishers}
        self.timer = self.create_timer(0.05, self._publish)

    def _receive(self, message):
        if message.header.stamp.sec < 0 or not 0 <= message.header.stamp.nanosec < 1000000000:
            return
        stamp = Time.from_msg(message.header.stamp)
        if stamp.nanoseconds <= 0 or not 0 <= (self.get_clock().now() - stamp).nanoseconds <= int(self.timeout * 1e9):
            return
        if self.state is None or Time.from_msg(message.header.stamp).nanoseconds > Time.from_msg(self.state.header.stamp).nanoseconds:
            self.state = message

    def _publish(self):
        now = self.get_clock().now()
        valid = (self.state is not None and self.state.scanner_state_known and
                 0 <= (now - Time.from_msg(self.state.header.stamp)).nanoseconds <= int(self.timeout * 1e9))
        for name, publisher in self.publishers.items():
            output = ScannerSafetyState()
            output.header = self.state.header if self.state is not None else output.header
            output.scanner_id = name
            output.communication_ok = valid
            output.protective_state_known = output.warning_state_known = output.ossd_state_known = valid
            if valid:
                output.protective_field_clear = getattr(self.state, name + '_protective_field_clear')
                output.warning_field_clear = getattr(self.state, name + '_warning_field_clear')
                output.ossd_active = getattr(self.state, name + '_ossd_active')
            output.source = 'commissioned safety-controller I/O; field-set number unavailable'
            publisher.publish(output)
            diagnostic = DiagnosticStatus()
            diagnostic.name = 'scanner/' + name
            diagnostic.hardware_id = name
            diagnostic.level = (DiagnosticStatus.ERROR if not valid else
                                DiagnosticStatus.WARN if not output.protective_field_clear or not output.ossd_active else
                                DiagnosticStatus.OK)
            diagnostic.message = ('Field monitoring unavailable' if not valid else
                                  'Protective stop' if diagnostic.level == DiagnosticStatus.WARN else 'Fields monitored')
            diagnostic.values = [KeyValue(key='source', value=output.source),
                                 KeyValue(key='field_set_known', value='false')]
            array = DiagnosticArray()
            array.header = output.header
            array.status = [diagnostic]
            self.diagnostics[name].publish(array)


def main(args=None):
    rclpy.init(args=args)
    node = ScannerSafetyAdapter()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
