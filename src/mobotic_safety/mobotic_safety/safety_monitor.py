import rclpy
from mobotic_interfaces.msg import SafetyIOState, SafetyState
from rclpy.node import Node
from std_msgs.msg import UInt8


class SafetyMonitor(Node):
    def __init__(self):
        super().__init__('safety_monitor')

        self.declare_parameter('status_timeout', 0.25)
        self.declare_parameter('publish_period', 0.05)
        self.declare_parameter('front_protective_ossd_mask', 0x03)
        self.declare_parameter('rear_protective_ossd_mask', 0x0C)
        self.declare_parameter('emergency_stop_mask', 0x10)
        self.declare_parameter('safety_override_mask', 0x20)
        self.declare_parameter('rear_warning_mask', 0x40)
        self.declare_parameter('front_warning_mask', 0x80)
        self.declare_parameter('safety_enable_mask', 0)
        self.declare_parameter('sto_mask', 0)
        self.declare_parameter('estop_implies_sto', True)

        self.status_timeout = float(self.get_parameter('status_timeout').value)
        self.publish_period = float(self.get_parameter('publish_period').value)
        self.front_ossd_mask = int(self.get_parameter('front_protective_ossd_mask').value)
        self.rear_ossd_mask = int(self.get_parameter('rear_protective_ossd_mask').value)
        self.estop_mask = int(self.get_parameter('emergency_stop_mask').value)
        self.override_mask = int(self.get_parameter('safety_override_mask').value)
        self.rear_warning_mask = int(self.get_parameter('rear_warning_mask').value)
        self.front_warning_mask = int(self.get_parameter('front_warning_mask').value)
        self.safety_enable_mask = int(self.get_parameter('safety_enable_mask').value)
        self.sto_mask = int(self.get_parameter('sto_mask').value)
        self.estop_implies_sto = bool(self.get_parameter('estop_implies_sto').value)

        if self.status_timeout <= 0.0 or self.publish_period <= 0.0:
            raise ValueError('status timeout and publish period must be positive')
        masks = [
            self.front_ossd_mask,
            self.rear_ossd_mask,
            self.estop_mask,
            self.override_mask,
            self.rear_warning_mask,
            self.front_warning_mask,
            self.safety_enable_mask,
            self.sto_mask,
        ]
        if any(mask < 0 or mask > 0xFF for mask in masks):
            raise ValueError('all safety masks must fit in one byte')

        self.io_publisher = self.create_publisher(SafetyIOState, 'safety/io_state', 1)
        self.state_publisher = self.create_publisher(SafetyState, 'safety/state', 1)
        self.subscription = self.create_subscription(
            UInt8, 'flexisoft/status_byte', self._status_callback, 10
        )
        self.timer = self.create_timer(self.publish_period, self._publish)

        self.last_status_time = None
        self.status_byte = 0

    def _status_callback(self, msg):
        self.status_byte = int(msg.data)
        self.last_status_time = self.get_clock().now()
        self._publish()

    def _communication_ok(self, now):
        return (
            self.last_status_time is not None
            and 0 <= (now - self.last_status_time).nanoseconds <= int(self.status_timeout * 1e9)
        )

    def _publish(self):
        now = self.get_clock().now()
        communication_ok = self._communication_ok(now)
        value = self.status_byte if communication_ok else 0

        front_ossd = communication_ok and bool(value & self.front_ossd_mask)
        rear_ossd = communication_ok and bool(value & self.rear_ossd_mask)
        emergency_stop = not communication_ok or bool(value & self.estop_mask)
        safety_override = communication_ok and bool(value & self.override_mask)
        front_warning_clear = communication_ok and not bool(value & self.front_warning_mask)
        rear_warning_clear = communication_ok and not bool(value & self.rear_warning_mask)
        safety_enable = communication_ok and (
            bool(value & self.safety_enable_mask) if self.safety_enable_mask else True
        )
        sto_active = not communication_ok or (
            bool(value & self.sto_mask) if self.sto_mask else False
        )
        if self.estop_implies_sto and emergency_stop:
            sto_active = True

        io_state = SafetyIOState()
        io_state.header.stamp = now.to_msg()
        io_state.emergency_stop_active = emergency_stop
        io_state.safety_enable_active = safety_enable
        io_state.sto_active = sto_active
        io_state.sto_state_known = communication_ok and bool(self.sto_mask)
        io_state.safety_enable_state_known = communication_ok and bool(self.safety_enable_mask)
        io_state.flexisoft_status_raw = self.status_byte
        io_state.front_left_protective_field_clear = front_ossd
        io_state.front_left_warning_field_clear = front_warning_clear
        io_state.front_left_ossd_active = front_ossd
        io_state.rear_right_protective_field_clear = rear_ossd
        io_state.rear_right_warning_field_clear = rear_warning_clear
        io_state.rear_right_ossd_active = rear_ossd
        self.io_publisher.publish(io_state)

        protective_stop = not front_ossd or not rear_ossd
        system_ready = communication_ok and safety_enable and not emergency_stop and not sto_active
        motion_permitted = system_ready and (safety_override or not protective_stop)

        state = SafetyState()
        state.header = io_state.header
        state.communication_ok = communication_ok
        state.system_ready = system_ready
        state.motion_permitted = motion_permitted
        state.protective_stop_active = protective_stop
        state.warning_field_active = not front_warning_clear or not rear_warning_clear
        state.emergency_stop_active = emergency_stop
        state.sto_active = sto_active
        state.safety_override_active = safety_override
        if not communication_ok:
            state.status_message = 'FlexiSoft status timeout'
        elif emergency_stop:
            state.status_message = 'emergency stop active'
        elif sto_active:
            state.status_message = 'safe torque off active'
        elif protective_stop and not safety_override:
            state.status_message = 'protective field occupied'
        elif safety_override:
            state.status_message = 'safety override active'
        else:
            state.status_message = 'safety system ready'
        self.state_publisher.publish(state)


def main(args=None):
    rclpy.init(args=args)
    node = SafetyMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
