"""Observability only. This node cannot enable drives or operate hardware STO."""
import time
import math

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from mobotic_interfaces.msg import BatterySystemState, OperatorState, SafetyIOState, SafetyState, VehicleModeState, ComponentHealth, ComponentHealthArray
from geometry_msgs.msg import TwistStamped
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry

from .diagnostic_health import OK, WARN, ERROR, freshness, battery_health, safety_health
from .host_health import read_host_health


class PlatformDiagnostics(Node):
    def __init__(self):
        super().__init__('platform_diagnostics')
        self.declare_parameter('monitor_manual', True)
        self.declare_parameter('monitor_scanners', True)
        self.declare_parameter('monitor_odometry', True)
        self.declare_parameter('host_state_path', '')
        self.host_state_path = str(self.get_parameter('host_state_path').value)
        self.declare_parameter('minimum_battery_percentage', 0.2)
        self.minimum_soc = float(self.get_parameter('minimum_battery_percentage').value)
        if not math.isfinite(self.minimum_soc) or not 0 <= self.minimum_soc <= 1:
            raise ValueError('minimum_battery_percentage must be a fraction in [0, 1]')
        self.cache = {}
        deadlines = {}
        for key, default in (('battery_timeout', 0.6), ('safety_timeout', 0.3),
                             ('mode_timeout', 0.3), ('manual_timeout', 0.25),
                             ('scanner_timeout', 0.5), ('odometry_timeout', 0.5),
                             ('publish_period', 0.5)):
            self.declare_parameter(key, default)
            deadlines[key] = float(self.get_parameter(key).value)
            if not math.isfinite(deadlines[key]) or deadlines[key] <= 0:
                raise ValueError(f'{key} must be finite and positive')
        self.channels = {
            'vehicle/measured_velocity': (TwistStamped, deadlines['odometry_timeout']),
            'battery/system_state': (BatterySystemState, deadlines['battery_timeout']),
            'safety/state': (SafetyState, deadlines['safety_timeout']),
            'safety/io_state': (SafetyIOState, deadlines['safety_timeout']),
            'vehicle/mode': (VehicleModeState, deadlines['mode_timeout']),
        }
        if self.get_parameter('monitor_manual').value:
            self.channels['operator/state'] = (OperatorState, deadlines['manual_timeout'])
        if self.get_parameter('monitor_scanners').value:
            for location in ('front_left', 'rear_right'):
                self.channels[f'scanner/{location}/scan'] = (LaserScan, deadlines['scanner_timeout'])
        if self.get_parameter('monitor_odometry').value:
            self.channels['vehicle/odom'] = (Odometry, deadlines['odometry_timeout'])
        self.input_subscriptions = [
            self.create_subscription(kind, topic, lambda msg, key=topic: self._receive(key, msg),
                                     qos_profile_sensor_data)
            for topic, (kind, _) in self.channels.items()
        ]
        self.publisher = self.create_publisher(DiagnosticArray, 'diagnostics', 10)
        self.health_publisher = self.create_publisher(ComponentHealthArray, 'internal/component_health', 1)
        self.steady_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.timer = self.create_timer(deadlines['publish_period'], self._publish, clock=self.steady_clock)
        self.health_timer = self.create_timer(0.05, self._publish_health, clock=self.steady_clock)

    def _publish_health(self):
        array = ComponentHealthArray()
        array.header.stamp = self.get_clock().now().to_msg()
        if self.host_state_path:
            record = ComponentHealth()
            record.header = array.header
            record.component_id = 'host'
            record.initialized = True
            record.monitoring_valid, record.fault_active, reason = read_host_health(
                self.host_state_path, time.time())
            record.reason_codes = [reason] if reason else []
            array.components.append(record)
        for topic, component in (('vehicle/measured_velocity', 'kinematics'), ('vehicle/odom', 'odometry')):
            if topic not in self.channels:
                continue
            record = ComponentHealth()
            record.component_id = component
            msg, received = self.cache.get(topic, (None, None))
            record.initialized = msg is not None
            if msg is not None:
                record.header = msg.header
                stamp_seconds = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
                age = self.get_clock().now().nanoseconds / 1e9 - stamp_seconds
                record.monitoring_valid = stamp_seconds > 0 and freshness(received, time.monotonic(), self.channels[topic][1], age)[0] == OK
                twist = msg.twist.twist if topic == 'vehicle/odom' else msg.twist
                record.fault_active = not all(math.isfinite(v) for v in (twist.linear.x, twist.linear.y, twist.angular.z))
            record.reason_codes = [] if record.monitoring_valid and not record.fault_active else ['MONITORING_INVALID']
            array.components.append(record)
        self.health_publisher.publish(array)

    def _receive(self, topic, msg):
        self.cache[topic] = (msg, time.monotonic())

    def _publish(self):
        array = DiagnosticArray()
        array.header.stamp = self.get_clock().now().to_msg()
        if self.host_state_path:
            valid, fault, reason = read_host_health(self.host_state_path, time.time())
            status = DiagnosticStatus()
            status.name = f'{self.get_namespace().rstrip("/")}/platform/host'
            status.hardware_id = 'MoboTerra host'
            status.level = bytes([OK if valid and not fault else ERROR])
            status.message = reason or 'Host CAN and process monitoring available'
            array.status.append(status)
        for topic, (_, timeout) in self.channels.items():
            status = DiagnosticStatus()
            status.name = f'{self.get_namespace().rstrip("/")}/platform/{topic}'
            status.hardware_id = 'MoboTerra'
            msg, received = self.cache.get(topic, (None, None))
            stamp_age = None
            if msg is not None:
                stamp = msg.header.stamp
                stamp_seconds = stamp.sec + stamp.nanosec / 1e9
                stamp_age = self.get_clock().now().nanoseconds / 1e9 - stamp_seconds
                if stamp_seconds == 0:
                    stamp_age = float('inf')
            level, status.message = freshness(received, time.monotonic(), timeout, stamp_age)
            if level == OK:
                if topic == 'battery/system_state':
                    level, status.message = battery_health(msg, self.minimum_soc)
                    status.values = [KeyValue(key='minimum_soc', value=str(msg.minimum_percentage)),
                                     KeyValue(key='stale_participants', value=','.join(msg.stale_batteries)),
                                     KeyValue(key='faulted_participants', value=','.join(msg.faulted_batteries))]
                elif topic == 'safety/state':
                    level, status.message = safety_health(msg)
                elif topic == 'safety/io_state':
                    level = OK if msg.sto_state_known and msg.safety_enable_state_known else WARN
                    status.message = 'Mapped hardware signals' if level == OK else 'Independent STO/enable signals unmapped; interlocks inferred'
                    status.values = [KeyValue(key='flexisoft_status_raw', value=hex(msg.flexisoft_status_raw)),
                                     KeyValue(key='sto_state_known', value=str(msg.sto_state_known)),
                                     KeyValue(key='safety_enable_state_known', value=str(msg.safety_enable_state_known))]
                elif topic == 'vehicle/mode':
                    level = ERROR if msg.error_active else OK if msg.motion_permitted else WARN
                    status.message = msg.status_message or 'Waiting for motion permission'
                    status.values = [KeyValue(key='mode', value=str(msg.current_mode)),
                                     KeyValue(key='transition_in_progress', value=str(msg.transition_in_progress))]
                elif topic == 'operator/state':
                    level = OK if msg.connected else WARN
                    status.message = msg.status_message or ('Connected' if msg.connected else 'Joystick disconnected')
                elif topic.startswith('scanner/') and not msg.ranges:
                    level, status.message = WARN, 'Empty laser scan'
            # diagnostic_msgs uses a ROS byte, represented by one byte in Python.
            status.level = bytes([level])
            array.status.append(status)
        self.publisher.publish(array)


def main(args=None):
    rclpy.init(args=args)
    node = PlatformDiagnostics()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
