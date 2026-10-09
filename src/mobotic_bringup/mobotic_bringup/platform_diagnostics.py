"""Observability only. This node cannot enable drives or operate hardware STO."""
import time
import math

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from mobotic_interfaces.msg import BatterySystemState, ManualControlState, SafetyIOState, SafetyState, VehicleModeState
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry

from .diagnostic_health import OK, WARN, ERROR, freshness, battery_health, safety_health


class PlatformDiagnostics(Node):
    def __init__(self):
        super().__init__('platform_diagnostics')
        self.declare_parameter('monitor_manual', True)
        self.declare_parameter('monitor_scanners', True)
        self.declare_parameter('monitor_odometry', True)
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
            'battery/system_state': (BatterySystemState, deadlines['battery_timeout']),
            'safety/state': (SafetyState, deadlines['safety_timeout']),
            'safety/io_state': (SafetyIOState, deadlines['safety_timeout']),
            'vehicle/mode_state': (VehicleModeState, deadlines['mode_timeout']),
        }
        if self.get_parameter('monitor_manual').value:
            self.channels['manual/state'] = (ManualControlState, deadlines['manual_timeout'])
        if self.get_parameter('monitor_scanners').value:
            for location in ('front_left', 'rear_right'):
                self.channels[f'scanner/{location}/scan'] = (LaserScan, deadlines['scanner_timeout'])
        if self.get_parameter('monitor_odometry').value:
            self.channels['odometry'] = (Odometry, deadlines['odometry_timeout'])
        self.input_subscriptions = [
            self.create_subscription(kind, topic, lambda msg, key=topic: self._receive(key, msg),
                                     qos_profile_sensor_data)
            for topic, (kind, _) in self.channels.items()
        ]
        self.publisher = self.create_publisher(DiagnosticArray, 'diagnostics', 10)
        self.steady_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.timer = self.create_timer(deadlines['publish_period'], self._publish, clock=self.steady_clock)

    def _receive(self, topic, msg):
        self.cache[topic] = (msg, time.monotonic())

    def _publish(self):
        array = DiagnosticArray()
        array.header.stamp = self.get_clock().now().to_msg()
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
            status.level, status.message = freshness(received, time.monotonic(), timeout, stamp_age)
            if status.level == OK:
                if topic == 'battery/system_state':
                    status.level, status.message = battery_health(msg, self.minimum_soc)
                    status.values = [KeyValue(key='minimum_soc', value=str(msg.minimum_percentage)),
                                     KeyValue(key='stale_participants', value=','.join(msg.stale_batteries)),
                                     KeyValue(key='faulted_participants', value=','.join(msg.faulted_batteries))]
                elif topic == 'safety/state':
                    status.level, status.message = safety_health(msg)
                elif topic == 'safety/io_state':
                    status.level = OK if msg.sto_state_known and msg.safety_enable_state_known else WARN
                    status.message = 'Mapped hardware signals' if status.level == OK else 'Independent STO/enable signals unmapped; interlocks inferred'
                    status.values = [KeyValue(key='flexisoft_status_raw', value=hex(msg.flexisoft_status_raw)),
                                     KeyValue(key='sto_state_known', value=str(msg.sto_state_known)),
                                     KeyValue(key='safety_enable_state_known', value=str(msg.safety_enable_state_known))]
                elif topic == 'vehicle/mode_state':
                    status.level = ERROR if 'timeout' in msg.status_message.lower() else OK if msg.motion_permitted else WARN
                    status.message = msg.status_message or 'Waiting for motion permission'
                    status.values = [KeyValue(key='mode', value=str(msg.current_mode)),
                                     KeyValue(key='transition_in_progress', value=str(msg.transition_in_progress))]
                elif topic == 'manual/state':
                    status.level = OK if msg.connected else WARN
                    status.message = msg.status_message or ('Connected' if msg.connected else 'Joystick disconnected')
                elif topic.startswith('scanner/') and not msg.ranges:
                    status.level, status.message = WARN, 'Empty laser scan'
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
