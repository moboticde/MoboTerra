import math
from dataclasses import dataclass, field

import rclpy
from can_msgs.msg import Frame
from mobotic_interfaces.msg import BatterySystemState
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.time import Time
from sensor_msgs.msg import BatteryState
from .telemetry import decode_soc, decode_electrical, values_valid


HIGH_VOLTAGE_CONNECTED = 1
OPERATIONAL = 2


def parse_integer(value):
    return int(str(value), 0)


@dataclass
class BatteryTelemetry:
    identifier: str
    source_address: int
    status_pgn: int
    soc_pgn: int
    voltage_pgn: int
    high_voltage_status: int = 15
    operational_status: int = 15
    percentage: float = math.nan
    voltage: float = math.nan
    current: float = math.nan
    status_stamp: object = None
    soc_stamp: object = None
    electrical_stamp: object = None
    acquisition_stamps: dict = field(default_factory=dict)


class VanguardBatteryMonitor(Node):
    def __init__(self):
        super().__init__('vanguard_battery_monitor')

        self.declare_parameter('batteries.ids', Parameter.Type.STRING_ARRAY)
        self.declare_parameter('batteries.source_addresses', Parameter.Type.STRING_ARRAY)
        self.declare_parameter('batteries.status_pgns', Parameter.Type.STRING_ARRAY)
        self.declare_parameter('batteries.soc_pgns', Parameter.Type.STRING_ARRAY)
        self.declare_parameter('batteries.voltage_pgns', Parameter.Type.STRING_ARRAY)
        self.declare_parameter('primary_battery_id', '')
        self.declare_parameter('telemetry_timeout', 0.5)
        self.declare_parameter('publish_period', 0.1)

        identifiers = list(self.get_parameter('batteries.ids').value)
        source_addresses = list(self.get_parameter('batteries.source_addresses').value)
        status_pgns = list(self.get_parameter('batteries.status_pgns').value)
        soc_pgns = list(self.get_parameter('batteries.soc_pgns').value)
        voltage_pgns = list(self.get_parameter('batteries.voltage_pgns').value)
        self.primary_battery_id = str(self.get_parameter('primary_battery_id').value)
        self.telemetry_timeout = float(self.get_parameter('telemetry_timeout').value)
        self.publish_period = float(self.get_parameter('publish_period').value)

        sizes = {
            len(identifiers),
            len(source_addresses),
            len(status_pgns),
            len(soc_pgns),
            len(voltage_pgns),
        }
        if len(sizes) != 1 or not identifiers:
            raise ValueError('all batteries.* arrays must have the same non-zero size')
        if len(set(identifiers)) != len(identifiers) or any(not name for name in identifiers):
            raise ValueError('battery identifiers must be non-empty and unique')
        if self.primary_battery_id not in identifiers:
            raise ValueError('primary_battery_id must name one configured battery')
        if (not all(math.isfinite(value) for value in (self.telemetry_timeout, self.publish_period))
                or self.telemetry_timeout <= 0.0 or self.publish_period <= 0.0):
            raise ValueError('telemetry timeout and publish period must be positive')

        self.batteries = {}
        for index, identifier in enumerate(identifiers):
            battery = BatteryTelemetry(
                identifier=identifier,
                source_address=parse_integer(source_addresses[index]),
                status_pgn=parse_integer(status_pgns[index]),
                soc_pgn=parse_integer(soc_pgns[index]),
                voltage_pgn=parse_integer(voltage_pgns[index]),
            )
            if not 0 <= battery.source_address <= 0xFF:
                raise ValueError(f'invalid J1939 source address for {identifier}')
            self.batteries[identifier] = battery

        self.primary_publisher = self.create_publisher(BatteryState, 'battery/state', 1)
        self.system_state_publisher = self.create_publisher(
            BatterySystemState, 'battery/system_state', 1
        )
        self.subscription = self.create_subscription(Frame, 'can_rx', self._can_callback, 100)
        self.timer = self.create_timer(self.publish_period, self._publish)

    @staticmethod
    def _j1939_fields(arbitration_id):
        source_address = arbitration_id & 0xFF
        pgn = (arbitration_id >> 8) & 0x3FFFF
        return source_address, pgn

    def _can_callback(self, frame):
        if not frame.is_extended or frame.is_rtr or frame.is_error or frame.dlc != 8:
            return
        source_address, pgn = self._j1939_fields(frame.id)
        now = self.get_clock().now()
        try:
            acquisition_stamp = Time.from_msg(frame.header.stamp, clock_type=now.clock_type)
        except ValueError:
            return
        timestamped = acquisition_stamp.nanoseconds != 0
        if not timestamped:
            # Some CAN transports do not supply acquisition time.
            acquisition_stamp = now
        age_ns = (now - acquisition_stamp).nanoseconds
        if not -100_000_000 <= age_ns <= int(self.telemetry_timeout * 1e9):
            return
        # Match the CAN driver's 100 ms skew tolerance without granting extra
        # freshness to samples slightly ahead of our clock.
        sample_stamp = acquisition_stamp if age_ns >= 0 else now

        for battery in self.batteries.values():
            if source_address != battery.source_address:
                continue
            if pgn == battery.status_pgn:
                stamp_field = 'status_stamp'
            elif pgn == battery.soc_pgn:
                stamp_field = 'soc_stamp'
            elif pgn == battery.voltage_pgn:
                stamp_field = 'electrical_stamp'
            else:
                continue
            previous = getattr(battery, stamp_field)
            previous_acquisition = battery.acquisition_stamps.get(stamp_field)
            if previous is not None and sample_stamp.nanoseconds < previous.nanoseconds:
                continue
            if previous_acquisition is not None and (
                acquisition_stamp.nanoseconds < previous_acquisition
                or (timestamped and acquisition_stamp.nanoseconds == previous_acquisition)
            ):
                # Track the original timestamp too: repeated future-skewed
                # frames must not renew a stamp clamped to local reception time.
                continue
            if pgn == battery.status_pgn:
                battery.high_voltage_status = int(frame.data[5] & 0x0F)
                battery.operational_status = int((frame.data[5] >> 4) & 0x0F)
            elif pgn == battery.soc_pgn:
                raw = int(frame.data[0]) | (int(frame.data[1]) << 8)
                battery.percentage = decode_soc(raw)
            elif pgn == battery.voltage_pgn:
                raw_voltage = int(frame.data[4]) | (int(frame.data[5]) << 8)
                raw_current = int(frame.data[6]) | (int(frame.data[7]) << 8)
                battery.voltage, battery.current = decode_electrical(raw_voltage, raw_current)
            setattr(battery, stamp_field, sample_stamp)
            battery.acquisition_stamps[stamp_field] = acquisition_stamp.nanoseconds

    def _fresh(self, stamp, now):
        return (
            stamp is not None
            and 0 <= (now - stamp).nanoseconds <= int(self.telemetry_timeout * 1e9)
        )

    def _message(self, battery, now):
        status_fresh = self._fresh(battery.status_stamp, now)
        soc_fresh = self._fresh(battery.soc_stamp, now)
        electrical_fresh = self._fresh(battery.electrical_stamp, now)

        msg = BatteryState()
        msg.header.stamp = now.to_msg()
        msg.header.frame_id = battery.identifier
        msg.location = battery.identifier
        msg.voltage = battery.voltage if electrical_fresh else math.nan
        msg.current = battery.current if electrical_fresh else math.nan
        msg.percentage = battery.percentage if soc_fresh else math.nan
        msg.temperature = math.nan
        msg.charge = math.nan
        msg.capacity = math.nan
        msg.design_capacity = math.nan
        msg.power_supply_technology = BatteryState.POWER_SUPPLY_TECHNOLOGY_UNKNOWN
        msg.present = status_fresh and battery.high_voltage_status == HIGH_VOLTAGE_CONNECTED

        if not status_fresh:
            msg.power_supply_health = BatteryState.POWER_SUPPLY_HEALTH_WATCHDOG_TIMER_EXPIRE
        elif (battery.operational_status == OPERATIONAL and soc_fresh and electrical_fresh
              and values_valid(battery.percentage, battery.voltage, battery.current)):
            msg.power_supply_health = BatteryState.POWER_SUPPLY_HEALTH_GOOD
        else:
            msg.power_supply_health = BatteryState.POWER_SUPPLY_HEALTH_UNSPEC_FAILURE

        if not electrical_fresh or not math.isfinite(battery.current):
            msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_UNKNOWN
        elif battery.current > 0.1:
            msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_CHARGING
        elif battery.current < -0.1:
            msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_DISCHARGING
        else:
            msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_NOT_CHARGING
        return msg

    def _publish(self):
        now = self.get_clock().now()
        primary_message = None
        stale_batteries = []
        faulted_batteries = []
        percentages = []
        all_high_voltage_connected = True
        for identifier, battery in self.batteries.items():
            message = self._message(battery, now)
            if identifier == self.primary_battery_id:
                primary_message = message
            fields_fresh = (
                self._fresh(battery.status_stamp, now)
                and self._fresh(battery.soc_stamp, now)
                and self._fresh(battery.electrical_stamp, now)
            )
            if not fields_fresh:
                stale_batteries.append(identifier)
            else:
                percentages.append(battery.percentage)
            if (
                self._fresh(battery.status_stamp, now)
                and (battery.operational_status != OPERATIONAL or
                     (fields_fresh and not values_valid(battery.percentage, battery.voltage, battery.current)))
            ):
                faulted_batteries.append(identifier)
            all_high_voltage_connected = all_high_voltage_connected and (
                self._fresh(battery.status_stamp, now)
                and battery.high_voltage_status == HIGH_VOLTAGE_CONNECTED
            )
        if primary_message is not None:
            self.primary_publisher.publish(primary_message)

        system_state = BatterySystemState()
        system_state.header.stamp = now.to_msg()
        system_state.communication_ok = not stale_batteries
        system_state.high_voltage_connected = all_high_voltage_connected
        system_state.system_ready = (
            system_state.communication_ok
            and system_state.high_voltage_connected
            and not faulted_batteries
        )
        system_state.minimum_percentage = (
            min(percentages) if not stale_batteries and not faulted_batteries and percentages else math.nan
        )
        system_state.stale_batteries = stale_batteries
        system_state.faulted_batteries = faulted_batteries
        if stale_batteries:
            system_state.status_message = 'stale battery telemetry'
        elif faulted_batteries:
            system_state.status_message = 'battery participant fault or invalid telemetry'
        elif not all_high_voltage_connected:
            system_state.status_message = 'high voltage disconnected'
        else:
            system_state.status_message = 'battery system ready'
        self.system_state_publisher.publish(system_state)


def main(args=None):
    rclpy.init(args=args)
    node = VanguardBatteryMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
