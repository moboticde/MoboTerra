#!/usr/bin/env python3
"""Publish and optionally record decoded Mobotic CAN telemetry.

Run this node together with the ros2_socketcan bridge. It publishes a JSON
snapshot on the `can_telemetry` topic. Recording can be started and stopped
with Trigger services; each recording writes snapshots to a timestamped file.

Position, velocity, actual current, controller temperature, and steering bus
voltage are decoded from TPDOs. Traction bus voltage remains an SDO value
because object 0x6079 is not PDO-mappable on the Elmo drive. The traction
motor temperature is read from 0x22A4:00 by SDO. Traction RMS
current is calculated from a rolling window of actual-current PDO samples.
Steering RMS current is supplied directly by its TPDO. With the default
parameters, PDO acquisition and file snapshots run at 50 Hz, traction voltage
SDOs run at 10 Hz, and diagnostics/configuration SDOs run at 1 Hz. Elmo
digital inputs (0x60FD), STO status (0x2086), and predefined error history
(0x1003) are captured by the slow SDO poller. The most recent nonzero
0x603F error code is latched separately from the live error-code value.

The same snapshots also contain read-only Vanguard battery and charger data
decoded from the separate battery CAN receive topic: SOC, pack voltage/current,
BMS requested charge current, charger output/status/limits, and the SOC-based
EFF3 controller state. No separate Vanguard recording file is required.

Important:
    - Traction motor and controller temperatures are recorded separately.
    - Steering records controller temperature only.
    - The flat JSON/CSV recording contains fields such as:
        ftr_motor_temp_c
        ftr_controller_temp_c
        ftr_serial_number
        btr_motor_temp_c
        btr_controller_temp_c
        btr_serial_number
        fstr_controller_temp_c
        fstr_serial_number
        bstr_controller_temp_c
        bstr_serial_number
"""

import csv
import json
import math
import struct
from collections import deque
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Any, Deque, Dict, List, Optional, Tuple

import rclpy
from can_msgs.msg import Frame
from geometry_msgs.msg import TwistStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32, String
from std_srvs.srv import Trigger

try:
    from terraspiro_sick_interface.msg import SickInfo
except ImportError:
    SickInfo = None


def parse_int(value: Any) -> int:
    if isinstance(value, int):
        return value
    return int(str(value), 0)


def parse_int_list(values: Any) -> List[int]:
    if isinstance(values, str):
        values = [item.strip() for item in values.split(",") if item.strip()]
    return [parse_int(value) for value in values]


def normalize_node_id(value: int) -> int:
    """Accept either node ID, e.g. 0x04, or SDO request COB-ID, e.g. 0x604."""
    if 0x600 <= value <= 0x67F:
        return value - 0x600
    return value


def parse_node_id_list(values: Any) -> List[int]:
    return [normalize_node_id(value) for value in parse_int_list(values)]


def hex_id(value: int) -> str:
    return f"0x{value:02X}"


def node_role(node_id: int, steering_node_ids: set, traction_node_ids: set) -> str:
    if node_id in steering_node_ids:
        return "steering"
    if node_id in traction_node_ids:
        return "traction"
    return "unknown"


TRACTION_NODE_LABELS = {0x04: "ftr", 0x08: "btr"}
STEERING_NODE_LABELS = {0x03: "fstr", 0x07: "bstr"}
DRIVE_NODE_LABELS = {
    **STEERING_NODE_LABELS,
    **TRACTION_NODE_LABELS,
}

BASE_CSV_FIELDNAMES = [
    "tmps",
    "stamp_sec",
    "elapsed_s",
    "ros_moving",
    "ros_operating_seconds",
    "ros_estop_operations",
]

VANGUARD_CSV_FIELDNAMES = [
    "vanguard_battery_soc_percent",
    "vanguard_battery_soc_fresh",
    "vanguard_battery_pack_voltage_v",
    "vanguard_battery_pack_current_a",
    "vanguard_battery_electrical_fresh",
    "vanguard_bms_charge_mode",
    "vanguard_bms_requested_current_a",
    "vanguard_bms_charge_fresh",
    "vanguard_charger_output_voltage_v",
    "vanguard_charger_output_current_a",
    "vanguard_charger_status",
    "vanguard_charger_ac_connected",
    "vanguard_charger_ac_voltage_v",
    "vanguard_charger_charging_active",
    "vanguard_charger_fault",
    "vanguard_charger_status_fresh",
    "vanguard_charger_voltage_limit_v",
    "vanguard_charger_current_limit_a",
    "vanguard_charger_power_rating_kw",
    "vanguard_charger_limits_fresh",
    "vanguard_eff3_requested_current_a",
    "vanguard_charge_rate_status",
]

VANGUARD_BMS_CHARGE_REQUEST_ID = 0x18FF10F3
VANGUARD_CHARGER_STATUS_ID = 0x0CFFBC49
VANGUARD_CHARGER_LIMITS_ID = 0x0CFFBD49
VANGUARD_DEFAULT_BATTERY_SOURCE_ADDRESS = 0xF3
VANGUARD_DEFAULT_BATTERY_SOC_PGN = 0xF091
VANGUARD_DEFAULT_BATTERY_ELECTRICAL_PGN = 0xF090


def little_endian_u16(low: int, high: int) -> int:
    return int(low) | (int(high) << 8)


def decode_offset_current(low: int, high: int) -> float:
    return little_endian_u16(low, high) * 0.05 - 1600.0


def decode_vanguard_frame(
    msg: Frame,
    battery_source_address: int = VANGUARD_DEFAULT_BATTERY_SOURCE_ADDRESS,
    battery_soc_pgn: int = VANGUARD_DEFAULT_BATTERY_SOC_PGN,
    battery_electrical_pgn: int = VANGUARD_DEFAULT_BATTERY_ELECTRICAL_PGN,
) -> Optional[Tuple[str, Dict[str, Any]]]:
    """Decode one supported Vanguard J1939 frame without transmitting anything."""
    if not msg.is_extended or msg.is_rtr or msg.is_error or msg.dlc != 8:
        return None

    # rclpy may expose uint8[] members as numpy.uint8 values. Convert every
    # byte at the decoder boundary so the resulting telemetry is JSON-safe.
    data = [int(value) for value in msg.data]
    if msg.id == VANGUARD_BMS_CHARGE_REQUEST_ID:
        return (
            "bms_charge",
            {
                "mode": data[0],
                "requested_current_a": decode_offset_current(data[1], data[2]),
            },
        )

    if msg.id == VANGUARD_CHARGER_STATUS_ID:
        output_current_a = decode_offset_current(data[2], data[3])
        status = data[4]
        return (
            "charger_status",
            {
                "output_voltage_v": little_endian_u16(data[0], data[1]) * 0.05,
                "output_current_a": output_current_a,
                "status": status,
                "ac_connected": data[5] == 1,
                "ac_voltage_v": little_endian_u16(data[6], data[7]) * 0.5,
                "charging_active": status in (3, 5) and output_current_a > 1.0,
                "fault": status == 254,
            },
        )

    if msg.id == VANGUARD_CHARGER_LIMITS_ID:
        return (
            "charger_limits",
            {
                "voltage_limit_v": little_endian_u16(data[0], data[1]) * 0.05,
                "current_limit_a": decode_offset_current(data[2], data[3]),
                "power_rating_kw": little_endian_u16(data[4], data[5]) * 0.05,
            },
        )

    source_address = msg.id & 0xFF
    pgn = (msg.id >> 8) & 0x3FFFF
    if source_address != battery_source_address:
        return None

    if pgn == battery_soc_pgn:
        raw_soc = little_endian_u16(data[0], data[1])
        if raw_soc == 0xFFFF:
            return None
        return "battery_soc", {"soc_percent": raw_soc * 0.001563}

    if pgn == battery_electrical_pgn:
        raw_voltage = little_endian_u16(data[4], data[5])
        raw_current = little_endian_u16(data[6], data[7])
        if raw_voltage == 0xFFFF or raw_current == 0xFFFF:
            return None
        return (
            "battery_electrical",
            {
                "pack_voltage_v": raw_voltage * 0.05,
                "pack_current_a": raw_current * 0.05 - 1600.0,
            },
        )

    return None


def build_telemetry_csv_fieldnames() -> List[str]:
    fields = list(BASE_CSV_FIELDNAMES)
    fields.extend(VANGUARD_CSV_FIELDNAMES)

    for label in STEERING_NODE_LABELS.values():
        fields.extend(
            [
                f"{label}_controller_temp_c",
                f"{label}_serial_number",
                f"{label}_status_feedback",
                f"{label}_enabled",
                f"{label}_error_feedback",
                f"{label}_can_estop_operations",
                f"{label}_bus_voltage_v",
            ]
        )

    for label in TRACTION_NODE_LABELS.values():
        fields.extend(
            [
                f"{label}_motor_temp_c",
                f"{label}_controller_temp_c",
                f"{label}_serial_number",
                f"{label}_status_feedback",
                f"{label}_mode_display",
                f"{label}_enabled",
                f"{label}_error_feedback",
                f"{label}_ec_error_code",
                f"{label}_ec_error_code_stamp_sec",
                f"{label}_last_nonzero_ec_error_code",
                f"{label}_last_nonzero_ec_error_code_stamp_sec",
                f"{label}_digital_inputs_raw",
                f"{label}_digital_inputs_stamp_sec",
                f"{label}_sto_status_raw",
                f"{label}_sto_status_stamp_sec",
                f"{label}_error_history_count",
                f"{label}_error_history_stamp_sec",
                f"{label}_error_history_raw",
                f"{label}_can_estop_operations",
                f"{label}_bus_voltage_v",
            ]
        )

    for label in TRACTION_NODE_LABELS.values():
        fields.extend(
            [
                f"{label}_velocity_raw",
                f"{label}_velocity_mm_s",
                f"{label}_acc_raw",
                f"{label}_acc_mm_s2",
                f"{label}_dec_raw",
                f"{label}_dec_mm_s2",
                f"{label}_current_actual_permille",
                f"{label}_current_actual_ma",
                f"{label}_rms_cur",
            ]
        )

    for label in STEERING_NODE_LABELS.values():
        fields.extend(
            [
                f"{label}_position_ticks",
                f"{label}_angle_deg",
                f"{label}_velocity_deg_s",
                f"{label}_current_actual_ma",
                f"{label}_rms_cur",
            ]
        )

    return fields


TELEMETRY_CSV_FIELDNAMES = build_telemetry_csv_fieldnames()


def csv_number(value: Any, digits: Optional[int] = None) -> str:
    if value is None:
        return ""
    if digits is None:
        return str(value)
    return f"{float(value):.{digits}f}"


def json_record_value(value: str) -> Any:
    if value == "":
        return None
    if value == "True":
        return True
    if value == "False":
        return False
    try:
        if any(marker in value for marker in (".", "e", "E")):
            return float(value)
        return int(value)
    except ValueError:
        return value


def json_numeric_scalar(value: Any) -> Any:
    """Convert NumPy-style scalar values while rejecting unsupported objects."""
    item = getattr(value, "item", None)
    if callable(item):
        return item()
    raise TypeError(f"Object of type {value.__class__.__name__} is not JSON serializable")


@dataclass(frozen=True)
class SdoQuery:
    name: str
    index: int
    subindex: int
    signed: bool = True
    scale: float = 1.0
    unit: str = ""
    group: str = "all"
    value_type: str = "int"
    poll_class: str = "fast"


@dataclass
class PendingRequest:
    node_id: int
    query: SdoQuery
    sent_time_ns: int
    attempts: int = 1


class CanRosbagRecorder(Node):
    def __init__(self) -> None:
        super().__init__("can_rosbag_recorder")

        self.declare_parameter("node_ids", ["0x03", "0x07", "0x04", "0x08"])
        self.declare_parameter("steering_node_ids", ["0x03", "0x07"])
        self.declare_parameter("traction_node_ids", ["0x04", "0x08"])

        self.declare_parameter("can_tx_topic", "/mobotic/can_tx")
        self.declare_parameter("can_rx_topic", "/mobotic/can_rx")
        self.declare_parameter("battery_can_rx_topic", "/mobotic/battery_can/can_rx")
        self.declare_parameter("output_topic", "can_telemetry")
        self.declare_parameter("cmd_vel_topic", "/mobotic/cmd_vel")
        self.declare_parameter("joint_states_topic", "/mobotic/joint_states")
        self.declare_parameter("sick_info_topic", "sick_info")
        self.declare_parameter(
            "charge_rate_requested_topic", "/mobotic/charge_rate/requested_current"
        )
        self.declare_parameter("charge_rate_status_topic", "/mobotic/charge_rate/status")

        self.declare_parameter("decode_traction_tpdo", True)
        self.declare_parameter("decode_telemetry_tpdo", True)
        self.declare_parameter("request_hz", 50.0)
        self.declare_parameter("sdo_poll_hz", 1.0)
        self.declare_parameter("sdo_slow_poll_hz", 0.1)
        self.declare_parameter("sdo_spacing_sec", 0.005)
        self.declare_parameter("sdo_timeout_sec", 0.05)
        self.declare_parameter("max_retries", 0)
        self.declare_parameter("motion_timeout_sec", 0.5)
        self.declare_parameter("cmd_vel_motion_threshold", 0.01)
        self.declare_parameter("joint_velocity_motion_threshold", 0.01)

        self.declare_parameter("steering_bus_voltage_scale", 0.001)
        self.declare_parameter("steering_gear_ratio", 121.0)
        self.declare_parameter("traction_bus_voltage_index", "0x6079")
        self.declare_parameter("traction_bus_voltage_subindex", "0x00")
        self.declare_parameter("traction_bus_voltage_scale", 0.001)
        self.declare_parameter("traction_current_scale_ma_per_permille", 60.0)
        self.declare_parameter("traction_rms_window_samples", 50)
        self.declare_parameter("traction_accel_index", "0x6083")
        self.declare_parameter("traction_accel_subindex", "0x00")
        self.declare_parameter("traction_decel_index", "0x6084")
        self.declare_parameter("traction_decel_subindex", "0x00")
        self.declare_parameter("traction_motor_temperature_index", "0x22A4")
        self.declare_parameter("traction_motor_temperature_subindex", "0x00")
        self.declare_parameter("traction_wheel_diameter_mm", 225.0)
        self.declare_parameter("traction_gear_ratio", 16.0)
        self.declare_parameter("traction_velocity_resolution", 4096.0)
        self.declare_parameter("ec_serial_number_index", "0x31E1")
        self.declare_parameter("ec_serial_number_subindex", "0x04")
        self.declare_parameter("ec_digital_inputs_index", "0x60FD")
        self.declare_parameter("ec_digital_inputs_subindex", "0x00")
        self.declare_parameter("ec_sto_status_index", "0x2086")
        self.declare_parameter("ec_sto_status_subindex", "0x00")
        self.declare_parameter("ec_error_history_index", "0x1003")
        self.declare_parameter("ec_error_history_max_entries", 16)
        self.declare_parameter("steering_position_resolution", 4096.0)
        self.declare_parameter("recording_dir", "can_telemetry_records")
        self.declare_parameter("recording_prefix", "can_telemetry")
        self.declare_parameter("recording_format", "json")
        self.declare_parameter("record_on_start", False)
        self.declare_parameter("sdo_warning_period_sec", 2.0)
        self.declare_parameter("sdo_warning_startup_grace_sec", 2.0)
        self.declare_parameter("csv_path", "")
        self.declare_parameter("vanguard_stale_timeout_sec", 0.75)
        self.declare_parameter("vanguard_battery_source_address", "0xF3")
        self.declare_parameter("vanguard_battery_soc_pgn", "0xF091")
        self.declare_parameter("vanguard_battery_electrical_pgn", "0xF090")

        self.node_ids = parse_node_id_list(self.get_parameter("node_ids").value)
        self.steering_node_ids = set(
            parse_node_id_list(self.get_parameter("steering_node_ids").value)
        )
        self.traction_node_ids = set(
            parse_node_id_list(self.get_parameter("traction_node_ids").value)
        )
        self.vanguard_battery_source_address = parse_int(
            self.get_parameter("vanguard_battery_source_address").value
        )
        self.vanguard_battery_soc_pgn = parse_int(
            self.get_parameter("vanguard_battery_soc_pgn").value
        )
        self.vanguard_battery_electrical_pgn = parse_int(
            self.get_parameter("vanguard_battery_electrical_pgn").value
        )
        self.vanguard_latest: Dict[str, Dict[str, Any]] = {}
        self.vanguard_eff3_requested_current_a: Optional[float] = None
        self.vanguard_charge_rate_status = ""

        self.pub_can = self.create_publisher(Frame, self.get_parameter("can_tx_topic").value, 10)
        self.pub_telemetry = self.create_publisher(String, self.get_parameter("output_topic").value, 10)
        self.create_subscription(Frame, self.get_parameter("can_rx_topic").value, self.can_rx_callback, 100)
        self.create_subscription(
            Frame,
            self.get_parameter("battery_can_rx_topic").value,
            self.can_rx_callback,
            100,
        )
        self.create_subscription(
            TwistStamped,
            self.get_parameter("cmd_vel_topic").value,
            self.cmd_vel_callback,
            10,
        )
        self.create_subscription(
            JointState,
            self.get_parameter("joint_states_topic").value,
            self.joint_state_callback,
            10,
        )
        self.create_subscription(
            Float32,
            self.get_parameter("charge_rate_requested_topic").value,
            self.charge_rate_requested_callback,
            QoSProfile(
                depth=1,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
            ),
        )
        self.create_subscription(
            String,
            self.get_parameter("charge_rate_status_topic").value,
            self.charge_rate_status_callback,
            QoSProfile(
                depth=1,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
            ),
        )

        self._sick_subscription = None
        if SickInfo is not None:
            self._sick_subscription = self.create_subscription(
                SickInfo,
                self.get_parameter("sick_info_topic").value,
                self.sick_info_callback,
                10,
            )
        else:
            self.get_logger().warn(
                "terraspiro_sick_interface is not available; ROS estop counter will stay at 0."
            )

        self.queries = self.build_queries()
        self.query_map = {(query.index, query.subindex): query for query in self.queries}
        self.queues: Dict[int, List[PendingRequest]] = {
            node_id: [] for node_id in self.node_ids
        }
        self.pending: Dict[int, PendingRequest] = {}
        self.latest: Dict[int, Dict[str, Any]] = {node_id: {} for node_id in self.node_ids}
        rms_window_samples = int(self.get_parameter("traction_rms_window_samples").value)
        if rms_window_samples <= 0:
            raise ValueError("traction_rms_window_samples must be positive")
        self._traction_current_samples: Dict[int, Deque[float]] = {
            node_id: deque(maxlen=rms_window_samples)
            for node_id in self.traction_node_ids
        }

        self.ros_operating_seconds = 0.0
        self.ros_estop_operations = 0
        self._last_ros_time = self.get_clock().now()
        self._start_time_ns = self._last_ros_time.nanoseconds
        self._last_cmd_vel_time = None
        self._last_joint_time = None
        self._last_cmd_vel_moving = False
        self._last_joint_moving = False
        self._moving = False
        self._last_estop_active = False
        self._estop_field_warning_sent = False
        self._last_steering_samples: Dict[int, Tuple[int, float]] = {}
        self._last_sdo_warning_ns: Dict[Tuple[int, str], int] = {}
        self.drive_node_labels = DRIVE_NODE_LABELS
        self.traction_node_labels = TRACTION_NODE_LABELS
        self.steering_node_labels = STEERING_NODE_LABELS

        self.recording_file = None
        self.recording_path: Optional[Path] = None
        self.recording_start_stamp_sec: Optional[float] = None
        self.json_record_count = 0
        self.recording_format = self.normalize_recording_format(
            str(self.get_parameter("recording_format").value)
        )
        self.csv_path_override = str(self.get_parameter("csv_path").value)
        self.csv_writer = None
        if self.csv_path_override:
            self.recording_format = "csv"

        self.create_service(Trigger, "~/start_recording", self.start_recording_callback)
        self.create_service(Trigger, "~/stop_recording", self.stop_recording_callback)

        record_hz = float(self.get_parameter("request_hz").value)
        sdo_poll_hz = float(self.get_parameter("sdo_poll_hz").value)
        sdo_slow_poll_hz = float(self.get_parameter("sdo_slow_poll_hz").value)
        if record_hz <= 0.0 or sdo_poll_hz <= 0.0 or sdo_slow_poll_hz <= 0.0:
            raise ValueError(
                "request_hz, sdo_poll_hz, and sdo_slow_poll_hz must be positive"
            )

        record_period = 1.0 / record_hz
        sdo_poll_period = 1.0 / sdo_poll_hz
        sdo_slow_poll_period = 1.0 / sdo_slow_poll_hz
        self.create_timer(sdo_poll_period, self.enqueue_poll_cycle)
        self.create_timer(sdo_slow_poll_period, self.enqueue_slow_poll_cycle)
        self.create_timer(float(self.get_parameter("sdo_spacing_sec").value), self.pump_sdo)
        self.create_timer(record_period, self.publish_snapshot)

        if bool(self.get_parameter("record_on_start").value) or self.csv_path_override:
            self.start_recording()

        self.get_logger().info(
            "CAN rosbag recorder is running for nodes "
            f"{[hex_id(node_id) for node_id in self.node_ids]}; "
            f"file={record_hz:g} Hz, fast SDO={sdo_poll_hz:g} Hz, "
            f"slow SDO={sdo_slow_poll_hz:g} Hz."
        )

    def build_queries(self) -> List[SdoQuery]:
        queries = [
            SdoQuery(
                "error_feedback",
                0x3001,
                0x00,
                signed=True,
                group="steering",
                poll_class="slow",
            ),
            SdoQuery(
                "error_feedback",
                0x1001,
                0x00,
                signed=False,
                group="traction",
                poll_class="passive",
            ),
            SdoQuery(
                "ec_error_code",
                0x603F,
                0x00,
                signed=False,
                group="traction",
                poll_class="fast",
            ),
            SdoQuery(
                "status_feedback",
                0x3004,
                0x00,
                signed=False,
                group="steering",
                poll_class="passive",
            ),
        ]

        ec_serial_number_index = parse_int(self.get_parameter("ec_serial_number_index").value)
        if ec_serial_number_index:
            queries.append(
                SdoQuery(
                    "serial_number",
                    ec_serial_number_index,
                    parse_int(self.get_parameter("ec_serial_number_subindex").value),
                    signed=False,
                    group="traction",
                    poll_class="slow",
                )
            )

        ec_digital_inputs_index = parse_int(
            self.get_parameter("ec_digital_inputs_index").value
        )
        if ec_digital_inputs_index:
            queries.append(
                SdoQuery(
                    "digital_inputs_raw",
                    ec_digital_inputs_index,
                    parse_int(
                        self.get_parameter("ec_digital_inputs_subindex").value
                    ),
                    signed=False,
                    group="traction",
                    poll_class="slow",
                )
            )

        ec_sto_status_index = parse_int(
            self.get_parameter("ec_sto_status_index").value
        )
        if ec_sto_status_index:
            queries.append(
                SdoQuery(
                    "sto_status_raw",
                    ec_sto_status_index,
                    parse_int(
                        self.get_parameter("ec_sto_status_subindex").value
                    ),
                    signed=False,
                    group="traction",
                    poll_class="slow",
                )
            )

        ec_error_history_index = parse_int(
            self.get_parameter("ec_error_history_index").value
        )
        error_history_max_entries = int(
            self.get_parameter("ec_error_history_max_entries").value
        )
        if not 0 <= error_history_max_entries <= 0xFE:
            raise ValueError("ec_error_history_max_entries must be in [0, 254]")
        self.error_history_max_entries = error_history_max_entries
        if ec_error_history_index:
            queries.append(
                SdoQuery(
                    "error_history_count",
                    ec_error_history_index,
                    0x00,
                    signed=False,
                    group="traction",
                    poll_class="slow",
                )
            )
            for entry in range(1, error_history_max_entries + 1):
                queries.append(
                    SdoQuery(
                        f"error_history_{entry:02d}_raw",
                        ec_error_history_index,
                        entry,
                        signed=False,
                        group="traction",
                        poll_class="slow",
                    )
                )

        traction_accel_index = parse_int(self.get_parameter("traction_accel_index").value)
        if traction_accel_index:
            queries.append(
                SdoQuery(
                    "traction_accel_profile_raw",
                    traction_accel_index,
                    parse_int(self.get_parameter("traction_accel_subindex").value),
                    signed=False,
                    group="traction",
                    poll_class="slow",
                )
            )

        traction_decel_index = parse_int(self.get_parameter("traction_decel_index").value)
        if traction_decel_index:
            queries.append(
                SdoQuery(
                    "traction_decel_profile_raw",
                    traction_decel_index,
                    parse_int(self.get_parameter("traction_decel_subindex").value),
                    signed=False,
                    group="traction",
                    poll_class="slow",
                )
            )

        traction_motor_temperature_index = parse_int(
            self.get_parameter("traction_motor_temperature_index").value
        )
        if traction_motor_temperature_index:
            queries.append(
                SdoQuery(
                    "motor_temperature_c",
                    traction_motor_temperature_index,
                    parse_int(
                        self.get_parameter(
                            "traction_motor_temperature_subindex"
                        ).value
                    ),
                    signed=False,
                    unit="degC",
                    group="traction",
                    poll_class="slow",
                )
            )

        traction_bus_voltage_index = parse_int(
            self.get_parameter("traction_bus_voltage_index").value
        )
        if traction_bus_voltage_index:
            queries.append(
                SdoQuery(
                    "bus_voltage_v",
                    traction_bus_voltage_index,
                    parse_int(self.get_parameter("traction_bus_voltage_subindex").value),
                    signed=False,
                    scale=float(self.get_parameter("traction_bus_voltage_scale").value),
                    unit="V",
                    group="traction",
                )
            )

        return queries

    def enqueue_poll_cycle(self) -> None:
        self.enqueue_sdo_class("fast")

    def enqueue_slow_poll_cycle(self) -> None:
        self.enqueue_sdo_class("slow")

    def enqueue_sdo_class(self, poll_class: str) -> None:
        for node_id in self.node_ids:
            for query in self.queries:
                if query.poll_class != poll_class:
                    continue
                if query.group == "traction" and node_id not in self.traction_node_ids:
                    continue
                if query.group == "steering" and node_id not in self.steering_node_ids:
                    continue

                pending = self.pending.get(node_id)
                if pending is not None and pending.query == query:
                    continue
                if any(request.query == query for request in self.queues[node_id]):
                    continue

                self.queues[node_id].append(
                    PendingRequest(node_id=node_id, query=query, sent_time_ns=0)
                )

    def pump_sdo(self) -> None:
        now_ns = self.get_clock().now().nanoseconds
        timeout_ns = int(float(self.get_parameter("sdo_timeout_sec").value) * 1e9)
        max_retries = int(self.get_parameter("max_retries").value)

        for node_id in self.node_ids:
            pending = self.pending.get(node_id)
            if pending is not None:
                if now_ns - pending.sent_time_ns < timeout_ns:
                    continue

                if pending.attempts <= max_retries:
                    pending.attempts += 1
                    self.send_sdo_upload(pending)
                    continue

                self.warn_missing_sdo(pending, now_ns)
                self.pending.pop(node_id, None)

            queue = self.queues[node_id]
            if not queue:
                continue

            request = queue.pop(0)
            self.pending[node_id] = request
            self.send_sdo_upload(request)

    def send_sdo_upload(self, request: PendingRequest) -> None:
        query = request.query
        msg = Frame()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "can"
        msg.id = 0x600 + request.node_id
        msg.is_rtr = False
        msg.is_extended = False
        msg.is_error = False
        msg.dlc = 8
        msg.data = [
            0x40,
            query.index & 0xFF,
            (query.index >> 8) & 0xFF,
            query.subindex & 0xFF,
            0x00,
            0x00,
            0x00,
            0x00,
        ]
        request.sent_time_ns = self.get_clock().now().nanoseconds
        self.pub_can.publish(msg)

    def can_rx_callback(self, msg: Frame) -> None:
        decoded_vanguard = decode_vanguard_frame(
            msg,
            self.vanguard_battery_source_address,
            self.vanguard_battery_soc_pgn,
            self.vanguard_battery_electrical_pgn,
        )
        if decoded_vanguard is not None:
            group, values = decoded_vanguard
            values["stamp_sec"] = self.now_seconds()
            self.vanguard_latest[group] = values

        if (
            self.decode_traction_tpdo(msg)
            or self.decode_steering_motion_tpdo(msg)
            or self.decode_telemetry_tpdo(msg)
        ):
            return

        node_id = msg.id - 0x580
        if node_id not in self.node_ids or msg.dlc < 4:
            return

        data = list(msg.data)
        index = data[1] | (data[2] << 8)
        subindex = data[3]
        query = self.query_map.get((index, subindex))
        if query is None:
            return

        if data[0] == 0x80:
            abort_code = int.from_bytes(bytes(data[4:8]), byteorder="little", signed=False)
            self.get_logger().warn(
                f"SDO abort from {hex_id(node_id)} for {query.name}: 0x{abort_code:08X}"
            )
            self.clear_matching_pending(node_id, query)
            return

        value = self.parse_sdo_value(data, query)
        if value is None:
            return

        stamp_sec = self.now_seconds()
        node_data = self.latest.setdefault(node_id, {})
        node_data[query.name] = value
        node_data[f"{query.name}_stamp_sec"] = stamp_sec
        if query.name == "ec_error_code" and int(value) != 0:
            node_data["last_nonzero_ec_error_code"] = value
            node_data["last_nonzero_ec_error_code_stamp_sec"] = stamp_sec
        if query.name.startswith("error_history_"):
            node_data["error_history_stamp_sec"] = stamp_sec

        pending = self.pending.get(node_id)
        if pending is not None and pending.query == query:
            self.pending.pop(node_id, None)

    def decode_traction_tpdo(self, msg: Frame) -> bool:
        if not bool(self.get_parameter("decode_traction_tpdo").value):
            return False

        node_id = msg.id - 0x180
        if node_id not in self.traction_node_ids or msg.dlc < 7:
            return False

        data = list(msg.data)
        stamp_sec = self.now_seconds()
        statusword = int.from_bytes(bytes(data[0:2]), byteorder="little", signed=False)
        mode_display = int.from_bytes(bytes(data[2:3]), byteorder="little", signed=True)
        velocity_raw = int.from_bytes(bytes(data[3:7]), byteorder="little", signed=True)

        node_data = self.latest.setdefault(node_id, {})
        node_data["status_feedback"] = statusword
        node_data["status_feedback_stamp_sec"] = stamp_sec
        node_data["enabled"] = (statusword & 0x006F) == 0x0027
        node_data["enabled_stamp_sec"] = stamp_sec
        node_data["traction_mode_display"] = mode_display
        node_data["traction_mode_display_stamp_sec"] = stamp_sec
        node_data["traction_velocity_raw"] = velocity_raw
        node_data["traction_velocity_raw_stamp_sec"] = stamp_sec
        return True

    def decode_steering_motion_tpdo(self, msg: Frame) -> bool:
        if not bool(self.get_parameter("decode_telemetry_tpdo").value):
            return False

        node_id = msg.id - 0x180
        if node_id not in self.steering_node_ids or msg.dlc < 8:
            return False

        data = bytes(msg.data)
        stamp_sec = self.now_seconds()
        position_ticks = int.from_bytes(
            data[0:4], byteorder="little", signed=True
        )
        velocity_raw = int.from_bytes(
            data[4:8], byteorder="little", signed=True
        )

        node_data = self.latest.setdefault(node_id, {})
        node_data["steering_position_ticks"] = position_ticks
        node_data["steering_position_ticks_stamp_sec"] = stamp_sec
        self.update_steering_feedback(
            node_id,
            position_ticks,
            velocity_raw=velocity_raw,
            stamp_sec=stamp_sec,
        )
        return True

    def decode_telemetry_tpdo(self, msg: Frame) -> bool:
        if not bool(self.get_parameter("decode_telemetry_tpdo").value):
            return False

        stamp_sec = self.now_seconds()
        tpdo2_node_id = msg.id - 0x280

        if tpdo2_node_id in self.traction_node_ids and msg.dlc >= 4:
            data = bytes(msg.data)
            node_data = self.latest.setdefault(tpdo2_node_id, {})
            current_permille = int.from_bytes(
                data[0:2], byteorder="little", signed=True
            )
            current_ma = (
                current_permille
                * float(
                    self.get_parameter(
                        "traction_current_scale_ma_per_permille"
                    ).value
                )
            )
            current_samples = self._traction_current_samples[tpdo2_node_id]
            current_samples.append(current_ma)
            rms_current_ma = math.sqrt(
                sum(sample * sample for sample in current_samples)
                / len(current_samples)
            )

            node_data["traction_current_actual_permille"] = current_permille
            node_data["traction_current_actual_ma"] = current_ma
            node_data["traction_rms_current"] = rms_current_ma
            node_data["controller_temperature_c"] = int.from_bytes(
                data[2:4], byteorder="little", signed=False
            )
            node_data["traction_current_actual_permille_stamp_sec"] = stamp_sec
            node_data["traction_current_actual_ma_stamp_sec"] = stamp_sec
            node_data["traction_rms_current_stamp_sec"] = stamp_sec
            node_data["controller_temperature_c_stamp_sec"] = stamp_sec
            return True

        if tpdo2_node_id in self.steering_node_ids and msg.dlc >= 8:
            data = bytes(msg.data)
            node_data = self.latest.setdefault(tpdo2_node_id, {})
            voltage_scale = float(
                self.get_parameter("steering_bus_voltage_scale").value
            )
            current_actual_ma = int.from_bytes(
                data[4:8], byteorder="little", signed=True
            )
            node_data["bus_voltage_v"] = (
                int.from_bytes(data[0:4], byteorder="little", signed=False)
                * voltage_scale
            )
            node_data["steering_current_actual_ma"] = current_actual_ma
            node_data["bus_voltage_v_stamp_sec"] = stamp_sec
            node_data["steering_current_actual_ma_stamp_sec"] = stamp_sec
            return True

        tpdo3_node_id = msg.id - 0x380
        if tpdo3_node_id in self.steering_node_ids and msg.dlc >= 4:
            data = bytes(msg.data)
            node_data = self.latest.setdefault(tpdo3_node_id, {})
            node_data["controller_temperature_c"] = (
                int.from_bytes(data[0:4], byteorder="little", signed=True) * 0.1
            )
            node_data["controller_temperature_c_stamp_sec"] = stamp_sec
            if msg.dlc >= 8:
                node_data["steering_rms_current"] = int.from_bytes(
                    data[4:8], byteorder="little", signed=True
                )
                node_data["steering_rms_current_stamp_sec"] = stamp_sec
            return True

        return False

    def warn_missing_sdo(self, request: PendingRequest, now_ns: int) -> None:
        startup_grace_ns = int(float(self.get_parameter("sdo_warning_startup_grace_sec").value) * 1e9)
        if now_ns - self._start_time_ns < startup_grace_ns:
            return

        warning_period_ns = int(float(self.get_parameter("sdo_warning_period_sec").value) * 1e9)
        key = (request.node_id, request.query.name)
        last_warning_ns = self._last_sdo_warning_ns.get(key)
        if last_warning_ns is not None and now_ns - last_warning_ns < warning_period_ns:
            return

        self._last_sdo_warning_ns[key] = now_ns
        self.get_logger().warn(
            "No SDO reply from "
            f"{hex_id(request.node_id)} for "
            f"{request.query.name} "
            f"0x{request.query.index:04X}:{request.query.subindex:02X}"
        )

    def clear_matching_pending(self, node_id: int, query: SdoQuery) -> None:
        pending = self.pending.get(node_id)
        if pending is not None and pending.query == query:
            self.pending.pop(node_id, None)

    def parse_sdo_value(self, data: List[int], query: SdoQuery) -> Optional[float]:
        command = data[0]
        length_by_command = {0x43: 4, 0x4B: 2, 0x4F: 1}
        length = length_by_command.get(command)

        if length is None:
            # Expedited upload response with size indicated.
            if (command & 0xE0) == 0x40 and (command & 0x02):
                length = 4 - ((command >> 2) & 0x03)
            else:
                self.get_logger().debug(
                    f"Unsupported SDO upload response command byte: 0x{command:02X}"
                )
                return None

        raw_bytes = bytes(data[4: 4 + length])
        if query.value_type == "float32":
            if length != 4:
                self.get_logger().debug(
                    f"Unsupported float32 SDO response length for {query.name}: {length}"
                )
                return None
            scaled = struct.unpack("<f", raw_bytes)[0] * query.scale
            return float(scaled)

        raw = int.from_bytes(raw_bytes, byteorder="little", signed=query.signed)
        scaled = raw * query.scale

        if query.scale == 1.0:
            return int(raw)
        return float(scaled)

    def steering_angle_deg(self, position_ticks: Any) -> Optional[float]:
        if position_ticks is None:
            return None

        resolution = float(self.get_parameter("steering_position_resolution").value)
        return float(position_ticks) * 360.0 / resolution

    def update_steering_feedback(
        self,
        node_id: int,
        position_ticks: int,
        velocity_raw: Optional[int] = None,
        stamp_sec: Optional[float] = None,
    ) -> None:
        if stamp_sec is None:
            stamp_sec = self.now_seconds()
        node_data = self.latest.setdefault(node_id, {})

        angle_deg = self.steering_angle_deg(position_ticks)
        if angle_deg is not None:
            node_data["steering_angle_deg"] = angle_deg
            node_data["steering_angle_deg_stamp_sec"] = stamp_sec

        if velocity_raw is not None:
            node_data["steering_velocity_raw"] = velocity_raw
            node_data["steering_velocity_raw_stamp_sec"] = stamp_sec
            gear_ratio = float(self.get_parameter("steering_gear_ratio").value)
            if gear_ratio != 0.0:
                # MiControl 0x3A04:01 is motor rpm. Convert to output deg/s.
                node_data["steering_velocity_deg_s"] = (
                    float(velocity_raw) * 6.0 / gear_ratio
                )
                node_data["steering_velocity_deg_s_stamp_sec"] = stamp_sec

        # Keep a differentiated-position fallback for recordings made without
        # the steering velocity member in TPDO1.
        if velocity_raw is None:
            previous = self._last_steering_samples.get(node_id)
            if previous is not None:
                previous_position_ticks, previous_stamp_sec = previous
                elapsed_sec = stamp_sec - previous_stamp_sec
                if elapsed_sec > 0.0:
                    delta_angle_deg = self.steering_angle_deg(
                        position_ticks - previous_position_ticks
                    )
                    if delta_angle_deg is not None:
                        node_data["steering_velocity_deg_s"] = (
                            delta_angle_deg / elapsed_sec
                        )
                        node_data["steering_velocity_deg_s_stamp_sec"] = stamp_sec

        self._last_steering_samples[node_id] = (position_ticks, stamp_sec)

    def traction_raw_to_mm(self, raw_value: Any) -> Optional[float]:
        if raw_value is None:
            return None

        wheel_diameter_mm = float(self.get_parameter("traction_wheel_diameter_mm").value)
        gear_ratio = float(self.get_parameter("traction_gear_ratio").value)
        resolution = float(self.get_parameter("traction_velocity_resolution").value)
        if gear_ratio == 0.0 or resolution == 0.0:
            return None

        return float(raw_value) * math.pi * wheel_diameter_mm / (resolution * gear_ratio)

    def update_traction_conversions(self, node_data: Dict[str, Any]) -> None:
        conversions = (
            ("traction_velocity_raw", "traction_velocity_mm_s"),
            ("traction_accel_profile_raw", "traction_accel_profile_mm_s2"),
            ("traction_decel_profile_raw", "traction_decel_profile_mm_s2"),
        )
        for raw_key, converted_key in conversions:
            converted = self.traction_raw_to_mm(node_data.get(raw_key))
            if converted is not None:
                node_data[converted_key] = converted

        current_ma = self.traction_current_actual_ma(node_data)
        if current_ma is not None:
            node_data["traction_current_actual_ma"] = current_ma

    def traction_current_actual_ma(self, node_data: Dict[str, Any]) -> Optional[float]:
        current_permille = node_data.get("traction_current_actual_permille")
        if current_permille is None:
            return None

        return float(current_permille) * float(
            self.get_parameter("traction_current_scale_ma_per_permille").value
        )

    def cmd_vel_callback(self, msg: TwistStamped) -> None:
        twist = msg.twist
        threshold = float(self.get_parameter("cmd_vel_motion_threshold").value)
        self._last_cmd_vel_moving = (
            abs(twist.linear.x) > threshold
            or abs(twist.linear.y) > threshold
            or abs(twist.angular.z) > threshold
        )
        self._last_cmd_vel_time = self.get_clock().now()

    def joint_state_callback(self, msg: JointState) -> None:
        threshold = float(self.get_parameter("joint_velocity_motion_threshold").value)
        self._last_joint_moving = any(abs(velocity) > threshold for velocity in msg.velocity)
        self._last_joint_time = self.get_clock().now()

    def sick_info_callback(self, msg: Any) -> None:
        if not hasattr(msg, "estop_active"):
            if not self._estop_field_warning_sent:
                self.get_logger().warn("SickInfo message has no estop_active field.")
                self._estop_field_warning_sent = True
            return

        estop_active = bool(msg.estop_active)
        if estop_active and not self._last_estop_active:
            self.ros_estop_operations += 1
        self._last_estop_active = estop_active

    def charge_rate_requested_callback(self, msg: Float32) -> None:
        self.vanguard_eff3_requested_current_a = float(msg.data)

    def charge_rate_status_callback(self, msg: String) -> None:
        self.vanguard_charge_rate_status = msg.data

    def build_vanguard_snapshot(self, stamp_sec: float) -> Dict[str, Any]:
        stale_timeout = float(self.get_parameter("vanguard_stale_timeout_sec").value)

        def group(name: str) -> Tuple[Dict[str, Any], bool]:
            values = self.vanguard_latest.get(name, {})
            value_stamp = values.get("stamp_sec")
            fresh = (
                value_stamp is not None
                and stamp_sec - float(value_stamp) <= stale_timeout
            )
            return values, fresh

        battery_soc, battery_soc_fresh = group("battery_soc")
        battery_electrical, battery_electrical_fresh = group(
            "battery_electrical"
        )
        bms_charge, bms_charge_fresh = group("bms_charge")
        charger_status, charger_status_fresh = group("charger_status")
        charger_limits, charger_limits_fresh = group("charger_limits")
        return {
            "battery_soc": battery_soc,
            "battery_soc_fresh": battery_soc_fresh,
            "battery_electrical": battery_electrical,
            "battery_electrical_fresh": battery_electrical_fresh,
            "bms_charge": bms_charge,
            "bms_charge_fresh": bms_charge_fresh,
            "charger_status": charger_status,
            "charger_status_fresh": charger_status_fresh,
            "charger_limits": charger_limits,
            "charger_limits_fresh": charger_limits_fresh,
            "eff3_requested_current_a": self.vanguard_eff3_requested_current_a,
            "charge_rate_status": self.vanguard_charge_rate_status,
        }

    def update_ros_operating_time(self) -> None:
        now = self.get_clock().now()
        elapsed_sec = (now - self._last_ros_time).nanoseconds / 1e9
        self._last_ros_time = now

        self._moving = self.is_ros_motion_active(now)
        if self._moving:
            self.ros_operating_seconds += max(0.0, elapsed_sec)

    def is_ros_motion_active(self, now: Any) -> bool:
        timeout = float(self.get_parameter("motion_timeout_sec").value)

        if self._last_joint_time is not None:
            joint_age = (now - self._last_joint_time).nanoseconds / 1e9
            if joint_age <= timeout and self._last_joint_moving:
                return True

        if self._last_cmd_vel_time is not None:
            cmd_age = (now - self._last_cmd_vel_time).nanoseconds / 1e9
            if cmd_age <= timeout and self._last_cmd_vel_moving:
                return True

        return False

    def publish_snapshot(self) -> None:
        self.update_ros_operating_time()

        nodes: Dict[str, Dict[str, Any]] = {}
        for node_id in self.node_ids:
            node_data = dict(self.latest.get(node_id, {}))
            role = node_role(node_id, self.steering_node_ids, self.traction_node_ids)

            node_data["role"] = role
            node_data["sdo_request_can_id"] = hex_id(0x600 + node_id)
            node_data["sdo_response_can_id"] = hex_id(0x580 + node_id)

            status = node_data.get("status_feedback")
            if status is not None:
                if role == "traction":
                    status_int = int(status)
                    node_data["enabled"] = (status_int & 0x006F) == 0x0027
                else:
                    node_data["enabled"] = bool(status)

            if role == "traction":
                self.update_traction_conversions(node_data)
                history_count = int(node_data.get("error_history_count") or 0)
                node_data["error_history_raw"] = [
                    node_data.get(f"error_history_{entry:02d}_raw")
                    for entry in range(
                        1,
                        min(history_count, self.error_history_max_entries) + 1,
                    )
                ]
                for entry in range(1, self.error_history_max_entries + 1):
                    node_data.pop(f"error_history_{entry:02d}_raw", None)
                    node_data.pop(
                        f"error_history_{entry:02d}_raw_stamp_sec",
                        None,
                    )

            if role == "steering" and "steering_position_ticks" in node_data:
                node_data["steering_angle_deg"] = self.steering_angle_deg(
                    node_data["steering_position_ticks"]
                )

            nodes[hex_id(node_id)] = node_data

        stamp_sec = self.now_seconds()
        payload = {
            "stamp_sec": stamp_sec,
            "ros": {
                "operating_seconds": self.ros_operating_seconds,
                "estop_operations": self.ros_estop_operations,
                "moving": self._moving,
            },
            "nodes": nodes,
            "vanguard": self.build_vanguard_snapshot(stamp_sec),
        }

        msg = String()
        msg.data = json.dumps(
            payload,
            sort_keys=True,
            separators=(",", ":"),
            default=json_numeric_scalar,
        )
        self.pub_telemetry.publish(msg)

        self.record_snapshot(payload, msg.data)

    def normalize_recording_format(self, value: str) -> str:
        normalized = value.strip().lower()
        if normalized not in {"json", "jsonl", "csv"}:
            self.get_logger().warn(
                f"Unsupported recording_format '{value}'. Falling back to json."
            )
            return "json"
        return normalized

    def build_recording_path(self) -> Path:
        if self.csv_path_override:
            return Path(self.csv_path_override).expanduser()

        output_dir = Path(str(self.get_parameter("recording_dir").value)).expanduser()
        prefix = str(self.get_parameter("recording_prefix").value).strip() or "can_telemetry"
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        suffix = f".{self.recording_format}"
        path = output_dir / f"{prefix}_{timestamp}{suffix}"

        counter = 1
        while path.exists():
            path = output_dir / f"{prefix}_{timestamp}_{counter:02d}{suffix}"
            counter += 1
        return path

    def start_recording_callback(self, _request: Any, response: Any) -> Any:
        try:
            path = self.start_recording()
        except OSError as exc:
            response.success = False
            response.message = f"Failed to start recording: {exc}"
            return response

        response.success = True
        response.message = f"Recording to {path}"
        return response

    def stop_recording_callback(self, _request: Any, response: Any) -> Any:
        path = self.stop_recording()
        response.success = True
        if path is None:
            response.message = "Recorder was not active."
        else:
            response.message = f"Stopped recording {path}"
        return response

    def start_recording(self) -> Path:
        if self.recording_file is not None and self.recording_path is not None:
            return self.recording_path

        self.recording_path = self.build_recording_path()
        self.recording_path.parent.mkdir(parents=True, exist_ok=True)

        # Use write mode for JSON/JSONL generated by this node to avoid appending after a closed array.
        # Use append mode only when a fixed CSV path is explicitly requested.
        file_mode = "a" if self.recording_format == "csv" and self.csv_path_override else "w"
        self.recording_file = open(self.recording_path, file_mode, newline="", encoding="utf-8")

        self.recording_start_stamp_sec = self.now_seconds()
        self.json_record_count = 0
        self.csv_writer = None

        if self.recording_format == "csv":
            self.csv_writer = csv.DictWriter(
                self.recording_file,
                fieldnames=TELEMETRY_CSV_FIELDNAMES,
            )
            if self.recording_file.tell() == 0:
                self.csv_writer.writeheader()
        elif self.recording_format == "json":
            self.recording_file.write("[\n")

        self.get_logger().info(f"Started telemetry recording: {self.recording_path}")
        return self.recording_path

    def stop_recording(self) -> Optional[Path]:
        if self.recording_file is None:
            return None

        path = self.recording_path

        if self.recording_format == "json":
            if self.json_record_count > 0:
                self.recording_file.write("\n")
            self.recording_file.write("]\n")

        self.recording_file.flush()
        self.recording_file.close()
        self.recording_file = None
        self.recording_path = None
        self.recording_start_stamp_sec = None
        self.json_record_count = 0
        self.csv_writer = None

        if rclpy.ok(context=self.context):
            self.get_logger().info(f"Stopped telemetry recording: {path}")
        return path

    def record_snapshot(self, payload: Dict[str, Any], payload_json: str) -> None:
        if self.recording_file is None:
            return

        if self.csv_writer is not None:
            self.csv_writer.writerow(self.build_csv_row(payload))
        elif self.recording_format == "json":
            self.write_pretty_json_record(payload)
        else:
            # JSONL keeps the full nested payload, including all timestamps and raw node fields.
            self.recording_file.write(payload_json)
            self.recording_file.write("\n")

        self.recording_file.flush()

    def write_pretty_json_record(self, payload: Dict[str, Any]) -> None:
        if self.json_record_count > 0:
            self.recording_file.write(",\n")

        record_json = json.dumps(
            self.build_json_record(payload),
            indent=2,
            sort_keys=False,
            default=json_numeric_scalar,
        )
        self.recording_file.write(record_json)
        self.json_record_count += 1

    def build_json_record(self, payload: Dict[str, Any]) -> Dict[str, Any]:
        csv_row = self.build_csv_row(payload)
        record = {
            fieldname: json_record_value(csv_row[fieldname])
            for fieldname in TELEMETRY_CSV_FIELDNAMES
        }
        for label in self.traction_node_labels.values():
            fieldname = f"{label}_error_history_raw"
            value = record.get(fieldname)
            if isinstance(value, str):
                record[fieldname] = json.loads(value)
        return record

    def build_csv_row(self, payload: Dict[str, Any]) -> Dict[str, str]:
        row = {
            "tmps": datetime.now().isoformat(timespec="milliseconds"),
            "stamp_sec": csv_number(payload["stamp_sec"], 6),
            "elapsed_s": csv_number(
                payload["stamp_sec"] - self.recording_start_stamp_sec
                if self.recording_start_stamp_sec is not None
                else None,
                3,
            ),
            "ros_moving": str(bool(payload["ros"]["moving"])),
            "ros_operating_seconds": csv_number(payload["ros"]["operating_seconds"], 3),
            "ros_estop_operations": csv_number(payload["ros"]["estop_operations"]),
        }

        nodes = payload.get("nodes", {})
        vanguard = payload.get("vanguard", {})
        battery_soc = vanguard.get("battery_soc", {})
        battery_electrical = vanguard.get("battery_electrical", {})
        bms_charge = vanguard.get("bms_charge", {})
        charger_status = vanguard.get("charger_status", {})
        charger_limits = vanguard.get("charger_limits", {})

        row.update(
            {
                "vanguard_battery_soc_percent": csv_number(
                    battery_soc.get("soc_percent"), 3
                ),
                "vanguard_battery_soc_fresh": str(
                    bool(vanguard.get("battery_soc_fresh"))
                ),
                "vanguard_battery_pack_voltage_v": csv_number(
                    battery_electrical.get("pack_voltage_v"), 3
                ),
                "vanguard_battery_pack_current_a": csv_number(
                    battery_electrical.get("pack_current_a"), 3
                ),
                "vanguard_battery_electrical_fresh": str(
                    bool(vanguard.get("battery_electrical_fresh"))
                ),
                "vanguard_bms_charge_mode": csv_number(bms_charge.get("mode")),
                "vanguard_bms_requested_current_a": csv_number(
                    bms_charge.get("requested_current_a"), 3
                ),
                "vanguard_bms_charge_fresh": str(
                    bool(vanguard.get("bms_charge_fresh"))
                ),
                "vanguard_charger_output_voltage_v": csv_number(
                    charger_status.get("output_voltage_v"), 3
                ),
                "vanguard_charger_output_current_a": csv_number(
                    charger_status.get("output_current_a"), 3
                ),
                "vanguard_charger_status": csv_number(
                    charger_status.get("status")
                ),
                "vanguard_charger_ac_connected": (
                    str(bool(charger_status["ac_connected"]))
                    if charger_status.get("ac_connected") is not None
                    else ""
                ),
                "vanguard_charger_ac_voltage_v": csv_number(
                    charger_status.get("ac_voltage_v"), 3
                ),
                "vanguard_charger_charging_active": (
                    str(bool(charger_status["charging_active"]))
                    if charger_status.get("charging_active") is not None
                    else ""
                ),
                "vanguard_charger_fault": (
                    str(bool(charger_status["fault"]))
                    if charger_status.get("fault") is not None
                    else ""
                ),
                "vanguard_charger_status_fresh": str(
                    bool(vanguard.get("charger_status_fresh"))
                ),
                "vanguard_charger_voltage_limit_v": csv_number(
                    charger_limits.get("voltage_limit_v"), 3
                ),
                "vanguard_charger_current_limit_a": csv_number(
                    charger_limits.get("current_limit_a"), 3
                ),
                "vanguard_charger_power_rating_kw": csv_number(
                    charger_limits.get("power_rating_kw"), 3
                ),
                "vanguard_charger_limits_fresh": str(
                    bool(vanguard.get("charger_limits_fresh"))
                ),
                "vanguard_eff3_requested_current_a": csv_number(
                    vanguard.get("eff3_requested_current_a"), 3
                ),
                "vanguard_charge_rate_status": str(
                    vanguard.get("charge_rate_status", "")
                ),
            }
        )

        for node_id, label in self.drive_node_labels.items():
            data = nodes.get(hex_id(node_id), {})

            if node_id in self.traction_node_ids:
                row[f"{label}_ec_error_code"] = csv_number(
                    data.get("ec_error_code")
                )
                row[f"{label}_ec_error_code_stamp_sec"] = csv_number(
                    data.get("ec_error_code_stamp_sec"), 6
                )
                row[f"{label}_last_nonzero_ec_error_code"] = csv_number(
                    data.get("last_nonzero_ec_error_code")
                )
                row[
                    f"{label}_last_nonzero_ec_error_code_stamp_sec"
                ] = csv_number(
                    data.get("last_nonzero_ec_error_code_stamp_sec"), 6
                )
                row[f"{label}_digital_inputs_raw"] = csv_number(
                    data.get("digital_inputs_raw")
                )
                row[f"{label}_digital_inputs_stamp_sec"] = csv_number(
                    data.get("digital_inputs_raw_stamp_sec"), 6
                )
                row[f"{label}_sto_status_raw"] = csv_number(
                    data.get("sto_status_raw")
                )
                row[f"{label}_sto_status_stamp_sec"] = csv_number(
                    data.get("sto_status_raw_stamp_sec"), 6
                )
                row[f"{label}_error_history_count"] = csv_number(
                    data.get("error_history_count")
                )
                row[f"{label}_error_history_stamp_sec"] = csv_number(
                    data.get("error_history_stamp_sec"), 6
                )
                row[f"{label}_error_history_raw"] = json.dumps(
                    data.get("error_history_raw", []),
                    separators=(",", ":"),
                    default=json_numeric_scalar,
                )
                row[f"{label}_motor_temp_c"] = csv_number(
                    data.get("motor_temperature_c")
                )

            row[f"{label}_controller_temp_c"] = csv_number(data.get("controller_temperature_c"))
            row[f"{label}_serial_number"] = csv_number(data.get("serial_number"))
            row[f"{label}_status_feedback"] = csv_number(data.get("status_feedback"))
            if node_id in self.traction_node_ids:
                row[f"{label}_mode_display"] = csv_number(data.get("traction_mode_display"))
            row[f"{label}_enabled"] = (
                str(bool(data["enabled"])) if data.get("enabled") is not None else ""
            )
            error_feedback = data.get("error_feedback")
            if error_feedback is None and node_id in self.traction_node_ids:
                status_feedback = data.get("status_feedback")
                if status_feedback is not None:
                    error_feedback = int((int(status_feedback) & 0x0008) != 0)
            row[f"{label}_error_feedback"] = csv_number(error_feedback)
            row[f"{label}_can_estop_operations"] = csv_number(data.get("can_estop_operations"))
            row[f"{label}_bus_voltage_v"] = csv_number(data.get("bus_voltage_v"), 3)

        for node_id, label in self.traction_node_labels.items():
            data = nodes.get(hex_id(node_id), {})
            current_ma = data.get("traction_current_actual_ma")
            row[f"{label}_velocity_raw"] = csv_number(data.get("traction_velocity_raw"))
            row[f"{label}_velocity_mm_s"] = csv_number(data.get("traction_velocity_mm_s"), 3)
            row[f"{label}_acc_raw"] = csv_number(data.get("traction_accel_profile_raw"))
            row[f"{label}_acc_mm_s2"] = csv_number(data.get("traction_accel_profile_mm_s2"), 3)
            row[f"{label}_dec_raw"] = csv_number(data.get("traction_decel_profile_raw"))
            row[f"{label}_dec_mm_s2"] = csv_number(data.get("traction_decel_profile_mm_s2"), 3)
            row[f"{label}_current_actual_permille"] = csv_number(
                data.get("traction_current_actual_permille")
            )
            row[f"{label}_current_actual_ma"] = csv_number(current_ma, 3)
            row[f"{label}_rms_cur"] = csv_number(data.get("traction_rms_current"), 3)

        for node_id, label in self.steering_node_labels.items():
            data = nodes.get(hex_id(node_id), {})
            row[f"{label}_position_ticks"] = csv_number(data.get("steering_position_ticks"))
            row[f"{label}_angle_deg"] = csv_number(data.get("steering_angle_deg"), 3)
            row[f"{label}_velocity_deg_s"] = csv_number(data.get("steering_velocity_deg_s"), 3)
            row[f"{label}_current_actual_ma"] = csv_number(
                data.get("steering_current_actual_ma"), 3
            )
            row[f"{label}_rms_cur"] = csv_number(
                data.get("steering_rms_current"), 3
            )

        return {fieldname: row.get(fieldname, "") for fieldname in TELEMETRY_CSV_FIELDNAMES}

    def now_seconds(self) -> float:
        now = self.get_clock().now()
        return now.nanoseconds / 1e9

    def destroy_node(self) -> bool:
        self.stop_recording()
        return super().destroy_node()


def main(args: Optional[List[str]] = None) -> None:
    rclpy.init(args=args)
    node = CanRosbagRecorder()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except RuntimeError:
        # Jazzy can surface a pybind take_message RuntimeError when SIGINT
        # invalidates the ROS context while the executor is taking a message.
        if rclpy.ok(context=node.context):
            raise
    finally:
        node.destroy_node()
        if rclpy.ok(context=node.context):
            rclpy.shutdown(context=node.context)


if __name__ == "__main__":
    main()
