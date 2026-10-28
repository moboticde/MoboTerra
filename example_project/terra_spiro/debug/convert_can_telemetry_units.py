#!/usr/bin/env python3
"""Add converted traction units to flat can_rosbag_recorder JSON files."""

import argparse
import json
import math
from pathlib import Path
from typing import Any, Dict, Optional


TRACTION_LABELS = ("ftr", "btr")
MOTOR_TEMPERATURE_FIXED_POINT_SCALE = 1.0 / (2**25)


def raw_to_mm(raw_value: Any, wheel_diameter_mm: float, gear_ratio: float, resolution: float) -> Optional[float]:
    if raw_value is None:
        return None
    if gear_ratio == 0.0 or resolution == 0.0:
        return None
    return float(raw_value) * math.pi * wheel_diameter_mm / (resolution * gear_ratio)


def add_conversions(
    record: Dict[str, Any],
    wheel_diameter_mm: float,
    gear_ratio: float,
    resolution: float,
) -> None:
    for label in TRACTION_LABELS:
        # Drop legacy derived statusword fields from converted recordings.
        record.pop(f"{label}_statusword_fault", None)
        record.pop(f"{label}_statusword_warning", None)
        record.pop(f"{label}_status_word_running", None)

        velocity = raw_to_mm(
            record.get(f"{label}_velocity_raw"),
            wheel_diameter_mm,
            gear_ratio,
            resolution,
        )
        if velocity is not None:
            record[f"{label}_velocity_mm_s"] = velocity

        acceleration = raw_to_mm(
            record.get(f"{label}_acc_raw"),
            wheel_diameter_mm,
            gear_ratio,
            resolution,
        )
        if acceleration is not None:
            record[f"{label}_acc_mm_s2"] = acceleration

        deceleration = raw_to_mm(
            record.get(f"{label}_dec_raw"),
            wheel_diameter_mm,
            gear_ratio,
            resolution,
        )
        if deceleration is not None:
            record[f"{label}_dec_mm_s2"] = deceleration

        motor_temperature = record.get(f"{label}_motor_temp_c")
        if isinstance(motor_temperature, (int, float)) and abs(motor_temperature) > 100000.0:
            record[f"{label}_motor_temp_c"] = (
                float(motor_temperature) * MOTOR_TEMPERATURE_FIXED_POINT_SCALE
            )

        bus_voltage = record.get(f"{label}_bus_voltage_v")
        if isinstance(bus_voltage, (int, float)) and abs(bus_voltage) > 1000.0:
            record[f"{label}_bus_voltage_v"] = float(bus_voltage) * 0.001

        status = record.get(f"{label}_status_feedback")
        if status is not None:
            status_int = int(status)
            record[f"{label}_enabled"] = (status_int & 0x006F) == 0x0027
            has_fault = int((status_int & 0x0008) != 0)
            record[f"{label}_error_feedback"] = has_fault
            if has_fault == 0:
                record[f"{label}_ec_error_code"] = 0


def default_output_path(input_path: Path) -> Path:
    return input_path.with_name(f"{input_path.stem}_converted{input_path.suffix}")


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("input", type=Path)
    parser.add_argument("-o", "--output", type=Path)
    parser.add_argument("--wheel-diameter-mm", type=float, default=225.0)
    parser.add_argument("--gear-ratio", type=float, default=16.0)
    parser.add_argument("--resolution", type=float, default=4096.0)
    args = parser.parse_args()

    input_path = args.input.expanduser()
    output_path = (args.output or default_output_path(input_path)).expanduser()

    with input_path.open(encoding="utf-8") as input_file:
        records = json.load(input_file)

    if not isinstance(records, list):
        raise TypeError(f"{input_path} does not contain a JSON list")

    for record in records:
        if isinstance(record, dict):
            add_conversions(
                record,
                args.wheel_diameter_mm,
                args.gear_ratio,
                args.resolution,
            )

    output_path.parent.mkdir(parents=True, exist_ok=True)
    with output_path.open("w", encoding="utf-8") as output_file:
        json.dump(records, output_file, indent=2)
        output_file.write("\n")

    print(output_path)


if __name__ == "__main__":
    main()
