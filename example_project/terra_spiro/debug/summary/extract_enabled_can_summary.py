#!/usr/bin/env python3
"""Extract traction-enabled CAN telemetry samples from a range of recordings.

Edit the CONFIG section, then run:

    python3 src/terra_spiro/debug/summary/extract_enabled_can_summary.py

The selected file range is inclusive. A sample is written only when both
``ftr_enabled`` and ``btr_enabled`` are true. Set ``EXTRACT_MODE`` to
``"steering"`` to write fstr/bstr steering telemetry, or ``"traction"`` to
write the original ftr/btr traction telemetry with integrated distance.
"""

import argparse
import json
import math
import os
import re
import sys
from datetime import datetime
from pathlib import Path
from typing import Any, Dict, Iterator, List, Optional, Sequence


# =============================================================================
# CONFIG - CHANGE HERE
# =============================================================================

RECORDS_DIR = Path("/home/mobotic/can_telemetry_records")

START_FILE = "can_telemetry_20260925_110744.json"
END_FILE = "can_telemetry_20260927_214039.json"

EXTRACT_MODE = "traction"

TRACTION_OUTPUT_PATH = Path(
    "/home/mobotic/romb_ws_ec/can_telemetry_reports/"
    "enabled_traction_sep.json"
)
STEERING_OUTPUT_PATH = Path(
    "/home/mobotic/romb_ws_ec/can_telemetry_reports/"
    "enabled_steering_general.json"
)

# Leave as None to use the default path for EXTRACT_MODE.
OUTPUT_PATH: Optional[Path] = None

# Distance is integrated only across nearby enabled samples. Larger timestamp
# gaps are treated as recording breaks, so stale velocity does not create travel.
DISTANCE_MAX_SAMPLE_GAP_S = 2.0

# =============================================================================


DEBUG_DIR = Path(__file__).resolve().parents[1]
if str(DEBUG_DIR) not in sys.path:
    sys.path.insert(0, str(DEBUG_DIR))

import plot_can_telemetry as pct  # noqa: E402


Record = Dict[str, Any]
MODE_CHOICES = ("traction", "steering")

TRACTION_OUTPUT_FIELDS = [
    "timestamp",
    "ros_moving",
    "ftr_rpm_raw",
    "btr_rpm_raw",
    "ftr_motor_temp_c",
    "btr_motor_temp_c",
    "ftr_velocity_mm_s",
    "btr_velocity_mm_s",
    "ftr_distance_traveled_m",
    "btr_distance_traveled_m",
    "ftr_current_actual_ma",
    "btr_current_actual_ma",
]

STEERING_OUTPUT_FIELDS = [
    "timestamp",
    "ros_moving",
    "ftr_enabled",
    "btr_enabled",
    "fstr_enabled",
    "bstr_enabled",
    "fstr_position_ticks",
    "bstr_position_ticks",
    "fstr_angle_deg",
    "bstr_angle_deg",
    "fstr_velocity_deg_s",
    "bstr_velocity_deg_s",
    "fstr_rms_cur",
    "bstr_rms_cur",
    "fstr_motor_temp_c",
    "bstr_motor_temp_c",
    "fstr_controller_temp_c",
    "bstr_controller_temp_c",
    "fstr_bus_voltage_v",
    "bstr_bus_voltage_v",
    "fstr_status_feedback",
    "bstr_status_feedback",
    "fstr_error_feedback",
    "bstr_error_feedback",
    "fstr_can_estop_operations",
    "bstr_can_estop_operations",
]

JSON_FIELD_LINE_RE = re.compile(
    r'^\s*"(?P<key>(?:\\.|[^"\\])*)"\s*:\s*(?P<value>.*?)\s*,?\s*$'
)


def finite_float(value: Any) -> Optional[float]:
    if value is None:
        return None

    try:
        number = float(value)
    except (TypeError, ValueError):
        return None

    if math.isnan(number) or math.isinf(number):
        return None

    return number


def numeric_value(record: Record, key: str) -> Optional[float]:
    """Read a numeric value, with current fallback for older recordings."""
    value = pct.numeric_value(record, key)
    if value is not None:
        return value

    # Older recordings wrote traction current under *_rms_cur. The recorder later
    # mapped the same value to *_current_actual_ma.
    if key == "ftr_current_actual_ma":
        return finite_float(record.get("ftr_rms_cur"))
    if key == "btr_current_actual_ma":
        return finite_float(record.get("btr_rms_cur"))

    return None


def rpm_raw_value(record: Record, label: str) -> Optional[float]:
    """Read the raw traction speed field from old or current recordings."""
    direct = finite_float(record.get(f"{label}_rpm_raw"))
    if direct is not None:
        return direct

    return numeric_value(record, f"{label}_velocity_raw")


def steering_numeric_value(record: Record, label: str, field: str) -> Optional[float]:
    value = numeric_value(record, f"{label}_{field}")
    if value is not None:
        return value

    # KeyboardControl steering recordings used these older names.
    if field == "angle_deg":
        return numeric_value(record, f"{label}_steering_position_deg")
    if field == "velocity_deg_s":
        return numeric_value(record, f"{label}_steering_velocity_deg_s")

    return None


def timestamp_value(record: Record) -> Any:
    """Return the best available timestamp value under the requested key name."""
    for key in ("timestamp", "tmps", "stamp_sec", "elapsed_s"):
        value = record.get(key)
        if value is not None:
            return value

    return None


def timestamp_seconds(value: Any) -> Optional[float]:
    if value is None:
        return None

    if isinstance(value, (int, float)):
        return float(value)

    text = str(value)
    try:
        return float(text)
    except ValueError:
        pass

    try:
        return datetime.fromisoformat(text).timestamp()
    except ValueError:
        return None


def traction_output_sample(record: Record) -> Record:
    return {
        "timestamp": timestamp_value(record),
        "ros_moving": record.get("ros_moving"),
        "ftr_rpm_raw": rpm_raw_value(record, "ftr"),
        "btr_rpm_raw": rpm_raw_value(record, "btr"),
        "ftr_motor_temp_c": numeric_value(record, "ftr_motor_temp_c"),
        "btr_motor_temp_c": numeric_value(record, "btr_motor_temp_c"),
        "ftr_velocity_mm_s": numeric_value(record, "ftr_velocity_mm_s"),
        "btr_velocity_mm_s": numeric_value(record, "btr_velocity_mm_s"),
        "ftr_distance_traveled_m": None,
        "btr_distance_traveled_m": None,
        "ftr_current_actual_ma": numeric_value(record, "ftr_current_actual_ma"),
        "btr_current_actual_ma": numeric_value(record, "btr_current_actual_ma"),
    }


def steering_output_sample(record: Record) -> Record:
    return {
        "timestamp": timestamp_value(record),
        "ros_moving": record.get("ros_moving"),
        "ftr_enabled": record.get("ftr_enabled"),
        "btr_enabled": record.get("btr_enabled"),
        "fstr_enabled": record.get("fstr_enabled"),
        "bstr_enabled": record.get("bstr_enabled"),
        "fstr_position_ticks": steering_numeric_value(record, "fstr", "position_ticks"),
        "bstr_position_ticks": steering_numeric_value(record, "bstr", "position_ticks"),
        "fstr_angle_deg": steering_numeric_value(record, "fstr", "angle_deg"),
        "bstr_angle_deg": steering_numeric_value(record, "bstr", "angle_deg"),
        "fstr_velocity_deg_s": steering_numeric_value(record, "fstr", "velocity_deg_s"),
        "bstr_velocity_deg_s": steering_numeric_value(record, "bstr", "velocity_deg_s"),
        "fstr_rms_cur": numeric_value(record, "fstr_rms_cur"),
        "bstr_rms_cur": numeric_value(record, "bstr_rms_cur"),
        "fstr_motor_temp_c": numeric_value(record, "fstr_motor_temp_c"),
        "bstr_motor_temp_c": numeric_value(record, "bstr_motor_temp_c"),
        "fstr_controller_temp_c": numeric_value(record, "fstr_controller_temp_c"),
        "bstr_controller_temp_c": numeric_value(record, "bstr_controller_temp_c"),
        "fstr_bus_voltage_v": numeric_value(record, "fstr_bus_voltage_v"),
        "bstr_bus_voltage_v": numeric_value(record, "bstr_bus_voltage_v"),
        "fstr_status_feedback": numeric_value(record, "fstr_status_feedback"),
        "bstr_status_feedback": numeric_value(record, "bstr_status_feedback"),
        "fstr_error_feedback": numeric_value(record, "fstr_error_feedback"),
        "bstr_error_feedback": numeric_value(record, "bstr_error_feedback"),
        "fstr_can_estop_operations": numeric_value(record, "fstr_can_estop_operations"),
        "bstr_can_estop_operations": numeric_value(record, "bstr_can_estop_operations"),
    }


def output_fields(mode: str) -> List[str]:
    if mode == "traction":
        return TRACTION_OUTPUT_FIELDS
    if mode == "steering":
        return STEERING_OUTPUT_FIELDS
    raise ValueError(f"Unsupported extract mode: {mode!r}")


def output_sample(record: Record, mode: str) -> Record:
    if mode == "traction":
        return traction_output_sample(record)
    if mode == "steering":
        return steering_output_sample(record)
    raise ValueError(f"Unsupported extract mode: {mode!r}")


def default_output_path(mode: str) -> Path:
    if mode == "traction":
        return TRACTION_OUTPUT_PATH
    if mode == "steering":
        return STEERING_OUTPUT_PATH
    raise ValueError(f"Unsupported extract mode: {mode!r}")


def traction_enabled(record: Record) -> bool:
    return record.get("ftr_enabled") is True and record.get("btr_enabled") is True


def has_closing_json_array_bracket(path: Path) -> bool:
    with path.open("rb") as handle:
        handle.seek(0, os.SEEK_END)
        size = handle.tell()
        offset = min(size, 8192)
        handle.seek(size - offset)
        tail = handle.read(offset).rstrip()

    return tail.endswith(b"]")


def empty_recovery_stats() -> Record:
    return {
        "missing_opening_array_bracket": False,
        "missing_closing_array_bracket": False,
        "objects_repaired": 0,
        "objects_recovered_from_fields": 0,
        "objects_skipped": 0,
        "incomplete_objects": 0,
        "ignored_non_json_lines": 0,
    }


def decode_json_field_value(text: str) -> Any:
    text = text.strip()
    if text.endswith(","):
        text = text[:-1].rstrip()

    if not text:
        return None

    try:
        return json.loads(text)
    except json.JSONDecodeError:
        pass

    if text in {"true", "false", "null"}:
        return {"true": True, "false": False, "null": None}[text]

    try:
        if any(marker in text for marker in (".", "e", "E")):
            return float(text)
        return int(text)
    except ValueError:
        pass

    if text.startswith('"'):
        return text[1:].rstrip('"')

    return text


def recover_object_fields(lines: Sequence[str]) -> Optional[Record]:
    record: Record = {}

    for line in lines:
        match = JSON_FIELD_LINE_RE.match(line)
        if match is None:
            continue

        try:
            key = json.loads(f'"{match.group("key")}"')
        except json.JSONDecodeError:
            key = match.group("key")

        record[key] = decode_json_field_value(match.group("value"))

    return record if record else None


def parse_json_record_text(
    lines: Sequence[str],
    path: Path,
    object_start_line: Optional[int],
    recovery_stats: Record,
    incomplete: bool = False,
) -> Optional[Record]:
    text = "".join(lines).strip()
    if text.endswith(","):
        text = text[:-1].rstrip()

    candidates = [text]
    if text.startswith("{") and not text.endswith("}"):
        candidates.append(re.sub(r",\s*$", "", text) + "\n}")

    for candidate in candidates:
        if not candidate:
            continue

        try:
            record = json.loads(candidate)
        except json.JSONDecodeError:
            continue

        if isinstance(record, dict):
            if candidate != text or incomplete:
                recovery_stats["objects_repaired"] += 1
            return record

        recovery_stats["objects_skipped"] += 1
        print(
            f"WARNING: expected JSON object in {path} at line "
            f"{object_start_line}, got {type(record).__name__}"
        )
        return None

    recovered = recover_object_fields(lines)
    if recovered is not None:
        recovery_stats["objects_recovered_from_fields"] += 1
        return recovered

    recovery_stats["objects_skipped"] += 1
    print(
        f"WARNING: skipping damaged JSON object in {path} "
        f"starting at line {object_start_line}"
    )
    return None


def iter_recoverable_json_records(
    path: Path,
    recovery_stats: Record,
) -> Iterator[Record]:
    """Yield as many flat recorder objects as possible from damaged JSON files.

    The normal recorder output is a pretty-printed JSON array of flat objects.
    This reader accepts missing array brackets, missing object braces around a
    record, and an interrupted final object. If a record cannot be parsed as JSON,
    it falls back to extracting individual ``"key": value`` lines from it.
    """
    object_lines: List[str] = []
    object_start_line: Optional[int] = None
    saw_array_start = False

    def start_object(line: str, line_number: int) -> Optional[Record]:
        nonlocal object_lines, object_start_line
        object_lines = [line]
        object_start_line = line_number

        if line.strip().startswith("{") and line.strip().rstrip(",").endswith("}"):
            record = parse_json_record_text(
                object_lines,
                path,
                object_start_line,
                recovery_stats,
            )
            object_lines = []
            object_start_line = None
            return record

        return None

    with path.open("r", encoding="utf-8") as handle:
        for line_number, line in enumerate(handle, start=1):
            stripped = line.strip()

            if not object_lines:
                if not stripped or stripped == ",":
                    continue

                if stripped == "[":
                    saw_array_start = True
                    continue

                if stripped == "]":
                    return

                if stripped.startswith("{"):
                    if not saw_array_start:
                        recovery_stats["missing_opening_array_bracket"] = True
                    record = start_object(line, line_number)
                    if record is not None:
                        yield record
                    continue

                if stripped.startswith('"') and ":" in stripped:
                    if not saw_array_start:
                        recovery_stats["missing_opening_array_bracket"] = True
                    object_lines = ["{\n", line]
                    object_start_line = line_number
                    continue

                recovery_stats["ignored_non_json_lines"] += 1
                continue

            if stripped.startswith("{"):
                recovery_stats["incomplete_objects"] += 1
                record = parse_json_record_text(
                    object_lines,
                    path,
                    object_start_line,
                    recovery_stats,
                    incomplete=True,
                )
                if record is not None:
                    yield record

                if not saw_array_start:
                    recovery_stats["missing_opening_array_bracket"] = True
                record = start_object(line, line_number)
                if record is not None:
                    yield record
                continue

            if stripped == "]":
                recovery_stats["incomplete_objects"] += 1
                record = parse_json_record_text(
                    object_lines,
                    path,
                    object_start_line,
                    recovery_stats,
                    incomplete=True,
                )
                if record is not None:
                    yield record
                return

            object_lines.append(line)

            if stripped.startswith("}"):
                record = parse_json_record_text(
                    object_lines,
                    path,
                    object_start_line,
                    recovery_stats,
                )
                if record is not None:
                    yield record
                object_lines = []
                object_start_line = None

    if not saw_array_start:
        recovery_stats["missing_opening_array_bracket"] = True

    if object_lines:
        recovery_stats["incomplete_objects"] += 1
        print(
            f"WARNING: recovering incomplete JSON object in {path} "
            f"starting at line {object_start_line}"
        )
        record = parse_json_record_text(
            object_lines,
            path,
            object_start_line,
            recovery_stats,
            incomplete=True,
        )
        if record is not None:
            yield record


def selected_recording_files(
    records_dir: Path,
    start_file: str,
    end_file: str,
) -> List[Path]:
    files = sorted(records_dir.glob("can_telemetry_*.json"))
    by_name = {path.name: path for path in files}

    missing = [name for name in (start_file, end_file) if name not in by_name]
    if missing:
        raise FileNotFoundError(
            "Recording file not found in "
            f"{records_dir}: {', '.join(missing)}"
        )

    if start_file > end_file:
        raise ValueError(
            f"START_FILE must be older/equal to END_FILE: {start_file} > {end_file}"
        )

    return [
        path
        for path in files
        if start_file <= path.name <= end_file
    ]


def iter_enabled_samples(path: Path, mode: str = EXTRACT_MODE) -> Iterator[Record]:
    distance_integrator = DistanceIntegrator(DISTANCE_MAX_SAMPLE_GAP_S)
    recovery_stats = empty_recovery_stats()
    recovery_stats["missing_closing_array_bracket"] = (
        not has_closing_json_array_bracket(path)
    )
    for record in iter_recoverable_json_records(path, recovery_stats):
        if traction_enabled(record):
            sample = output_sample(record, mode)
            if mode == "traction":
                distance_integrator.update(sample)
            yield sample


def write_json_value(handle: Any, value: Any) -> None:
    handle.write(
        json.dumps(
            value,
            ensure_ascii=False,
            allow_nan=False,
            separators=(",", ":"),
        )
    )


class DistanceIntegrator:
    """Integrate front/back traction distance from enabled velocity samples."""

    def __init__(self, max_sample_gap_s: float) -> None:
        self.max_sample_gap_s = max_sample_gap_s
        self.previous_time_s: Optional[float] = None
        self.previous_ftr_velocity_mm_s: Optional[float] = None
        self.previous_btr_velocity_mm_s: Optional[float] = None
        self.ftr_distance_m = 0.0
        self.btr_distance_m = 0.0
        self.integrated_segments = 0
        self.skipped_gap_segments = 0
        self.skipped_nonmonotonic_segments = 0
        self.skipped_missing_timestamp_samples = 0
        self.skipped_missing_velocity_segments = 0

    def update(self, sample: Record) -> None:
        current_time_s = timestamp_seconds(sample.get("timestamp"))
        current_ftr_velocity_mm_s = finite_float(sample.get("ftr_velocity_mm_s"))
        current_btr_velocity_mm_s = finite_float(sample.get("btr_velocity_mm_s"))

        if current_time_s is None:
            self.skipped_missing_timestamp_samples += 1
            sample["ftr_distance_traveled_m"] = self.ftr_distance_m
            sample["btr_distance_traveled_m"] = self.btr_distance_m
            return

        if self.previous_time_s is not None:
            dt_s = current_time_s - self.previous_time_s
            if 0.0 < dt_s <= self.max_sample_gap_s:
                integrated_any = False
                if (
                    self.previous_ftr_velocity_mm_s is not None
                    and current_ftr_velocity_mm_s is not None
                ):
                    self.ftr_distance_m += (
                        (
                            abs(self.previous_ftr_velocity_mm_s)
                            + abs(current_ftr_velocity_mm_s)
                        )
                        * 0.5
                        * dt_s
                        / 1000.0
                    )
                    integrated_any = True

                if (
                    self.previous_btr_velocity_mm_s is not None
                    and current_btr_velocity_mm_s is not None
                ):
                    self.btr_distance_m += (
                        (
                            abs(self.previous_btr_velocity_mm_s)
                            + abs(current_btr_velocity_mm_s)
                        )
                        * 0.5
                        * dt_s
                        / 1000.0
                    )
                    integrated_any = True

                if integrated_any:
                    self.integrated_segments += 1
                else:
                    self.skipped_missing_velocity_segments += 1
            elif dt_s > self.max_sample_gap_s:
                self.skipped_gap_segments += 1
            else:
                self.skipped_nonmonotonic_segments += 1

        self.previous_time_s = current_time_s
        self.previous_ftr_velocity_mm_s = current_ftr_velocity_mm_s
        self.previous_btr_velocity_mm_s = current_btr_velocity_mm_s
        sample["ftr_distance_traveled_m"] = self.ftr_distance_m
        sample["btr_distance_traveled_m"] = self.btr_distance_m

    def summary(self) -> Record:
        return {
            "ftr_distance_traveled_m": self.ftr_distance_m,
            "btr_distance_traveled_m": self.btr_distance_m,
            "ftr_distance_traveled_km": self.ftr_distance_m / 1000.0,
            "btr_distance_traveled_km": self.btr_distance_m / 1000.0,
            "distance_integrated_segments": self.integrated_segments,
            "distance_skipped_gap_segments": self.skipped_gap_segments,
            "distance_skipped_nonmonotonic_segments": (
                self.skipped_nonmonotonic_segments
            ),
            "distance_skipped_missing_timestamp_samples": (
                self.skipped_missing_timestamp_samples
            ),
            "distance_skipped_missing_velocity_segments": (
                self.skipped_missing_velocity_segments
            ),
        }


def extract_summary(
    records_dir: Path,
    start_file: str,
    end_file: str,
    output_path: Path,
    mode: str = EXTRACT_MODE,
) -> Record:
    if mode not in MODE_CHOICES:
        raise ValueError(f"mode must be one of {', '.join(MODE_CHOICES)}")

    files = selected_recording_files(records_dir, start_file, end_file)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    fields = output_fields(mode)

    metadata = {
        "created_at": datetime.now().isoformat(timespec="seconds"),
        "source_directory": str(records_dir),
        "start_file": start_file,
        "end_file": end_file,
        "included_file_count": len(files),
        "mode": mode,
        "filter": {"ftr_enabled": True, "btr_enabled": True},
        "fields": fields,
        "timestamp_note": (
            "timestamp is taken from timestamp, tmps, stamp_sec, then elapsed_s, "
            "using the first field present in each record"
        ),
        "json_recovery_note": (
            "Input JSON recordings are read with a recovery parser. Missing array "
            "brackets, interrupted final objects, and flat key/value lines from "
            "damaged objects are repaired where possible; unrecoverable objects "
            "are skipped."
        ),
    }

    if mode == "traction":
        metadata.update(
            {
                "rpm_raw_note": (
                    "ftr_rpm_raw/btr_rpm_raw use those fields when present; newer "
                    "recordings fall back to ftr_velocity_raw/btr_velocity_raw "
                    "because they contain the same raw traction feedback value."
                ),
                "current_note": (
                    "ftr_current_actual_ma/btr_current_actual_ma use the requested "
                    "fields when present; older recordings fall back to "
                    "ftr_rms_cur/btr_rms_cur"
                ),
                "distance_note": (
                    "ftr_distance_traveled_m/btr_distance_traveled_m are cumulative "
                    "distances integrated from absolute enabled velocity using "
                    "trapezoids. Segments with timestamp gaps > "
                    f"{DISTANCE_MAX_SAMPLE_GAP_S:g}s are treated as recording "
                    "breaks and skipped."
                ),
            }
        )
    else:
        metadata["steering_note"] = (
            "fstr_angle_deg/bstr_angle_deg and fstr_velocity_deg_s/"
            "bstr_velocity_deg_s use the CAN recorder fields when present; "
            "older KeyboardControl steering recordings fall back to "
            "*_steering_position_deg and *_steering_velocity_deg_s."
        )

    total_records_scanned = 0
    total_enabled_samples = 0
    files_without_closing_array_bracket: List[str] = []
    files_with_json_recovery: List[str] = []
    file_summaries: List[Record] = []
    distance_integrator = DistanceIntegrator(DISTANCE_MAX_SAMPLE_GAP_S)

    with output_path.open("w", encoding="utf-8") as handle:
        handle.write("{\n")
        handle.write('"metadata":')
        write_json_value(handle, metadata)
        handle.write(',\n"samples":[\n')

        first_sample = True
        for index, path in enumerate(files, start=1):
            if not has_closing_json_array_bracket(path):
                files_without_closing_array_bracket.append(path.name)

            file_records_scanned = 0
            file_enabled_samples = 0
            recovery_stats = empty_recovery_stats()
            recovery_stats["missing_closing_array_bracket"] = (
                path.name in files_without_closing_array_bracket
            )

            for record in iter_recoverable_json_records(path, recovery_stats):
                file_records_scanned += 1

                if not traction_enabled(record):
                    continue

                sample = output_sample(record, mode)
                if mode == "traction":
                    distance_integrator.update(sample)
                if first_sample:
                    first_sample = False
                else:
                    handle.write(",\n")
                write_json_value(handle, sample)
                file_enabled_samples += 1

            total_records_scanned += file_records_scanned
            total_enabled_samples += file_enabled_samples
            recovery_used = any(
                value
                for key, value in recovery_stats.items()
                if key != "missing_closing_array_bracket"
            ) or recovery_stats["missing_closing_array_bracket"]
            if recovery_used:
                files_with_json_recovery.append(path.name)

            file_summaries.append(
                {
                    "file": path.name,
                    "records_scanned": file_records_scanned,
                    "enabled_samples": file_enabled_samples,
                    "json_recovery": recovery_stats,
                }
            )

            print(
                f"{index}/{len(files)} {path.name}: "
                f"{file_enabled_samples} traction-enabled {mode} samples",
                flush=True,
            )

        summary = {
            "output_path": str(output_path),
            "mode": mode,
            "total_records_scanned": total_records_scanned,
            "total_enabled_samples": total_enabled_samples,
            "files_without_closing_array_bracket": files_without_closing_array_bracket,
            "files_with_json_recovery": files_with_json_recovery,
        }
        if mode == "traction":
            summary.update(distance_integrator.summary())

        handle.write('\n],\n"summary":')
        write_json_value(handle, summary)
        handle.write(',\n"files":')
        write_json_value(handle, file_summaries)
        handle.write("\n}\n")

    return {
        **summary,
        "included_file_count": len(files),
        "start_file": start_file,
        "end_file": end_file,
    }


def parse_args(argv: Optional[Sequence[str]] = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Extract traction or steering telemetry from CAN telemetry files "
            "while both traction drives are enabled."
        )
    )
    parser.add_argument("--records-dir", type=Path, default=RECORDS_DIR)
    parser.add_argument("--start-file", default=START_FILE)
    parser.add_argument("--end-file", default=END_FILE)
    parser.add_argument("--mode", choices=MODE_CHOICES, default=EXTRACT_MODE)
    parser.add_argument(
        "--output",
        type=Path,
        default=OUTPUT_PATH,
        help=(
            "Output path. Defaults to the configured path for the selected mode "
            "when omitted."
        ),
    )
    return parser.parse_args(argv)


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = parse_args(argv)
    output_path = args.output if args.output is not None else default_output_path(args.mode)
    result = extract_summary(
        records_dir=args.records_dir,
        start_file=args.start_file,
        end_file=args.end_file,
        output_path=output_path,
        mode=args.mode,
    )

    print("\nDone")
    print(f"Mode: {result['mode']}")
    print(f"Output: {result['output_path']}")
    print(f"Files included: {result['included_file_count']}")
    print(f"Records scanned: {result['total_records_scanned']}")
    print(f"Traction-enabled samples: {result['total_enabled_samples']}")
    if result["mode"] == "traction":
        print(
            "Front traction distance: "
            f"{result['ftr_distance_traveled_m']:.3f} m "
            f"({result['ftr_distance_traveled_km']:.6f} km)"
        )
        print(
            "Back traction distance: "
            f"{result['btr_distance_traveled_m']:.3f} m "
            f"({result['btr_distance_traveled_km']:.6f} km)"
        )

    if result["files_without_closing_array_bracket"]:
        print(
            "Files without closing JSON array bracket: "
            + ", ".join(result["files_without_closing_array_bracket"])
        )

    if result["files_with_json_recovery"]:
        print(
            "Files read with JSON recovery: "
            + ", ".join(result["files_with_json_recovery"])
        )

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
