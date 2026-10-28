#!/usr/bin/env python3
"""Calculate acquisition KPIs from CAN telemetry recording coverage.

Example (times without an offset are interpreted in Europe/Berlin):

    python3 calculate_can_acquisition_summary.py \
        --start 2026-09-24T18:00:00 \
        --end 2026-09-25T18:00:00

Vehicle power-on time is the sum of consecutive recorded intervals. Front and
rear traction operation time is accumulated independently while the respective
drive is enabled and its absolute wheel speed exceeds the configured threshold.
Recording gaps larger than ``--max-sample-gap`` are excluded from all time and
distance totals.
"""

import argparse
import csv
import json
import math
import re
import shutil
import subprocess
import sys
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Any, Dict, Iterator, List, Optional, Sequence, Tuple
from zoneinfo import ZoneInfo


SUMMARY_DIR = Path(__file__).resolve().parent
if str(SUMMARY_DIR) not in sys.path:
    sys.path.insert(0, str(SUMMARY_DIR))

import extract_enabled_can_summary as extractor  # noqa: E402


DEFAULT_RECORDS_DIR = Path("/home/mobotic/can_telemetry_records")
DEFAULT_OUTPUT_DIR = Path("/home/mobotic/romb_ws_ec/can_telemetry_reports")
DEFAULT_TIMEZONE = "Europe/Berlin"
DEFAULT_MAX_SAMPLE_GAP_S = 2.0
DEFAULT_TRACTION_SPEED_THRESHOLD_MM_S = 0.0
RECORDING_NAME_RE = re.compile(
    r"^can_telemetry_(?P<date>\d{8})_(?P<time>\d{6})(?:_\d+)?\.json$"
)

CSV_FIELDS = [
    "Acquisition date",
    "Acquisition time",
    "Vehicle power-on time [s]",
    "Front Motorwheel Traction operation time [s]",
    "Rear Motorwheel Traction operation time [s]",
    "Front Motorwheel distance traveled in traction [m]",
    "Rear Motorwheel distance traveled in traction [m]",
    "Vehicle power-on hours [h]",
    "Power-on time delta since previous acquisition [h]",
    "Front Motorwheel Traction operation hours [h]",
    "Rear Motorwheel Traction operation hours [h]",
    "Traction time delta since previous acquisition [h]",
    "Front Motorwheel Average traction speed [mm/s]",
    "Rear Motorwheel Average traction speed [mm/s]",
    "Front Motorwheel Traction hours / power-on hours ratio [%]",
    "Rear Motorwheel Traction hours / power-on hours ratio [%]",
]


Record = Dict[str, Any]


def finite_float(value: Any) -> Optional[float]:
    try:
        number = float(value)
    except (TypeError, ValueError):
        return None
    return number if math.isfinite(number) else None


def record_timestamp_seconds(record: Record, timezone: ZoneInfo) -> Optional[float]:
    """Return epoch seconds, preferring the recorder's numeric ROS timestamp."""
    stamp_sec = finite_float(record.get("stamp_sec"))
    if stamp_sec is not None:
        return stamp_sec

    for key in ("timestamp", "tmps"):
        value = record.get(key)
        if value in (None, ""):
            continue
        try:
            parsed = datetime.fromisoformat(str(value))
        except ValueError:
            continue
        if parsed.tzinfo is None:
            parsed = parsed.replace(tzinfo=timezone)
        return parsed.timestamp()

    return None


def bool_value(value: Any) -> Optional[bool]:
    if isinstance(value, bool):
        return value
    if isinstance(value, (int, float)):
        return bool(value)
    if isinstance(value, str):
        normalized = value.strip().lower()
        if normalized == "true":
            return True
        if normalized == "false":
            return False
    return None


def velocity_mm_s(record: Record, label: str) -> Optional[float]:
    direct = extractor.numeric_value(record, f"{label}_velocity_mm_s")
    if direct is not None:
        return direct

    raw = extractor.rpm_raw_value(record, label)
    if raw is None:
        return None
    return extractor.pct.traction_raw_to_mm(raw)


def parse_local_datetime(value: str, timezone: ZoneInfo) -> datetime:
    parsed = datetime.fromisoformat(value)
    if parsed.tzinfo is None:
        parsed = parsed.replace(tzinfo=timezone)
    return parsed.astimezone(timezone)


def recording_start(path: Path, timezone: ZoneInfo) -> Optional[datetime]:
    match = RECORDING_NAME_RE.match(path.name)
    if match is None:
        return None
    return datetime.strptime(
        match.group("date") + match.group("time"), "%Y%m%d%H%M%S"
    ).replace(tzinfo=timezone)


def select_candidate_files(
    records_dir: Path,
    start: datetime,
    end: datetime,
    timezone: ZoneInfo,
) -> List[Path]:
    """Select the last recording started before the window plus later files."""
    dated: List[Tuple[datetime, Path]] = []
    for path in records_dir.glob("can_telemetry_*.json"):
        file_start = recording_start(path, timezone)
        if file_start is not None and file_start <= end:
            dated.append((file_start, path))
    dated.sort(key=lambda item: (item[0], item[1].name))

    before_or_at_start = [index for index, item in enumerate(dated) if item[0] <= start]
    first_index = before_or_at_start[-1] if before_or_at_start else 0
    return [path for _, path in dated[first_index:]]


def parse_fast_value(value: bytes) -> Any:
    value = value.strip().rstrip(b",")
    if value == b"true":
        return True
    if value == b"false":
        return False
    if value == b"null" or not value:
        return None
    try:
        return float(value)
    except ValueError:
        return None


def iter_fast_kpi_records(path: Path) -> Iterator[Record]:
    """Use ripgrep to stream only the five fields needed for these KPIs."""
    pattern = (
        rb'^  "(stamp_sec|ftr_enabled|btr_enabled|'
        rb'ftr_velocity_mm_s|btr_velocity_mm_s)":'
    )
    process = subprocess.Popen(
        ["rg", "--no-filename", "--no-line-number", pattern, path],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
    )
    if process.stdout is None:
        raise RuntimeError("Failed to open ripgrep output")

    record: Record = {}
    for line in process.stdout:
        key_text, separator, value = line.strip().partition(b":")
        if not separator:
            continue
        key = key_text.strip().strip(b'"').decode("ascii")
        if key == "stamp_sec" and record:
            yield record
            record = {}
        record[key] = parse_fast_value(value)

    if record:
        yield record

    stderr = process.stderr.read().decode("utf-8", errors="replace") if process.stderr else ""
    return_code = process.wait()
    if return_code not in (0, 1):
        raise RuntimeError(f"ripgrep failed for {path}: {stderr.strip()}")


def iter_kpi_records(path: Path) -> Iterator[Record]:
    if shutil.which("rg") is not None:
        yield from iter_fast_kpi_records(path)
        return

    recovery_stats = extractor.empty_recovery_stats()
    yield from extractor.iter_recoverable_json_records(path, recovery_stats)


def trapezoid_abs_integral(
    previous_value: Optional[float],
    current_value: Optional[float],
    segment_start: float,
    segment_end: float,
    overlap_start: float,
    overlap_end: float,
) -> Optional[float]:
    """Integrate absolute endpoint values over a clipped linear segment."""
    if previous_value is None or current_value is None:
        return None
    duration = segment_end - segment_start
    if duration <= 0.0:
        return None
    start_fraction = (overlap_start - segment_start) / duration
    end_fraction = (overlap_end - segment_start) / duration
    overlap_previous = previous_value + (current_value - previous_value) * start_fraction
    overlap_current = previous_value + (current_value - previous_value) * end_fraction
    return (
        (abs(overlap_previous) + abs(overlap_current))
        * 0.5
        * (overlap_end - overlap_start)
    )


@dataclass
class Sample:
    timestamp_s: float
    ftr_enabled: Optional[bool]
    btr_enabled: Optional[bool]
    ftr_velocity_mm_s: Optional[float]
    btr_velocity_mm_s: Optional[float]


class AcquisitionAccumulator:
    def __init__(
        self,
        start_s: float,
        end_s: float,
        max_sample_gap_s: float,
        traction_speed_threshold_mm_s: float,
    ) -> None:
        self.start_s = start_s
        self.end_s = end_s
        self.max_sample_gap_s = max_sample_gap_s
        self.traction_speed_threshold_mm_s = traction_speed_threshold_mm_s
        self.previous: Optional[Sample] = None
        self.vehicle_power_on_s = 0.0
        self.ftr_operation_s = 0.0
        self.btr_operation_s = 0.0
        self.shared_traction_s = 0.0
        self.ftr_distance_mm = 0.0
        self.btr_distance_mm = 0.0
        self.records_scanned = 0
        self.valid_timestamp_records = 0
        self.integrated_segments = 0
        self.skipped_gap_segments = 0
        self.skipped_nonmonotonic_segments = 0
        self.missing_ftr_velocity_segments = 0
        self.missing_btr_velocity_segments = 0
        self.first_covered_s: Optional[float] = None
        self.last_covered_s: Optional[float] = None

    def add(self, record: Record, timezone: ZoneInfo) -> bool:
        """Add a record. Return False once the requested end is passed."""
        self.records_scanned += 1
        timestamp_s = record_timestamp_seconds(record, timezone)
        if timestamp_s is None:
            return True
        self.valid_timestamp_records += 1

        current = Sample(
            timestamp_s=timestamp_s,
            ftr_enabled=bool_value(record.get("ftr_enabled")),
            btr_enabled=bool_value(record.get("btr_enabled")),
            ftr_velocity_mm_s=velocity_mm_s(record, "ftr"),
            btr_velocity_mm_s=velocity_mm_s(record, "btr"),
        )

        if self.previous is not None:
            self._integrate_segment(self.previous, current)
        self.previous = current
        return timestamp_s <= self.end_s

    def _integrate_segment(self, previous: Sample, current: Sample) -> None:
        dt_s = current.timestamp_s - previous.timestamp_s
        if dt_s <= 0.0:
            self.skipped_nonmonotonic_segments += 1
            return
        if dt_s > self.max_sample_gap_s:
            self.skipped_gap_segments += 1
            return

        overlap_start = max(previous.timestamp_s, self.start_s)
        overlap_end = min(current.timestamp_s, self.end_s)
        if overlap_end <= overlap_start:
            return

        overlap_s = overlap_end - overlap_start
        self.vehicle_power_on_s += overlap_s
        self.integrated_segments += 1
        if self.first_covered_s is None:
            self.first_covered_s = overlap_start
        self.last_covered_s = overlap_end

        # Enabled and moving are treated as sample-and-hold over this <=2 s
        # interval. At the normal 50 Hz rate, a transition contributes at most
        # one sample period of uncertainty to an operation-time total.
        ftr_active = self._traction_active(
            previous.ftr_enabled, previous.ftr_velocity_mm_s
        )
        btr_active = self._traction_active(
            previous.btr_enabled, previous.btr_velocity_mm_s
        )
        if previous.ftr_enabled is True:
            distance = trapezoid_abs_integral(
                previous.ftr_velocity_mm_s,
                current.ftr_velocity_mm_s,
                previous.timestamp_s,
                current.timestamp_s,
                overlap_start,
                overlap_end,
            )
            if distance is None:
                self.missing_ftr_velocity_segments += 1
            else:
                self.ftr_distance_mm += distance
        if ftr_active:
            self.ftr_operation_s += overlap_s

        if previous.btr_enabled is True:
            distance = trapezoid_abs_integral(
                previous.btr_velocity_mm_s,
                current.btr_velocity_mm_s,
                previous.timestamp_s,
                current.timestamp_s,
                overlap_start,
                overlap_end,
            )
            if distance is None:
                self.missing_btr_velocity_segments += 1
            else:
                self.btr_distance_mm += distance
        if btr_active:
            self.btr_operation_s += overlap_s

        if ftr_active and btr_active:
            self.shared_traction_s += overlap_s

    def _traction_active(
        self, enabled: Optional[bool], velocity: Optional[float]
    ) -> bool:
        return (
            enabled is True
            and velocity is not None
            and abs(velocity) > self.traction_speed_threshold_mm_s
        )


def rounded(value: Optional[float], digits: int = 6) -> Optional[float]:
    return None if value is None else round(value, digits)


def safe_ratio(numerator: float, denominator: float, scale: float = 1.0) -> Optional[float]:
    return numerator / denominator * scale if denominator > 0.0 else None


def calculate_summary(
    records_dir: Path,
    start: datetime,
    end: datetime,
    timezone: ZoneInfo,
    max_sample_gap_s: float,
    traction_speed_threshold_mm_s: float,
) -> Record:
    if end <= start:
        raise ValueError("--end must be later than --start")
    if max_sample_gap_s <= 0.0:
        raise ValueError("--max-sample-gap must be positive")
    if traction_speed_threshold_mm_s < 0.0:
        raise ValueError("--traction-speed-threshold must be non-negative")

    paths = select_candidate_files(records_dir, start, end, timezone)
    if not paths:
        raise FileNotFoundError(f"No CAN telemetry JSON files found in {records_dir}")

    accumulator = AcquisitionAccumulator(
        start.timestamp(),
        end.timestamp(),
        max_sample_gap_s,
        traction_speed_threshold_mm_s,
    )
    files_scanned: List[str] = []
    for path in paths:
        files_scanned.append(path.name)
        print(f"Scanning {path.name} ...", flush=True)
        for record in iter_kpi_records(path):
            if not accumulator.add(record, timezone):
                break
        if accumulator.previous is not None and accumulator.previous.timestamp_s > end.timestamp():
            break

    ftr_average = safe_ratio(accumulator.ftr_distance_mm, accumulator.ftr_operation_s)
    btr_average = safe_ratio(accumulator.btr_distance_mm, accumulator.btr_operation_s)
    power_hours = accumulator.vehicle_power_on_s / 3600.0
    ftr_hours = accumulator.ftr_operation_s / 3600.0
    btr_hours = accumulator.btr_operation_s / 3600.0
    shared_hours = accumulator.shared_traction_s / 3600.0

    values = {
        "acquisition_date": end.date().isoformat(),
        "acquisition_time": end.strftime("%H:%M:%S"),
        "vehicle_power_on_time_s": rounded(accumulator.vehicle_power_on_s, 3),
        "front_traction_operation_time_s": rounded(accumulator.ftr_operation_s, 3),
        "rear_traction_operation_time_s": rounded(accumulator.btr_operation_s, 3),
        "front_distance_traveled_in_traction_m": rounded(
            accumulator.ftr_distance_mm / 1000.0, 3
        ),
        "rear_distance_traveled_in_traction_m": rounded(
            accumulator.btr_distance_mm / 1000.0, 3
        ),
        "vehicle_power_on_hours_h": rounded(power_hours),
        "power_on_time_delta_since_previous_acquisition_h": rounded(power_hours),
        "front_traction_operation_hours_h": rounded(ftr_hours),
        "rear_traction_operation_hours_h": rounded(btr_hours),
        "traction_time_delta_since_previous_acquisition_h": rounded(shared_hours),
        "front_average_traction_speed_mm_s": rounded(ftr_average, 3),
        "rear_average_traction_speed_mm_s": rounded(btr_average, 3),
        "front_traction_to_power_on_ratio_percent": rounded(
            safe_ratio(accumulator.ftr_operation_s, accumulator.vehicle_power_on_s, 100.0),
            3,
        ),
        "rear_traction_to_power_on_ratio_percent": rounded(
            safe_ratio(accumulator.btr_operation_s, accumulator.vehicle_power_on_s, 100.0),
            3,
        ),
    }

    def local_iso(timestamp_s: Optional[float]) -> Optional[str]:
        if timestamp_s is None:
            return None
        return datetime.fromtimestamp(timestamp_s, timezone).isoformat(timespec="milliseconds")

    return {
        "metadata": {
            "timezone": str(timezone),
            "window_start": start.isoformat(),
            "window_end": end.isoformat(),
            "source_directory": str(records_dir),
            "files_scanned": files_scanned,
            "max_sample_gap_s": max_sample_gap_s,
            "traction_speed_threshold_mm_s": traction_speed_threshold_mm_s,
            "definitions": {
                "vehicle_power_on_time": (
                    "Sum of consecutive telemetry coverage inside the acquisition window; "
                    "recording gaps are excluded."
                ),
                "traction_operation_time": (
                    "Per-drive time for which that traction drive reports enabled and "
                    "its absolute wheel speed is greater than the configured threshold."
                ),
                "distance": (
                    "Integral of absolute per-drive wheel velocity while that drive reports "
                    "enabled; stationary samples contribute zero distance."
                ),
                "power_on_delta": (
                    "For this one-row interval report, equal to vehicle power-on hours "
                    "accumulated since the window start (the previous acquisition)."
                ),
                "traction_time_delta": (
                    "Hours in the interval for which both front and rear traction drives "
                    "report enabled and moving."
                ),
                "average_speed": (
                    "Per-drive distance divided by that drive's enabled operation time."
                ),
            },
        },
        "values": values,
        "quality": {
            "records_scanned": accumulator.records_scanned,
            "valid_timestamp_records": accumulator.valid_timestamp_records,
            "integrated_segments": accumulator.integrated_segments,
            "skipped_recording_gap_segments": accumulator.skipped_gap_segments,
            "skipped_nonmonotonic_segments": accumulator.skipped_nonmonotonic_segments,
            "missing_front_velocity_segments": accumulator.missing_ftr_velocity_segments,
            "missing_rear_velocity_segments": accumulator.missing_btr_velocity_segments,
            "first_covered_time": local_iso(accumulator.first_covered_s),
            "last_covered_time": local_iso(accumulator.last_covered_s),
            "window_wall_clock_hours": (end.timestamp() - start.timestamp()) / 3600.0,
            "recording_coverage_percent": rounded(
                safe_ratio(
                    accumulator.vehicle_power_on_s,
                    end.timestamp() - start.timestamp(),
                    100.0,
                ),
                3,
            ),
        },
    }


def csv_row(values: Record) -> Record:
    return dict(
        zip(
            CSV_FIELDS,
            [
                values["acquisition_date"],
                values["acquisition_time"],
                values["vehicle_power_on_time_s"],
                values["front_traction_operation_time_s"],
                values["rear_traction_operation_time_s"],
                values["front_distance_traveled_in_traction_m"],
                values["rear_distance_traveled_in_traction_m"],
                values["vehicle_power_on_hours_h"],
                values["power_on_time_delta_since_previous_acquisition_h"],
                values["front_traction_operation_hours_h"],
                values["rear_traction_operation_hours_h"],
                values["traction_time_delta_since_previous_acquisition_h"],
                values["front_average_traction_speed_mm_s"],
                values["rear_average_traction_speed_mm_s"],
                values["front_traction_to_power_on_ratio_percent"],
                values["rear_traction_to_power_on_ratio_percent"],
            ],
        )
    )


def write_outputs(summary: Record, output_prefix: Path) -> Tuple[Path, Path]:
    json_path = output_prefix.with_suffix(".json")
    csv_path = output_prefix.with_suffix(".csv")
    json_path.parent.mkdir(parents=True, exist_ok=True)
    with json_path.open("w", encoding="utf-8") as handle:
        json.dump(summary, handle, indent=2, ensure_ascii=False, allow_nan=False)
        handle.write("\n")
    with csv_path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=CSV_FIELDS)
        writer.writeheader()
        writer.writerow(csv_row(summary["values"]))
    return json_path, csv_path


def parse_args(argv: Optional[Sequence[str]] = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--start", required=True, help="Previous acquisition boundary")
    parser.add_argument("--end", required=True, help="Current acquisition boundary")
    parser.add_argument("--timezone", default=DEFAULT_TIMEZONE)
    parser.add_argument("--records-dir", type=Path, default=DEFAULT_RECORDS_DIR)
    parser.add_argument(
        "--max-sample-gap", type=float, default=DEFAULT_MAX_SAMPLE_GAP_S
    )
    parser.add_argument(
        "--traction-speed-threshold",
        type=float,
        default=DEFAULT_TRACTION_SPEED_THRESHOLD_MM_S,
        help=(
            "Count traction operation only above this absolute wheel speed in mm/s "
            "(default: 0, so exact stationary samples are excluded)"
        ),
    )
    parser.add_argument(
        "--output-prefix",
        type=Path,
        help="Output path without extension; defaults under can_telemetry_reports",
    )
    return parser.parse_args(argv)


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = parse_args(argv)
    timezone = ZoneInfo(args.timezone)
    start = parse_local_datetime(args.start, timezone)
    end = parse_local_datetime(args.end, timezone)
    output_prefix = args.output_prefix or (
        DEFAULT_OUTPUT_DIR / f"can_acquisition_summary_{end.strftime('%Y%m%d_%H%M%S')}"
    )
    summary = calculate_summary(
        args.records_dir,
        start,
        end,
        timezone,
        args.max_sample_gap,
        args.traction_speed_threshold,
    )
    json_path, csv_path = write_outputs(summary, output_prefix)
    print(json.dumps(summary["values"], indent=2, ensure_ascii=False))
    print(f"JSON: {json_path}")
    print(f"CSV:  {csv_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
