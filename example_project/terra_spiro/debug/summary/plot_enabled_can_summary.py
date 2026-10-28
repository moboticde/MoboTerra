#!/usr/bin/env python3
"""Create a PDF plot report from an enabled CAN telemetry summary JSON.

Run the extractor first:

    python3 src/terra_spiro/debug/summary/extract_enabled_can_summary.py

Then run this script:

    python3 src/terra_spiro/debug/summary/plot_enabled_can_summary.py
"""

import argparse
import json
import math
import os
from datetime import datetime
from pathlib import Path
from typing import Any, Dict, Iterable, Iterator, List, Optional, Sequence, Tuple


# =============================================================================
# CONFIG - CHANGE HERE
# =============================================================================

INPUT_SUMMARY_PATH = Path(
    "/home/mobotic/romb_ws_ec/can_telemetry_reports/"
    "enabled_can_telemetry_summary_reliability.json"
)

OUTPUT_PDF_PATH = Path(
    "/home/mobotic/romb_ws_ec/can_telemetry_reports/"
    "enabled_can_telemetry_summary_reliability.pdf"
)

# Options: "wall_clock" keeps calendar gaps, "enabled_time" collapses long
# gaps between recordings so plots show only measurement time.
TIME_AXIS_MODE = "enabled_time"

# Options: "seconds", "minutes", "hours"
TIME_UNIT = "hours"

# Used only when TIME_AXIS_MODE is "enabled_time". Gaps larger than this are
# treated as recording breaks and removed from the x axis.
MAX_CONTINUOUS_GAP_S = 2.0

# Options: "magnitude" plots selected bidirectional motor signals with one sign,
# "raw" keeps the signs exactly as recorded.
VALUE_MODE = "magnitude"

# Options: "minmax" plots min/average/max curves per time bin, "sample" plots
# decimated sample points directly.
PLOT_REDUCTION_MODE = "minmax"

# Used when PLOT_REDUCTION_MODE is "minmax".
PLOT_MINMAX_PERIOD_S = 10.0 * 60.0 * 60.0

# Cumulative distance is plotted separately as sampled points, not min/avg/max.
DISTANCE_PLOT_PERIOD_S = 10.0 * 60.0 * 60.0

# Show numeric labels at reduced datapoints. By default this labels the reduced
# average curve; add "min" or "max" here if you want those labeled too.
PLOT_POINT_VALUE_LABELS = True
PLOT_POINT_VALUE_LABEL_KINDS = ("avg",)
PLOT_POINT_VALUE_LABEL_FONT_SIZE = 7

# If True, connect curves across collapsed recording gaps. If False, insert
# visible line breaks at recording boundaries.
CONNECT_COLLAPSED_GAPS = True

# For velocity max curves only, ignore values below this magnitude. Set to None
# to allow low-speed values to become max points in low-speed bins.
MIN_VELOCITY_FOR_MAX_MM_S: Optional[float] = 0.0

# Similar optional max-curve filters for other plots. Keep disabled unless you
# have a concrete threshold for what is meaningful motor behavior.
MIN_CURRENT_FOR_MAX_MA: Optional[float] = 80000.0
MIN_MOTOR_TEMP_FOR_MAX_C: Optional[float] = 32.06

# Extra PDF page: current samples whose matching same-sample motor speed is in
# this interval. Averages are computed from all matching samples.
SPEED_FILTERED_CURRENT_MIN_MM_S = 1200.0
SPEED_FILTERED_CURRENT_MAX_MM_S = 1400.0

# Extra PDF page: one average-current point per time bin, using only samples
# whose same-sample motor speed is in SPEED_FILTERED_CURRENT_MIN/MAX_MM_S.
# The x axis for this page is always shown in hours.
SPEED_FILTERED_CURRENT_AVG_PERIOD_S = 1.0 * 60.0 * 60.0

# Set to None to keep all points. For long reports, 1-5 Hz is usually enough.
PLOT_SAMPLE_HZ: Optional[float] = 5.0

# =============================================================================


Record = Dict[str, Any]

PLOT_GROUPS = [
    (
        "Traction Motor Temperature [C]",
        ["ftr_motor_temp_c", "btr_motor_temp_c"],
    ),
    (
        "Traction Velocity [mm/s]",
        ["ftr_velocity_mm_s", "btr_velocity_mm_s"],
    ),
    (
        "Traction Actual Current [A]",
        ["ftr_current_actual_ma", "btr_current_actual_ma"],
    ),
]

DISTANCE_PLOT_GROUP = (
    "Traction Distance Traveled [km]",
    ["ftr_distance_traveled_m", "btr_distance_traveled_m"],
)

MAGNITUDE_FIELDS = {
    "ftr_velocity_mm_s",
    "btr_velocity_mm_s",
    "ftr_current_actual_ma",
    "btr_current_actual_ma",
}

VELOCITY_FIELDS = {
    "ftr_velocity_mm_s",
    "btr_velocity_mm_s",
}

CURRENT_FIELDS = {
    "ftr_current_actual_ma",
    "btr_current_actual_ma",
}

CURRENT_TO_SPEED_FIELD = {
    "ftr_current_actual_ma": "ftr_velocity_mm_s",
    "btr_current_actual_ma": "btr_velocity_mm_s",
}

MOTOR_TEMP_FIELDS = {
    "ftr_motor_temp_c",
    "btr_motor_temp_c",
}

DISTANCE_FIELDS = {
    "ftr_distance_traveled_m",
    "btr_distance_traveled_m",
}


def configure_matplotlib_cache() -> None:
    if "MPLCONFIGDIR" in os.environ:
        return

    cache_dir = Path("/tmp") / f"matplotlib-{os.getuid()}"
    cache_dir.mkdir(parents=True, exist_ok=True)
    os.environ["MPLCONFIGDIR"] = str(cache_dir)


def iter_summary_samples(path: Path) -> Iterator[Record]:
    """Yield sample objects from the generated summary JSON without loading all."""
    in_samples = False

    with path.open("r", encoding="utf-8") as handle:
        for line in handle:
            stripped = line.strip()

            if not in_samples:
                if stripped == '"samples":[':
                    in_samples = True
                continue

            if stripped == "]," or stripped == "]":
                return

            if not stripped:
                continue

            if stripped.endswith(","):
                stripped = stripped[:-1]

            record = json.loads(stripped)
            if isinstance(record, dict):
                yield record


def load_top_level_without_samples(path: Path) -> Record:
    """Read metadata/summary/files while replacing the large samples array."""
    text = path.read_text(encoding="utf-8")
    samples_key = '"samples":'
    start = text.find(samples_key)
    if start < 0:
        return json.loads(text)

    array_start = text.find("[", start + len(samples_key))
    if array_start < 0:
        return json.loads(text)

    depth = 0
    in_string = False
    escape = False
    array_end = None

    for index in range(array_start, len(text)):
        char = text[index]
        if in_string:
            if escape:
                escape = False
            elif char == "\\":
                escape = True
            elif char == '"':
                in_string = False
            continue

        if char == '"':
            in_string = True
        elif char == "[":
            depth += 1
        elif char == "]":
            depth -= 1
            if depth == 0:
                array_end = index
                break

    if array_end is None:
        return json.loads(text)

    compact_text = text[:array_start] + "[]" + text[array_end + 1:]
    return json.loads(compact_text)


def numeric_value(record: Record, key: str) -> Optional[float]:
    value = record.get(key)
    if value is None:
        return None

    try:
        number = float(value)
    except (TypeError, ValueError):
        return None

    if math.isnan(number) or math.isinf(number):
        return None

    return number


def report_value(record: Record, key: str, value_mode: str) -> Optional[float]:
    value = numeric_value(record, key)
    if value is None:
        return None

    if value_mode == "raw":
        return value
    if value_mode == "magnitude":
        return abs(value) if key in MAGNITUDE_FIELDS else value

    raise ValueError(f"Unsupported VALUE_MODE: {value_mode}")


def display_values(key: str, values: Sequence[float]) -> List[float]:
    if key in CURRENT_FIELDS or key in DISTANCE_FIELDS:
        return [value / 1000.0 for value in values]

    return list(values)


def display_value(key: str, value: float) -> float:
    if key in CURRENT_FIELDS or key in DISTANCE_FIELDS:
        return value / 1000.0

    return value


def point_value_label(key: str, value: float) -> str:
    displayed = display_value(key, value)
    if key in CURRENT_FIELDS:
        return f"{displayed:.1f}"
    if key in DISTANCE_FIELDS:
        return f"{displayed:.3f}"
    if key in MOTOR_TEMP_FIELDS:
        return f"{displayed:.1f}"

    return f"{displayed:.0f}"


def annotate_point_values(
    axis: Any,
    key: str,
    kind: str,
    x_values: Sequence[float],
    y_values: Sequence[float],
    font_size: int,
    color: Optional[str] = None,
) -> None:
    x_offset = -6 if key.startswith("btr_") else 6
    y_offset = -11 if key.startswith("btr_") and kind == "avg" else 8
    if kind == "min":
        y_offset = -10
    elif kind == "max":
        y_offset = 10

    for x_value, y_value in zip(x_values, y_values):
        if math.isnan(x_value) or math.isnan(y_value):
            continue

        axis.annotate(
            point_value_label(key, y_value),
            xy=(x_value, display_value(key, y_value)),
            xytext=(x_offset, y_offset),
            textcoords="offset points",
            ha="center",
            va="bottom" if y_offset >= 0 else "top",
            fontsize=font_size,
            color=color,
        )


def eligible_for_max_curve(
    key: str,
    value: float,
    min_velocity_for_max_mm_s: Optional[float],
    min_current_for_max_ma: Optional[float],
    min_motor_temp_for_max_c: Optional[float],
) -> bool:
    if key in VELOCITY_FIELDS:
        return min_velocity_for_max_mm_s is None or abs(value) >= min_velocity_for_max_mm_s
    if key in CURRENT_FIELDS:
        return min_current_for_max_ma is None or abs(value) >= min_current_for_max_ma
    if key in MOTOR_TEMP_FIELDS:
        return min_motor_temp_for_max_c is None or value >= min_motor_temp_for_max_c

    return True


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


def time_scale(unit: str) -> Tuple[float, str]:
    if unit == "seconds":
        return 1.0, "time [s]"
    if unit == "minutes":
        return 1.0 / 60.0, "time [min]"
    if unit == "hours":
        return 1.0 / 3600.0, "time [h]"

    raise ValueError(f"Unsupported TIME_UNIT: {unit}")


def collect_plot_series(
    samples: Iterable[Record],
    fields: Sequence[str],
    sample_hz: Optional[float],
    reduction_mode: str,
    minmax_period_s: float,
    axis_mode: str,
    value_mode: str,
    unit: str,
    max_continuous_gap_s: float,
    connect_collapsed_gaps: bool,
    min_velocity_for_max_mm_s: Optional[float],
    min_current_for_max_ma: Optional[float],
    min_motor_temp_for_max_c: Optional[float],
) -> Tuple[Dict[str, List[float]], Dict[str, List[float]], int, int, int]:
    if reduction_mode == "minmax":
        series_keys = [
            f"{field}_{kind}"
            for field in fields
            for kind in ("min", "avg", "max")
        ]
    else:
        series_keys = list(fields)

    x_values = {key: [] for key in series_keys}
    y_values = {key: [] for key in series_keys}
    last_kept_time: Optional[float] = None
    last_raw_time: Optional[float] = None
    first_time: Optional[float] = None
    enabled_time = 0.0
    scanned_samples = 0
    skipped_values = 0
    collapsed_gaps = 0
    min_period = None
    bin_start: Optional[float] = None
    bin_min: Dict[str, Optional[float]] = {key: None for key in fields}
    bin_sum: Dict[str, float] = {key: 0.0 for key in fields}
    bin_count: Dict[str, int] = {key: 0 for key in fields}
    bin_max: Dict[str, Optional[float]] = {key: None for key in fields}

    if axis_mode not in {"wall_clock", "enabled_time"}:
        raise ValueError(f"Unsupported TIME_AXIS_MODE: {axis_mode}")
    if value_mode not in {"raw", "magnitude"}:
        raise ValueError(f"Unsupported VALUE_MODE: {value_mode}")
    if reduction_mode not in {"sample", "minmax"}:
        raise ValueError(f"Unsupported PLOT_REDUCTION_MODE: {reduction_mode}")

    if reduction_mode == "minmax":
        if minmax_period_s <= 0.0:
            raise ValueError("PLOT_MINMAX_PERIOD_S must be positive")
    elif sample_hz is not None:
        if sample_hz <= 0.0:
            raise ValueError("PLOT_SAMPLE_HZ must be positive, or None")
        min_period = 1.0 / sample_hz

    scale, _label = time_scale(unit)

    def flush_minmax_bin() -> None:
        nonlocal bin_start, bin_min, bin_sum, bin_count, bin_max
        if bin_start is None:
            return

        scaled_time = (bin_start + minmax_period_s / 2.0) * scale
        for field in fields:
            min_value = bin_min[field]
            count = bin_count[field]
            max_value = bin_max[field]

            min_key = f"{field}_min"
            avg_key = f"{field}_avg"
            max_key = f"{field}_max"
            if min_value is not None:
                x_values[min_key].append(scaled_time)
                y_values[min_key].append(min_value)
            if count > 0:
                x_values[avg_key].append(scaled_time)
                y_values[avg_key].append(bin_sum[field] / count)
            if max_value is not None:
                x_values[max_key].append(scaled_time)
                y_values[max_key].append(max_value)

        bin_start = None
        bin_min = {key: None for key in fields}
        bin_sum = {key: 0.0 for key in fields}
        bin_count = {key: 0 for key in fields}
        bin_max = {key: None for key in fields}

    def append_gap_marker() -> None:
        for key in series_keys:
            if x_values[key]:
                x_values[key].append(float("nan"))
                y_values[key].append(float("nan"))

    for sample in samples:
        scanned_samples += 1
        raw_time = timestamp_seconds(sample.get("timestamp"))
        if raw_time is None:
            skipped_values += len(fields)
            continue

        if first_time is None:
            first_time = raw_time

        if axis_mode == "wall_clock":
            relative_time = raw_time - first_time
        else:
            gap_was_collapsed = False
            if last_raw_time is not None:
                dt = raw_time - last_raw_time
                if 0.0 < dt <= max_continuous_gap_s:
                    enabled_time += dt
                elif dt > max_continuous_gap_s:
                    collapsed_gaps += 1
                    gap_was_collapsed = True
                    last_kept_time = None

            if gap_was_collapsed and not connect_collapsed_gaps:
                if reduction_mode == "minmax":
                    flush_minmax_bin()
                append_gap_marker()

            relative_time = enabled_time

        last_raw_time = raw_time

        if reduction_mode == "minmax":
            current_bin_start = math.floor(relative_time / minmax_period_s) * minmax_period_s
            if bin_start is None:
                bin_start = current_bin_start
            elif current_bin_start != bin_start:
                flush_minmax_bin()
                bin_start = current_bin_start

            for key in fields:
                value = report_value(sample, key, value_mode)
                if value is None:
                    skipped_values += 1
                    continue

                bin_min[key] = value if bin_min[key] is None else min(bin_min[key], value)
                bin_sum[key] += value
                bin_count[key] += 1
                allow_as_max = eligible_for_max_curve(
                    key=key,
                    value=value,
                    min_velocity_for_max_mm_s=min_velocity_for_max_mm_s,
                    min_current_for_max_ma=min_current_for_max_ma,
                    min_motor_temp_for_max_c=min_motor_temp_for_max_c,
                )
                if allow_as_max:
                    bin_max[key] = value if bin_max[key] is None else max(bin_max[key], value)

            continue

        if (
            min_period is not None
            and last_kept_time is not None
            and relative_time - last_kept_time < min_period
        ):
            continue

        kept_any_value = False
        scaled_time = relative_time * scale
        for key in fields:
            value = report_value(sample, key, value_mode)
            if value is None:
                skipped_values += 1
                continue

            x_values[key].append(scaled_time)
            y_values[key].append(value)
            kept_any_value = True

        if kept_any_value:
            last_kept_time = relative_time

    if reduction_mode == "minmax":
        flush_minmax_bin()

    return x_values, y_values, scanned_samples, skipped_values, collapsed_gaps


def collect_periodic_distance_series(
    samples: Iterable[Record],
    fields: Sequence[str],
    period_s: float,
    axis_mode: str,
    unit: str,
    max_continuous_gap_s: float,
    connect_collapsed_gaps: bool,
) -> Tuple[Dict[str, List[float]], Dict[str, List[float]], int, int, int]:
    if period_s <= 0.0:
        raise ValueError("DISTANCE_PLOT_PERIOD_S must be positive")
    if axis_mode not in {"wall_clock", "enabled_time"}:
        raise ValueError(f"Unsupported TIME_AXIS_MODE: {axis_mode}")

    x_values = {key: [] for key in fields}
    y_values = {key: [] for key in fields}
    first_time: Optional[float] = None
    last_raw_time: Optional[float] = None
    previous_relative_time: Optional[float] = None
    previous_values: Dict[str, Optional[float]] = {key: None for key in fields}
    next_point_time = 0.0
    enabled_time = 0.0
    scanned_samples = 0
    skipped_values = 0
    collapsed_gaps = 0
    scale, _label = time_scale(unit)

    def append_gap_marker() -> None:
        for key in fields:
            if x_values[key]:
                x_values[key].append(float("nan"))
                y_values[key].append(float("nan"))

    def append_point(target_time: float, values: Dict[str, Optional[float]]) -> None:
        nonlocal skipped_values
        scaled_time = target_time * scale
        for key in fields:
            value = values[key]
            if value is None:
                skipped_values += 1
                continue

            x_values[key].append(scaled_time)
            y_values[key].append(value)

    for sample in samples:
        scanned_samples += 1
        raw_time = timestamp_seconds(sample.get("timestamp"))
        if raw_time is None:
            skipped_values += len(fields)
            continue

        if first_time is None:
            first_time = raw_time

        if axis_mode == "wall_clock":
            relative_time = raw_time - first_time
        else:
            gap_was_collapsed = False
            if last_raw_time is not None:
                dt = raw_time - last_raw_time
                if 0.0 < dt <= max_continuous_gap_s:
                    enabled_time += dt
                elif dt > max_continuous_gap_s:
                    collapsed_gaps += 1
                    gap_was_collapsed = True

            if gap_was_collapsed and not connect_collapsed_gaps:
                append_gap_marker()
                previous_relative_time = None

            relative_time = enabled_time

        last_raw_time = raw_time
        current_values = {key: numeric_value(sample, key) for key in fields}

        if previous_relative_time is None:
            while next_point_time <= relative_time:
                append_point(next_point_time, current_values)
                next_point_time += period_s

            previous_relative_time = relative_time
            previous_values = current_values
            continue

        if relative_time < previous_relative_time:
            previous_relative_time = relative_time
            previous_values = current_values
            continue

        while next_point_time <= relative_time:
            if next_point_time < previous_relative_time:
                next_point_time += period_s
                continue

            if relative_time == previous_relative_time:
                interpolated_values = current_values
            else:
                fraction = (
                    (next_point_time - previous_relative_time)
                    / (relative_time - previous_relative_time)
                )
                interpolated_values = {}
                for key in fields:
                    previous_value = previous_values[key]
                    current_value = current_values[key]
                    if previous_value is None or current_value is None:
                        interpolated_values[key] = None
                    else:
                        interpolated_values[key] = (
                            previous_value
                            + (current_value - previous_value) * fraction
                        )

            append_point(next_point_time, interpolated_values)
            next_point_time += period_s

        previous_relative_time = relative_time
        previous_values = current_values

    return x_values, y_values, scanned_samples, skipped_values, collapsed_gaps


def collect_speed_filtered_current_series(
    samples: Iterable[Record],
    speed_min_mm_s: float,
    speed_max_mm_s: float,
    sample_hz: Optional[float],
    axis_mode: str,
    value_mode: str,
    unit: str,
    max_continuous_gap_s: float,
    connect_collapsed_gaps: bool,
) -> Tuple[
    Dict[str, List[float]],
    Dict[str, List[float]],
    Dict[str, Optional[float]],
    Dict[str, int],
    int,
    int,
    int,
]:
    if speed_min_mm_s > speed_max_mm_s:
        raise ValueError("Speed-filtered current min speed must be <= max speed")
    if axis_mode not in {"wall_clock", "enabled_time"}:
        raise ValueError(f"Unsupported TIME_AXIS_MODE: {axis_mode}")
    if value_mode not in {"raw", "magnitude"}:
        raise ValueError(f"Unsupported VALUE_MODE: {value_mode}")
    if sample_hz is not None and sample_hz <= 0.0:
        raise ValueError("PLOT_SAMPLE_HZ must be positive, or None")

    current_fields = list(CURRENT_TO_SPEED_FIELD)
    x_values = {key: [] for key in current_fields}
    y_values = {key: [] for key in current_fields}
    sums = {key: 0.0 for key in current_fields}
    counts = {key: 0 for key in current_fields}
    last_kept_time: Dict[str, Optional[float]] = {key: None for key in current_fields}
    min_period = None if sample_hz is None else 1.0 / sample_hz
    first_time: Optional[float] = None
    last_raw_time: Optional[float] = None
    enabled_time = 0.0
    scanned_samples = 0
    skipped_values = 0
    collapsed_gaps = 0
    scale, _label = time_scale(unit)

    def append_gap_marker() -> None:
        for key in current_fields:
            if x_values[key]:
                x_values[key].append(float("nan"))
                y_values[key].append(float("nan"))

    def speed_in_range(value: Optional[float]) -> bool:
        return value is not None and speed_min_mm_s <= value <= speed_max_mm_s

    for sample in samples:
        scanned_samples += 1
        raw_time = timestamp_seconds(sample.get("timestamp"))
        if raw_time is None:
            skipped_values += len(current_fields)
            continue

        if first_time is None:
            first_time = raw_time

        if axis_mode == "wall_clock":
            relative_time = raw_time - first_time
        else:
            gap_was_collapsed = False
            if last_raw_time is not None:
                dt = raw_time - last_raw_time
                if 0.0 < dt <= max_continuous_gap_s:
                    enabled_time += dt
                elif dt > max_continuous_gap_s:
                    collapsed_gaps += 1
                    gap_was_collapsed = True
                    last_kept_time = {key: None for key in current_fields}

            if gap_was_collapsed and not connect_collapsed_gaps:
                append_gap_marker()

            relative_time = enabled_time

        last_raw_time = raw_time
        scaled_time = relative_time * scale

        for current_key, speed_key in CURRENT_TO_SPEED_FIELD.items():
            speed = report_value(sample, speed_key, value_mode)
            current = report_value(sample, current_key, value_mode)
            if speed is None or current is None:
                skipped_values += 1
                continue
            if not speed_in_range(speed):
                continue

            sums[current_key] += current
            counts[current_key] += 1

            previous_kept_time = last_kept_time[current_key]
            if (
                min_period is not None
                and previous_kept_time is not None
                and relative_time - previous_kept_time < min_period
            ):
                continue

            x_values[current_key].append(scaled_time)
            y_values[current_key].append(current)
            last_kept_time[current_key] = relative_time

    averages = {
        key: (sums[key] / counts[key] if counts[key] else None)
        for key in current_fields
    }

    return (
        x_values,
        y_values,
        averages,
        counts,
        scanned_samples,
        skipped_values,
        collapsed_gaps,
    )


def collect_speed_filtered_current_average_by_time_series(
    samples: Iterable[Record],
    speed_min_mm_s: float,
    speed_max_mm_s: float,
    period_s: float,
    axis_mode: str,
    value_mode: str,
    max_continuous_gap_s: float,
) -> Tuple[
    Dict[str, List[float]],
    Dict[str, List[float]],
    Dict[str, List[int]],
    int,
    int,
    int,
]:
    """Average current by elapsed time, filtered by same-sample speed.

    The x axis is always returned in hours. With TIME_AXIS_MODE="enabled_time",
    long recording gaps are collapsed before binning. With TIME_AXIS_MODE="wall_clock",
    calendar gaps are kept.
    """
    if speed_min_mm_s > speed_max_mm_s:
        raise ValueError("Speed-filtered current min speed must be <= max speed")
    if period_s <= 0.0:
        raise ValueError("SPEED_FILTERED_CURRENT_AVG_PERIOD_S must be positive")
    if axis_mode not in {"wall_clock", "enabled_time"}:
        raise ValueError(f"Unsupported TIME_AXIS_MODE: {axis_mode}")
    if value_mode not in {"raw", "magnitude"}:
        raise ValueError(f"Unsupported VALUE_MODE: {value_mode}")

    current_fields = list(CURRENT_TO_SPEED_FIELD)
    bin_sums: Dict[str, Dict[int, float]] = {key: {} for key in current_fields}
    bin_counts: Dict[str, Dict[int, int]] = {key: {} for key in current_fields}

    first_time: Optional[float] = None
    last_raw_time: Optional[float] = None
    enabled_time = 0.0
    scanned_samples = 0
    skipped_values = 0
    collapsed_gaps = 0

    def speed_in_range(value: Optional[float]) -> bool:
        return value is not None and speed_min_mm_s <= value <= speed_max_mm_s

    for sample in samples:
        scanned_samples += 1
        raw_time = timestamp_seconds(sample.get("timestamp"))
        if raw_time is None:
            skipped_values += len(current_fields)
            continue

        if first_time is None:
            first_time = raw_time

        if axis_mode == "wall_clock":
            relative_time = raw_time - first_time
        else:
            if last_raw_time is not None:
                dt = raw_time - last_raw_time
                if 0.0 < dt <= max_continuous_gap_s:
                    enabled_time += dt
                elif dt > max_continuous_gap_s:
                    collapsed_gaps += 1
            relative_time = enabled_time

        last_raw_time = raw_time
        bin_index = int(math.floor(relative_time / period_s))

        for current_key, speed_key in CURRENT_TO_SPEED_FIELD.items():
            speed = report_value(sample, speed_key, value_mode)
            current = report_value(sample, current_key, value_mode)
            if speed is None or current is None:
                skipped_values += 1
                continue
            if not speed_in_range(speed):
                continue

            bin_sums[current_key][bin_index] = (
                bin_sums[current_key].get(bin_index, 0.0) + current
            )
            bin_counts[current_key][bin_index] = (
                bin_counts[current_key].get(bin_index, 0) + 1
            )

    x_values: Dict[str, List[float]] = {key: [] for key in current_fields}
    y_values: Dict[str, List[float]] = {key: [] for key in current_fields}
    counts: Dict[str, List[int]] = {key: [] for key in current_fields}

    for key in current_fields:
        for bin_index in sorted(bin_counts[key]):
            count = bin_counts[key][bin_index]
            if count <= 0:
                continue
            x_hour = ((bin_index * period_s) + period_s / 2.0) / 3600.0
            average_current = bin_sums[key][bin_index] / count
            x_values[key].append(x_hour)
            y_values[key].append(average_current)
            counts[key].append(count)

    return (
        x_values,
        y_values,
        counts,
        scanned_samples,
        skipped_values,
        collapsed_gaps,
    )


def summary_lines(
    input_path: Path,
    sample_hz: Optional[float],
    reduction_mode: str,
    minmax_period_s: float,
    connect_collapsed_gaps: bool,
    axis_mode: str,
    value_mode: str,
    max_continuous_gap_s: float,
    data: Record,
    scanned_samples: int,
    skipped_values: int,
    collapsed_gaps: int,
    min_velocity_for_max_mm_s: Optional[float],
    min_current_for_max_ma: Optional[float],
    min_motor_temp_for_max_c: Optional[float],
) -> List[str]:
    metadata = data.get("metadata", {})
    summary = data.get("summary", {})
    start_file = metadata.get("start_file", metadata.get("first_file", "unknown"))
    end_file = metadata.get("end_file", metadata.get("last_file", "unknown"))
    file_count = metadata.get("included_file_count", metadata.get("file_count", "unknown"))
    lines = [
        f"Input summary: {input_path}",
        f"Start file: {start_file}",
        f"End file: {end_file}",
        f"Included files: {file_count}",
        f"Created by extractor: {metadata.get('created_at', 'unknown')}",
        f"Records scanned by extractor: {summary.get('total_records_scanned', 'unknown')}",
        f"Enabled samples in summary: {summary.get('total_enabled_samples', 'unknown')}",
        f"Samples scanned for plots: {scanned_samples}",
        f"Time axis mode: {axis_mode}",
        f"Value mode: {value_mode}",
        f"Plot reduction mode: {reduction_mode}",
        f"Connect collapsed gaps: {connect_collapsed_gaps}",
        f"Min velocity eligible for max curve: {min_velocity_for_max_mm_s if min_velocity_for_max_mm_s is not None else 'disabled'} mm/s",
        f"Min current eligible for max curve: {min_current_for_max_ma if min_current_for_max_ma is not None else 'disabled'} mA",
        f"Min motor temp eligible for max curve: {min_motor_temp_for_max_c if min_motor_temp_for_max_c is not None else 'disabled'} C",
        f"Skipped null/non-numeric plot values: {skipped_values}",
    ]

    if reduction_mode == "minmax":
        lines.append(f"Min/average/max bin period: {minmax_period_s:g}s")
    else:
        lines.append(f"Plot sample rate: {sample_hz if sample_hz is not None else 'all'} Hz")

    if axis_mode == "enabled_time":
        lines.append(f"Collapsed gaps > {max_continuous_gap_s:g}s: {collapsed_gaps}")

    incomplete_files = summary.get("files_without_closing_array_bracket") or []
    if incomplete_files:
        lines.append("Incomplete JSON source files: " + ", ".join(incomplete_files))

    return lines


def plot_file_bar_chart(axis: Any, files: Sequence[Record]) -> None:
    top_files = sorted(
        files,
        key=lambda item: int(item.get("enabled_samples") or 0),
        reverse=True,
    )[:20]
    top_files.reverse()

    names = [str(item.get("file", "unknown")).removeprefix("can_telemetry_") for item in top_files]
    values = [int(item.get("enabled_samples") or 0) for item in top_files]

    axis.barh(names, values, color="#3b82a0")
    axis.set_title("Top Files By Enabled Samples")
    axis.set_xlabel("enabled samples")
    axis.grid(True, axis="x", alpha=0.35)


def plot_report(
    input_path: Path,
    output_path: Path,
    axis_mode: str,
    value_mode: str,
    reduction_mode: str,
    minmax_period_s: float,
    distance_period_s: float,
    speed_filtered_current_avg_period_s: float,
    connect_collapsed_gaps: bool,
    time_unit: str,
    sample_hz: Optional[float],
    max_continuous_gap_s: float,
    min_velocity_for_max_mm_s: Optional[float],
    min_current_for_max_ma: Optional[float],
    min_motor_temp_for_max_c: Optional[float],
    point_value_labels: bool = PLOT_POINT_VALUE_LABELS,
    point_value_label_kinds: Sequence[str] = PLOT_POINT_VALUE_LABEL_KINDS,
    point_value_label_font_size: int = PLOT_POINT_VALUE_LABEL_FONT_SIZE,
) -> Record:
    configure_matplotlib_cache()

    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.backends.backend_pdf import PdfPages

    fields = [field for _title, group_fields in PLOT_GROUPS for field in group_fields]
    (
        x_values,
        y_values,
        scanned_samples,
        skipped_values,
        collapsed_gaps,
    ) = collect_plot_series(
        samples=iter_summary_samples(input_path),
        fields=fields,
        sample_hz=sample_hz,
        reduction_mode=reduction_mode,
        minmax_period_s=minmax_period_s,
        axis_mode=axis_mode,
        value_mode=value_mode,
        unit=time_unit,
        max_continuous_gap_s=max_continuous_gap_s,
        connect_collapsed_gaps=connect_collapsed_gaps,
        min_velocity_for_max_mm_s=min_velocity_for_max_mm_s,
        min_current_for_max_ma=min_current_for_max_ma,
        min_motor_temp_for_max_c=min_motor_temp_for_max_c,
    )
    distance_title, distance_fields = DISTANCE_PLOT_GROUP
    (
        distance_x_values,
        distance_y_values,
        distance_scanned_samples,
        distance_skipped_values,
        distance_collapsed_gaps,
    ) = collect_periodic_distance_series(
        samples=iter_summary_samples(input_path),
        fields=distance_fields,
        period_s=distance_period_s,
        axis_mode=axis_mode,
        unit=time_unit,
        max_continuous_gap_s=max_continuous_gap_s,
        connect_collapsed_gaps=connect_collapsed_gaps,
    )
    (
        speed_current_x_values,
        speed_current_y_values,
        speed_current_averages,
        speed_current_counts,
        speed_current_scanned_samples,
        speed_current_skipped_values,
        speed_current_collapsed_gaps,
    ) = collect_speed_filtered_current_series(
        samples=iter_summary_samples(input_path),
        speed_min_mm_s=SPEED_FILTERED_CURRENT_MIN_MM_S,
        speed_max_mm_s=SPEED_FILTERED_CURRENT_MAX_MM_S,
        sample_hz=sample_hz,
        axis_mode=axis_mode,
        value_mode=value_mode,
        unit=time_unit,
        max_continuous_gap_s=max_continuous_gap_s,
        connect_collapsed_gaps=connect_collapsed_gaps,
    )
    (
        speed_current_avg_x_values,
        speed_current_avg_y_values,
        speed_current_avg_counts,
        speed_current_avg_scanned_samples,
        speed_current_avg_skipped_values,
        speed_current_avg_collapsed_gaps,
    ) = collect_speed_filtered_current_average_by_time_series(
        samples=iter_summary_samples(input_path),
        speed_min_mm_s=SPEED_FILTERED_CURRENT_MIN_MM_S,
        speed_max_mm_s=SPEED_FILTERED_CURRENT_MAX_MM_S,
        period_s=speed_filtered_current_avg_period_s,
        axis_mode=axis_mode,
        value_mode=value_mode,
        max_continuous_gap_s=max_continuous_gap_s,
    )
    _scale, x_label = time_scale(time_unit)

    output_path.parent.mkdir(parents=True, exist_ok=True)

    with PdfPages(output_path) as pdf:
        for title, group_fields in PLOT_GROUPS:
            figure, axis = plt.subplots(1, 1, figsize=(11.69, 8.27))
            plotted = False
            for key in group_fields:
                if reduction_mode == "minmax":
                    min_key = f"{key}_min"
                    avg_key = f"{key}_avg"
                    max_key = f"{key}_max"
                    if y_values[min_key]:
                        (line,) = axis.plot(
                            x_values[min_key],
                            display_values(key, y_values[min_key]),
                            label=f"{key} min",
                            linewidth=1.0,
                            linestyle="--",
                        )
                        if point_value_labels and "min" in point_value_label_kinds:
                            annotate_point_values(
                                axis,
                                key,
                                "min",
                                x_values[min_key],
                                y_values[min_key],
                                point_value_label_font_size,
                                line.get_color(),
                            )
                        plotted = True
                    if y_values[avg_key]:
                        (line,) = axis.plot(
                            x_values[avg_key],
                            display_values(key, y_values[avg_key]),
                            label=f"{key} avg",
                            linewidth=1.4,
                            marker="o",
                            markersize=3.5,
                        )
                        if point_value_labels and "avg" in point_value_label_kinds:
                            annotate_point_values(
                                axis,
                                key,
                                "avg",
                                x_values[avg_key],
                                y_values[avg_key],
                                point_value_label_font_size,
                                line.get_color(),
                            )
                        plotted = True
                    if y_values[max_key]:
                        (line,) = axis.plot(
                            x_values[max_key],
                            display_values(key, y_values[max_key]),
                            label=f"{key} max",
                            linewidth=1.1,
                        )
                        if point_value_labels and "max" in point_value_label_kinds:
                            annotate_point_values(
                                axis,
                                key,
                                "max",
                                x_values[max_key],
                                y_values[max_key],
                                point_value_label_font_size,
                                line.get_color(),
                            )
                        plotted = True
                else:
                    if not y_values[key]:
                        continue
                    axis.plot(
                        x_values[key],
                        display_values(key, y_values[key]),
                        label=key,
                        linewidth=1.1,
                    )
                    plotted = True

            if not plotted:
                plt.close(figure)
                continue

            axis.set_title(title)
            axis.set_xlabel(x_label)
            if all(key in CURRENT_FIELDS for key in group_fields):
                axis.set_ylabel("current [A]")
                axis.set_ylim(0.0, 170.0)
            else:
                axis.set_ylabel("magnitude" if value_mode == "magnitude" else "value")
            axis.grid(True, alpha=0.35)
            legend_location = (
                "upper right"
                if title == "Traction Velocity [mm/s]"
                else "best"
            )
            axis.legend(loc=legend_location)
            figure.tight_layout()
            pdf.savefig(figure)
            plt.close(figure)

        figure, axis = plt.subplots(1, 1, figsize=(11.69, 8.27))
        plotted = False
        for key in distance_fields:
            if not distance_y_values[key]:
                continue

            (line,) = axis.plot(
                distance_x_values[key],
                display_values(key, distance_y_values[key]),
                label=key,
                linewidth=1.3,
                marker="o",
                markersize=3.5,
            )
            if point_value_labels:
                annotate_point_values(
                    axis,
                    key,
                    "avg",
                    distance_x_values[key],
                    distance_y_values[key],
                    point_value_label_font_size,
                    line.get_color(),
                )
            plotted = True

        if plotted:
            axis.set_title(distance_title)
            axis.set_xlabel(x_label)
            axis.set_ylabel("distance [km]")
            axis.grid(True, alpha=0.35)
            axis.legend(loc="best")
            figure.tight_layout()
            pdf.savefig(figure)
        plt.close(figure)

        figure, axis = plt.subplots(1, 1, figsize=(11.69, 8.27))
        plotted = False
        for key in CURRENT_TO_SPEED_FIELD:
            values = speed_current_y_values[key]
            if not values:
                continue

            (line,) = axis.plot(
                speed_current_x_values[key],
                display_values(key, values),
                label=f"{key} samples",
                linewidth=0.9,
                marker=".",
                markersize=2.0,
            )
            average = speed_current_averages[key]
            count = speed_current_counts[key]
            if average is not None:
                axis.axhline(
                    display_value(key, average),
                    color=line.get_color(),
                    linestyle="--",
                    linewidth=1.2,
                    label=(
                        f"{key} overall avg {display_value(key, average):.3f} A "
                        f"(n={count})"
                    ),
                )
            plotted = True

        if plotted:
            axis.set_title(
                "Traction Actual Current Samples at "
                f"{SPEED_FILTERED_CURRENT_MIN_MM_S:g}-"
                f"{SPEED_FILTERED_CURRENT_MAX_MM_S:g} mm/s"
            )
            axis.set_xlabel(x_label)
            axis.set_ylabel("current [A]")
            axis.set_ylim(0.0, 170.0)
            axis.grid(True, alpha=0.35)
            axis.legend(loc="best")
            figure.tight_layout()
            pdf.savefig(figure)
        plt.close(figure)

        figure, axis = plt.subplots(1, 1, figsize=(11.69, 8.27))
        plotted = False
        for key in CURRENT_TO_SPEED_FIELD:
            values = speed_current_avg_y_values[key]
            if not values:
                continue

            axis.plot(
                speed_current_avg_x_values[key],
                display_values(key, values),
                label=f"{key} average per time bin",
                linewidth=1.8,
                marker="o",
                markersize=3.5,
            )
            plotted = True

        if plotted:
            axis.set_title(
                "Average Traction Current Over Time at "
                f"{SPEED_FILTERED_CURRENT_MIN_MM_S:g}-"
                f"{SPEED_FILTERED_CURRENT_MAX_MM_S:g} mm/s"
            )
            axis.set_xlabel(
                "active time [h]"
                if axis_mode == "enabled_time"
                else "wall-clock time [h]"
            )
            axis.set_ylabel("average current [A]")
            axis.set_ylim(0.0, 170.0)
            axis.grid(True, alpha=0.35)
            axis.legend(loc="best")
            figure.tight_layout()
            pdf.savefig(figure)
        plt.close(figure)

    return {
        "output_path": str(output_path),
        "scanned_samples": scanned_samples,
        "skipped_values": skipped_values,
        "collapsed_gaps": collapsed_gaps,
        "distance_scanned_samples": distance_scanned_samples,
        "distance_skipped_values": distance_skipped_values,
        "distance_collapsed_gaps": distance_collapsed_gaps,
        "speed_filtered_current_scanned_samples": speed_current_scanned_samples,
        "speed_filtered_current_skipped_values": speed_current_skipped_values,
        "speed_filtered_current_collapsed_gaps": speed_current_collapsed_gaps,
        "speed_filtered_current_avg_scanned_samples": speed_current_avg_scanned_samples,
        "speed_filtered_current_avg_skipped_values": speed_current_avg_skipped_values,
        "speed_filtered_current_avg_collapsed_gaps": speed_current_avg_collapsed_gaps,
        "speed_filtered_current_avg_period_s": speed_filtered_current_avg_period_s,
        "speed_filtered_current_avg_counts": speed_current_avg_counts,
        "speed_filtered_current_min_mm_s": SPEED_FILTERED_CURRENT_MIN_MM_S,
        "speed_filtered_current_max_mm_s": SPEED_FILTERED_CURRENT_MAX_MM_S,
        "speed_filtered_current_counts": speed_current_counts,
        "speed_filtered_current_averages_a": {
            key: (display_value(key, value) if value is not None else None)
            for key, value in speed_current_averages.items()
        },
        "reduction_mode": reduction_mode,
        "minmax_period_s": minmax_period_s,
        "distance_period_s": distance_period_s,
        "min_velocity_for_max_mm_s": min_velocity_for_max_mm_s,
        "min_current_for_max_ma": min_current_for_max_ma,
        "min_motor_temp_for_max_c": min_motor_temp_for_max_c,
        "connect_collapsed_gaps": connect_collapsed_gaps,
        "plotted_points": (
            sum(len(values) for values in y_values.values())
            + sum(len(values) for values in distance_y_values.values())
            + sum(len(values) for values in speed_current_y_values.values())
            + sum(len(values) for values in speed_current_avg_y_values.values())
        ),
    }


def parse_args(argv: Optional[Sequence[str]] = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Plot a PDF report from enabled CAN telemetry summary JSON."
    )
    parser.add_argument("--input", type=Path, default=INPUT_SUMMARY_PATH)
    parser.add_argument("--output", type=Path, default=OUTPUT_PDF_PATH)
    parser.add_argument(
        "--time-axis",
        choices=("wall_clock", "enabled_time"),
        default=TIME_AXIS_MODE,
        help="Use wall_clock for real calendar gaps or enabled_time to collapse recording breaks.",
    )
    parser.add_argument(
        "--time-unit",
        choices=("seconds", "minutes", "hours"),
        default=TIME_UNIT,
    )
    parser.add_argument(
        "--value-mode",
        choices=("raw", "magnitude"),
        default=VALUE_MODE,
        help="Use magnitude to plot velocity/current with one sign, or raw to keep recorded signs.",
    )
    parser.add_argument(
        "--reduction-mode",
        choices=("sample", "minmax"),
        default=PLOT_REDUCTION_MODE,
        help="Use minmax for envelope curves per bin, or sample for decimated raw samples.",
    )
    parser.add_argument(
        "--minmax-period-s",
        type=float,
        default=PLOT_MINMAX_PERIOD_S,
        help="Time bin size in seconds for min/average/max reduction.",
    )
    parser.add_argument(
        "--distance-period-s",
        type=float,
        default=DISTANCE_PLOT_PERIOD_S,
        help="Time spacing in seconds for cumulative distance datapoints.",
    )
    parser.add_argument(
        "--speed-filtered-current-avg-period-s",
        type=float,
        default=SPEED_FILTERED_CURRENT_AVG_PERIOD_S,
        help=(
            "Time bin size in seconds for the average-current-over-time plot "
            "inside the configured speed interval."
        ),
    )
    parser.add_argument(
        "--connect-gaps",
        action=argparse.BooleanOptionalAction,
        default=CONNECT_COLLAPSED_GAPS,
        help="Connect curves across collapsed recording gaps.",
    )
    parser.add_argument(
        "--max-continuous-gap-s",
        type=float,
        default=MAX_CONTINUOUS_GAP_S,
        help="When using enabled_time, collapse gaps larger than this many seconds.",
    )
    parser.add_argument(
        "--min-velocity-for-max-mm-s",
        type=float,
        default=MIN_VELOCITY_FOR_MAX_MM_S,
        help="For velocity max curves only, ignore values below this magnitude. Use 0 to disable.",
    )
    parser.add_argument(
        "--min-current-for-max-ma",
        type=float,
        default=MIN_CURRENT_FOR_MAX_MA,
        help="For current max curves only, ignore magnitudes below this value. Use 0 to disable.",
    )
    parser.add_argument(
        "--min-motor-temp-for-max-c",
        type=float,
        default=MIN_MOTOR_TEMP_FOR_MAX_C,
        help="For motor temperature max curves only, ignore temperatures below this value. Use 0 to disable.",
    )
    parser.add_argument(
        "--sample-hz",
        type=float,
        default=PLOT_SAMPLE_HZ,
        help="Plot at most this many samples per second. Use 0 to keep all.",
    )
    return parser.parse_args(argv)


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = parse_args(argv)
    sample_hz = None if args.sample_hz == 0 else args.sample_hz
    min_velocity_for_max_mm_s = (
        None
        if args.min_velocity_for_max_mm_s == 0
        else args.min_velocity_for_max_mm_s
    )
    min_current_for_max_ma = (
        None
        if args.min_current_for_max_ma in (None, 0)
        else args.min_current_for_max_ma
    )
    min_motor_temp_for_max_c = (
        None
        if args.min_motor_temp_for_max_c in (None, 0)
        else args.min_motor_temp_for_max_c
    )

    if not args.input.exists():
        print(f"ERROR: input summary does not exist: {args.input}")
        return 1

    result = plot_report(
        input_path=args.input,
        output_path=args.output,
        axis_mode=args.time_axis,
        value_mode=args.value_mode,
        reduction_mode=args.reduction_mode,
        minmax_period_s=args.minmax_period_s,
        distance_period_s=args.distance_period_s,
        speed_filtered_current_avg_period_s=args.speed_filtered_current_avg_period_s,
        connect_collapsed_gaps=args.connect_gaps,
        time_unit=args.time_unit,
        sample_hz=sample_hz,
        max_continuous_gap_s=args.max_continuous_gap_s,
        min_velocity_for_max_mm_s=min_velocity_for_max_mm_s,
        min_current_for_max_ma=min_current_for_max_ma,
        min_motor_temp_for_max_c=min_motor_temp_for_max_c,
    )

    print(f"Saved PDF report: {result['output_path']}")
    print(f"Scanned samples: {result['scanned_samples']}")
    print(f"Plotted points: {result['plotted_points']}")
    print(f"Min velocity eligible for max curve: {result['min_velocity_for_max_mm_s']}")
    print(f"Min current eligible for max curve: {result['min_current_for_max_ma']}")
    print(f"Min motor temp eligible for max curve: {result['min_motor_temp_for_max_c']}")
    print(
        "Speed-filtered current interval: "
        f"{result['speed_filtered_current_min_mm_s']}.."
        f"{result['speed_filtered_current_max_mm_s']} mm/s"
    )
    for key, average in result["speed_filtered_current_averages_a"].items():
        count = result["speed_filtered_current_counts"][key]
        if average is not None:
            print(f"{key} speed-filtered average: {average:.3f} A (n={count})")
    print(f"Reduction mode: {result['reduction_mode']}")
    if result["reduction_mode"] == "minmax":
        print(f"Min/average/max period: {result['minmax_period_s']} s")
    print(f"Distance plot period: {result['distance_period_s']} s")
    print(
        "Speed-filtered average-current period: "
        f"{result['speed_filtered_current_avg_period_s']} s "
        f"({result['speed_filtered_current_avg_period_s'] / 3600.0:.3f} h)"
    )
    print(f"Connect collapsed gaps: {result['connect_collapsed_gaps']}")
    if args.time_axis == "enabled_time":
        print(f"Collapsed gaps: {result['collapsed_gaps']}")
    if result["skipped_values"]:
        print(f"Skipped null/non-numeric values: {result['skipped_values']}")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
