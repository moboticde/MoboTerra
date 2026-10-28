#!/usr/bin/env python3
"""Create a PDF report with CAN telemetry variables versus time.

Change only the CONFIG section below, then run:

    python3 src/terra_spiro/debug/plot_can_telemetry.py

Supported input formats:
    .json   JSON array
    .jsonl  one JSON object per line
    .csv    CSV table
"""

import csv
import json
import math
import os
from pathlib import Path
from typing import Any, Dict, Iterable, Iterator, List, Optional, Sequence, Tuple


# =============================================================================
# CONFIG — CHANGE HERE
# =============================================================================

INPUT_PATH = Path("/home/mobotic/can_telemetry_records/can_telemetry_20260925_162457.json")

OUTPUT_DIR = Path("/home/mobotic/romb_ws_ec/can_telemetry_plots")

# Output PDF filename inside OUTPUT_DIR.
OUTPUT_NAME = f"{INPUT_PATH.stem}_report.pdf"

# Same EC traction conversion used by KeyboardControl/KeyboardControlEC.py.
TRACTION_WHEEL_DIAMETER_MM = 225.0
TRACTION_GEAR_RATIO = 16.0
TRACTION_VELOCITY_RESOLUTION = 4096.0
MOTOR_TEMPERATURE_FIXED_POINT_SCALE = 1.0 / (2**25)


FIELD_GROUPS = [
    ("ROS moving", ["ros_moving"]),
    ("ROS operating seconds", ["ros_operating_seconds"]),
    ("ROS estop operations", ["ros_estop_operations"]),

    ("Battery state of charge [%]", ["vanguard_battery_soc_percent"]),
    ("Battery pack voltage [V]", ["vanguard_battery_pack_voltage_v"]),
    ("Battery pack current [A]", ["vanguard_battery_pack_current_a"]),
    ("Charger output current [A]", ["vanguard_charger_output_current_a"]),
    ("Charger output voltage [V]", ["vanguard_charger_output_voltage_v"]),

    ("Traction motor temperature [C]", ["ftr_motor_temp_c", "btr_motor_temp_c"]),
    ("Traction controller temperature [C]", ["ftr_controller_temp_c", "btr_controller_temp_c"]),
    #("Traction status feedback", ["ftr_status_feedback", "btr_status_feedback"]),
    ("Traction enabled", ["ftr_enabled", "btr_enabled"]),
    ("Traction error feedback", ["ftr_error_feedback", "btr_error_feedback"]),
    ("Traction EC error code", ["ftr_ec_error_code", "btr_ec_error_code"]),
    #("Traction CAN estop operations", ["ftr_can_estop_operations", "btr_can_estop_operations"]),
    ("Traction bus voltage [V]", ["ftr_bus_voltage_v", "btr_bus_voltage_v"]),
    ("Traction velocity [mm/s]", ["ftr_velocity_mm_s", "btr_velocity_mm_s"]),
    ("Traction acceleration [mm/s^2]", ["ftr_acc_mm_s2", "btr_acc_mm_s2"]),
    ("Traction deceleration [mm/s^2]", ["ftr_dec_mm_s2", "btr_dec_mm_s2"]),
    ("Traction actual current [mA]", ["ftr_current_actual_ma", "btr_current_actual_ma"]),
    ("Traction calculated RMS current [mA]", ["ftr_rms_cur", "btr_rms_cur"]),

    ("Steering controller temperature [C]", ["fstr_controller_temp_c", "bstr_controller_temp_c"]),
    #("Steering status feedback", ["fstr_status_feedback", "bstr_status_feedback"]),
    ("Steering enabled", ["fstr_enabled", "bstr_enabled"]),
    ("Steering error feedback", ["fstr_error_feedback", "bstr_error_feedback"]),
    #("Steering CAN estop operations", ["fstr_can_estop_operations", "bstr_can_estop_operations"]),
    ("Steering bus voltage [V]", ["fstr_bus_voltage_v", "bstr_bus_voltage_v"]),
    ("Steering position ticks", ["fstr_position_ticks", "bstr_position_ticks"]),
    ("Steering angle [deg]", ["fstr_angle_deg", "bstr_angle_deg"]),
    ("Steering velocity [deg/s]", ["fstr_velocity_deg_s", "bstr_velocity_deg_s"]),
    (
        "Steering actual current PDO 0x3262:01 [mA]",
        ["fstr_current_actual_ma", "bstr_current_actual_ma"],
    ),
    (
        "Steering controller RMS current PDO 0x3262:00 [mA]",
        ["fstr_rms_cur", "bstr_rms_cur"],
    ),
]

# Time field in your telemetry file.
TIME_KEY = "elapsed_s"

# Older recordings used rpm_raw for the same raw traction feedback field.
FIELD_ALIASES = {
    "ftr_velocity_raw": ("ftr_rpm_raw",),
    "btr_velocity_raw": ("btr_rpm_raw",),
}

# Do not draw cached Vanguard measurements after their CAN messages become
# stale. If a legacy recording has no freshness field, keep plotting it for
# backward compatibility.
FIELD_FRESHNESS_KEYS = {
    "vanguard_battery_soc_percent": "vanguard_battery_soc_fresh",
    "vanguard_battery_pack_voltage_v": "vanguard_battery_electrical_fresh",
    "vanguard_battery_pack_current_a": "vanguard_battery_electrical_fresh",
    "vanguard_charger_output_current_a": "vanguard_charger_status_fresh",
    "vanguard_charger_output_voltage_v": "vanguard_charger_status_fresh",
}

DERIVED_TRACTION_FIELDS = {
    "ftr_velocity_mm_s": ("ftr_velocity_raw", "ftr_rpm_raw"),
    "btr_velocity_mm_s": ("btr_velocity_raw", "btr_rpm_raw"),
    "ftr_acc_mm_s2": ("ftr_acc_raw",),
    "btr_acc_mm_s2": ("btr_acc_raw",),
    "ftr_dec_mm_s2": ("ftr_dec_raw",),
    "btr_dec_mm_s2": ("btr_dec_raw",),
}

# Options: "seconds", "minutes", "hours"
TIME_UNIT = "seconds"

# Number of plots per PDF page.
PLOTS_PER_PAGE = 4

# If True, first timestamp becomes 0.
NORMALIZE_TIME_TO_ZERO = True

# Plot at most this many samples per second for each selected field.
# The scanner still uses all 50 Hz records for extrema and keeps every state
# change. Set to None only for short recordings where every point is needed.
PLOT_SAMPLE_HZ: Optional[float] = 10.0

# Keep implausible traction velocity spikes out of the report. Bounds are
# inclusive and use the same units as the corresponding telemetry fields.
FIELD_VALUE_RANGES = {
    "ftr_velocity_mm_s": (-2000.0, 2000.0),
    "btr_velocity_mm_s": (-2000.0, 2000.0),
}

# If True, mark each field's extrema and include them in the legend. Negative-only
# series show their minimum but skip the misleading negative "max" value.
SHOW_EXTREME_VALUES = True

# These fields are usually state/counter/error signals. Keep every value change
# for them so short fault or enable transitions are not hidden by downsampling.
KEEP_EVERY_CHANGE_FOR_FIELDS = {
    "ros_moving",
    "ros_estop_operations",
    "ftr_enabled",
    "btr_enabled",
    "ftr_error_feedback",
    "btr_error_feedback",
    "ftr_ec_error_code",
    "btr_ec_error_code",
    "fstr_enabled",
    "bstr_enabled",
    "fstr_error_feedback",
    "bstr_error_feedback",
}

# Plot these lifetime counters as "increase since recording start".
# Absolute controller counters can be hundreds of thousands of seconds, which
# makes short changes hard to see when front/back values share one axis.
DELTA_FROM_FIRST_FIELDS = {
}

EC_ERROR_DESCRIPTIONS = {
    0x3120: "Mains/input voltage too low",
    0x8110: "CAN overrun",
    0xFF10: "Manufacturer-specific error FF10",
    0xFF20: "Safety switch sensed; drive in safety state",
}

ERROR_REGISTER_BITS = {
    0: "generic",
    1: "current",
    2: "voltage",
    3: "temperature",
    4: "communication",
    5: "device-profile",
    7: "manufacturer",
}

# If True, prints available fields from the first record and exits.
LIST_FIELDS_FIRST_RECORD = False

# Open matplotlib window on the NUC.
# Usually keep False when working through SSH/terminal.
SHOW_PLOT = False

# =============================================================================


Record = Dict[str, Any]
SeriesExtrema = Dict[str, Optional[Tuple[float, float]]]


def iter_json_array_records(path: Path, chunk_size: int = 1024 * 1024) -> Iterator[Record]:
    """Yield objects from a recorder JSON array without loading the whole file."""
    del chunk_size  # Kept for API symmetry with the recovery parser.

    saw_array_start = False
    saw_array_end = False
    object_lines: List[str] = []
    object_start_line: Optional[int] = None

    with path.open("r", encoding="utf-8") as handle:
        for line_number, line in enumerate(handle, start=1):
            stripped = line.strip()

            if not saw_array_start:
                if not stripped:
                    continue
                if stripped != "[":
                    raise ValueError(f"{path} is not a JSON array recording")
                saw_array_start = True
                continue

            if not object_lines:
                if not stripped or stripped == ",":
                    continue
                if stripped == "]":
                    saw_array_end = True
                    return
                if stripped.startswith("{"):
                    object_lines = [line]
                    object_start_line = line_number
                    continue

                raise ValueError(
                    f"Unexpected JSON token {stripped!r} in {path} at line {line_number}"
                )

            object_lines.append(line)

            if stripped in {"}", "},"}:
                record_text = "".join(object_lines).rstrip()
                if record_text.endswith(","):
                    record_text = record_text[:-1]

                record = json.loads(record_text)
                if not isinstance(record, dict):
                    raise ValueError(
                        f"Expected object in {path} at line {object_start_line}, "
                        f"got {type(record).__name__}"
                    )

                yield record
                object_lines = []
                object_start_line = None

    if not saw_array_start:
        return

    if object_lines:
        print(
            f"WARNING: ignoring incomplete JSON object in {path} "
            f"starting at line {object_start_line}"
        )
    elif not saw_array_end:
        print(
            f"WARNING: {path} has no closing JSON array bracket. "
            "Using complete records found before end of file."
        )


def iter_recovered_json_array_records(
    path: Path,
    chunk_size: int = 1024 * 1024,
) -> Iterator[Record]:
    """Yield objects from a JSON array without loading the whole file into RAM.

    The recorder writes a pretty-printed JSON array. If recording is interrupted,
    the final closing bracket may be missing; this scanner still yields complete
    objects that were fully written.
    """
    saw_array_start = False
    object_chars: List[str] = []
    object_start_line: Optional[int] = None
    depth = 0
    in_string = False
    escape = False
    line_number = 1

    with path.open("r", encoding="utf-8") as handle:
        while True:
            chunk = handle.read(chunk_size)
            if not chunk:
                break

            for char in chunk:
                if not saw_array_start:
                    if char.isspace():
                        pass
                    elif char == "[":
                        saw_array_start = True
                    else:
                        raise ValueError(f"{path} is not a JSON array recording")

                    if char == "\n":
                        line_number += 1
                    continue

                if depth == 0:
                    if char == "{":
                        object_chars = [char]
                        object_start_line = line_number
                        depth = 1
                        in_string = False
                        escape = False
                    elif char == "]":
                        return
                    elif char.isspace() or char == ",":
                        pass
                    else:
                        raise ValueError(
                            f"Unexpected JSON token {char!r} in {path} at line {line_number}"
                        )

                    if char == "\n":
                        line_number += 1
                    continue

                object_chars.append(char)

                if in_string:
                    if escape:
                        escape = False
                    elif char == "\\":
                        escape = True
                    elif char == '"':
                        in_string = False
                elif char == '"':
                    in_string = True
                elif char == "{":
                    depth += 1
                elif char == "}":
                    depth -= 1
                    if depth == 0:
                        record_text = "".join(object_chars)
                        record = json.loads(record_text)

                        if not isinstance(record, dict):
                            raise ValueError(
                                f"Expected object in {path} at line {object_start_line}, "
                                f"got {type(record).__name__}"
                            )

                        yield record
                        object_chars = []
                        object_start_line = None

                if char == "\n":
                    line_number += 1

    if not saw_array_start:
        return

    if depth > 0:
        print(
            f"WARNING: ignoring incomplete JSON object in {path} "
            f"starting at line {object_start_line}"
        )


def iter_jsonl_records(path: Path) -> Iterator[Record]:
    """Yield objects from JSONL file."""
    with path.open("r", encoding="utf-8") as handle:
        for line_number, line in enumerate(handle, start=1):
            stripped = line.strip()
            if not stripped:
                continue

            record = json.loads(stripped)

            if not isinstance(record, dict):
                raise ValueError(
                    f"Expected JSON object on line {line_number}, got {type(record).__name__}"
                )

            yield record


def parse_csv_value(value: str) -> Any:
    """Convert CSV strings to int/float/bool/None when possible."""
    if value == "":
        return None

    if value in {"True", "False"}:
        return value == "True"

    try:
        if any(marker in value for marker in (".", "e", "E")):
            return float(value)
        return int(value)
    except ValueError:
        return value


def iter_csv_records(path: Path) -> Iterator[Record]:
    """Yield objects from CSV file."""
    with path.open("r", newline="", encoding="utf-8") as handle:
        for row in csv.DictReader(handle):
            yield {key: parse_csv_value(value) for key, value in row.items()}


def iter_records(path: Path) -> Iterator[Record]:
    """Select reader depending on file extension."""
    suffix = path.suffix.lower()

    if suffix == ".csv":
        yield from iter_csv_records(path)
    elif suffix == ".jsonl":
        yield from iter_jsonl_records(path)
    else:
        yield from iter_json_array_records(path)


def traction_raw_to_mm(raw_value: Any) -> Optional[float]:
    if raw_value is None:
        return None

    if TRACTION_GEAR_RATIO == 0.0 or TRACTION_VELOCITY_RESOLUTION == 0.0:
        return None

    try:
        raw = float(raw_value)
    except (TypeError, ValueError):
        return None

    return (
        raw
        * math.pi
        * TRACTION_WHEEL_DIAMETER_MM
        / (TRACTION_VELOCITY_RESOLUTION * TRACTION_GEAR_RATIO)
    )


def derived_numeric_value(record: Record, key: str) -> Optional[float]:
    raw_keys = DERIVED_TRACTION_FIELDS.get(key)
    if raw_keys is None:
        return None

    for raw_key in raw_keys:
        value = record.get(raw_key)
        if value is not None:
            return traction_raw_to_mm(value)

    return None


def normalize_known_units(key: str, number: float) -> float:
    if key in {"ftr_motor_temp_c", "btr_motor_temp_c"} and abs(number) > 100000.0:
        return number * MOTOR_TEMPERATURE_FIXED_POINT_SCALE

    if key in {"ftr_bus_voltage_v", "btr_bus_voltage_v"} and abs(number) > 1000.0:
        return number * 0.001

    return number


def numeric_value(record: Record, key: str) -> Optional[float]:
    """Read a numeric value from record."""
    freshness_key = FIELD_FRESHNESS_KEYS.get(key)
    if freshness_key is not None and record.get(freshness_key) is False:
        return None

    value = record.get(key)

    if value is None:
        for alias in FIELD_ALIASES.get(key, ()):
            value = record.get(alias)
            if value is not None:
                break

    if value is None:
        return derived_numeric_value(record, key)

    if isinstance(value, bool):
        return 1.0 if value else 0.0

    if isinstance(value, (int, float)):
        number = float(value)
    else:
        try:
            number = float(str(value))
        except ValueError:
            return None

    if math.isnan(number) or math.isinf(number):
        return None

    return normalize_known_units(key, number)


def collect_series(
    records: Iterable[Record],
    value_keys: Sequence[str],
    time_key: str,
) -> Tuple[
    Dict[str, List[float]],
    Dict[str, List[float]],
    SeriesExtrema,
    SeriesExtrema,
    int,
    int,
    int,
]:
    """Collect x/y values for all selected fields."""
    x_values = {key: [] for key in value_keys}
    y_values = {key: [] for key in value_keys}
    min_values: SeriesExtrema = {key: None for key in value_keys}
    max_values: SeriesExtrema = {key: None for key in value_keys}
    last_plotted_time: Dict[str, Optional[float]] = {key: None for key in value_keys}
    last_plotted_value: Dict[str, Optional[float]] = {key: None for key in value_keys}

    scanned_records = 0
    skipped_values = 0
    filtered_values = 0
    first_time: Optional[float] = None
    sample_period_s = None

    if PLOT_SAMPLE_HZ is not None:
        if PLOT_SAMPLE_HZ <= 0.0:
            raise ValueError("PLOT_SAMPLE_HZ must be positive, or None to keep every sample.")
        sample_period_s = 1.0 / PLOT_SAMPLE_HZ

    for record in records:
        scanned_records += 1

        time_value = numeric_value(record, time_key)
        if time_value is None:
            skipped_values += len(value_keys)
            continue

        if NORMALIZE_TIME_TO_ZERO:
            if first_time is None:
                first_time = time_value
            time_value = time_value - first_time

        for key in value_keys:
            value = numeric_value(record, key)

            if value is None:
                skipped_values += 1
                continue

            value_range = FIELD_VALUE_RANGES.get(key)
            if value_range is not None:
                minimum, maximum = value_range
                if not minimum <= value <= maximum:
                    filtered_values += 1
                    continue

            current_max = max_values[key]
            if current_max is None or value > current_max[1]:
                max_values[key] = (time_value, value)

            current_min = min_values[key]
            if current_min is None or value < current_min[1]:
                min_values[key] = (time_value, value)

            if sample_period_s is not None:
                previous_time = last_plotted_time[key]
                previous_value = last_plotted_value[key]
                keep_change = (
                    key in KEEP_EVERY_CHANGE_FOR_FIELDS
                    and previous_value is not None
                    and value != previous_value
                )
                if (
                    not keep_change
                    and previous_time is not None
                    and time_value - previous_time < sample_period_s
                ):
                    continue
                last_plotted_time[key] = time_value
                last_plotted_value[key] = value

            x_values[key].append(time_value)
            y_values[key].append(value)

    return (
        x_values,
        y_values,
        min_values,
        max_values,
        scanned_records,
        skipped_values,
        filtered_values,
    )


def time_scale_and_label(unit: str) -> Tuple[float, str]:
    scale_by_unit = {
        "seconds": (1.0, "time [s]"),
        "minutes": (1.0 / 60.0, "time [min]"),
        "hours": (1.0 / 3600.0, "time [h]"),
    }
    return scale_by_unit[unit]


def scale_time(
    values: Dict[str, List[float]],
    unit: str,
) -> Tuple[Dict[str, List[float]], str]:
    """Convert seconds to selected time unit."""
    scale, label = time_scale_and_label(unit)

    if scale == 1.0:
        return values, label

    return {
        key: [value * scale for value in series]
        for key, series in values.items()
    }, label


def scale_extreme_times(extreme_values: SeriesExtrema, unit: str) -> SeriesExtrema:
    """Convert stored extrema timestamps to the selected time unit."""
    scale, _label = time_scale_and_label(unit)

    if scale == 1.0:
        return extreme_values

    return {
        key: None if value is None else (value[0] * scale, value[1])
        for key, value in extreme_values.items()
    }


def normalize_delta_fields(
    values: Dict[str, List[float]],
    delta_fields: Iterable[str],
) -> Dict[str, float]:
    """Convert selected y-series to delta from first plotted sample."""
    first_values: Dict[str, float] = {}

    for key in delta_fields:
        series = values.get(key)
        if not series:
            continue

        first_value = series[0]
        first_values[key] = first_value
        for index, value in enumerate(series):
            series[index] = value - first_value

    return first_values


def normalize_delta_extreme_values(
    extreme_values: SeriesExtrema,
    delta_first_values: Dict[str, float],
) -> None:
    """Apply the same delta offset to stored extrema."""
    for key, first_value in delta_first_values.items():
        extreme_value = extreme_values.get(key)
        if extreme_value is None:
            continue

        time_value, value = extreme_value
        extreme_values[key] = (time_value, value - first_value)


def format_plot_number(value: float) -> str:
    """Format numeric plot annotations without excessive decimal noise."""
    if value == 0.0:
        return "0"

    abs_value = abs(value)
    if 0.001 <= abs_value < 10000.0:
        return f"{value:.3f}".rstrip("0").rstrip(".")

    return f"{value:.3g}"


def visible_extrema(
    min_value: Optional[Tuple[float, float]],
    max_value: Optional[Tuple[float, float]],
) -> List[Tuple[str, Tuple[float, float]]]:
    """Return extrema worth displaying for one series.

    When a signed signal's negative side is larger in magnitude, the minimum is
    the useful peak and a small positive max would be noise in the report.
    """
    if min_value is None or max_value is None:
        return []

    _min_time, min_y = min_value
    _max_time, max_y = max_value

    if min_y == max_y:
        return [("value", max_value)]

    if min_y < 0.0 and abs(min_y) > abs(max_y):
        return [("min", min_value)]

    if max_y > 0.0 and abs(max_y) > abs(min_y):
        return [("max", max_value)]

    return [("min", min_value), ("max", max_value)]


def describe_error_value(key: str, value: int) -> str:
    """Return a compact description for known traction error fields."""
    if key.endswith("_ec_error_code"):
        return EC_ERROR_DESCRIPTIONS.get(value, "Unknown/manufacturer-specific error")

    if key.endswith("_error_feedback"):
        descriptions = [
            description
            for bit, description in ERROR_REGISTER_BITS.items()
            if value & (1 << bit)
        ]
        return "+".join(descriptions) if descriptions else "none"

    return ""


def compact_error_summary_lines(
    x_values: Dict[str, List[float]],
    y_values: Dict[str, List[float]],
) -> List[str]:
    """Summarize nonzero traction error episodes for the report title page."""
    lines: List[str] = []
    error_fields = [
        "ftr_ec_error_code",
        "btr_ec_error_code",
        "ftr_error_feedback",
        "btr_error_feedback",
    ]

    for key in error_fields:
        times = x_values.get(key, [])
        values = y_values.get(key, [])
        episodes: Dict[int, List[float]] = {}
        previous_value = 0

        for time_value, raw_value in zip(times, values):
            value = int(raw_value)
            if value != 0 and value != previous_value:
                episodes.setdefault(value, []).append(time_value)
            previous_value = value

        for value, starts in episodes.items():
            if key.endswith("_ec_error_code"):
                formatted_value = f"0x{value:04X}"
            else:
                formatted_value = str(value)

            description = describe_error_value(key, value)
            lines.append(
                f"{key:<22} {formatted_value:<6} {description:<34} "
                f"x{len(starts):<3} first {starts[0]:7.1f}s  last {starts[-1]:7.1f}s"
            )

    return lines


def configure_matplotlib_cache() -> None:
    """Avoid matplotlib cache permission problems on Linux."""
    if "MPLCONFIGDIR" in os.environ:
        return

    cache_dir = Path("/tmp") / f"matplotlib-{os.getuid()}"
    cache_dir.mkdir(parents=True, exist_ok=True)
    os.environ["MPLCONFIGDIR"] = str(cache_dir)


def safe_filename(text: str) -> str:
    """Create safe filename part from telemetry field name."""
    return "".join(char if char.isalnum() or char in "._-" else "_" for char in text)


def selected_fields() -> List[str]:
    """Return unique selected fields while preserving config order."""
    fields: List[str] = []
    seen = set()

    for _title, group_fields in FIELD_GROUPS:
        for field in group_fields:
            if field in seen:
                continue
            fields.append(field)
            seen.add(field)

    return fields


def list_fields(path: Path) -> int:
    """Print field names from first record."""
    for record in iter_records(path):
        print("\nAvailable fields:\n")
        for key in record.keys():
            print(key)
        return 0

    print(f"No records found in {path}")
    return 1


def plot_grouped_axes(
    x_values: Dict[str, List[float]],
    y_values: Dict[str, List[float]],
    min_values: SeriesExtrema,
    max_values: SeriesExtrema,
    x_label: str,
    output_path: Path,
    scanned_records: int,
    skipped_values: int,
    filtered_values: int,
) -> None:
    """Plot configured field groups into a multi-page PDF report."""
    import matplotlib.pyplot as plt
    from matplotlib.backends.backend_pdf import PdfPages

    populated_groups = [
        (title, [key for key in group_fields if y_values[key]])
        for title, group_fields in FIELD_GROUPS
    ]
    populated_groups = [
        (title, group_fields)
        for title, group_fields in populated_groups
        if group_fields
    ]

    if not populated_groups:
        raise ValueError("No configured fields have numeric samples.")

    with PdfPages(output_path) as pdf:
        title_figure = plt.figure(figsize=(11.69, 8.27))
        title_figure.suptitle("CAN Telemetry Report", fontsize=22, y=0.92)
        summary_lines = [
            f"Input file: {INPUT_PATH}",
            f"Time field: {TIME_KEY}",
            f"Time axis: {x_label}",
            f"Plot sample rate: {PLOT_SAMPLE_HZ if PLOT_SAMPLE_HZ is not None else 'all'} Hz",
            f"Scanned records: {scanned_records}",
            f"Plotted samples: {sum(len(series) for series in y_values.values())}",
            f"Skipped non-numeric/null values: {skipped_values}",
            f"Filtered out-of-range values: {filtered_values}",
            f"Populated plots: {len(populated_groups)}",
        ]
        title_figure.text(
            0.06,
            0.84,
            "\n".join(summary_lines),
            fontsize=10,
            va="top",
            family="monospace",
        )

        error_lines = compact_error_summary_lines(x_values, y_values)
        title_figure.text(
            0.06,
            0.55,
            "Nonzero traction error episodes\n"
            + ("\n".join(error_lines) if error_lines else "None"),
            fontsize=8.5,
            va="top",
            family="monospace",
        )
        pdf.savefig(title_figure)
        if SHOW_PLOT:
            plt.show()
        plt.close(title_figure)

        plots_per_page = max(1, PLOTS_PER_PAGE)
        for page_start in range(0, len(populated_groups), plots_per_page):
            page_groups = populated_groups[page_start: page_start + plots_per_page]
            figure, axes = plt.subplots(
                len(page_groups),
                1,
                figsize=(11.69, 8.27),
                sharex=True,
            )

            if len(page_groups) == 1:
                axes = [axes]

            for axis, (title, group_fields) in zip(axes, page_groups):
                for key in group_fields:
                    label = key
                    extrema = []
                    min_value = min_values.get(key)
                    max_value = max_values.get(key)
                    if SHOW_EXTREME_VALUES:
                        extrema = visible_extrema(min_value, max_value)
                        if extrema:
                            extrema_label = ", ".join(
                                f"{name} {format_plot_number(value)} "
                                f"@ {format_plot_number(time_value)}"
                                for name, (time_value, value) in extrema
                            )
                            label = f"{key} ({extrema_label})"

                    (line,) = axis.plot(
                        x_values[key],
                        y_values[key],
                        label=label,
                        linewidth=1.2,
                    )

                    for name, (time_value, value) in extrema:
                        axis.scatter(
                            [time_value],
                            [value],
                            color=line.get_color(),
                            marker="^" if name == "max" else "v",
                            s=28,
                            zorder=4,
                        )
                        axis.annotate(
                            f"{name} {format_plot_number(value)}",
                            xy=(time_value, value),
                            xytext=(4, 4),
                            textcoords="offset points",
                            color=line.get_color(),
                            fontsize=7,
                        )

                axis.set_ylabel("value")
                axis.set_title(title)
                axis.grid(True, alpha=0.35)
                axis.legend(loc="best")

            axes[-1].set_xlabel(x_label)
            figure.tight_layout()
            pdf.savefig(figure)

            if SHOW_PLOT:
                plt.show()

            plt.close(figure)

    print(f"Saved PDF report: {output_path}")


def main() -> int:
    if not INPUT_PATH.exists():
        print(f"ERROR: input file does not exist: {INPUT_PATH}")
        return 1

    if LIST_FIELDS_FIRST_RECORD:
        return list_fields(INPUT_PATH)

    fields = selected_fields()

    if not fields:
        print("ERROR: FIELD_GROUPS is empty.")
        return 1

    configure_matplotlib_cache()

    if not SHOW_PLOT:
        import matplotlib
        matplotlib.use("Agg")

    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)

    (
        x_values,
        y_values,
        min_values,
        max_values,
        scanned_records,
        skipped_values,
        filtered_values,
    ) = collect_series(
        records=iter_records(INPUT_PATH),
        value_keys=fields,
        time_key=TIME_KEY,
    )

    empty_fields = [key for key in fields if not y_values[key]]
    populated_fields = [key for key in fields if y_values[key]]

    if not populated_fields:
        print("ERROR: none of the configured fields contain numeric samples.")
        print("Set LIST_FIELDS_FIRST_RECORD = True to check available field names.")
        return 1

    if empty_fields:
        print(f"Skipping fields without numeric samples: {', '.join(empty_fields)}")

    delta_first_values = normalize_delta_fields(y_values, DELTA_FROM_FIRST_FIELDS)
    normalize_delta_extreme_values(min_values, delta_first_values)
    normalize_delta_extreme_values(max_values, delta_first_values)
    x_values, x_label = scale_time(x_values, TIME_UNIT)
    min_values = scale_extreme_times(min_values, TIME_UNIT)
    max_values = scale_extreme_times(max_values, TIME_UNIT)

    output_path = OUTPUT_DIR / safe_filename(OUTPUT_NAME)
    plot_grouped_axes(
        x_values,
        y_values,
        min_values,
        max_values,
        x_label,
        output_path,
        scanned_records,
        skipped_values,
        filtered_values,
    )

    print(f"Scanned records: {scanned_records}")
    print(f"Plotted samples: {sum(len(series) for series in y_values.values())}")
    if PLOT_SAMPLE_HZ is not None:
        print(f"Plot sample rate: {PLOT_SAMPLE_HZ} Hz")

    if SHOW_EXTREME_VALUES:
        print("Extreme values:")
        for key in populated_fields:
            min_value = min_values.get(key)
            max_value = max_values.get(key)
            extrema = visible_extrema(min_value, max_value)
            if not extrema:
                continue
            extrema_text = ", ".join(
                f"{name} {format_plot_number(value)} "
                f"at {format_plot_number(time_value)} {x_label}"
                for name, (time_value, value) in extrema
            )
            print(f"  {key}: {extrema_text}")

    if skipped_values:
        print(f"Skipped non-numeric/null values: {skipped_values}")

    if filtered_values:
        print(f"Filtered out-of-range values: {filtered_values}")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
