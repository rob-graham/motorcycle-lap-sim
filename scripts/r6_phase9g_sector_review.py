"""Review the dominant sectors from a Phase 9G sector-diagnostics CSV.

This report is intentionally post-processing only.  It does not rerun the lap
simulation or alter any model parameters.  It highlights the sectors that
dominate finite-roll time loss, roll-aware time recovery, and any offsetting
roll-aware time losses, then prints the local physical diagnostics already
stored in the sector CSV.
"""

import argparse
import csv
import math
from pathlib import Path


REQUIRED_COLUMNS = (
    "sector_index",
    "sector_start_m",
    "sector_end_m",
    "finite_roll_time_penalty_s",
    "roll_aware_time_gain_s",
    "finite_roll_speed_mae_improvement_mps",
    "roll_aware_speed_mae_change_vs_frozen_mps",
    "roll_aware_offset_mae_change_m",
    "frozen_roll_binding_fraction",
    "roll_aware_binding_fraction",
    "frozen_roll_peak_abs_roll_rate_radps",
    "roll_aware_peak_abs_roll_rate_radps",
    "frozen_roll_peak_abs_curvature_gradient_1pm2",
    "roll_aware_peak_abs_curvature_gradient_1pm2",
    "frozen_roll_min_longitudinal_acceleration_mps2",
    "frozen_roll_max_longitudinal_acceleration_mps2",
    "roll_aware_min_longitudinal_acceleration_mps2",
    "roll_aware_max_longitudinal_acceleration_mps2",
    "line_max_abs_offset_change_m",
)


def _positive_int(text):
    value = int(text)
    if value <= 0:
        raise argparse.ArgumentTypeError("value must be positive")
    return value


def build_parser():
    parser = argparse.ArgumentParser()
    parser.add_argument("sector_csv", type=Path)
    parser.add_argument("--top-sectors", type=_positive_int, default=6)
    return parser


def _load_rows(path):
    with Path(path).open(newline="", encoding="utf-8") as stream:
        reader = csv.DictReader(stream)
        fieldnames = tuple(reader.fieldnames or ())
        missing = [name for name in REQUIRED_COLUMNS if name not in fieldnames]
        if missing:
            raise ValueError("sector CSV is missing required columns: " + ", ".join(missing))
        raw_rows = list(reader)
    if not raw_rows:
        raise ValueError("sector CSV contains no data rows")

    rows = []
    for row_number, raw in enumerate(raw_rows, start=2):
        row = {}
        for name in REQUIRED_COLUMNS:
            value = raw.get(name, "")
            if value is None or value.strip() == "":
                row[name] = math.nan
                continue
            try:
                row[name] = float(value)
            except ValueError as exc:
                raise ValueError(
                    f"sector CSV row {row_number} column {name} is not numeric") from exc
        sector_value = row["sector_index"]
        if not math.isfinite(sector_value) or not sector_value.is_integer():
            raise ValueError(f"sector CSV row {row_number} has non-integer sector_index")
        row["sector_index"] = int(sector_value)
        rows.append(row)
    return rows


def _finite(rows, key):
    return [row for row in rows if math.isfinite(float(row[key]))]


def _rank(rows, key, count, descending=True, predicate=None):
    candidates = _finite(rows, key)
    if predicate is not None:
        candidates = [row for row in candidates if predicate(float(row[key]))]
    candidates.sort(key=lambda row: float(row[key]), reverse=descending)
    return candidates[:min(count, len(candidates))]


def _print_rank(label, rows, key):
    print(f"{label}:")
    if not rows:
        print("  none")
        return
    for row in rows:
        print(
            f"  sector={row['sector_index']:02d} "
            f"chainage_m={row['sector_start_m']:.1f}:{row['sector_end_m']:.1f} "
            f"{key}={float(row[key]):.9f}")


def _format(value, digits=6):
    return "not_available" if not math.isfinite(float(value)) else f"{float(value):.{digits}f}"


def _print_details(rows):
    print("dominant_sector_details:")
    for row in sorted(rows, key=lambda item: item["sector_index"]):
        print(
            f"  sector={row['sector_index']:02d} "
            f"chainage_m={row['sector_start_m']:.1f}:{row['sector_end_m']:.1f} "
            f"roll_penalty_s={_format(row['finite_roll_time_penalty_s'], 6)} "
            f"roll_aware_gain_s={_format(row['roll_aware_time_gain_s'], 6)} "
            f"speed_mae_improvement_from_roll_mps={_format(row['finite_roll_speed_mae_improvement_mps'], 6)} "
            f"roll_aware_speed_mae_change_mps={_format(row['roll_aware_speed_mae_change_vs_frozen_mps'], 6)} "
            f"roll_aware_offset_mae_change_m={_format(row['roll_aware_offset_mae_change_m'], 6)} "
            f"roll_binding_fraction={_format(row['frozen_roll_binding_fraction'], 4)}->{_format(row['roll_aware_binding_fraction'], 4)} "
            f"peak_roll_rate_radps={_format(row['frozen_roll_peak_abs_roll_rate_radps'], 4)}->{_format(row['roll_aware_peak_abs_roll_rate_radps'], 4)} "
            f"peak_abs_curvature_gradient_1pm2={_format(row['frozen_roll_peak_abs_curvature_gradient_1pm2'], 7)}->{_format(row['roll_aware_peak_abs_curvature_gradient_1pm2'], 7)} "
            f"longitudinal_accel_frozen_mps2={_format(row['frozen_roll_min_longitudinal_acceleration_mps2'], 3)}:{_format(row['frozen_roll_max_longitudinal_acceleration_mps2'], 3)} "
            f"longitudinal_accel_roll_aware_mps2={_format(row['roll_aware_min_longitudinal_acceleration_mps2'], 3)}:{_format(row['roll_aware_max_longitudinal_acceleration_mps2'], 3)} "
            f"line_max_abs_move_m={_format(row['line_max_abs_offset_change_m'], 4)}")


def main(argv=None):
    args = build_parser().parse_args(argv)
    rows = _load_rows(args.sector_csv)

    top_penalty = _rank(
        rows, "finite_roll_time_penalty_s", args.top_sectors, descending=True)
    top_gain = _rank(
        rows, "roll_aware_time_gain_s", args.top_sectors, descending=True,
        predicate=lambda value: value > 0.0)
    top_loss = _rank(
        rows, "roll_aware_time_gain_s", args.top_sectors, descending=False,
        predicate=lambda value: value < 0.0)

    print(f"sector_csv={args.sector_csv}")
    print(f"sector_count={len(rows)}")
    _print_rank(
        "top_finite_roll_time_penalty_sectors", top_penalty,
        "finite_roll_time_penalty_s")
    _print_rank(
        "top_roll_aware_time_gain_sectors", top_gain,
        "roll_aware_time_gain_s")
    _print_rank(
        "top_roll_aware_time_loss_sectors", top_loss,
        "roll_aware_time_gain_s")

    selected = {}
    for row in (*top_penalty, *top_gain, *top_loss):
        selected[row["sector_index"]] = row
    _print_details(tuple(selected.values()))
    print("review_note=start/finish is a periodic seam; sectors at the end and beginning of the chainage range should be interpreted together")
    print("calibration_note=this report only reads the existing sector CSV and changes no model parameters")


if __name__ == "__main__":
    main()
