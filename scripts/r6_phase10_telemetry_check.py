"""Initial Phase 10 Mallala R6 telemetry-ingestion diagnostic."""

import argparse
import math
from pathlib import Path

import numpy as np

from motorcycle_lap_sim.telemetry import lap_slices, load_aim_workbook


def build_parser():
    parser = argparse.ArgumentParser()
    parser.add_argument("workbook", type=Path)
    parser.add_argument("--sheet", default="Updated")
    return parser


def finite_summary(values):
    values = np.asarray(values, dtype=float)
    finite = values[np.isfinite(values)]
    if len(finite) == 0:
        return "count=0"
    return (f"count={len(finite)} min={np.min(finite):.9g} "
            f"median={np.median(finite):.9g} max={np.max(finite):.9g}")


def main(argv=None):
    args = build_parser().parse_args(argv)
    session = load_aim_workbook(args.workbook, sheet_name=args.sheet)
    print(f"source={args.workbook}")
    print(f"sheet={session.source_sheet}")
    print(f"samples={len(session.time_s)}")
    print(f"time_start_s={session.time_s[0]:.6f}")
    print(f"time_end_s={session.time_s[-1]:.6f}")
    print(f"duration_s={session.time_s[-1] - session.time_s[0]:.6f}")
    print(f"median_sample_interval_s={np.median(np.diff(session.time_s)):.9f}")
    print(f"speed_mps {finite_summary(session.speed_mps)}")
    print(f"roll_rate_radps {finite_summary(session.roll_rate_radps)}")
    print(f"roll_rate_abs_radps {finite_summary(np.abs(session.roll_rate_radps))}")
    print(f"gps_lateral_acceleration_mps2 {finite_summary(session.lateral_acceleration_mps2)}")
    print(f"gps_longitudinal_acceleration_mps2 {finite_summary(session.longitudinal_acceleration_mps2)}")
    for lap in lap_slices(session):
        start, stop = lap.start_index, lap.stop_index
        # Add one nominal sample interval to a sample-centre duration when the
        # local timing is regular, while retaining the raw endpoint duration.
        intervals = np.diff(session.time_s[start:stop])
        nominal = float(np.median(intervals)) if len(intervals) else math.nan
        covered = lap.duration_s + nominal if math.isfinite(nominal) else math.nan
        print(f"lap_id={lap.lap_id} start_index={start} stop_index={stop} "
              f"endpoint_duration_s={lap.duration_s:.6f} "
              f"sample_covered_duration_s={covered:.6f}")


if __name__ == "__main__":
    main()
