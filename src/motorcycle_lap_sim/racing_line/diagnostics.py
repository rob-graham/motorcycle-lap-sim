"""Command-line diagnostics for a constant-offset supplied racing line."""

import argparse
import csv
from pathlib import Path
import matplotlib.pyplot as plt
import numpy as np

from motorcycle_lap_sim.track import Track, sample_track
from .geometry import build_racing_line_path
from .offsets import LateralOffsetProfile
from .plotting import plot_racing_line


def write_csv(filename: Path, samples, profile, path) -> None:
    with filename.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow(("centreline_s_m", "offset_m", "x_m", "y_m",
                         "racing_line_q_m", "curvature_1pm"))
        writer.writerows(zip(samples.s_m, profile.offset_m, path.x_m, path.y_m,
                             path.q_m, path.curvature_1pm))


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("track", type=Path)
    parser.add_argument("--spacing", type=float, default=1.0)
    parser.add_argument("--constant-offset", type=float, default=0.0)
    parser.add_argument("--boundary-margin", type=float, default=0.0)
    parser.add_argument("--csv", type=Path)
    parser.add_argument("--output-png", type=Path)
    args = parser.parse_args()
    samples = sample_track(Track.from_yaml(args.track), args.spacing)
    profile = LateralOffsetProfile(samples, np.full(len(samples.s_m), args.constant_offset),
                                   args.boundary_margin)
    path = build_racing_line_path(samples, profile)
    print(f"sample count: {len(path.q_m)}")
    print(f"centreline length: {samples.total_length_m:.9f} m")
    print(f"racing-line length: {path.total_length_m:.9f} m")
    print(f"offset range: {profile.offset_m.min():.6f} to {profile.offset_m.max():.6f} m")
    print(f"minimum distance to boundary: {profile.minimum_boundary_clearance_m(samples):.6f} m")
    print(f"curvature range: {path.curvature_1pm.min():.9f} to {path.curvature_1pm.max():.9f} 1/m")
    print(f"closed: {path.closed}")
    if args.csv:
        write_csv(args.csv, samples, profile, path)
    if args.output_png:
        plot_racing_line(samples, path)
        plt.savefig(args.output_png, dpi=150, bbox_inches="tight")


if __name__ == "__main__":
    main()
