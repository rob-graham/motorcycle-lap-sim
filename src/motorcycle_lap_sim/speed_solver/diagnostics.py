"""Command-line diagnostics for the Phase 3 fixed-path solver."""

import argparse
import csv
from pathlib import Path

from motorcycle_lap_sim.motorcycle.config import load_motorcycle_config
from motorcycle_lap_sim.track.sampling import sample_track
from motorcycle_lap_sim.track.track import Track
from .solver import solve_fixed_path


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("track")
    parser.add_argument("motorcycle")
    parser.add_argument("--spacing", type=float, default=1.0, help="sample spacing in metres")
    parser.add_argument("--csv", type=Path, help="write s and speed columns")
    args = parser.parse_args()
    sampled = sample_track(Track.from_yaml(args.track), args.spacing)
    profile = solve_fixed_path(sampled, load_motorcycle_config(args.motorcycle))
    print(f"samples: {len(sampled.s_m)}")
    print(f"lap time: {profile.lap_time_s:.6f} s")
    print(f"iterations: {profile.iterations}")
    if args.csv:
        with args.csv.open("w", newline="", encoding="utf-8") as stream:
            writer = csv.writer(stream)
            writer.writerow(("s_m", "speed_mps"))
            writer.writerows(zip(sampled.s_m, profile.speed_mps))


if __name__ == "__main__":
    main()
