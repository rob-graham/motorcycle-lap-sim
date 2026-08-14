"""Plot sampled track centreline and boundaries."""

import argparse
from pathlib import Path

import matplotlib.pyplot as plt
from matplotlib.axes import Axes

from motorcycle_lap_sim.track.boundaries import calculate_boundaries
from motorcycle_lap_sim.track.sampling import SampledTrack, sample_track
from motorcycle_lap_sim.track.track import Track


def plot_track(samples: SampledTrack, ax: Axes | None = None) -> Axes:
    ax = ax if ax is not None else plt.subplots()[1]
    boundaries = calculate_boundaries(samples)
    ax.plot(samples.x_m, samples.y_m, "--", label="centreline")
    ax.plot(boundaries.left_x_m, boundaries.left_y_m, label="left boundary")
    ax.plot(boundaries.right_x_m, boundaries.right_y_m, label="right boundary")
    ax.set_aspect("equal", adjustable="box")
    ax.set_xlabel("x [m]"); ax.set_ylabel("y [m]"); ax.legend()
    return ax


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("track", type=Path)
    parser.add_argument("--spacing", type=float, default=1.0)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    track = Track.from_yaml(args.track)
    samples = sample_track(track, args.spacing)
    plot_track(samples)
    closure = track.closure_diagnostic()
    print(f"centreline length: {track.total_length_m:.9f} m")
    print(f"closure: dx={closure.x_error_m:.3e} m, dy={closure.y_error_m:.3e} m, "
          f"position={closure.position_error_m:.3e} m, heading={closure.heading_error_rad:.3e} rad")
    if args.output:
        plt.savefig(args.output, dpi=150, bbox_inches="tight")
    else:
        plt.show()


if __name__ == "__main__":
    main()
