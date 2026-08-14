"""Plotting helpers kept separate from racing-line numerical calculations."""

import matplotlib.pyplot as plt
from matplotlib.axes import Axes

from motorcycle_lap_sim.track.boundaries import calculate_boundaries


def plot_racing_line(sampled_track, path, ax: Axes | None = None) -> Axes:
    """Plot track edges, centreline, and the supplied (not optimised) line."""
    ax = ax if ax is not None else plt.subplots()[1]
    edges = calculate_boundaries(sampled_track)
    ax.plot(edges.left_x_m, edges.left_y_m, label="left boundary")
    ax.plot(edges.right_x_m, edges.right_y_m, label="right boundary")
    ax.plot(sampled_track.x_m, sampled_track.y_m, "--", label="centreline")
    ax.plot(path.x_m, path.y_m, label="supplied racing line")
    ax.set_aspect("equal", adjustable="box")
    ax.set_xlabel("x [m]"); ax.set_ylabel("y [m]"); ax.legend()
    return ax
