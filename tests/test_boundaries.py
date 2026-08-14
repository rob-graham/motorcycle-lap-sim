import numpy as np
import pytest

from motorcycle_lap_sim.track.boundaries import calculate_boundaries
from motorcycle_lap_sim.track.primitives import Pose, Straight
from motorcycle_lap_sim.track.sampling import sample_track
from motorcycle_lap_sim.track.track import Track


def test_boundaries_follow_left_normal_at_requested_distances() -> None:
    samples = sample_track(Track((Straight(10.0),), Pose(0, 0, 0), 3.0, 2.0), 1.0)
    boundaries = calculate_boundaries(samples)
    left_distance = np.hypot(boundaries.left_x_m - samples.x_m, boundaries.left_y_m - samples.y_m)
    right_distance = np.hypot(boundaries.right_x_m - samples.x_m, boundaries.right_y_m - samples.y_m)
    assert left_distance == pytest.approx(3.0)
    assert right_distance == pytest.approx(2.0)
    assert boundaries.left_y_m == pytest.approx(3.0)
    assert boundaries.right_y_m == pytest.approx(-2.0)
