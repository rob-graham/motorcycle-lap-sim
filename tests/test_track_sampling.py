import math
from pathlib import Path

import numpy as np
import pytest

from motorcycle_lap_sim.track.primitives import CircularArc, Pose, Straight
from motorcycle_lap_sim.track.sampling import sample_track
from motorcycle_lap_sim.track.track import Track

OVAL = Path(__file__).parents[1] / "examples/tracks/test_oval.yaml"


def test_sampling_is_monotonic_approximately_uniform_and_has_no_closed_duplicate() -> None:
    samples = sample_track(Track.from_yaml(OVAL), 1.3)
    differences = np.diff(samples.s_m)
    assert np.all(differences > 0)
    assert differences == pytest.approx(np.full_like(differences, differences[0]))
    assert differences[0] <= 1.3
    assert samples.s_m[-1] < samples.total_length_m
    assert math.hypot(samples.x_m[-1] - samples.x_m[0], samples.y_m[-1] - samples.y_m[0]) > 0


def test_closed_endpoint_can_be_requested() -> None:
    samples = sample_track(Track.from_yaml(OVAL), 3.0, include_endpoint=True)
    assert samples.s_m[-1] == pytest.approx(samples.total_length_m)
    assert samples.x_m[-1] == pytest.approx(samples.x_m[0], abs=1e-12)
    assert samples.y_m[-1] == pytest.approx(samples.y_m[0], abs=1e-12)


def test_curvature_and_frame_are_analytic() -> None:
    track = Track((Straight(10.0), CircularArc(5.0, math.pi / 2)), Pose(0, 0, 0))
    samples = sample_track(track, 0.25)
    assert samples.curvature_1pm[0] == pytest.approx(0.0)
    arc_samples = samples.s_m > 10.0
    assert samples.curvature_1pm[arc_samples] == pytest.approx(0.2)
    assert np.hypot(samples.tangent_x, samples.tangent_y) == pytest.approx(1.0)
    assert np.hypot(samples.normal_x, samples.normal_y) == pytest.approx(1.0)
    dot = samples.tangent_x * samples.normal_x + samples.tangent_y * samples.normal_y
    assert dot == pytest.approx(0.0, abs=1e-15)


def test_sampling_resolution_does_not_change_analytic_length() -> None:
    track = Track.from_yaml(OVAL)
    coarse = sample_track(track, 10.0)
    fine = sample_track(track, 0.1)
    assert coarse.total_length_m == fine.total_length_m == pytest.approx(200 + 60 * math.pi)
