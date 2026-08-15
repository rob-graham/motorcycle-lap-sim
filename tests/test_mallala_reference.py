import math
from pathlib import Path

import numpy as np
import pytest

from motorcycle_lap_sim.track import CircularArc, Straight, Track, sample_track

TRACK_PATH = Path(__file__).parents[1] / "examples/tracks/mallala_reference.yaml"


def test_mallala_v03_reference_geometry_and_closure():
    track = Track.from_yaml(TRACK_PATH)
    assert track.closed and len(track.primitives) == 23
    assert track.total_length_m == pytest.approx(2557.188177, abs=1e-6)
    arcs = [p for p in track.primitives if isinstance(p, CircularArc)]
    assert sum(p.turn_angle_rad for p in arcs) == pytest.approx(-2 * math.pi, abs=3e-11)
    closure = track.closure_diagnostic()
    assert closure.position_error_m < 2e-8
    assert abs(closure.heading_error_rad) < 3e-11
    assert all(isinstance(p, CircularArc) and p.turn_angle_rad < 0
               for p in track.primitives[5:8])
    assert all(isinstance(p, CircularArc) and p.turn_angle_rad < 0
               for p in track.primitives[16:18])
    assert isinstance(track.primitives[0], Straight)
    assert isinstance(track.primitives[-1], Straight)


def test_mallala_resolved_widths_and_sampled_analytic_curvature():
    track = Track.from_yaml(TRACK_PATH)
    assert track.primitive_width_left_m[0] == track.primitive_width_left_m[-1] == 5
    assert track.primitive_width_right_m[0] == track.primitive_width_right_m[-1] == 5
    assert set(track.primitive_width_left_m[1:-1]) == {4}
    assert set(track.primitive_width_right_m[1:-1]) == {4}
    samples = sample_track(track, 1.0)
    assert set(np.unique(samples.width_left_m)) == {4, 5}
    assert set(np.unique(samples.width_right_m)) == {4, 5}
    assert np.any(samples.curvature_1pm == 0)
    assert np.any(samples.curvature_1pm > 0)
    assert np.any(samples.curvature_1pm < 0)
    assert samples.s_m[-1] < track.total_length_m
    assert not np.array_equal(samples.x_m[[0]], samples.x_m[[-1]])
