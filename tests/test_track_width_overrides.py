from pathlib import Path

import numpy as np
import pytest

from motorcycle_lap_sim.racing_line import LateralOffsetProfile, build_smooth_racing_line_path
from motorcycle_lap_sim.track import CircularArc, Pose, Straight, Track, sample_track, sample_track_stations
from motorcycle_lap_sim.track.boundaries import calculate_boundaries

ROOT = Path(__file__).parents[1]


def test_existing_yaml_and_positional_constructor_keep_global_widths():
    existing = sample_track(Track.from_yaml(ROOT / "examples/tracks/test_oval.yaml"), 3.0)
    assert np.array_equal(existing.width_left_m, np.full(len(existing.s_m), 4.0))
    assert np.array_equal(existing.width_right_m, np.full(len(existing.s_m), 4.0))
    direct = Track((Straight(1.0),), Pose(0, 0, 0), 3.0, 2.0, False)
    assert direct.primitive_width_left_m == (3.0,)
    assert direct.primitive_width_right_m == (2.0,)


def test_per_primitive_and_one_sided_widths_use_next_primitive_at_join():
    track = Track((Straight(10), Straight(10)), Pose(0, 0, 0), 4, 4, True,
                  (None, 5), (None, None))
    samples = sample_track_stations(track, [0, 9, 10, 19])
    assert samples.width_left_m.tolist() == [4, 4, 5, 5]
    assert samples.width_right_m.tolist() == [4, 4, 4, 4]


def test_variable_widths_flow_to_boundaries_and_offset_validation():
    track = Track((Straight(10), Straight(10)), Pose(0, 0, 0), 4, 4, True,
                  (None, 5), (None, 5))
    samples = sample_track_stations(track, [5, 15])
    boundaries = calculate_boundaries(samples)
    assert boundaries.left_y_m.tolist() == [4, 5]
    assert boundaries.right_y_m.tolist() == [-4, -5]
    LateralOffsetProfile(samples, [4, 5])
    with pytest.raises(ValueError, match="outside"):
        LateralOffsetProfile(samples, [4.01, 5])


def test_smooth_planar_validation_uses_primitive_widths():
    track = Track((CircularArc(30, np.pi), CircularArc(30, np.pi)),
                  Pose(0, 0, 0), 5, 5, True, (3, None), (3, None))
    with pytest.raises(ValueError, match="boundary"):
        build_smooth_racing_line_path(track, np.full(8, 4.0), sample_spacing_m=2.0)


@pytest.mark.parametrize("bad", [0, -1, float("nan"), float("inf"), "wide", True])
def test_invalid_programmatic_override_width_is_rejected(bad):
    with pytest.raises(ValueError, match="numeric, finite, and positive"):
        Track((Straight(1),), primitive_width_left_m=(bad,))


@pytest.mark.parametrize("yaml_value", ["0", "-1", ".nan", ".inf", "wide", "true"])
def test_invalid_yaml_override_width_is_rejected(tmp_path, yaml_value):
    path = tmp_path / "track.yaml"
    path.write_text(f"""width_left_m: 4\nwidth_right_m: 4\nprimitives:\n  - type: straight\n    length_m: 1\n    width_left_m: {yaml_value}\n""")
    with pytest.raises(ValueError, match="numeric, finite, and positive"):
        Track.from_yaml(path)
