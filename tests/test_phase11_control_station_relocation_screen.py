import importlib.util
from pathlib import Path

import numpy as np
import pytest

from motorcycle_lap_sim.optimisation import (
    REFERENCE_PLANAR_CONTROL_POLICY,
    generate_planar_control_stations,
)
from motorcycle_lap_sim.track import Track


SCRIPT = Path(__file__).resolve().parents[1] / "scripts" / "r6_phase11_control_station_relocation_screen.py"
SPEC = importlib.util.spec_from_file_location("phase11_control_station_relocation_screen_test", SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
SPEC.loader.exec_module(MODULE)


def mallala():
    return Track.from_yaml(Path(__file__).resolve().parents[1] / "examples" / "tracks" / "mallala_reference.yaml")


def test_parser_defaults_keep_bounded_screen_small():
    args = MODULE.build_parser().parse_args(["controls.csv", "out"])
    assert args.margin_m == pytest.approx(0.25)
    assert args.screen_shifts_m == MODULE.DEFAULT_SCREEN_SHIFTS_M
    assert args.minimum_station_gap_m == pytest.approx(5.0)
    assert args.optimisation_spacing_m == pytest.approx(1.0)
    assert args.common_spacing_m == pytest.approx(0.125)
    assert args.boundary_check_spacing_m == pytest.approx(0.125)


def test_relocation_protects_primitive_boundaries_but_allows_periodic_seam():
    track = mallala()
    stations = generate_planar_control_stations(track, REFERENCE_PLANAR_CONTROL_POLICY)
    eligible = MODULE.relocation_eligible_mask(track, stations)
    boundaries = MODULE.phase11screen.primitive_boundary_stations(track)
    assert stations[0] == pytest.approx(0.0)
    assert eligible[0]
    for index, station in enumerate(stations[1:], start=1):
        if np.any(np.isclose(station, boundaries, rtol=0.0, atol=1e-9)):
            assert not eligible[index]


def test_seam_station_can_move_forward_without_crossing_neighbour():
    track = mallala()
    stations = generate_planar_control_stations(track, REFERENCE_PLANAR_CONTROL_POLICY)
    moved = MODULE.relocated_stations(stations, 0, 20.0, track.total_length_m, 5.0)
    assert moved is not None
    assert moved[0] == pytest.approx(20.0)
    assert np.all(np.diff(moved) > 0.0)


def test_relocation_rejects_shift_that_crosses_bounded_interval():
    track = mallala()
    stations = generate_planar_control_stations(track, REFERENCE_PLANAR_CONTROL_POLICY)
    assert MODULE.relocated_stations(
        stations, 0, float(stations[1]), track.total_length_m, 5.0) is None


def test_start_finish_segment_spans_track_width_at_zero():
    track = mallala()
    left, right = MODULE._start_finish_segment(track)
    assert left[0] == pytest.approx(0.0)
    assert right[0] == pytest.approx(0.0)
    assert np.hypot(left[0] - right[0], left[1] - right[1]) == pytest.approx(10.0)
