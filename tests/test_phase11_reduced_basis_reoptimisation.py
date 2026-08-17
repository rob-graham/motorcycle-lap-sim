import importlib.util
from pathlib import Path

import numpy as np
import pytest


SCRIPT = Path(__file__).resolve().parents[1] / "scripts" / "r6_phase11_reduced_basis_reoptimisation.py"
spec = importlib.util.spec_from_file_location("phase11_reduced_basis_reoptimisation", SCRIPT)
phase11 = importlib.util.module_from_spec(spec)
assert spec.loader is not None
spec.loader.exec_module(phase11)


def test_parser_defaults_match_reduced_basis_contract():
    args = phase11.build_parser().parse_args(["controls.csv", "out"])

    assert args.delete_index == 26
    assert args.margin_m == 0.25
    assert args.max_roll_rate_radps == 0.8
    assert args.initial_step_m == 0.125
    assert args.minimum_step_m == 0.0625
    assert args.max_sweeps == 12
    assert args.max_evaluations == 4000
    assert args.workers == 16
    assert args.speed_backend == "numba"
    assert args.optimisation_spacing_m == 1.0
    assert args.common_spacing_m == 0.125
    assert args.boundary_check_spacing_m == 0.125
    assert args.plot_dpi == 400


def test_reduced_basis_removes_selected_station_and_control():
    stations = np.array([0.0, 10.0, 20.0, 30.0, 40.0])
    controls = np.array([1.0, 2.0, 3.0, 4.0, 5.0])

    reduced_s, reduced_controls = phase11.reduced_basis(stations, controls, 2)

    assert np.array_equal(reduced_s, [0.0, 10.0, 30.0, 40.0])
    assert np.array_equal(reduced_controls, [1.0, 2.0, 4.0, 5.0])


def test_start_finish_segment_spans_reference_track_edges():
    track = phase11.Track.from_yaml(phase11.phase9.DEFAULT_TRACK)
    left, right = phase11._start_finish_segment(track)
    sampled = phase11.sample_track_stations(track, np.array([0.0]))

    centre = np.array([sampled.x_m[0], sampled.y_m[0]])
    normal = np.array([sampled.normal_x[0], sampled.normal_y[0]])
    expected_left = centre + sampled.width_left_m[0] * normal
    expected_right = centre - sampled.width_right_m[0] * normal

    assert np.array(left) == pytest.approx(expected_left)
    assert np.array(right) == pytest.approx(expected_right)


def test_compact_worker_result_drops_heavy_artifacts():
    evaluation = phase11.PlanarObjectiveEvaluation(
        feasible=True,
        lap_time_s=71.5,
        smooth_line=object(),
        speed_profile=object(),
    )

    compact = phase11._compact(evaluation)

    assert compact.feasible
    assert compact.lap_time_s == pytest.approx(71.5)
    assert compact.smooth_line is None
    assert compact.speed_profile is None


def test_reference_reduced_basis_has_51_controls_after_deleting_26():
    track = phase11.Track.from_yaml(phase11.phase9.DEFAULT_TRACK)
    stations = phase11.generate_planar_control_stations(
        track, phase11.REFERENCE_PLANAR_CONTROL_POLICY)
    controls = np.zeros(len(stations))

    reduced_s, reduced_controls = phase11.reduced_basis(stations, controls, 26)

    assert len(stations) == 52
    assert len(reduced_s) == 51
    assert len(reduced_controls) == 51
    assert not np.any(np.isclose(reduced_s, stations[26], rtol=0.0, atol=1e-12))
