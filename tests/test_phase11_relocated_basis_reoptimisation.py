import importlib.util
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest


SCRIPT = Path(__file__).resolve().parents[1] / "scripts" / "r6_phase11_relocated_basis_reoptimisation.py"
spec = importlib.util.spec_from_file_location("phase11_relocated_basis_reopt_test", SCRIPT)
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


def test_parser_defaults():
    args = module.build_parser().parse_args(["controls.csv", "output"])
    assert args.relocate_index == 27
    assert args.relocate_shift_m == 5.0
    assert args.minimum_station_gap_m == 5.0
    assert args.margin_m == 0.25
    assert args.max_roll_rate_radps == 0.8
    assert args.initial_step_m == 0.125
    assert args.minimum_step_m == 0.0625
    assert args.max_sweeps == 12
    assert args.max_evaluations == 4000
    assert args.workers == 1
    assert args.optimisation_spacing_m == 1.0
    assert args.common_spacing_m == 0.125
    assert args.boundary_check_spacing_m == 0.125
    assert args.plot_dpi == 400


def test_relocated_basis_moves_only_requested_station():
    stations = np.array([0.0, 20.0, 40.0, 60.0])
    moved = module.relocated_basis(stations, 2, 5.0, 80.0, 5.0)
    np.testing.assert_allclose(moved, [0.0, 20.0, 45.0, 60.0])
    np.testing.assert_allclose(stations, [0.0, 20.0, 40.0, 60.0])


def test_relocated_basis_rejects_neighbour_crossing():
    stations = np.array([0.0, 20.0, 40.0, 60.0])
    with pytest.raises(ValueError, match="bounded neighbour interval"):
        module.relocated_basis(stations, 2, 16.0, 80.0, 5.0)


def test_relocation_shift_must_be_nonzero_in_main(tmp_path):
    with pytest.raises(ValueError, match="finite and non-zero"):
        module.main(["missing.csv", str(tmp_path), "--relocate-shift-m", "0"])


def test_compact_discards_heavy_candidate_results():
    evaluation = module.PlanarObjectiveEvaluation(
        True, 71.25, smooth_line=object(), speed_profile=object())

    compact = module._compact(evaluation)

    assert compact.feasible
    assert compact.lap_time_s == 71.25
    assert compact.smooth_line is None
    assert compact.speed_profile is None


def test_guide_point_uses_singular_stored_offset_field(monkeypatch):
    sampled = SimpleNamespace(
        x_m=np.array([10.0]), y_m=np.array([20.0]),
        normal_x=np.array([0.0]), normal_y=np.array([1.0]))
    monkeypatch.setattr(module, "sample_track_stations", lambda track, stations: sampled)
    smooth_line = SimpleNamespace(guide_offset_m=np.array([1.5, -2.0]))

    point = module._guide_point(object(), smooth_line, 1, 40.0)

    assert point == (10.0, 18.0)


def test_racing_line_plot_uses_singular_stored_offset_field(tmp_path):
    track = module.Track.from_yaml("examples/tracks/test_oval.yaml")
    checked_s = np.array([0.0, 20.0, 40.0])
    checked_track = module.sample_track_stations(track, checked_s)

    spline = SimpleNamespace(evaluate=lambda stations: (
        module.sample_track_stations(track, stations).x_m,
        module.sample_track_stations(track, stations).y_m,
        np.ones(len(stations)),
        np.zeros(len(stations)),
    ))
    smooth_line = SimpleNamespace(
        evaluated_track_s_m=checked_s,
        spline=spline,
        guide_offset_m=np.array([0.0]),
        guide_x_m=np.array([checked_track.x_m[0]]),
        guide_y_m=np.array([checked_track.y_m[0]]),
    )
    evaluation = SimpleNamespace(smooth_line=smooth_line)
    output = tmp_path / "line.png"

    module._write_racing_line_png(
        output,
        track,
        evaluation,
        evaluation,
        0,
        0.0,
        1.0,
        margin_m=0.25,
        dpi=50,
    )

    assert output.is_file()
    assert output.stat().st_size > 0
