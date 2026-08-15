"""Lightweight tests for the Phase 8 diagnostic command-line selection helpers."""

from pathlib import Path
import math
import runpy
from types import SimpleNamespace

import numpy as np
import pytest


SCRIPT = runpy.run_path(Path(__file__).parents[1] / "scripts/r6_phase8_planar_optimisation_check.py")
build_parser = SCRIPT["build_parser"]
selected_policies = SCRIPT["selected_policies"]
selected_tracks = SCRIPT["selected_tracks"]
half_lap_symmetry_differences = SCRIPT["half_lap_symmetry_differences"]
optimisation_config = SCRIPT["optimisation_config"]
load_initial_controls_csv = SCRIPT["load_initial_controls_csv"]
run_selected_optimisation = SCRIPT["run_selected_optimisation"]
metrics = SCRIPT["metrics"]
project_initial_controls = SCRIPT["project_initial_controls"]
EXTRA_FINE_POLICY = SCRIPT["EXTRA_FINE_POLICY"]
SCRIPT_GLOBALS = run_selected_optimisation.__globals__


def write_controls(path, stations=(0.0, 10.0), controls=(1.0, -1.0),
                   lower=(-2.0, -2.0), upper=(2.0, 2.0), indices=(0, 1)):
    lines = ["index,control_s_m,best_offset_m,lower_bound_m,upper_bound_m"]
    lines += [f"{i},{s},{control},{lo},{hi}" for i, s, control, lo, hi in
              zip(indices, stations, controls, lower, upper)]
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def test_targeted_cli_selection():
    args = build_parser().parse_args(("--track", "oval", "--policy", "fine",
                                      "--max-evaluations", "25", "--max-sweeps", "100",
                                      "--workers", "2"))

    assert args.track == "oval"
    assert args.policy == "fine"
    assert args.max_evaluations == 25
    assert args.max_sweeps == 100
    assert optimisation_config(args).max_sweeps == 100
    assert optimisation_config(args).max_evaluations == 25
    assert optimisation_config(args).parallel_workers == 2
    assert optimisation_config(args).initial_step_m == 1.0
    assert [name for name, _ in selected_tracks(args.track)] == ["oval"]
    assert [name for name, _ in selected_policies(args.policy)] == ["fine"]


def test_extra_fine_is_explicit_policy_with_expected_parameters():
    args = build_parser().parse_args(("--track", "mallala", "--policy", "extra-fine"))
    assert [name for name, _ in selected_policies(args.policy)] == ["extra-fine"]
    assert EXTRA_FINE_POLICY.max_spacing_m == 50.0
    assert EXTRA_FINE_POLICY.max_arc_heading_change_rad == pytest.approx(math.radians(20.0))


def test_parser_accepts_initial_controls_csv():
    args = build_parser().parse_args(("--track", "mallala", "--policy", "reference",
                                      "--initial-controls-csv", "saved.csv"))
    assert args.initial_controls_csv == Path("saved.csv")


def test_projection_cli_options_are_explicit():
    args = build_parser().parse_args((
        "--track", "mallala", "--policy", "extra-fine",
        "--project-source-policy", "reference",
        "--project-initial-controls-csv", "saved.csv"))
    assert args.project_initial_controls_csv == Path("saved.csv")
    assert args.project_source_policy == "reference"


def test_projection_and_restart_are_mutually_exclusive(capsys):
    with pytest.raises(SystemExit):
        build_parser().parse_args((
            "--track", "mallala", "--policy", "fine",
            "--initial-controls-csv", "restart.csv",
            "--project-initial-controls-csv", "source.csv",
            "--project-source-policy", "coarse"))
    assert "mutually exclusive" in capsys.readouterr().err


@pytest.mark.parametrize("option,value,message", [
    ("--track", "both", "one explicit --track"),
    ("--policy", "all", "one explicit target --policy"),
])
def test_projection_rejects_ambiguous_target(option, value, message, capsys):
    arguments = ["--track", "oval", "--policy", "fine", option, value,
                 "--project-initial-controls-csv", "saved.csv",
                 "--project-source-policy", "coarse"]
    with pytest.raises(SystemExit):
        build_parser().parse_args(arguments)
    assert message in capsys.readouterr().err


def test_projection_requires_source_policy(capsys):
    with pytest.raises(SystemExit):
        build_parser().parse_args((
            "--track", "oval", "--policy", "fine",
            "--project-initial-controls-csv", "saved.csv"))
    assert "--project-source-policy is required" in capsys.readouterr().err


def test_initial_step_cli_is_passed_to_optimisation_config():
    args = build_parser().parse_args(("--initial-step-m", "0.25"))

    assert optimisation_config(args).initial_step_m == 0.25


@pytest.mark.parametrize("value", ["0", "-0.25"])
def test_non_positive_initial_step_uses_config_validation(value):
    args = build_parser().parse_args(("--initial-step-m", value))

    with pytest.raises(ValueError, match="step and spacing values must be finite and positive"):
        optimisation_config(args)


@pytest.mark.parametrize("option,value,message", [
    ("--track", "both", "requires --track oval or mallala"),
    ("--policy", "all", "requires --policy coarse, reference, or fine"),
])
def test_restart_rejects_ambiguous_selection(option, value, message, capsys):
    arguments = ["--track", "oval", "--policy", "fine", option, value,
                 "--initial-controls-csv", "saved.csv"]
    with pytest.raises(SystemExit):
        build_parser().parse_args(arguments)
    assert message in capsys.readouterr().err


def test_valid_controls_csv_loads(tmp_path):
    path = tmp_path / "controls.csv"
    write_controls(path)
    loaded = load_initial_controls_csv(path, [0.0, 10.0], [-2.0, -2.0], [2.0, 2.0])
    assert np.array_equal(loaded, [1.0, -1.0])


@pytest.mark.parametrize("change,message", [
    ({"stations": (0.0,)}, "row count"),
    ({"indices": (1, 0)}, "sequential index"),
    ({"controls": (float("nan"), -1.0)}, "non-finite"),
    ({"stations": (0.0, 10.1)}, "control_s_m"),
    ({"lower": (-2.0, -2.1)}, "lower_bound_m"),
    ({"upper": (2.0, 2.1)}, "upper_bound_m"),
    ({"controls": (3.0, -1.0)}, "outside current bounds"),
])
def test_invalid_controls_csv_is_rejected(tmp_path, change, message):
    path = tmp_path / "controls.csv"
    write_controls(path, **change)
    with pytest.raises(ValueError, match=message):
        load_initial_controls_csv(path, [0.0, 10.0], [-2.0, -2.0], [2.0, 2.0])


def test_projection_validates_source_layout_and_uses_spline_normal_projection(tmp_path):
    track = SCRIPT["Track"].from_yaml("examples/tracks/test_oval.yaml")
    source_policy = SCRIPT["FINE_PLANAR_CONTROL_POLICY"]
    target_policy = EXTRA_FINE_POLICY
    source_s = SCRIPT["generate_planar_control_stations"](track, source_policy)
    source_lower, source_upper = SCRIPT["planar_control_bounds"](track, source_s, 0.25)
    source_controls = np.zeros(len(source_s))
    path = tmp_path / "source.csv"
    write_controls(path, source_s, source_controls, source_lower, source_upper,
                   np.arange(len(source_s)))

    projected, loaded_source_s, target_s, target_line, _ = project_initial_controls(
        track, source_policy, target_policy, path, 0.25)
    source_line = SCRIPT["build_smooth_racing_line_path"](
        track, source_controls, guide_s_m=source_s, sample_spacing_m=1.0,
        boundary_margin_m=0.25, boundary_check_spacing_m=0.25)
    exact_track = SCRIPT["sample_track_stations"](track, target_s)
    x, y, *_ = source_line.spline.evaluate(target_s)
    expected = ((x - exact_track.x_m) * exact_track.normal_x
                + (y - exact_track.y_m) * exact_track.normal_y)
    assert np.array_equal(loaded_source_s, source_s)
    assert np.allclose(projected, expected, rtol=0.0, atol=1e-13)
    # Interpolating the constant source controls would return exactly zero.
    assert np.max(np.abs(projected)) > 1e-6
    lower, upper = SCRIPT["planar_control_bounds"](track, target_s, 0.25)
    assert np.all(projected >= lower) and np.all(projected <= upper)
    assert target_line.minimum_boundary_clearance_m >= -1e-10
    assert target_line.minimum_forward_progress > 0.0

    wrong_layout = tmp_path / "wrong.csv"
    target_lower, target_upper = SCRIPT["planar_control_bounds"](track, target_s, 0.25)
    write_controls(wrong_layout, target_s, np.ones(len(target_s)), target_lower,
                   target_upper, np.arange(len(target_s)))
    with pytest.raises(ValueError, match="row count"):
        project_initial_controls(track, source_policy, target_policy, wrong_layout, 0.25)


def test_out_of_bounds_projection_fails_without_clipping(monkeypatch, tmp_path):
    path = tmp_path / "source.csv"
    write_controls(path, stations=(0., 1., 2., 3.), controls=(0.,) * 4,
                   lower=(-1.,) * 4, upper=(1.,) * 4, indices=range(4))
    fake_track = SimpleNamespace()
    monkeypatch.setitem(SCRIPT_GLOBALS, "generate_planar_control_stations",
                        lambda track, policy: np.array([0., 1., 2., 3.]))
    monkeypatch.setitem(SCRIPT_GLOBALS, "planar_control_bounds",
                        lambda track, stations, margin: (np.full(4, -1.), np.full(4, 1.)))
    sampled = SimpleNamespace(x_m=np.zeros(4), y_m=np.zeros(4),
                              normal_x=np.ones(4), normal_y=np.zeros(4))
    monkeypatch.setitem(SCRIPT_GLOBALS, "sample_track_stations", lambda *args: sampled)
    spline = SimpleNamespace(evaluate=lambda stations: (
        np.full(4, 2.), np.zeros(4), np.zeros(4), np.zeros(4), np.zeros(4), np.zeros(4)))
    monkeypatch.setitem(SCRIPT_GLOBALS, "build_smooth_racing_line_path",
                        lambda *args, **kwargs: SimpleNamespace(spline=spline))

    with pytest.raises(ValueError, match=r"index 0.*station 0.*value 2.*upper bound 1"):
        project_initial_controls(fake_track, object(), object(), path, 0.0)


def test_no_restart_preserves_zero_start_and_metric_wording(monkeypatch):
    args = build_parser().parse_args(("--track", "oval", "--policy", "fine"))
    seen = {}
    monkeypatch.setitem(SCRIPT_GLOBALS, "timed_optimisation",
                        lambda *args, initial_controls_m=None: seen.setdefault(
                            "initial", initial_controls_m) or object())
    monkeypatch.setitem(SCRIPT_GLOBALS, "metrics",
                        lambda label, result, restarted=False: seen.setdefault("restarted", restarted))
    result = run_selected_optimisation(object(), args, object(), object(), "fine",
                                       object(), SimpleNamespace(boundary_margin_m=0.25), [0.0])
    assert result is not None
    assert seen == {"initial": None, "restarted": False}


def test_restart_passes_loaded_controls_and_uses_restart_metric(monkeypatch, tmp_path):
    path = tmp_path / "controls.csv"
    write_controls(path)
    args = build_parser().parse_args(("--track", "oval", "--policy", "fine",
                                      "--initial-controls-csv", str(path)))
    seen = {}
    result = object()

    def fake_timed(*args, initial_controls_m=None):
        seen["initial"] = initial_controls_m
        return result

    monkeypatch.setitem(SCRIPT_GLOBALS, "planar_control_bounds",
                        lambda track, stations, margin: (np.array([-2., -2.]), np.array([2., 2.])))
    monkeypatch.setitem(SCRIPT_GLOBALS, "timed_optimisation", fake_timed)
    monkeypatch.setitem(SCRIPT_GLOBALS, "metrics",
                        lambda label, result, restarted=False: seen.setdefault("restarted", restarted))
    run_selected_optimisation(object(), args, object(), object(), "fine", object(),
                              SimpleNamespace(boundary_margin_m=0.25), np.array([0., 10.]))
    assert np.array_equal(seen["initial"], [1.0, -1.0])
    assert seen["restarted"] is True


def test_restarted_metrics_does_not_describe_initial_candidate_as_zero(capsys):
    path = SimpleNamespace(total_length_m=10.0, curvature_1pm=np.array([0.1]))
    speed = SimpleNamespace(curvature_gradient_1pm2=np.array([0.0]),
                            curvature_rate_1pmps=np.array([0.0]))
    result = SimpleNamespace(sampled_path=path, speed_profile=speed,
                             control_s_m=np.array([0.0]), initial_lap_time_s=2.0,
                             best_lap_time_s=1.5, improvement_s=0.5, evaluations=2,
                             sweeps=1, minimum_boundary_clearance_m=1.0,
                             minimum_forward_progress=1.0, best_controls_m=np.array([0.5]),
                             termination_reason="test")
    metrics("fine", result, restarted=True)
    output = capsys.readouterr().out
    assert "initial_lap_s=2.000000000" in output
    assert "zero_lap_s" not in output


def test_complete_diagnostic_is_default():
    args = build_parser().parse_args(())

    assert [name for name, _ in selected_tracks(args.track)] == ["oval", "mallala"]
    assert [name for name, _ in selected_policies(args.policy)] == [
        "coarse", "reference", "fine",
    ]
    assert args.workers == 1
    assert args.initial_step_m == 1.0


def test_half_lap_symmetry_differences_require_repeated_even_station_layout():
    stations = np.array([0.0, 2.0, 5.0, 10.0, 12.0, 15.0])
    controls = np.array([1.0, 2.0, 4.0, 0.5, 3.0, 2.0])

    assert np.array_equal(half_lap_symmetry_differences(stations, controls, 20.0),
                          [0.5, -1.0, 2.0])
    assert half_lap_symmetry_differences(stations[:-1], controls[:-1], 20.0) is None
    assert half_lap_symmetry_differences(stations + [0, 0, 0, 0, 0, 0.1],
                                         controls, 20.0) is None
