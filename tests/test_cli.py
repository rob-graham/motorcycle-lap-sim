from pathlib import Path
import subprocess
import sys
import tomllib
from types import SimpleNamespace

import pytest

from motorcycle_lap_sim import cli
from motorcycle_lap_sim.optimisation import (
    COARSE_PLANAR_CONTROL_POLICY, FINE_PLANAR_CONTROL_POLICY,
    REFERENCE_PLANAR_CONTROL_POLICY, PlanarOptimisationConfig,
    write_planar_controls_csv)
from motorcycle_lap_sim.runoff import retained_export


ROOT = Path(__file__).resolve().parents[1]


@pytest.mark.parametrize("arguments", [
    ["--help"],
    ["--version"],
    ["optimise", "--help"],
    ["export", "--help"],
    ["export", "runoff", "--help"],
])
def test_help_and_version_succeed(arguments):
    with pytest.raises(SystemExit) as raised:
        cli.main(arguments)
    assert raised.value.code == 0


def test_missing_optimise_files_fail_clearly(tmp_path, capsys):
    motorcycle = tmp_path / "motorcycle.yaml"
    motorcycle.write_text("test", encoding="utf-8")
    with pytest.raises(SystemExit) as raised:
        cli.main(["optimise", str(tmp_path / "missing track.yaml"), str(motorcycle)])
    assert raised.value.code == 2
    assert "track file does not exist" in capsys.readouterr().err

    track = tmp_path / "track.yaml"
    track.write_text("test", encoding="utf-8")
    with pytest.raises(SystemExit) as raised:
        cli.main(["optimise", str(track), str(tmp_path / "missing motorcycle.yaml")])
    assert raised.value.code == 2
    assert "motorcycle file does not exist" in capsys.readouterr().err


def _fake_optimisation(monkeypatch, tmp_path):
    track_file = tmp_path / "my track.yaml"
    motorcycle_file = tmp_path / "my motorcycle.yaml"
    track_file.write_text("track", encoding="utf-8")
    motorcycle_file.write_text("motorcycle", encoding="utf-8")
    track, motorcycle = object(), object()
    monkeypatch.setattr(cli.Track, "from_yaml", lambda path: track)
    monkeypatch.setattr(cli, "load_motorcycle_config", lambda path: motorcycle)
    result = SimpleNamespace(
        control_s_m=[0.0, 10.0], best_controls_m=[1.0, -1.0],
        lower_bounds_m=[-2.0, -2.0], upper_bounds_m=[2.0, 2.0],
        initial_lap_time_s=10.0, best_lap_time_s=9.0, improvement_s=1.0,
        evaluations=5, sweeps=1, final_step_m=0.5,
        termination_reason="maximum sweeps reached",
        minimum_boundary_clearance_m=0.25)
    calls = []
    monkeypatch.setattr(
        cli, "optimise_planar_racing_line",
        lambda *arguments: calls.append(arguments) or result)
    return track_file, motorcycle_file, track, motorcycle, calls


@pytest.mark.parametrize(("name", "policy"), [
    ("coarse", COARSE_PLANAR_CONTROL_POLICY),
    ("reference", REFERENCE_PLANAR_CONTROL_POLICY),
    ("fine", FINE_PLANAR_CONTROL_POLICY),
])
def test_optimise_delegates_policy_and_returns_zero(tmp_path, monkeypatch, name, policy):
    track_file, motorcycle_file, track, motorcycle, calls = _fake_optimisation(
        monkeypatch, tmp_path)
    output = tmp_path / "results with spaces" / "controls file.csv"
    output.parent.mkdir()

    assert cli.main(["optimise", str(track_file), str(motorcycle_file),
                     "--policy", name, "--output", str(output)]) == 0
    assert calls[0][0:3] == (track, motorcycle, policy)
    assert output.is_file()


def test_optimise_default_output_and_exact_config_defaults(tmp_path, monkeypatch):
    track_file, motorcycle_file, _, _, calls = _fake_optimisation(monkeypatch, tmp_path)
    monkeypatch.chdir(tmp_path)

    assert cli.main(["optimise", str(track_file), str(motorcycle_file)]) == 0
    assert (tmp_path / "my track_controls.csv").is_file()
    assert calls[0][3] == PlanarOptimisationConfig()


def test_optimise_maps_every_config_option(tmp_path, monkeypatch):
    track_file, motorcycle_file, _, _, calls = _fake_optimisation(monkeypatch, tmp_path)
    cli.main(["optimise", str(track_file), str(motorcycle_file),
              "--initial-step-m", "2", "--minimum-step-m", ".2",
              "--step-reduction", ".4", "--lap-time-improvement-tolerance-s", ".01",
              "--max-sweeps", "4", "--max-evaluations", "99",
              "--boundary-margin-m", ".3", "--boundary-check-spacing-m", ".4",
              "--optimisation-sample-spacing-m", ".5", "--workers", "2",
              "--speed-backend", "numba", "--output", str(tmp_path / "controls.csv")])
    assert calls[0][3] == PlanarOptimisationConfig(
        initial_step_m=2, minimum_step_m=.2, step_reduction=.4,
        lap_time_improvement_tolerance_s=.01, max_sweeps=4, max_evaluations=99,
        boundary_margin_m=.3, boundary_check_spacing_m=.4,
        optimisation_sample_spacing_m=.5, parallel_workers=2, speed_backend="numba")


def test_controls_writer_exact_header_and_ordered_rows(tmp_path):
    output = tmp_path / "controls.csv"
    result = SimpleNamespace(control_s_m=[0.0, 12.5], best_controls_m=[1.0, -1.5],
                             lower_bounds_m=[-3.0, -4.0], upper_bounds_m=[3.0, 4.0])
    write_planar_controls_csv(output, result)
    assert output.read_text(encoding="utf-8").splitlines() == [
        "index,control_s_m,best_offset_m,lower_bound_m,upper_bound_m",
        "0,0.0,1.0,-3.0,3.0", "1,12.5,-1.5,-4.0,4.0"]


def test_actual_user_command_small_numerical_smoke(tmp_path):
    output = tmp_path / "oval smoke.csv"
    assert cli.main([
        "optimise", str(ROOT / "examples/tracks/test_oval.yaml"),
        str(ROOT / "examples/motorcycles/test_motorcycle.yaml"),
        "--policy", "fine", "--max-sweeps", "1", "--max-evaluations", "1",
        "--output", str(output)]) == 0
    assert output.read_text(encoding="utf-8").startswith(
        "index,control_s_m,best_offset_m,lower_bound_m,upper_bound_m\n")


def test_missing_controls_argument_fails_clearly(capsys):
    with pytest.raises(SystemExit) as raised:
        cli.main(["export", "runoff"])
    assert raised.value.code == 2
    assert "CONTROLS.csv" in capsys.readouterr().err


def test_nonexistent_controls_file_fails_clearly(tmp_path, capsys):
    missing = tmp_path / "missing controls.csv"
    with pytest.raises(SystemExit) as raised:
        cli.main(["export", "runoff", str(missing)])
    assert raised.value.code == 2
    assert "controls file does not exist" in capsys.readouterr().err


def _capture_export(monkeypatch):
    captured = []
    monkeypatch.setattr(retained_export, "run_export", lambda args: captured.append(args))
    return captured


def test_runoff_defaults_and_canonical_settings(tmp_path, monkeypatch):
    controls = tmp_path / "results with spaces" / "controls.csv"
    controls.parent.mkdir()
    controls.write_text("test", encoding="utf-8")
    captured = _capture_export(monkeypatch)

    exit_code = cli.main(["export", "runoff", str(controls)])

    assert exit_code == 0
    args = captured[0]
    assert args.output_dir == controls.parent / "runoff-bundle"
    assert args.georeference_json == (
        ROOT / "examples/tracks/mallala_reference.georeference.json")
    assert args.delete_index == 26
    assert args.margin_m == pytest.approx(0.25)
    assert args.max_roll_rate_radps == pytest.approx(0.8)
    assert args.spacing_m == pytest.approx(0.125)
    assert args.boundary_check_spacing_m == pytest.approx(0.125)
    assert args.expected_lap_s == pytest.approx(71.396583646)
    assert args.lap_tolerance_s == pytest.approx(2e-6)


def test_success_does_not_return_export_package_as_console_exit_status(
        tmp_path, monkeypatch):
    controls = tmp_path / "controls.csv"
    controls.write_text("test", encoding="utf-8")
    package = object()
    monkeypatch.setattr(retained_export, "run_export", lambda args: package)

    assert cli.main(["export", "runoff", str(controls)]) == 0


def test_output_and_georeference_overrides(tmp_path, monkeypatch):
    controls = tmp_path / "controls.csv"
    controls.write_text("test", encoding="utf-8")
    georeference = tmp_path / "custom georeference.json"
    georeference.write_text("{}", encoding="utf-8")
    output = tmp_path / "custom output"
    captured = _capture_export(monkeypatch)

    cli.main(["export", "runoff", str(controls), "--output", str(output),
              "--georeference-json", str(georeference)])

    assert captured[0].output_dir == output
    assert captured[0].georeference_json == georeference


def test_no_georeference_passes_none(tmp_path, monkeypatch):
    controls = tmp_path / "controls.csv"
    controls.write_text("test", encoding="utf-8")
    captured = _capture_export(monkeypatch)

    cli.main(["export", "runoff", str(controls), "--no-georeference"])

    assert captured[0].georeference_json is None


def test_missing_default_georeference_fails_clearly(tmp_path, monkeypatch, capsys):
    controls = tmp_path / "controls.csv"
    controls.write_text("test", encoding="utf-8")
    monkeypatch.setattr(cli, "_default_georeference_path", lambda: tmp_path / "missing.json")
    with pytest.raises(SystemExit) as raised:
        cli.main(["export", "runoff", str(controls)])
    assert raised.value.code == 2
    assert "georeference JSON does not exist" in capsys.readouterr().err


def test_historical_script_help_still_works():
    result = subprocess.run(
        [sys.executable, "scripts/r6_phase12b_runoff_export.py", "--help"],
        cwd=ROOT, capture_output=True, text=True)
    assert result.returncode == 0, result.stderr


def test_existing_console_entry_points_remain_present():
    project = tomllib.loads((ROOT / "pyproject.toml").read_text(encoding="utf-8"))
    scripts = project["project"]["scripts"]
    assert scripts == {
        "motorcycle-lap-sim": "motorcycle_lap_sim.cli:main",
        "plot-example-track": "motorcycle_lap_sim.plotting.track_plot:main",
        "fixed-path-diagnostics": "motorcycle_lap_sim.speed_solver.diagnostics:main",
        "racing-line-diagnostics": "motorcycle_lap_sim.racing_line.diagnostics:main",
        "racing-line-optimisation": "motorcycle_lap_sim.optimisation.diagnostics:main",
    }
