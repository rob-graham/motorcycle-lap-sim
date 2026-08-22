from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

from motorcycle_lap_sim import cli
from motorcycle_lap_sim.optimisation import load_planar_controls_csv


def _result():
    return SimpleNamespace(
        control_s_m=[0.0, 10.0], best_controls_m=[1.0, -1.0],
        lower_bounds_m=[-2.0, -2.0], upper_bounds_m=[2.0, 2.0],
        initial_lap_time_s=10.0, best_lap_time_s=9.0, improvement_s=1.0,
        evaluations=5, sweeps=1, final_step_m=0.5,
        termination_reason="maximum sweeps reached",
        minimum_boundary_clearance_m=0.25)


def _input_files(tmp_path):
    track = tmp_path / "track.yaml"
    motorcycle = tmp_path / "motorcycle.yaml"
    track.write_text("track", encoding="utf-8")
    motorcycle.write_text("motorcycle", encoding="utf-8")
    return track, motorcycle


def test_restart_aliases_parse_to_same_destination():
    parser = cli.build_parser()
    a = parser.parse_args([
        "optimise", "track.yaml", "motorcycle.yaml",
        "--restart-controls", "saved.csv"])
    b = parser.parse_args([
        "optimise", "track.yaml", "motorcycle.yaml",
        "--initial-controls-csv", "saved.csv"])
    assert a.initial_controls_csv == Path("saved.csv")
    assert b.initial_controls_csv == Path("saved.csv")


def test_missing_restart_controls_fail_clearly(tmp_path, capsys):
    track, motorcycle = _input_files(tmp_path)
    missing = tmp_path / "missing.csv"
    with pytest.raises(SystemExit) as raised:
        cli.main([
            "optimise", str(track), str(motorcycle),
            "--restart-controls", str(missing)])
    assert raised.value.code == 2
    assert "restart controls file does not exist" in capsys.readouterr().err


def test_restart_controls_are_loaded_and_passed_to_optimiser(tmp_path, monkeypatch):
    track_file, motorcycle_file = _input_files(tmp_path)
    restart = tmp_path / "saved.csv"
    restart.write_text("placeholder", encoding="utf-8")
    output = tmp_path / "output.csv"

    track = object()
    motorcycle = object()
    stations = np.array([0.0, 10.0])
    lower = np.array([-2.0, -2.0])
    upper = np.array([2.0, 2.0])
    controls = np.array([0.75, -0.5])
    captured = {}

    monkeypatch.setattr(cli.Track, "from_yaml", lambda path: track)
    monkeypatch.setattr(cli, "load_motorcycle_config", lambda path: motorcycle)
    monkeypatch.setattr(cli, "generate_planar_control_stations", lambda *args: stations)
    monkeypatch.setattr(cli, "planar_control_bounds", lambda *args: (lower, upper))

    def load(path, actual_stations, actual_lower, actual_upper):
        assert path == restart.resolve()
        assert np.array_equal(actual_stations, stations)
        assert np.array_equal(actual_lower, lower)
        assert np.array_equal(actual_upper, upper)
        return controls

    def optimise(actual_track, actual_motorcycle, policy, config, *, initial_controls_m):
        captured["track"] = actual_track
        captured["motorcycle"] = actual_motorcycle
        captured["policy"] = policy
        captured["config"] = config
        captured["controls"] = initial_controls_m
        return _result()

    monkeypatch.setattr(cli, "load_planar_controls_csv", load)
    monkeypatch.setattr(cli, "optimise_planar_racing_line", optimise)

    assert cli.main([
        "optimise", str(track_file), str(motorcycle_file),
        "--restart-controls", str(restart), "--output", str(output)]) == 0
    assert captured["track"] is track
    assert captured["motorcycle"] is motorcycle
    assert np.array_equal(captured["controls"], controls)
    assert output.is_file()


def test_incompatible_restart_controls_fail_as_cli_error(tmp_path, monkeypatch, capsys):
    track_file, motorcycle_file = _input_files(tmp_path)
    restart = tmp_path / "saved.csv"
    restart.write_text("placeholder", encoding="utf-8")

    monkeypatch.setattr(cli.Track, "from_yaml", lambda path: object())
    monkeypatch.setattr(cli, "load_motorcycle_config", lambda path: object())
    monkeypatch.setattr(cli, "generate_planar_control_stations", lambda *args: np.array([0.0]))
    monkeypatch.setattr(
        cli, "planar_control_bounds",
        lambda *args: (np.array([-1.0]), np.array([1.0])))
    monkeypatch.setattr(
        cli, "load_planar_controls_csv",
        lambda *args: (_ for _ in ()).throw(ValueError("station mismatch")))

    with pytest.raises(SystemExit) as raised:
        cli.main([
            "optimise", str(track_file), str(motorcycle_file),
            "--restart-controls", str(restart)])
    assert raised.value.code == 2
    assert "restart controls are incompatible: station mismatch" in capsys.readouterr().err


def _write_controls(path, rows):
    path.write_text(
        "index,control_s_m,best_offset_m,lower_bound_m,upper_bound_m\n"
        + "\n".join(rows) + "\n",
        encoding="utf-8")


def test_planar_controls_loader_accepts_exact_layout(tmp_path):
    path = tmp_path / "controls.csv"
    _write_controls(path, [
        "0,0.0,0.75,-2.0,2.0",
        "1,10.0,-0.5,-3.0,3.0",
    ])
    controls = load_planar_controls_csv(
        path, np.array([0.0, 10.0]), np.array([-2.0, -3.0]), np.array([2.0, 3.0]))
    assert np.array_equal(controls, np.array([0.75, -0.5]))


@pytest.mark.parametrize(("rows", "message"), [
    (["0,0.1,0.75,-2.0,2.0", "1,10.0,-0.5,-3.0,3.0"], "control_s_m"),
    (["0,0.0,0.75,-2.1,2.0", "1,10.0,-0.5,-3.0,3.0"], "lower_bound_m"),
    (["0,0.0,2.5,-2.0,2.0", "1,10.0,-0.5,-3.0,3.0"], "outside current bounds"),
])
def test_planar_controls_loader_rejects_incompatible_restart(tmp_path, rows, message):
    path = tmp_path / "controls.csv"
    _write_controls(path, rows)
    with pytest.raises(ValueError, match=message):
        load_planar_controls_csv(
            path, np.array([0.0, 10.0]), np.array([-2.0, -3.0]), np.array([2.0, 3.0]))
