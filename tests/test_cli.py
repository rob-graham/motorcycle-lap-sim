from pathlib import Path
import subprocess
import sys
import tomllib

import pytest

from motorcycle_lap_sim import cli
from motorcycle_lap_sim.runoff import retained_export


ROOT = Path(__file__).resolve().parents[1]


@pytest.mark.parametrize("arguments", [
    ["--help"],
    ["--version"],
    ["export", "--help"],
    ["export", "runoff", "--help"],
])
def test_help_and_version_succeed(arguments):
    with pytest.raises(SystemExit) as raised:
        cli.main(arguments)
    assert raised.value.code == 0


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

    cli.main(["export", "runoff", str(controls)])

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
