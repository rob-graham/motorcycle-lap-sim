import importlib.util
from pathlib import Path

import numpy as np
import pytest

from motorcycle_lap_sim.optimisation import (
    REFERENCE_PLANAR_CONTROL_POLICY,
    generate_planar_control_stations,
    planar_control_bounds,
)
from motorcycle_lap_sim.track import Track


SCRIPT_PATH = Path(__file__).parents[1] / "scripts" / "r6_phase9_baseline_check.py"
SPEC = importlib.util.spec_from_file_location("r6_phase9_baseline_check", SCRIPT_PATH)
MODULE = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
SPEC.loader.exec_module(MODULE)
ROOT = Path(__file__).parents[1]


def test_frozen_controls_have_expected_identity_station_policy_and_bounds():
    controls_path = ROOT / MODULE.DEFAULT_CONTROLS
    track_path = ROOT / MODULE.DEFAULT_TRACK
    track = Track.from_yaml(track_path)
    stations = generate_planar_control_stations(track, REFERENCE_PLANAR_CONTROL_POLICY)
    lower, upper = planar_control_bounds(track, stations, MODULE.BOUNDARY_MARGIN_M)

    controls = MODULE.load_frozen_controls(controls_path, stations, lower, upper)

    assert MODULE.sha256_file(controls_path) == MODULE.EXPECTED_CONTROLS_SHA256
    assert len(stations) == MODULE.EXPECTED_CONTROL_COUNT == 52
    assert np.all(controls >= lower)
    assert np.all(controls <= upper)


def test_frozen_controls_reject_changed_stored_bound(tmp_path):
    source = ROOT / MODULE.DEFAULT_CONTROLS
    text = source.read_text(encoding="utf-8")
    lines = text.splitlines()
    fields = lines[1].split(",")
    fields[3] = str(float(fields[3]) - 0.1)
    lines[1] = ",".join(fields)
    changed = tmp_path / "changed.csv"
    changed.write_text("\n".join(lines) + "\n", encoding="utf-8")

    track = Track.from_yaml(ROOT / MODULE.DEFAULT_TRACK)
    stations = generate_planar_control_stations(track, REFERENCE_PLANAR_CONTROL_POLICY)
    lower, upper = planar_control_bounds(track, stations, MODULE.BOUNDARY_MARGIN_M)

    with pytest.raises(ValueError, match="lower bounds"):
        MODULE.load_frozen_controls(changed, stations, lower, upper)


def test_frozen_baseline_fixed_geometry_evaluates_at_all_documented_resolutions(monkeypatch):
    monkeypatch.chdir(ROOT)
    _, stations, controls, evaluations = MODULE.evaluate_baseline(speed_backend="python")

    assert len(stations) == len(controls) == 52
    assert len(evaluations) == len(MODULE.OUTPUT_SPACINGS_M) == 3
    assert all(evaluation.feasible for evaluation in evaluations)
    assert all(np.isfinite(evaluation.lap_time_s) for evaluation in evaluations)
    assert all(evaluation.lap_time_s > 0.0 for evaluation in evaluations)
