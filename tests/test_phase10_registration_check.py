import csv
import importlib.util
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

from motorcycle_lap_sim.telemetry.repeatability import CrossLapEnvelope
from motorcycle_lap_sim.track import Straight, Track


SCRIPT_PATH = Path(__file__).parents[1] / "scripts" / "r6_phase10_registration_check.py"
SPEC = importlib.util.spec_from_file_location("r6_phase10_registration_check", SCRIPT_PATH)
MODULE = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
SPEC.loader.exec_module(MODULE)


def _envelope(median, p10, p90, lap_count):
    median = np.asarray(median, dtype=float)
    p10 = np.asarray(p10, dtype=float)
    p90 = np.asarray(p90, dtype=float)
    lap_count = np.asarray(lap_count, dtype=np.int64)
    count = len(median)
    chainage = np.arange(count, dtype=float) * 10.0 + 5.0
    return CrossLapEnvelope(
        chainage_m=chainage,
        median=median,
        p10=p10,
        p90=p90,
        minimum=p10.copy(),
        maximum=p90.copy(),
        lap_count=lap_count,
        per_lap_values=np.full((3, count), np.nan),
    )


def _track(length_m):
    return Track((Straight(length_m),), width_left_m=4.0, width_right_m=4.0, closed=True)


def test_registration_guard_rejects_nonconverged_result():
    result = SimpleNamespace(converged=False)
    with pytest.raises(RuntimeError, match="did not converge"):
        MODULE._require_converged_registration(result)


def test_corridor_ignores_empty_and_incomplete_bins(capsys):
    envelope = _envelope(
        [np.nan, 6.0, 7.0, 0.0],
        [np.nan, 5.0, 6.0, -1.0],
        [np.nan, 7.0, 8.0, 1.0],
        [0, 3, 2, 3],
    )
    eligible = envelope.lap_count == 3

    _, corridor_eligible, excess, touches = MODULE._corridor_diagnostics(
        _track(40.0), envelope, eligible)

    assert np.array_equal(corridor_eligible, [False, True, False, True])
    assert np.isnan(excess[0])
    assert excess[1] == pytest.approx(2.0)
    assert np.isnan(excess[2])
    assert excess[3] == pytest.approx(0.0)
    assert np.array_equal(touches, [False, True, False, False])
    output = capsys.readouterr().out
    assert "corridor_eligible_bins=2/4" in output
    assert "median_outside_model_corridor_bins=1/2" in output
    assert "maximum_median_model_corridor_excess_m=2.000000000" in output


def test_corridor_reports_no_eligible_bins(capsys):
    envelope = _envelope(
        [np.nan, np.nan], [np.nan, np.nan], [np.nan, np.nan], [0, 0])

    _, eligible, excess, touches = MODULE._corridor_diagnostics(
        _track(20.0), envelope, np.array([False, False]))

    assert not np.any(eligible)
    assert np.all(np.isnan(excess))
    assert not np.any(touches)
    output = capsys.readouterr().out
    assert "corridor_eligible_bins=0/2" in output
    assert "maximum_median_model_corridor_excess_m=not_available" in output


def test_circular_flag_runs_merges_start_finish_sector():
    runs = MODULE._circular_flag_runs(np.array([True, True, False, False, True]))
    assert len(runs) == 1
    assert np.array_equal(runs[0], [4, 0, 1])


def test_envelope_csv_preserves_missing_corridor_evidence(tmp_path):
    envelope = _envelope(
        [6.0, np.nan], [5.0, np.nan], [7.0, np.nan], [3, 0])
    speed = _envelope(
        [20.0, np.nan], [19.0, np.nan], [21.0, np.nan], [3, 0])
    reference, eligible, excess, touches = MODULE._corridor_diagnostics(
        _track(20.0), envelope, np.array([True, False]))
    path = tmp_path / "envelope.csv"

    MODULE._write_envelope_csv(
        path, envelope, speed, reference, eligible, excess, touches)

    with path.open(newline="", encoding="utf-8") as stream:
        rows = list(csv.DictReader(stream))
    assert rows[0]["corridor_evidence_complete"] == "1"
    assert rows[0]["p10_p90_touches_outside_model_corridor"] == "1"
    assert rows[1]["corridor_evidence_complete"] == "0"
    assert rows[1]["median_model_corridor_excess_m"] == ""
    assert rows[1]["p10_p90_touches_outside_model_corridor"] == ""
