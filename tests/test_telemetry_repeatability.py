import numpy as np
import pytest

from motorcycle_lap_sim.telemetry import (
    chainage_progress_diagnostics,
    cross_lap_envelope,
    unwrap_closed_chainage,
)


def test_unwrap_closed_chainage_removes_only_start_finish_seam():
    raw = np.array([95.0, 98.0, 1.0, 4.0, 3.5, 7.0])
    unwrapped, wraps = unwrap_closed_chainage(raw, 100.0)

    assert wraps == 1
    assert np.allclose(unwrapped, [95.0, 98.0, 101.0, 104.0, 103.5, 107.0])


def test_chainage_progress_reports_local_backtracking():
    raw = np.array([95.0, 98.0, 1.0, 4.0, 3.5, 7.0])
    diagnostics = chainage_progress_diagnostics(raw, 100.0, backward_tolerance_m=0.25)

    assert diagnostics.wrap_count == 1
    assert diagnostics.backward_step_count == 1
    assert diagnostics.total_backward_m == pytest.approx(0.5)
    assert diagnostics.largest_backward_step_m == pytest.approx(0.5)
    assert diagnostics.net_progress_m == pytest.approx(12.0)


def test_cross_lap_envelope_weights_laps_equally_within_bins():
    lap_chainage = [
        np.array([1.0, 2.0, 3.0, 12.0, 13.0]),
        np.array([1.5, 11.0]),
    ]
    lap_values = [
        np.array([0.0, 2.0, 100.0, 10.0, 14.0]),
        np.array([20.0, 30.0]),
    ]

    envelope = cross_lap_envelope(lap_chainage, lap_values, 20.0, bin_width_m=10.0)

    # Lap 1 contributes median 2.0 to the first bin, not three separately
    # weighted samples; Lap 2 contributes 20.0. The cross-lap median is 11.0.
    assert np.allclose(envelope.chainage_m, [5.0, 15.0])
    assert np.allclose(envelope.median, [11.0, 21.0])
    assert np.array_equal(envelope.lap_count, [2, 2])
    assert np.allclose(envelope.per_lap_values, [[2.0, 12.0], [20.0, 30.0]])
