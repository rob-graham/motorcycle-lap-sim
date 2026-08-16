import numpy as np
import pytest

from motorcycle_lap_sim.telemetry.speed_comparison import (
    compare_speed_envelope,
    periodic_interpolate,
    summarize_speed_comparison,
    uniform_closed_parameter_grid,
)


def test_uniform_closed_parameter_grid_matches_periodic_sampling():
    stations = uniform_closed_parameter_grid(100.0, 4)
    assert np.array_equal(stations, [0.0, 25.0, 50.0, 75.0])


def test_periodic_interpolation_wraps_across_start_finish():
    source_s = np.array([0.0, 25.0, 50.0, 75.0])
    source_v = np.array([10.0, 20.0, 30.0, 40.0])
    values = periodic_interpolate(source_s, source_v, [-5.0, 5.0, 95.0, 105.0], 100.0)
    assert np.allclose(values, [16.0, 12.0, 16.0, 12.0])


def test_compare_speed_envelope_uses_only_complete_lap_bins():
    comparison = compare_speed_envelope(
        chainage_m=[5.0, 15.0, 25.0],
        measured_median_mps=[20.0, 30.0, 40.0],
        measured_p10_mps=[19.0, 29.0, 39.0],
        measured_p90_mps=[21.0, 31.0, 41.0],
        measured_lap_count=[5, 4, 5],
        simulated_chainage_m=[0.0, 10.0, 20.0, 30.0],
        simulated_speed_mps=[20.0, 22.0, 42.0, 40.0],
        total_length_m=40.0,
        required_lap_count=5,
    )
    assert np.array_equal(comparison.eligible_mask, [True, False, True])
    assert comparison.sim_minus_median_mps[0] == pytest.approx(1.0)
    assert np.isnan(comparison.sim_minus_median_mps[1])
    assert comparison.sim_minus_median_mps[2] == pytest.approx(1.0)

    summary = summarize_speed_comparison(comparison)
    assert summary.eligible_bins == 2
    assert summary.mean_bias_mps == pytest.approx(1.0)
    assert summary.mean_absolute_error_mps == pytest.approx(1.0)
    assert summary.within_p10_p90_bins == 2
    assert summary.above_p90_bins == 0
    assert summary.below_p10_bins == 0


def test_compare_speed_envelope_reports_above_and_below_band():
    comparison = compare_speed_envelope(
        chainage_m=[5.0, 15.0, 25.0],
        measured_median_mps=[20.0, 30.0, 40.0],
        measured_p10_mps=[19.0, 29.0, 39.0],
        measured_p90_mps=[21.0, 31.0, 41.0],
        measured_lap_count=[5, 5, 5],
        simulated_chainage_m=[5.0, 15.0, 25.0],
        simulated_speed_mps=[18.0, 30.0, 43.0],
        total_length_m=30.0,
        required_lap_count=5,
    )
    summary = summarize_speed_comparison(comparison)
    assert summary.within_p10_p90_bins == 1
    assert summary.above_p90_bins == 1
    assert summary.below_p10_bins == 1
    assert summary.maximum_absolute_error_mps == pytest.approx(3.0)
    assert summary.maximum_absolute_error_chainage_m == pytest.approx(25.0)


def test_compare_speed_envelope_rejects_unordered_percentiles():
    with pytest.raises(ValueError, match="p10 <= median <= p90"):
        compare_speed_envelope(
            chainage_m=[5.0, 15.0],
            measured_median_mps=[20.0, 30.0],
            measured_p10_mps=[21.0, 29.0],
            measured_p90_mps=[22.0, 31.0],
            measured_lap_count=[5, 5],
            simulated_chainage_m=[0.0, 10.0],
            simulated_speed_mps=[20.0, 30.0],
            total_length_m=20.0,
            required_lap_count=5,
        )
