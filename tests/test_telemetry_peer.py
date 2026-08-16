import numpy as np

from motorcycle_lap_sim.telemetry.peer import peer_trajectory_deviation


def test_peer_trajectory_deviation_flags_one_displaced_lap_region():
    x = np.linspace(0.0, 100.0, 51)
    lap1_y = np.zeros_like(x)
    lap2_y = np.full_like(x, 0.2)
    lap3_y = np.zeros_like(x)
    lap3_y[(x >= 40.0) & (x <= 60.0)] = 5.0

    result = peer_trajectory_deviation(
        [x, x, x],
        [lap1_y, lap2_y, lap3_y],
    )

    deviation3 = result.median_nearest_distance_m[2]
    outside = (x < 40.0) | (x > 60.0)
    inside = ~outside
    assert np.max(deviation3[outside]) < 0.25
    assert np.min(deviation3[inside]) > 2.0


def test_peer_trajectory_deviation_is_independent_of_sample_counts():
    lap1_x = np.linspace(0.0, 20.0, 41)
    lap2_x = np.linspace(0.0, 20.0, 11)
    lap1_y = np.zeros_like(lap1_x)
    lap2_y = np.full_like(lap2_x, 1.0)

    result = peer_trajectory_deviation(
        [lap1_x, lap2_x],
        [lap1_y, lap2_y],
    )

    assert np.all(result.median_nearest_distance_m[0] >= 1.0)
    assert np.all(result.median_nearest_distance_m[1] >= 1.0)
