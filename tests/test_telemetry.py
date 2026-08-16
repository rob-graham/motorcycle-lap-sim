import math

import numpy as np
import pytest

from motorcycle_lap_sim.telemetry import fit_rigid_registration, lap_slices, load_aim_workbook
from motorcycle_lap_sim.telemetry.map_match import Rigid2DTransform, map_match_nearest
from motorcycle_lap_sim.track import Pose, Straight, Track, sample_track
from motorcycle_lap_sim.track.sampling import SampledTrack


HEADERS = [
    "Time", "Distance on GPS Speed", "east", "north", "GPS Speed",
    "GPS LatAcc", "GPS LonAcc", "GPS Slope", "GPS Heading", "GPS Gyro",
    "GPS Latitude", "GPS Longitude", "RollRate", "PitchRate", "YawRate",
    "ECU RPM", "ECU GEAR", "ECU THROTTLE", "ECU TPS HAND", "Dist from Start",
    None, None,
]
UNITS = ["s", "m", "m", "m", "km/h", "g", "g", "deg", "deg", "deg/s",
         "deg", "deg", "deg/s", "deg/s", "deg/s", "rpm", "gear", "deg", "%", "m",
         None, None]


def test_aim_workbook_import_converts_units_and_lap_ids(tmp_path):
    openpyxl = pytest.importorskip("openpyxl")
    workbook = openpyxl.Workbook()
    sheet = workbook.active
    sheet.title = "Updated"
    sheet.append(HEADERS)
    sheet.append(UNITS)
    sheet.append([10.0, 100.0, 1.0, 2.0, 72.0, 1.0, -0.5, 10.0, 90.0, 180.0,
                  -34.4, 138.5, 90.0, 0.0, -45.0, 12000.0, 3, 30.0, 50.0, 2.2,
                  "start", 5])
    # AiM can interpolate the gear channel across a shift. Preserve that raw
    # numeric value at import; integer classification is a later cleaning step.
    sheet.append([10.05, 101.0, 1.5, 2.0, 36.0, 0.0, 0.0, 0.0, 91.0, 0.0,
                  -34.4, 138.5, 0.0, 0.0, 0.0, 11000.0, 3.42, 20.0, 25.0, 2.5,
                  None, 5])
    path = tmp_path / "telemetry.xlsx"
    workbook.save(path)

    session = load_aim_workbook(path)

    assert np.array_equal(session.speed_mps, [20.0, 10.0])
    assert session.lateral_acceleration_mps2[0] == pytest.approx(9.80665)
    assert session.longitudinal_acceleration_mps2[0] == pytest.approx(-0.5 * 9.80665)
    assert session.heading_rad[0] == pytest.approx(math.pi / 2)
    assert session.roll_rate_radps[0] == pytest.approx(math.pi / 2)
    assert session.hand_throttle_fraction[0] == pytest.approx(0.5)
    assert np.array_equal(session.gear_number, [3.0, 3.42])
    assert session.marker == ("start", None)
    laps = lap_slices(session)
    assert len(laps) == 1 and laps[0].lap_id == 5
    assert laps[0].start_index == 0 and laps[0].stop_index == 2
    assert laps[0].duration_s == pytest.approx(0.05)


def test_world_to_local_uses_explicit_bearing():
    transform = Rigid2DTransform(100.0, 200.0, math.pi / 2)
    x, y = transform.world_to_local([101.0, 100.0], [200.0, 201.0])
    assert np.allclose(x, [1.0, 0.0], atol=1e-12)
    assert np.allclose(y, [0.0, 1.0], atol=1e-12)


def test_map_match_reports_signed_offset_for_straight():
    track = Track((Straight(20.0),), Pose(0.0, 0.0, 0.0), 5.0, 5.0, False)
    sampled = sample_track(track, 1.0)

    match = map_match_nearest([5.0, 10.0], [2.0, -3.0], sampled)

    assert np.allclose(match.chainage_m, [5.0, 10.0])
    assert np.allclose(match.lateral_offset_m, [2.0, -3.0])
    assert np.allclose(match.reference_distance_m, [2.0, 3.0])


def _sampled_reference_points(x, y):
    x = np.asarray(x, dtype=float)
    y = np.asarray(y, dtype=float)
    count = len(x)
    s = np.arange(count, dtype=float) * 10.0
    zeros = np.zeros(count)
    ones = np.ones(count)
    return SampledTrack(
        s_m=s,
        x_m=x,
        y_m=y,
        heading_rad=zeros,
        tangent_x=ones,
        tangent_y=zeros,
        normal_x=zeros,
        normal_y=ones,
        curvature_1pm=zeros,
        width_left_m=np.full(count, 5.0),
        width_right_m=np.full(count, 5.0),
        total_length_m=float(max(10.0, s[-1] + 10.0)),
        closed=False,
    )


def _registration_fixture():
    target_x = np.array([0.0, 12.0, 25.0, 31.0, 24.0, 10.0, -2.0, -7.0])
    target_y = np.array([0.0, 2.0, 8.0, 20.0, 31.0, 34.0, 25.0, 11.0])
    sampled = _sampled_reference_points(target_x, target_y)

    true_transform = Rigid2DTransform(270780.0, 6188972.0, math.radians(-91.0))
    theta = true_transform.local_x_bearing_rad - math.pi / 2.0
    c, s = math.cos(theta), math.sin(theta)
    rotation = np.array([[c, -s], [s, c]])
    local = np.column_stack((target_x, target_y))
    world = np.array([true_transform.origin_east_m, true_transform.origin_north_m]) + local @ rotation
    return sampled, true_transform, world[:, 0].copy(), world[:, 1].copy()


def test_rigid_registration_recovers_transform_and_trims_position_outlier():
    sampled, true_transform, east, north = _registration_fixture()

    # Simulate a transient GPS position error at one sample.  The registration
    # should not move the track transform to explain it.
    east[-1] += 40.0
    north[-1] -= 25.0

    initial = Rigid2DTransform(270781.0, 6188971.0, math.radians(-90.5))
    result = fit_rigid_registration(
        east, north, sampled, initial, trim_fraction=0.875, max_iterations=20)

    assert result.transform.origin_east_m == pytest.approx(true_transform.origin_east_m, abs=1e-6)
    assert result.transform.origin_north_m == pytest.approx(true_transform.origin_north_m, abs=1e-6)
    assert result.transform.local_x_bearing_rad == pytest.approx(
        true_transform.local_x_bearing_rad, abs=1e-8)
    assert np.count_nonzero(result.inlier_mask) == 7
    assert not result.inlier_mask[-1]
    assert result.converged
    assert result.final_translation_delta_m <= 1e-4
    assert result.final_bearing_delta_rad <= 1e-7
    assert result.rms_residual_m < 1e-6


def test_rigid_registration_reports_iteration_limit_without_claiming_convergence():
    sampled, _, east, north = _registration_fixture()
    initial = Rigid2DTransform(270781.0, 6188971.0, math.radians(-90.5))

    result = fit_rigid_registration(
        east, north, sampled, initial, trim_fraction=1.0, max_iterations=1)

    assert result.iterations == 1
    assert not result.converged
    assert (result.final_translation_delta_m > 1e-4
            or result.final_bearing_delta_rad > 1e-7)
