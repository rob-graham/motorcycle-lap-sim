import math

import numpy as np
import pytest

from motorcycle_lap_sim.telemetry import lap_slices, load_aim_workbook
from motorcycle_lap_sim.telemetry.map_match import Rigid2DTransform, map_match_nearest
from motorcycle_lap_sim.track import Pose, Straight, Track, sample_track


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
