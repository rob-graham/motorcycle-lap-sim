import numpy as np
import pytest

from motorcycle_lap_sim.telemetry import (
    GPSQualitySeries,
    gps_quality_mask,
    load_aim_gps_quality,
    require_time_alignment,
)


def test_aim_gps_quality_import_converts_units(tmp_path):
    openpyxl = pytest.importorskip("openpyxl")
    workbook = openpyxl.Workbook()
    sheet = workbook.active
    sheet.title = "R6MallalaP4"
    sheet.append(["Time", "GPS Nsat", "GPS PosAccuracy", "GPS SpdAccuracy"])
    sheet.append(["s", "count", "mm", "km/h"])
    sheet.append([10.0, 12, 850.0, 1.8])
    sheet.append([10.05, 9, 1400.0, 3.6])
    path = tmp_path / "quality.xlsx"
    workbook.save(path)

    quality = load_aim_gps_quality(path)

    assert np.array_equal(quality.time_s, [10.0, 10.05])
    assert np.array_equal(quality.satellites, [12.0, 9.0])
    assert np.allclose(quality.position_accuracy_m, [0.85, 1.4])
    assert np.allclose(quality.speed_accuracy_mps, [0.5, 1.0])


def test_gps_quality_mask_uses_only_explicit_thresholds():
    quality = GPSQualitySeries(
        time_s=np.array([0.0, 0.05, 0.10]),
        satellites=np.array([12.0, 8.0, 11.0]),
        position_accuracy_m=np.array([0.8, 0.9, 2.5]),
        speed_accuracy_mps=np.array([0.2, 0.3, 0.4]),
        source_sheet="raw",
    )

    assert np.array_equal(gps_quality_mask(quality), [True, True, True])
    assert np.array_equal(
        gps_quality_mask(quality, min_satellites=10, max_position_accuracy_m=2.0),
        [True, False, False],
    )


def test_require_time_alignment_rejects_shifted_quality_series():
    quality = GPSQualitySeries(
        time_s=np.array([0.01, 0.06]),
        satellites=np.array([10.0, 10.0]),
        position_accuracy_m=np.array([1.0, 1.0]),
        speed_accuracy_mps=np.array([0.2, 0.2]),
        source_sheet="raw",
    )

    with pytest.raises(ValueError, match="sample-aligned"):
        require_time_alignment(np.array([0.0, 0.05]), quality)
