import importlib.util
from pathlib import Path

import numpy as np


def _load_script():
    path = Path("scripts/r6_phase10_trajectory_export.py").resolve()
    spec = importlib.util.spec_from_file_location("r6_phase10_trajectory_export_test", path)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_trajectory_export_columns_align_and_write(tmp_path):
    export = _load_script()
    track, _, _, evaluations = export.phase9.evaluate_baseline(speed_backend="python")
    evaluation = evaluations[0]
    assert evaluation.smooth_line is not None
    assert evaluation.speed_profile is not None
    bike = export.load_motorcycle_config(export.phase9.DEFAULT_MOTORCYCLE)

    columns = export.trajectory_columns(
        track, evaluation.smooth_line, evaluation.speed_profile, bike)

    count = len(evaluation.speed_profile.speed_mps)
    assert tuple(columns) == export.CSV_FIELDS
    assert all(len(np.asarray(columns[field])) == count for field in export.CSV_FIELDS)
    assert np.all(columns["left_boundary_clearance_m"] >= -1e-8)
    assert np.all(columns["right_boundary_clearance_m"] >= -1e-8)
    assert np.allclose(
        columns["roll_angle_rad"],
        np.arctan(
            evaluation.speed_profile.speed_mps ** 2
            * evaluation.smooth_line.sampled_path.curvature_1pm
            / bike.environment.gravity_mps2
        ),
        rtol=0.0,
        atol=1e-15,
    )
    assert set(np.unique(columns["roll_rate_limited"])) <= {0, 1}
    assert set(np.unique(columns["wheelie_limited"])) <= {0, 1}
    assert set(np.unique(columns["stoppie_limited"])) <= {0, 1}
    assert set(np.unique(columns["traction_limited"])) <= {0, 1}

    output = tmp_path / "trajectory.csv"
    export.write_trajectory_csv(output, columns)
    lines = output.read_text(encoding="utf-8").splitlines()
    assert lines[0].split(",") == list(export.CSV_FIELDS)
    assert len(lines) == count + 1
