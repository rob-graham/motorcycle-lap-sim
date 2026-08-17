import importlib.util
import math
from pathlib import Path
from types import SimpleNamespace

import numpy as np


SCRIPT = Path(__file__).resolve().parents[1] / "scripts" / "r6_phase11_resolution_margin_sensitivity.py"
spec = importlib.util.spec_from_file_location("phase11_resolution_margin", SCRIPT)
phase11 = importlib.util.module_from_spec(spec)
assert spec.loader is not None
spec.loader.exec_module(phase11)


def test_build_case_matrix_separates_three_sensitivity_dimensions():
    args = SimpleNamespace(
        output_spacings_m=[0.5, 0.25, 0.5],
        boundary_check_spacings_m=[0.25, 0.125],
        boundary_margins_m=[0.25, 0.0, 0.25],
        common_output_spacing_m=0.25,
        common_boundary_check_spacing_m=0.125,
    )

    rows = phase11.build_case_matrix(args)

    assert [row["study"] for row in rows] == [
        "fixed_spline_output_resolution",
        "fixed_spline_output_resolution",
        "corridor_check_resolution",
        "corridor_check_resolution",
        "boundary_margin",
        "boundary_margin",
    ]
    assert [row["sample_spacing_m"] for row in rows[:2]] == [0.25, 0.5]
    assert all(
        row["boundary_margin_m"] == phase11.phase9.BOUNDARY_MARGIN_M
        for row in rows[:4]
    )
    assert [row["boundary_margin_m"] for row in rows[-2:]] == [0.0, 0.25]
    assert all(row["sample_spacing_m"] == 0.25 for row in rows[-2:])
    assert all(row["boundary_check_spacing_m"] == 0.125 for row in rows[-2:])


def test_evaluation_row_reports_physical_edge_clearance_without_moving_path():
    case = {
        "study": "boundary_margin",
        "sample_spacing_m": 0.25,
        "boundary_check_spacing_m": 0.125,
        "boundary_margin_m": 0.4,
    }
    path = SimpleNamespace(
        total_length_m=123.0,
        q_m=np.array([0.0, 1.0, 2.0]),
        curvature_1pm=np.array([-0.1, 0.02, 0.05]),
    )
    smooth = SimpleNamespace(
        sampled_path=path,
        minimum_boundary_clearance_m=0.15,
        minimum_forward_progress=0.9,
    )
    evaluation = SimpleNamespace(
        feasible=True,
        lap_time_s=12.5,
        smooth_line=smooth,
        speed_profile=object(),
        failure_reason=None,
    )

    row = phase11.evaluation_row(case, evaluation)

    assert row["feasible"] is True
    assert row["lap_time_s"] == 12.5
    assert row["path_length_m"] == 123.0
    assert row["sample_count"] == 3
    assert row["minimum_boundary_clearance_m"] == 0.15
    assert math.isclose(row["minimum_edge_clearance_m"], 0.55)
    assert row["curvature_min_1pm"] == -0.1
    assert row["curvature_max_1pm"] == 0.05
    assert row["failure_reason"] == ""


def test_evaluation_row_preserves_infeasibility_instead_of_fabricating_metrics():
    case = {
        "study": "boundary_margin",
        "sample_spacing_m": 0.25,
        "boundary_check_spacing_m": 0.125,
        "boundary_margin_m": 0.5,
    }
    evaluation = SimpleNamespace(
        feasible=False,
        lap_time_s=math.inf,
        smooth_line=None,
        speed_profile=None,
        failure_reason="ValueError: guide offset violates track boundary margin",
    )

    row = phase11.evaluation_row(case, evaluation)

    assert row["feasible"] is False
    assert math.isnan(row["lap_time_s"])
    assert math.isnan(row["minimum_boundary_clearance_m"])
    assert row["sample_count"] == 0
    assert row["failure_reason"].startswith("ValueError:")
