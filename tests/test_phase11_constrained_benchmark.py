import csv
import importlib.util
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest


SCRIPT = Path(__file__).resolve().parents[1] / "scripts" / "r6_phase11_constrained_benchmark.py"
spec = importlib.util.spec_from_file_location("phase11_constrained_benchmark", SCRIPT)
phase11 = importlib.util.module_from_spec(spec)
assert spec.loader is not None
spec.loader.exec_module(phase11)


def test_parser_defaults_match_nonlinear_constrained_benchmark():
    args = phase11.build_parser().parse_args([
        "reviewed.csv",
        "start_dir",
        "output_dir",
    ])

    assert args.margins_m == phase11.DEFAULT_MARGINS_M
    assert args.max_roll_rate_radps == 0.8
    assert args.max_evaluations == 4000
    assert args.max_iterations == 2000
    assert args.initial_tr_radius_m == 0.05
    assert args.final_tr_radius_m == 0.002
    assert args.feasibility_tol == 1e-10
    assert args.speed_backend == "numba"
    assert args.common_spacing_m == 0.125
    assert args.boundary_check_spacing_m == 0.125
    assert args.plot_dpi == 400
    assert args.show_control_points is True


def test_parser_can_hide_development_control_points():
    args = phase11.build_parser().parse_args([
        "reviewed.csv",
        "start_dir",
        "output_dir",
        "--no-show-control-points",
    ])

    assert args.show_control_points is False


@pytest.mark.parametrize(
    ("values", "expected"),
    [
        ([0.0, 0.0, 1.0], True),
        ([-1e-10, 0.0, 1.0], True),
        ([-1.01e-10, 0.0, 1.0], False),
        ([0.0, 0.0, 0.0], False),
        ([0.0, 0.0, float("nan")], False),
    ],
)
def test_production_feasible_constraint_values_match_fail_closed_contract(values, expected):
    assert phase11.production_feasible_constraint_values(values) is expected


def test_racing_line_artifact_paths_use_stable_margin_filename(tmp_path):
    csv_path, png_path = phase11.racing_line_artifact_paths(tmp_path, 0.25)

    assert csv_path == tmp_path / "margin_0.250m_racing_line.csv"
    assert png_path == tmp_path / "margin_0.250m_racing_line.png"


def test_write_racing_line_csv_includes_path_track_and_margin_coordinates(tmp_path, monkeypatch):
    track = SimpleNamespace(total_length_m=4.0)
    sampled_track = SimpleNamespace(
        x_m=np.array([0.0, 1.0, 2.0, 3.0]),
        y_m=np.zeros(4),
        normal_x=np.zeros(4),
        normal_y=np.ones(4),
        width_left_m=np.full(4, 4.0),
        width_right_m=np.full(4, 4.0),
    )
    monkeypatch.setattr(phase11, "sample_track_stations", lambda _track, _s: sampled_track)

    sampled_path = SimpleNamespace(
        q_m=np.array([0.0, 1.0, 2.0, 3.0]),
        x_m=np.array([0.0, 1.0, 2.0, 3.0]),
        y_m=np.full(4, 1.5),
        curvature_1pm=np.array([0.1, 0.2, 0.3, 0.4]),
    )
    evaluation = SimpleNamespace(
        smooth_line=SimpleNamespace(sampled_path=sampled_path),
        speed_profile=SimpleNamespace(speed_mps=np.array([10.0, 11.0, 12.0, 13.0])),
    )
    path = tmp_path / "line.csv"

    phase11.write_racing_line_csv(path, track, evaluation, 0.5)

    with path.open(newline="", encoding="utf-8") as stream:
        rows = list(csv.DictReader(stream))
    assert len(rows) == 4
    assert float(rows[0]["projected_offset_m"]) == pytest.approx(1.5)
    assert float(rows[0]["left_edge_y_m"]) == pytest.approx(4.0)
    assert float(rows[0]["right_edge_y_m"]) == pytest.approx(-4.0)
    assert float(rows[0]["margin_left_y_m"]) == pytest.approx(3.5)
    assert float(rows[0]["margin_right_y_m"]) == pytest.approx(-3.5)
    assert float(rows[3]["speed_mps"]) == pytest.approx(13.0)


def test_scipy_is_loaded_only_when_requested(monkeypatch):
    import builtins

    original_import = builtins.__import__

    def blocked_import(name, *args, **kwargs):
        if name == "scipy.optimize" or name.startswith("scipy"):
            raise ModuleNotFoundError("blocked for test")
        return original_import(name, *args, **kwargs)

    monkeypatch.setattr(builtins, "__import__", blocked_import)
    with pytest.raises(RuntimeError, match="SciPy is required only for this benchmark"):
        phase11._load_scipy_tools()
