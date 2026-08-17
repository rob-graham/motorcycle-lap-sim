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


def test_parser_defaults_match_scalar_constrained_benchmark():
    args = phase11.build_parser().parse_args([
        "reviewed.csv",
        "start_dir",
        "output_dir",
    ])

    assert args.margins_m == phase11.DEFAULT_MARGINS_M
    assert args.max_roll_rate_radps == 0.8
    assert args.max_evaluations == 4000
    assert args.max_iterations == 2000
    assert args.initial_trust_region_radius_m == 0.05
    assert args.final_trust_region_radius_m == 0.002
    assert args.feasibility_tol == 1e-10
    assert args.speed_backend == "numba"
    assert args.common_spacing_m == 0.125
    assert args.boundary_check_spacing_m == 0.125
    assert args.plot_dpi == 400
    assert args.show_control_points is True


def test_parser_accepts_legacy_trust_region_flag_aliases():
    args = phase11.build_parser().parse_args([
        "reviewed.csv",
        "start_dir",
        "output_dir",
        "--initial-tr-radius-m", "0.1",
        "--final-tr-radius-m", "0.005",
    ])

    assert args.initial_trust_region_radius_m == 0.1
    assert args.final_trust_region_radius_m == 0.005


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


def test_production_constraint_violation_matches_contract():
    assert phase11.production_constraint_violation([0.0, 0.0, 1.0]) == 0.0
    assert phase11.production_constraint_violation([-2e-10, 0.0, 1.0]) == pytest.approx(1e-10)
    assert phase11.production_constraint_violation([0.0, 0.0, -0.25]) == pytest.approx(0.25)
    assert np.isinf(phase11.production_constraint_violation([0.0, np.nan, 1.0]))


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


def test_fixed_station_affine_geometry_matches_direct_spline_on_canonical_track():
    track = phase11.Track.from_yaml(phase11.phase9.DEFAULT_TRACK)
    stations = phase11.generate_planar_control_stations(
        track,
        phase11.REFERENCE_PLANAR_CONTROL_POLICY,
    )
    geometry = phase11.FixedStationAffineGeometry(track, stations, 5.0)

    controls = 0.35 * np.sin(np.arange(len(stations), dtype=float) * 0.71)
    direct_projection, direct_forward = geometry.direct_projection_forward(controls)
    affine_projection, affine_forward = geometry.projection_forward(controls)

    assert np.allclose(affine_projection, direct_projection, rtol=0.0, atol=2e-12)
    assert np.allclose(affine_forward, direct_forward, rtol=0.0, atol=2e-12)


def test_scalar_constraints_reduce_affine_geometry_without_scipy():
    track = phase11.Track.from_yaml(phase11.phase9.DEFAULT_TRACK)
    stations = phase11.generate_planar_control_stations(
        track,
        phase11.REFERENCE_PLANAR_CONTROL_POLICY,
    )
    geometry = phase11.FixedStationAffineGeometry(track, stations, 10.0)
    controls = 0.2 * np.cos(np.arange(len(stations), dtype=float) * 0.43)
    margin = 0.25
    constraint = phase11.ScalarGeometryConstraints(geometry, margin)

    projection, forward = geometry.projection_forward(controls)
    values = constraint(controls)

    assert values[0] == pytest.approx(np.min(geometry.checked_track.width_left_m - margin - projection))
    assert values[1] == pytest.approx(np.min(geometry.checked_track.width_right_m - margin + projection))
    assert values[2] == pytest.approx(np.min(forward))
    assert values.shape == (3,)


def test_scalar_constraints_cache_identical_control_vector():
    class Geometry:
        def __init__(self):
            self.calls = 0

        def constraint_values(self, controls, margin):
            self.calls += 1
            value = float(np.sum(controls))
            return np.array([margin + value, margin - value, 1.0])

    geometry = Geometry()
    constraint = phase11.ScalarGeometryConstraints(geometry, 0.25)
    controls = np.array([0.1, -0.1])

    first = constraint(controls)
    second = constraint(controls.copy())

    assert np.array_equal(first, second)
    assert geometry.calls == 1
    assert constraint.calls == 2
    assert constraint.unique_evaluations == 1
    assert constraint.cache_hits == 1


def test_production_density_constraint_interface_stays_three_scalar_values():
    track = phase11.Track.from_yaml(phase11.phase9.DEFAULT_TRACK)
    stations = phase11.generate_planar_control_stations(
        track,
        phase11.REFERENCE_PLANAR_CONTROL_POLICY,
    )
    geometry = phase11.FixedStationAffineGeometry(track, stations, 0.125)
    margin = 0.25
    lower, upper = phase11.planar_control_bounds(track, stations, margin)
    controls = np.clip(np.zeros(len(stations)), lower, upper)
    constraint = phase11.ScalarGeometryConstraints(geometry, margin)

    values = constraint(controls)

    assert geometry.check_count > 20_000
    assert values.shape == (3,)
    assert constraint.unique_evaluations == 1
    assert geometry.response_matrix_bytes < 20 * 1024 * 1024


def test_bounded_solver_smoke_uses_three_scalar_constraints_and_revalidates(monkeypatch, tmp_path):
    class FakeConstraint:
        def __init__(self, fun, lb, ub):
            self.fun = fun
            self.lb = np.asarray(lb, dtype=float)
            self.ub = np.asarray(ub, dtype=float)

    observed = {}

    def fake_minimize(fun, x0, method, bounds, constraints, options):
        assert method == "COBYQA"
        assert len(constraints) == 1
        values = constraints[0].fun(np.asarray(x0, dtype=float))
        observed["constraint_shape"] = values.shape
        observed["options"] = dict(options)
        fun(np.asarray(x0, dtype=float))
        return SimpleNamespace(
            x=np.asarray(x0, dtype=float),
            nit=1,
            success=True,
            status=0,
            message="bounded smoke",
        )

    monkeypatch.setattr(phase11, "_load_scipy_tools", lambda: (fake_minimize, FakeConstraint))
    monkeypatch.setattr(phase11.phase9f, "_require_canonical_inputs", lambda: None)
    monkeypatch.setattr(phase11.phase9, "sha256_file", lambda _path: "dummy")

    track = phase11.Track.from_yaml(phase11.phase9.DEFAULT_TRACK)
    stations = phase11.generate_planar_control_stations(track, phase11.REFERENCE_PLANAR_CONTROL_POLICY)
    reviewed = np.zeros(len(stations))
    start_dir = tmp_path / "start"
    output_dir = tmp_path / "out"
    start_dir.mkdir()
    reviewed_csv = tmp_path / "reviewed.csv"
    reviewed_csv.write_text("dummy", encoding="utf-8")

    monkeypatch.setattr(
        phase11.phase8,
        "load_initial_controls_csv",
        lambda _path, _stations, _lower, _upper: reviewed.copy(),
    )
    monkeypatch.setattr(
        phase11.phase8,
        "atomic_write_controls_csv",
        lambda *args, **kwargs: None,
    )
    monkeypatch.setattr(phase11, "write_racing_line_csv", lambda *args, **kwargs: None)
    monkeypatch.setattr(phase11, "write_racing_line_png", lambda *args, **kwargs: None)

    sampled_path = SimpleNamespace(
        q_m=np.array([0.0, 1.0, 2.0]),
        x_m=np.array([0.0, 1.0, 2.0]),
        y_m=np.zeros(3),
        curvature_1pm=np.zeros(3),
    )
    smooth_line = SimpleNamespace(
        sampled_path=sampled_path,
        minimum_boundary_clearance_m=1.0,
        evaluated_track_s_m=np.array([0.0, 1.0, 2.0]),
        spline=SimpleNamespace(evaluate=lambda s: (np.asarray(s), np.zeros_like(s), np.ones_like(s), np.zeros_like(s), np.zeros_like(s), np.zeros_like(s))),
        guide_x_m=np.zeros(len(stations)),
        guide_y_m=np.zeros(len(stations)),
    )
    feasible_evaluation = SimpleNamespace(
        feasible=True,
        failure_reason=None,
        smooth_line=smooth_line,
        speed_profile=SimpleNamespace(lap_time_s=72.0, speed_mps=np.ones(3)),
        lap_time_s=72.0,
    )
    monkeypatch.setattr(phase11, "_evaluate", lambda *args, **kwargs: feasible_evaluation)

    class FakeObjective(phase11.ConstrainedObjective):
        def __call__(self, controls):
            self.evaluations += 1
            values = self.scalar_constraints(controls)
            if self.best_controls is None and phase11.production_feasible_constraint_values(values):
                self.best_controls = np.asarray(controls, dtype=float).copy()
                self.best_lap_time_s = 72.0
                self.best_constraint_values = values.copy()
            return 72.0

    monkeypatch.setattr(phase11, "ConstrainedObjective", FakeObjective)
    monkeypatch.setattr(
        phase11.FixedStationAffineGeometry,
        "constraint_values",
        lambda self, controls, margin: np.array([1.0, 1.0, 1.0]),
    )

    rows = phase11.main([
        str(reviewed_csv),
        str(start_dir),
        str(output_dir),
        "--margins-m", "0.25",
        "--max-evaluations", "4",
        "--max-iterations", "2",
        "--boundary-check-spacing-m", "0.125",
        "--common-spacing-m", "1.0",
        "--speed-backend", "python",
        "--no-show-control-points",
    ])

    assert observed["constraint_shape"] == (3,)
    assert observed["options"]["maxfev"] == 4
    assert rows[0]["success"] is True
    assert rows[0]["max_constraint_violation"] == 0.0


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
