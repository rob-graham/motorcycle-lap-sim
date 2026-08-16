import math
import builtins
from types import SimpleNamespace
import numpy as np
import pytest

from motorcycle_lap_sim.motorcycle.config import load_motorcycle_config
from motorcycle_lap_sim.optimisation import (COARSE_PLANAR_CONTROL_POLICY,
    FINE_PLANAR_CONTROL_POLICY, REFERENCE_PLANAR_CONTROL_POLICY,
    PlanarControlStationPolicy, PlanarOptimisationConfig, evaluate_planar_racing_line,
    generate_planar_control_stations, optimise_planar_racing_line, planar_control_bounds,
    resample_planar_result)
from motorcycle_lap_sim.track import CircularArc, Pose, Straight, Track, sample_track_stations
from motorcycle_lap_sim.optimisation.planar import (
    _BACKEND_LAP_TIME_ATOL_S, _BACKEND_SPEED_ATOL_MPS, _BACKEND_SPEED_RTOL,
    _best_improvement_pattern_search, _validate_speed_backend_equivalence)
from motorcycle_lap_sim.optimisation.planar import SpeedBackendUnavailableError
from motorcycle_lap_sim.speed_solver import solve_speed_profile


def bike(): return load_motorcycle_config("examples/motorcycles/test_motorcycle.yaml")


def backend_result(lap_time_s=60.0, speeds=(10.0, 40.0, 0.0)):
    return SimpleNamespace(lap_time_s=lap_time_s, speed_mps=np.asarray(speeds))


def test_backend_equivalence_accepts_identical_profiles():
    result = backend_result()
    _validate_speed_backend_equivalence(result, result)


def test_backend_equivalence_accepts_observed_harmless_speed_scale():
    python = backend_result(speeds=(10.0, 40.0, 60.0))
    numba = backend_result(
        lap_time_s=python.lap_time_s + 4.01215061174e-10,
        speeds=python.speed_mps + np.array((0.0, 1.95256664171e-8, 0.0)))
    _validate_speed_backend_equivalence(python, numba)


def test_backend_equivalence_tolerance_boundary():
    python = backend_result(speeds=(25.0,))
    allowed = _BACKEND_SPEED_ATOL_MPS + _BACKEND_SPEED_RTOL * 25.0
    _validate_speed_backend_equivalence(
        python, backend_result(speeds=(25.0 + 0.999 * allowed,)))
    with pytest.raises(RuntimeError, match="validation failed"):
        _validate_speed_backend_equivalence(
            python, backend_result(speeds=(25.0 + 1.001 * allowed,)))


def test_backend_equivalence_failure_reports_worst_speed_details():
    python = backend_result(speeds=(10.0, 20.0, 30.0))
    numba = backend_result(speeds=(10.0, 20.000001, 30.0000001))
    with pytest.raises(RuntimeError) as error:
        _validate_speed_backend_equivalence(python, numba)
    message = str(error.value)
    for expected in ("lap difference=", "allowed lap tolerance=",
                     "maximum speed absolute difference=", "worst speed index=1",
                     "Python speed=20", "Numba speed=20.000001",
                     "allowed speed tolerance=", "relative discrepancy="):
        assert expected in message


def test_backend_equivalence_rejects_material_lap_time_difference():
    with pytest.raises(RuntimeError, match="lap difference"):
        _validate_speed_backend_equivalence(
            backend_result(), backend_result(lap_time_s=60.0 + 10 * _BACKEND_LAP_TIME_ATOL_S))


def test_backend_equivalence_rejects_material_speed_difference():
    with pytest.raises(RuntimeError, match="maximum speed absolute difference"):
        _validate_speed_backend_equivalence(
            backend_result(), backend_result(speeds=(10.0, 40.01, 0.0)))


def test_geometry_aware_reference_station_counts_and_primitive_starts():
    oval=Track.from_yaml("examples/tracks/test_oval.yaml")
    mallala=Track.from_yaml("examples/tracks/mallala_reference.yaml")
    policies=(COARSE_PLANAR_CONTROL_POLICY,REFERENCE_PLANAR_CONTROL_POLICY,FINE_PLANAR_CONTROL_POLICY)
    assert [len(generate_planar_control_stations(oval,p)) for p in policies] == [8,10,16]
    assert [len(generate_planar_control_stations(mallala,p)) for p in policies] == [41,52,67]
    stations=generate_planar_control_stations(mallala,REFERENCE_PLANAR_CONTROL_POLICY)
    assert stations[0] == 0 and stations[-1] < mallala.total_length_m and np.all(np.diff(stations)>0)
    assert all(np.any(np.isclose(stations,start,atol=1e-10))
               for start in mallala.primitive_start_s_m[:-1])


def test_station_subdivisions_respect_spacing_and_arc_heading():
    track=Track((Straight(230),CircularArc(20,1.8)),closed=False)
    policy=PlanarControlStationPolicy(100,.5)
    stations=generate_planar_control_stations(track,policy)
    assert np.allclose(stations[:3],[0,230/3,460/3])
    arc=stations[3:]
    assert len(arc)==4 and 20*1.8/len(arc)<=100 and 1.8/len(arc)<=.5


def test_direct_controls_and_local_asymmetric_bounds():
    track=Track.from_yaml("examples/tracks/mallala_reference.yaml")
    stations=generate_planar_control_stations(track,REFERENCE_PLANAR_CONTROL_POLICY)
    lower,upper=planar_control_bounds(track,stations,.25)
    sampled=sample_track_stations(track,stations)
    assert np.allclose(lower,-(sampled.width_right_m-.25))
    assert np.allclose(upper,sampled.width_left_m-.25)
    zero=evaluate_planar_racing_line(np.zeros(len(stations)),track,bike(),stations)
    if zero.feasible:
        assert np.allclose(zero.smooth_line.guide_x_m,sampled.x_m)
        assert np.allclose(zero.smooth_line.guide_y_m,sampled.y_m)
    moved=np.zeros(len(stations)); moved[0]=1
    candidate=evaluate_planar_racing_line(moved,track,bike(),stations)
    if candidate.feasible:
        delta=np.array([candidate.smooth_line.guide_x_m[0]-sampled.x_m[0],
                        candidate.smooth_line.guide_y_m[0]-sampled.y_m[0]])
        assert np.allclose(delta,[sampled.normal_x[0],sampled.normal_y[0]])


def test_planar_pattern_search_is_deterministic_bounded_and_resampleable():
    track=Track((CircularArc(30,2*math.pi),),Pose(0,0,0),5,3,True)
    policy=PlanarControlStationPolicy(100,math.pi/2)
    config=PlanarOptimisationConfig(max_sweeps=3,max_evaluations=30,
                                    boundary_check_spacing_m=1,
                                    optimisation_sample_spacing_m=2)
    first=optimise_planar_racing_line(track,bike(),policy,config)
    second=optimise_planar_racing_line(track,bike(),policy,config)
    assert first.best_lap_time_s <= first.initial_lap_time_s
    assert first.best_lap_time_s == second.best_lap_time_s
    assert np.array_equal(first.best_controls_m,second.best_controls_m)
    assert np.all(first.best_controls_m>=first.lower_bounds_m)
    assert np.all(first.best_controls_m<=first.upper_bounds_m)
    path,_=resample_planar_result(first,bike(),.5)
    assert path.total_length_m == first.sampled_path.total_length_m
    with pytest.raises(ValueError): first.best_controls_m[0]=99


def test_invalid_planar_candidate_is_deterministic_infeasibility():
    track=Track((CircularArc(20,2*math.pi),),Pose(0,0,0),5,5,True)
    stations=generate_planar_control_stations(track,PlanarControlStationPolicy(100,math.pi/2))
    first=evaluate_planar_racing_line(np.full(len(stations),20.),track,bike(),stations)
    second=evaluate_planar_racing_line(np.full(len(stations),20.),track,bike(),stations)
    assert not first.feasible and first.lap_time_s == math.inf
    assert first.failure_reason == second.failure_reason


def test_planar_pattern_search_accepts_bounded_initial_controls():
    track=Track((CircularArc(30,2*math.pi),),Pose(0,0,0),5,3,True)
    policy=PlanarControlStationPolicy(100,math.pi/2)
    stations=generate_planar_control_stations(track,policy)
    initial=np.full(len(stations),.25)
    config=PlanarOptimisationConfig(max_sweeps=1,max_evaluations=10,
                                    boundary_check_spacing_m=1,
                                    optimisation_sample_spacing_m=2)
    result=optimise_planar_racing_line(track,bike(),policy,config,initial)
    assert np.array_equal(result.initial_controls_m,initial)
    with pytest.raises(ValueError,match="one value per"):
        optimise_planar_racing_line(track,bike(),policy,config,initial[:-1])
    with pytest.raises(ValueError,match="finite"):
        optimise_planar_racing_line(track,bike(),policy,config,np.full(len(stations),np.nan))
    with pytest.raises(ValueError,match="local bounds"):
        optimise_planar_racing_line(track,bike(),policy,config,np.full(len(stations),99.))


def test_coupled_bump_escapes_coordinate_stationary_objective():
    """Every unit coordinate move is uphill, while a smooth bump is downhill."""
    def evaluate(controls):
        value = np.sum(controls ** 2) - 1.3 * np.sum(controls * np.roll(controls, 1))
        return SimpleNamespace(feasible=True, lap_time_s=float(value))

    controls = np.zeros(5)
    initial = evaluate(controls)
    config = PlanarOptimisationConfig(initial_step_m=1, minimum_step_m=.75,
                                      max_sweeps=1, max_evaluations=40)
    result, best, evaluations, polls, _, _ = _best_improvement_pattern_search(
        controls, np.full(5, -2.), np.full(5, 2.), initial, evaluate, config)

    assert all(evaluate(np.eye(5)[index]).lap_time_s > initial.lap_time_s
               for index in range(5))
    assert best.lap_time_s < initial.lap_time_s
    assert np.count_nonzero(result) > 1
    assert evaluations > 1 and polls == 1


@pytest.mark.parametrize("workers", [0, -1, 1.5, True])
def test_planar_parallel_worker_count_must_be_a_positive_integer(workers):
    with pytest.raises(ValueError, match="worker"):
        PlanarOptimisationConfig(parallel_workers=workers)


def test_invalid_speed_backend_is_rejected():
    with pytest.raises(ValueError, match="speed_backend"):
        PlanarOptimisationConfig(speed_backend="gpu")
    track = Track((CircularArc(30, 2 * math.pi),), Pose(0, 0, 0), 5, 3, True)
    stations = generate_planar_control_stations(
        track, PlanarControlStationPolicy(100, math.pi / 2))
    with pytest.raises(ValueError, match="speed_backend"):
        evaluate_planar_racing_line(
            np.zeros(len(stations)), track, bike(), stations, speed_backend="gpu")


def test_missing_numba_is_a_backend_error_not_candidate_infeasibility(monkeypatch):
    original_import = builtins.__import__

    def unavailable(name, *args, **kwargs):
        if name == "motorcycle_lap_sim.speed_solver.numba_backend":
            raise ModuleNotFoundError("simulated missing numba")
        return original_import(name, *args, **kwargs)

    monkeypatch.setattr(builtins, "__import__", unavailable)
    track = Track((CircularArc(30, 2 * math.pi),), Pose(0, 0, 0), 5, 3, True)
    stations = generate_planar_control_stations(
        track, PlanarControlStationPolicy(100, math.pi / 2))
    with pytest.raises(SpeedBackendUnavailableError, match="accelerated extra"):
        evaluate_planar_racing_line(
            np.zeros(len(stations)), track, bike(), stations, speed_backend="numba")


def test_numba_direct_serial_and_restart_match_python():
    pytest.importorskip("numba")
    track = Track((CircularArc(30, 2 * math.pi),), Pose(0, 0, 0), 5, 3, True)
    policy = PlanarControlStationPolicy(100, math.pi / 2)
    stations = generate_planar_control_stations(track, policy)
    controls = np.full(len(stations), 0.25)
    python_evaluation = evaluate_planar_racing_line(
        controls, track, bike(), stations, sample_spacing_m=2,
        boundary_check_spacing_m=1, speed_backend="python")
    numba_evaluation = evaluate_planar_racing_line(
        controls, track, bike(), stations, sample_spacing_m=2,
        boundary_check_spacing_m=1, speed_backend="numba")
    assert python_evaluation.feasible and numba_evaluation.feasible
    _validate_speed_backend_equivalence(
        python_evaluation.speed_profile, numba_evaluation.speed_profile)

    common = dict(max_sweeps=1, max_evaluations=30,
                  boundary_check_spacing_m=1, optimisation_sample_spacing_m=2)
    python_result = optimise_planar_racing_line(
        track, bike(), policy, PlanarOptimisationConfig(**common), controls)
    numba_result = optimise_planar_racing_line(
        track, bike(), policy,
        PlanarOptimisationConfig(**common, speed_backend="numba"), controls)
    assert np.array_equal(python_result.initial_controls_m, numba_result.initial_controls_m)
    assert np.array_equal(python_result.best_controls_m, numba_result.best_controls_m)
    assert abs(python_result.best_lap_time_s - numba_result.best_lap_time_s) <= 1e-9
    canonical = solve_speed_profile(numba_result.sampled_path, bike())
    assert np.array_equal(numba_result.speed_profile.speed_mps, canonical.speed_mps)
    assert (python_result.evaluations, python_result.sweeps, python_result.final_step_m,
            python_result.termination_reason) == (
                numba_result.evaluations, numba_result.sweeps, numba_result.final_step_m,
                numba_result.termination_reason)


def test_spawn_numba_poll_matches_serial_numba():
    pytest.importorskip("numba")
    track = Track((CircularArc(30, 2 * math.pi),), Pose(0, 0, 0), 5, 3, True)
    policy = PlanarControlStationPolicy(100, math.pi / 2)
    common = dict(max_sweeps=1, max_evaluations=30,
                  boundary_check_spacing_m=1, optimisation_sample_spacing_m=2,
                  speed_backend="numba")
    serial = optimise_planar_racing_line(
        track, bike(), policy, PlanarOptimisationConfig(**common))
    parallel = optimise_planar_racing_line(
        track, bike(), policy,
        PlanarOptimisationConfig(**common, parallel_workers=2))
    assert np.array_equal(serial.best_controls_m, parallel.best_controls_m)
    assert serial.best_lap_time_s == parallel.best_lap_time_s
    assert (serial.evaluations, serial.sweeps, serial.final_step_m,
            serial.termination_reason) == (
        parallel.evaluations, parallel.sweeps, parallel.final_step_m,
        parallel.termination_reason)


def test_one_worker_preserves_default_serial_behaviour():
    track = Track((CircularArc(30, 2 * math.pi),), Pose(0, 0, 0), 5, 3, True)
    policy = PlanarControlStationPolicy(100, math.pi / 2)
    common = dict(max_sweeps=1, max_evaluations=30,
                  boundary_check_spacing_m=1,
                  optimisation_sample_spacing_m=2)
    default = optimise_planar_racing_line(
        track, bike(), policy, PlanarOptimisationConfig(**common))
    explicit = optimise_planar_racing_line(
        track, bike(), policy,
        PlanarOptimisationConfig(**common, parallel_workers=1))

    assert np.array_equal(default.best_controls_m, explicit.best_controls_m)
    assert default.best_lap_time_s == explicit.best_lap_time_s
    assert (default.evaluations, default.sweeps, default.final_step_m,
            default.termination_reason) == (
                explicit.evaluations, explicit.sweeps, explicit.final_step_m,
                explicit.termination_reason)


def test_spawn_parallel_poll_matches_serial_result_exactly():
    track = Track((CircularArc(30, 2 * math.pi),), Pose(0, 0, 0), 5, 3, True)
    policy = PlanarControlStationPolicy(100, math.pi / 2)
    common = dict(max_sweeps=1, max_evaluations=30,
                  boundary_check_spacing_m=1,
                  optimisation_sample_spacing_m=2)
    serial = optimise_planar_racing_line(
        track, bike(), policy,
        PlanarOptimisationConfig(**common, parallel_workers=1))
    parallel = optimise_planar_racing_line(
        track, bike(), policy,
        PlanarOptimisationConfig(**common, parallel_workers=2))

    assert np.array_equal(serial.best_controls_m, parallel.best_controls_m)
    assert serial.best_lap_time_s == parallel.best_lap_time_s
    assert (serial.evaluations, serial.sweeps, serial.final_step_m,
            serial.termination_reason) == (
                parallel.evaluations, parallel.sweeps, parallel.final_step_m,
                parallel.termination_reason)


def test_out_of_order_poll_completion_retains_candidate_tie_breaking():
    """A batch may finish backwards, but its returned records retain input order."""
    def evaluate(controls):
        return SimpleNamespace(feasible=True, lap_time_s=5.0)

    completion_order = []

    def complete_backwards_in_input_order(candidates):
        candidates = list(candidates)
        results = [None] * len(candidates)
        for index in reversed(range(len(candidates))):
            completion_order.append(index)
            results[index] = evaluate(candidates[index])
        return results

    controls = np.zeros(3)
    initial = SimpleNamespace(feasible=True, lap_time_s=10.0)
    config = PlanarOptimisationConfig(max_sweeps=1, max_evaluations=30)
    result, _, _, polls, _, _ = _best_improvement_pattern_search(
        controls, np.full(3, -2.0), np.full(3, 2.0), initial, evaluate,
        config, complete_backwards_in_input_order)

    assert completion_order == sorted(completion_order, reverse=True)
    assert polls == 1
    # All candidates tie, so direction 0/sign 0 (+ first coordinate) wins.
    assert np.array_equal(result, np.array([1.0, 0.0, 0.0]))
