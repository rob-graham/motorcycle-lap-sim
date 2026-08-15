import math
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
from motorcycle_lap_sim.optimisation.planar import _best_improvement_pattern_search


def bike(): return load_motorcycle_config("examples/motorcycles/test_motorcycle.yaml")


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
