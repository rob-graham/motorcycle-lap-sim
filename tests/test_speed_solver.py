import numpy as np
import pytest
from dataclasses import replace
from motorcycle_lap_sim.motorcycle.config import load_motorcycle_config
from motorcycle_lap_sim.path import SampledPath
from motorcycle_lap_sim.motorcycle.limits import maximum_longitudinal_force_n
from motorcycle_lap_sim.speed_solver import (best_gear, braking_capability,
    forward_acceleration_capability, lateral_speed_limit_mps,
    maximum_rev_limited_speed_mps, road_speed_at_rpm_mps, solve_speed_profile)
from motorcycle_lap_sim.speed_solver.solver import lap_time_seconds
from motorcycle_lap_sim.track import Track, sample_track
from motorcycle_lap_sim.path import from_sampled_track

@pytest.fixture
def bike():
    return load_motorcycle_config("examples/motorcycles/test_motorcycle.yaml")

def path(k=(0.,0.,0.), length=30.):
    return SampledPath(np.array([0.,10.,20.]),np.zeros(3),np.zeros(3),np.array(k),length)

def test_path_validation_and_wrap():
    assert np.allclose(path().segment_lengths_m,[10,10,10])
    with pytest.raises(ValueError): SampledPath(np.array([0.,0.,2.]),np.zeros(3),np.zeros(3),np.zeros(3),3)
    with pytest.raises(ValueError): SampledPath(np.arange(3.),np.zeros(3),np.zeros(3),np.zeros(3),3,False)

def test_lateral_limits(bike):
    assert np.isinf(lateral_speed_limit_mps(0,bike))
    assert lateral_speed_limit_mps(.02,bike)==pytest.approx(lateral_speed_limit_mps(-.02,bike))
    assert lateral_speed_limit_mps(.04,bike)<lateral_speed_limit_mps(.01,bike)

def test_rev_speed(bike):
    expected=bike.powertrain.rev_limit_rpm*2*np.pi/60/bike.powertrain.primary_ratio/bike.powertrain.gear_ratios[0]/bike.powertrain.final_drive_ratio*bike.motorcycle.wheel_radius_m
    assert road_speed_at_rpm_mps(bike.powertrain.rev_limit_rpm,1,bike)==pytest.approx(expected)
    assert maximum_rev_limited_speed_mps(bike)>=expected

def test_rev_ceiling_uses_all_gears_not_last_gear(bike):
    powertrain=replace(bike.powertrain,gear_ratios=(1.0,2.0))
    unusual=replace(bike,powertrain=powertrain)
    assert maximum_rev_limited_speed_mps(unusual)==pytest.approx(road_speed_at_rpm_mps(powertrain.rev_limit_rpm,1,unusual))

def test_best_gear_is_deterministic_and_reports_no_usable_gear(bike):
    assert best_gear(30.,bike).gear_number==2
    assert best_gear(1.,bike).gear_number==0
    assert best_gear(1.,bike).drive_force_n==0.

def test_synthetic_bike_limits_and_known_acceleration(bike):
    assert forward_acceleration_capability(10.,0.,bike).acceleration_mps2==pytest.approx(9.81)
    assert braking_capability(10.,0.,bike).deceleration_mps2==pytest.approx(9.81)
    assert forward_acceleration_capability(30.,0.,bike).acceleration_mps2==pytest.approx(7.398,abs=.001)

def test_load_transfer_rear_traction_and_combined_braking(bike):
    drive=forward_acceleration_capability(30.,0.,bike)
    brake=braking_capability(10.,0.,bike)
    static=bike.motorcycle.mass_kg*bike.environment.gravity_mps2/2
    assert drive.rear_normal_load_n>static and drive.front_normal_load_n<static
    assert brake.front_normal_load_n>static and brake.rear_normal_load_n==pytest.approx(0.,abs=1e-8)
    rear_capacity=maximum_longitudinal_force_n(0.,drive.rear_normal_load_n,bike.tyres.mu_longitudinal,bike.tyres.mu_lateral)
    whole_bike_capacity=bike.tyres.mu_longitudinal*bike.motorcycle.mass_kg*bike.environment.gravity_mps2
    assert drive.rear_traction_capacity_n==pytest.approx(rear_capacity)
    assert drive.rear_traction_capacity_n<whole_bike_capacity
    # Service braking sums both axle ellipses (at this stoppie limit the rear
    # contribution naturally reaches zero).
    front_capacity=maximum_longitudinal_force_n(0.,brake.front_normal_load_n,bike.tyres.mu_longitudinal,bike.tyres.mu_lateral)
    rear_capacity=maximum_longitudinal_force_n(0.,brake.rear_normal_load_n,bike.tyres.mu_longitudinal,bike.tyres.mu_lateral)
    assert brake.tyre_capacity_n==pytest.approx(front_capacity+rear_capacity)

def test_forward_capability_can_be_negative_without_a_usable_gear(bike):
    capability=forward_acceleration_capability(1.,0.,bike)
    expected=-(capability.drag_n+capability.rolling_resistance_n)/bike.motorcycle.mass_kg
    assert capability.acceleration_mps2==pytest.approx(expected)
    assert capability.acceleration_mps2<0

def test_constant_lap_time(): assert lap_time_seconds(path(),np.full(3,5.))==pytest.approx(6.)

def test_closed_solution_positive_and_within_limits(bike):
    result=solve_speed_profile(path((.02,.02,.02)),bike)
    assert result.speed_mps[0]>0
    assert np.all(result.speed_mps<=result.speed_limit_lateral_mps+1e-10)
    assert np.all(result.speed_mps<=result.speed_limit_powertrain_mps+1e-10)

def corner_path(spacing=5., shift=0):
    length=240.; q=np.arange(0.,length,spacing); curvature=np.zeros(len(q))
    curvature[(q>=100)&(q<140)]=.04
    curvature=np.roll(curvature,shift)
    return SampledPath(q,np.zeros(len(q)),np.zeros(len(q)),curvature,length)

def test_brakes_before_and_accelerates_after_corner(bike):
    result=solve_speed_profile(corner_path(),bike)
    assert np.any(result.longitudinal_acceleration_mps2[12:20]<0)
    assert np.any(result.longitudinal_acceleration_mps2[28:38]>0)
    corner=slice(20,28)
    assert np.allclose(result.speed_mps[corner],result.speed_limit_lateral_mps[corner])
    assert np.all(result.speed_mps<=result.speed_limit_powertrain_mps+1e-9)

def test_every_forward_and_backward_segment_is_feasible_including_wrap(bike):
    p=corner_path(); result=solve_speed_profile(p,bike); ds=p.segment_lengths_m
    for i in range(len(p.q_m)):
        j=(i+1)%len(p.q_m)
        forward=forward_acceleration_capability(result.speed_mps[i],p.curvature_1pm[i],bike).acceleration_mps2
        braking=braking_capability(result.speed_mps[j],p.curvature_1pm[j],bike).deceleration_mps2
        assert result.speed_mps[j]**2<=result.speed_mps[i]**2+2*forward*ds[i]+1e-6
        assert result.speed_mps[i]**2<=result.speed_mps[j]**2+2*braking*ds[i]+1e-6

def test_periodic_solution_has_nonzero_start_and_is_start_finish_invariant(bike):
    original=solve_speed_profile(corner_path(),bike)
    shifted=solve_speed_profile(corner_path(shift=11),bike)
    assert original.speed_mps[0]>0
    assert np.allclose(original.speed_mps,np.roll(shifted.speed_mps,-11),atol=1e-6)
    assert shifted.lap_time_s==pytest.approx(original.lap_time_s)

def test_sampling_convergence_at_two_one_and_half_metre(bike):
    track=Track.from_yaml("examples/tracks/test_oval.yaml")
    times=[solve_speed_profile(from_sampled_track(sample_track(track,s)),bike).lap_time_s for s in (2.,1.,.5)]
    assert abs(times[2]-times[1])<abs(times[1]-times[0])
    assert times==pytest.approx([17.4286,17.4052,17.3933],abs=1e-4)
