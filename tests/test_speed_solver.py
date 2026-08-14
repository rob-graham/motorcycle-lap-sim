import numpy as np
import pytest
from motorcycle_lap_sim.motorcycle.config import load_motorcycle_config
from motorcycle_lap_sim.path import SampledPath
from motorcycle_lap_sim.speed_solver import lateral_speed_limit_mps, maximum_rev_limited_speed_mps, road_speed_at_rpm_mps, solve_speed_profile
from motorcycle_lap_sim.speed_solver.solver import lap_time_seconds

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

def test_constant_lap_time(): assert lap_time_seconds(path(),np.full(3,5.))==pytest.approx(6.)

def test_closed_solution_positive_and_within_limits(bike):
    result=solve_speed_profile(path((.02,.02,.02)),bike)
    assert result.speed_mps[0]>0
    assert np.all(result.speed_mps<=result.speed_limit_lateral_mps+1e-10)
    assert np.all(result.speed_mps<=result.speed_limit_powertrain_mps+1e-10)
