import math
import numpy as np
import pytest
from motorcycle_lap_sim.motorcycle.config import load_motorcycle_config
from motorcycle_lap_sim.racing_line import PeriodicPlanarSpline, build_smooth_racing_line_path
from motorcycle_lap_sim.speed_solver import solve_speed_profile
from motorcycle_lap_sim.track import Track, sample_track_stations

def parametric_spline(count, x, y, period=2*math.pi):
    s = np.arange(count) * period / count
    return PeriodicPlanarSpline(s, x(s), y(s), period)

def test_periodic_c2_continuity_at_every_knot():
    spline = parametric_spline(9, lambda s: np.cos(s)+.13*np.cos(3*s), lambda s: .7*np.sin(s)-.08*np.sin(2*s))
    epsilon = 1e-8
    for knot in np.r_[spline.guide_s_m, spline.period_m]:
        left, right = spline.evaluate(knot-epsilon), spline.evaluate(knot+epsilon)
        for pair in ((0,1),(2,3),(4,5)):
            assert np.allclose([left[i] for i in pair], [right[i] for i in pair], atol=2e-6)

def test_circle_length_and_positive_curvature_converge():
    errors=[]
    for count in (12,24,48):
        r=20.; spline=parametric_spline(count,lambda s:r*np.cos(s),lambda s:r*np.sin(s)); path=spline.sampled_path(.2)
        errors.append(abs(path.total_length_m-2*math.pi*r)); assert np.all(path.curvature_1pm>0)
        assert np.max(abs(path.curvature_1pm-1/r)) < .003
    assert errors[2] < errors[1] < errors[0]

def test_ellipse_analytic_curvature_converges():
    errors=[]
    for count in (16,32,64):
        a,b=12.,7.; spline=parametric_spline(count,lambda s:a*np.cos(s),lambda s:b*np.sin(s))
        s=np.linspace(0,2*math.pi,501,endpoint=False); _,_,dx,dy,ddx,ddy=spline.evaluate(s)
        actual=(dx*ddy-dy*ddx)/np.hypot(dx,dy)**3
        expected=a*b/(a*a*np.sin(s)**2+b*b*np.cos(s)**2)**1.5; errors.append(np.max(abs(actual-expected)))
    assert errors[2] < errors[1] < errors[0]

def test_fixed_spline_output_resolution_preserves_integrated_length():
    spline=parametric_spline(32,lambda s:10*np.cos(s),lambda s:6*np.sin(s))
    paths=[spline.sampled_path(v) for v in (1.,.5,.25)]
    assert len({p.total_length_m for p in paths}) == 1 and len({len(p.q_m) for p in paths}) == 3

def test_dense_corridor_check_rejects_between_guide_overshoot():
    track=Track.from_yaml('examples/tracks/test_oval.yaml'); offsets=np.resize([3.74,3.74,-3.74,-3.74],24)
    with pytest.raises(ValueError,match='overshoots'):
        build_smooth_racing_line_path(track,offsets,sample_spacing_m=1,boundary_margin_m=.25,boundary_check_spacing_m=.05)

@pytest.mark.parametrize('stations',[[0,1,1,3],[1,2,3,4],[0,1,np.nan,3]])
def test_invalid_guide_stations_are_rejected(stations):
    with pytest.raises(ValueError): PeriodicPlanarSpline(stations,[0,1,0,-1],[1,0,-1,0],4)

def test_degenerate_and_invalid_inputs_are_rejected():
    with pytest.raises(ValueError,match='at least four'): PeriodicPlanarSpline([0,1,2],[0,1,0],[0,0,0],3)
    degenerate=PeriodicPlanarSpline([0,1,2,3],[0]*4,[0]*4,4)
    with pytest.raises(ValueError,match='tangent'): degenerate.sampled_path(.1)
    track=Track.from_yaml('examples/tracks/test_oval.yaml')
    for spacing in (0,-1,math.inf):
        with pytest.raises(ValueError): build_smooth_racing_line_path(track,np.zeros(8),sample_spacing_m=spacing)
        with pytest.raises(ValueError): build_smooth_racing_line_path(track,np.zeros(8),sample_spacing_m=1,boundary_check_spacing_m=spacing)

def test_exact_track_stations_and_speed_solver_interoperate():
    track=Track.from_yaml('examples/tracks/test_oval.yaml'); stations=np.arange(24)*track.total_length_m/24
    exact=sample_track_stations(track,stations); result=build_smooth_racing_line_path(track,np.zeros(24),sample_spacing_m=1)
    assert np.allclose(result.guide_x_m,exact.x_m)
    profile=solve_speed_profile(result.sampled_path,load_motorcycle_config('examples/motorcycles/test_motorcycle.yaml'))
    assert profile.converged and profile.lap_time_s>0 and np.all(np.isfinite(profile.speed_mps))
    assert np.all(np.isfinite(profile.curvature_gradient_1pm2))

def test_nonuniform_guides_interpolate_and_are_c2_periodic():
    stations=np.array([0.,.4,1.7,2.2,4.8,5.5])
    x=np.cos(stations); y=.7*np.sin(stations)
    spline=PeriodicPlanarSpline(stations,x,y,2*math.pi)
    evaluated=spline.evaluate(stations)
    assert np.allclose(evaluated[0],x,atol=1e-13) and np.allclose(evaluated[1],y,atol=1e-13)
    for knot in np.r_[stations,2*math.pi]:
        left=spline.evaluate(knot-1e-8); right=spline.evaluate(knot+1e-8)
        assert all(np.allclose(left[i],right[i],atol=2e-6) for i in range(6))
    assert np.all(np.isfinite(spline.sampled_path(.1).curvature_1pm))

def test_explicit_uniform_stations_preserve_implicit_uniform_result():
    track=Track.from_yaml('examples/tracks/test_oval.yaml'); offsets=np.zeros(16)
    stations=np.arange(16)*track.total_length_m/16
    implicit=build_smooth_racing_line_path(track,offsets,sample_spacing_m=1)
    explicit=build_smooth_racing_line_path(track,offsets,guide_s_m=stations,sample_spacing_m=1)
    assert np.array_equal(implicit.sampled_path.x_m,explicit.sampled_path.x_m)
    assert np.array_equal(implicit.sampled_path.curvature_1pm,explicit.sampled_path.curvature_1pm)
    assert implicit.minimum_forward_progress > 0
