"""Deterministic first-order cyclic forward/backward propagation."""
from dataclasses import dataclass
import numpy as np
from motorcycle_lap_sim.motorcycle.roll import (
    curvature_transition_roll_rate_radps,
    demanded_lean_rad,
    roll_rate_speed_limit_mps,
)
from motorcycle_lap_sim.path import curvature_gradient_1pm2, curvature_transient_speed_limit_mps
from .capabilities import best_gear, braking_capability, forward_acceleration_capability, lateral_speed_limit_mps, maximum_rev_limited_speed_mps
from .results import SpeedProfileResult

@dataclass(frozen=True)
class SolverConfig:
    speed_tolerance_mps: float=1e-6
    max_iterations: int=1000

def lap_time_seconds(path, speed):
    dq=path.segment_lengths_m; following=np.roll(speed,-1); denomin=speed+following
    if np.any(denomin<=0) or not np.all(np.isfinite(denomin)): raise ValueError("finite lap requires positive endpoint speeds")
    return float(np.sum(2*dq/denomin))

def solve_speed_profile(path,bike,config=SolverConfig()):
    n=len(path.q_m); dq=path.segment_lengths_m
    lateral=np.array([lateral_speed_limit_mps(k,bike) for k in path.curvature_1pm])
    power=np.full(n,maximum_rev_limited_speed_mps(bike))
    gradient=curvature_gradient_1pm2(path)
    handling=bike.handling
    curvature_limit=(
        np.full(n,np.inf)
        if handling is None or handling.max_path_curvature_rate_1pmps is None
        else curvature_transient_speed_limit_mps(
            path,handling.max_path_curvature_rate_1pmps))
    pre_roll_cap=np.minimum(np.minimum(lateral,power),curvature_limit)
    roll_limit=(
        np.full(n,np.inf)
        if handling is None or handling.max_roll_rate_radps is None
        else roll_rate_speed_limit_mps(
            path.curvature_1pm,gradient,pre_roll_cap,handling.max_roll_rate_radps,
            gravity_mps2=bike.environment.gravity_mps2))
    speed=np.minimum(pre_roll_cap,roll_limit)
    converged=False
    for iteration in range(1,config.max_iterations+1):
        old=speed.copy()
        for i in range(n):
            j=(i+1)%n; a=forward_acceleration_capability(speed[i],path.curvature_1pm[i],bike).acceleration_mps2
            speed[j]=min(speed[j],sqrt_nonnegative(speed[i]**2+2*a*dq[i]))
        for i in range(n-1,-1,-1):
            j=(i+1)%n; d=braking_capability(speed[j],path.curvature_1pm[j],bike).deceleration_mps2
            speed[i]=min(speed[i],sqrt_nonnegative(speed[j]**2+2*d*dq[i]))
        if float(np.max(np.abs(old-speed))) < config.speed_tolerance_mps: converged=True; break
    if not converged: raise RuntimeError(f"periodic speed solver did not converge in {config.max_iterations} iterations")
    following=np.roll(speed,-1); ax=(following**2-speed**2)/(2*dq)
    ay=speed**2*np.abs(path.curvature_1pm); gears=[]; rpms=[]
    for v in speed:
        choice=best_gear(v,bike); gears.append(choice.gear_number); rpms.append(choice.engine_rpm)
    curvature_rate=speed*gradient
    lean=demanded_lean_rad(speed,path.curvature_1pm,gravity_mps2=bike.environment.gravity_mps2)
    roll_rate=curvature_transition_roll_rate_radps(
        speed,path.curvature_1pm,gradient,gravity_mps2=bike.environment.gravity_mps2)
    arrays=[speed,lateral,power,gradient,curvature_rate,curvature_limit,roll_limit,lean,roll_rate,ay,ax,
            np.asarray(gears,dtype=int),np.asarray(rpms)]
    for a in arrays:a.setflags(write=False)
    return SpeedProfileResult(path.q_m,speed,lateral,power,gradient,curvature_rate,curvature_limit,
        roll_limit,lean,roll_rate,ay,ax,arrays[11],arrays[12],lap_time_seconds(path,speed),iteration,True)

def sqrt_nonnegative(value): return float(np.sqrt(max(0.,value)))
