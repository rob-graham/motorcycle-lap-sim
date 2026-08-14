"""Pure speed ceilings, ideal gearing, and combined-force capabilities."""
from dataclasses import dataclass
from math import inf, pi, sqrt
from motorcycle_lap_sim.motorcycle.config import MotorcycleConfig
from motorcycle_lap_sim.motorcycle.engine import available_engine_torque_nm
from motorcycle_lap_sim.motorcycle.forces import aerodynamic_drag_n, axle_normal_loads_n, rolling_resistance_n
from motorcycle_lap_sim.motorcycle.limits import effective_lateral_acceleration_mps2, maximum_longitudinal_force_n, stoppie_deceleration_mps2, wheelie_acceleration_mps2
from motorcycle_lap_sim.motorcycle.powertrain import engine_speed_rpm, overall_ratio, rear_wheel_drive_force_n

@dataclass(frozen=True)
class NumericalConfig:
    acceleration_tolerance_mps2: float = 1e-7
    bisection_iterations: int = 60

@dataclass(frozen=True)
class GearChoice:
    gear_number: int; engine_rpm: float; drive_force_n: float

@dataclass(frozen=True)
class AccelerationCapability:
    acceleration_mps2: float; gear_number: int; engine_rpm: float; available_drive_force_n: float
    rear_traction_capacity_n: float; drag_n: float; rolling_resistance_n: float
    front_normal_load_n: float; rear_normal_load_n: float; limiting_reason: str

@dataclass(frozen=True)
class BrakingCapability:
    deceleration_mps2: float; tyre_capacity_n: float; drag_n: float; rolling_resistance_n: float
    front_normal_load_n: float; rear_normal_load_n: float; limiting_reason: str

def lateral_speed_limit_mps(curvature_1pm: float, bike: MotorcycleConfig) -> float:
    if curvature_1pm == 0: return inf
    ay = effective_lateral_acceleration_mps2(bike.tyres.mu_lateral, bike.tyres.max_lean_angle_rad,
                                             bike.environment.gravity_mps2)
    return sqrt(ay / abs(curvature_1pm))

def road_speed_at_rpm_mps(rpm: float, gear_number: int, bike: MotorcycleConfig) -> float:
    return rpm * 2*pi/60 / overall_ratio(bike.powertrain, gear_number) * bike.motorcycle.wheel_radius_m

def maximum_rev_limited_speed_mps(bike: MotorcycleConfig) -> float:
    return max(road_speed_at_rpm_mps(bike.powertrain.rev_limit_rpm, g, bike)
               for g in range(1, len(bike.powertrain.gear_ratios)+1))

def best_gear(speed_mps: float, bike: MotorcycleConfig) -> GearChoice:
    best = GearChoice(0, 0.0, 0.0)
    for gear in range(1, len(bike.powertrain.gear_ratios)+1):
        ratio = overall_ratio(bike.powertrain, gear)
        rpm = engine_speed_rpm(speed_mps, bike.motorcycle.wheel_radius_m, ratio)
        # Converting the rev ceiling to road speed and back can land a few
        # ulps above the ceiling.  Treat that round trip as exactly on-limit.
        if abs(rpm-bike.powertrain.rev_limit_rpm) <= 1e-9*bike.powertrain.rev_limit_rpm:
            rpm=bike.powertrain.rev_limit_rpm
        torque = available_engine_torque_nm(rpm, bike.powertrain)
        force = rear_wheel_drive_force_n(torque, ratio, bike.powertrain.driveline_efficiency,
                                         bike.motorcycle.wheel_radius_m) if torque > 0 else 0.0
        if force > best.drive_force_n: best = GearChoice(gear, rpm, force)
    return best

def _resistance(speed, bike):
    return (aerodynamic_drag_n(speed, bike.environment.air_density_kgpm3, bike.aerodynamics.cda_m2),
            rolling_resistance_n(bike.rolling_resistance.crr, bike.motorcycle.mass_kg,
                                 bike.environment.gravity_mps2))

def _axle(speed, curvature, acceleration, bike):
    m=bike.motorcycle.mass_kg; g=bike.environment.gravity_mps2
    loads=axle_normal_loads_n(m,g,bike.motorcycle.wheelbase_m,bike.motorcycle.cg_height_m,
                              bike.motorcycle.cg_from_rear_m,acceleration)
    if loads.front_n < 0 or loads.rear_n < 0: return loads, 0., 0.
    fy=m*speed**2*abs(curvature)
    return loads, fy*loads.front_n/(m*g), fy*loads.rear_n/(m*g)

def forward_acceleration_capability(speed_mps, curvature_1pm, bike, numerical=NumericalConfig()):
    m=bike.motorcycle.mass_kg; drag,roll=_resistance(speed_mps,bike); choice=best_gear(speed_mps,bike)
    upper=wheelie_acceleration_mps2(bike.environment.gravity_mps2,bike.motorcycle.cg_from_rear_m,bike.motorcycle.cg_height_m)
    def margin(a):
        loads,_,fyr=_axle(speed_mps,curvature_1pm,a,bike)
        if loads.front_n < 0 or loads.rear_n < 0: return -inf
        tyre=maximum_longitudinal_force_n(fyr,loads.rear_n,bike.tyres.mu_longitudinal,bike.tyres.mu_lateral)
        return min(choice.drive_force_n,tyre)-(m*a+drag+roll)
    low=-(drag+roll)/m
    # ``low`` is the exact coasting solution when no propulsive force is
    # available.  Floating-point roundoff in ``m * low + drag + roll`` must
    # not turn that useful (and physically meaningful) lower bound into the
    # deliberately remote fallback bound.
    if margin(low) < -numerical.acceleration_tolerance_mps2 * m:
        low=-bike.environment.gravity_mps2*10
    hi=upper
    for _ in range(numerical.bisection_iterations):
        mid=(low+hi)/2
        if margin(mid)>=0: low=mid
        else: hi=mid
    a=low; loads,_,fyr=_axle(speed_mps,curvature_1pm,a,bike)
    tyre=maximum_longitudinal_force_n(fyr,max(0.,loads.rear_n),bike.tyres.mu_longitudinal,bike.tyres.mu_lateral)
    required=m*a+drag+roll
    reason="coasting/resistance" if a<0 else ("wheelie" if abs(a-upper)<1e-5 else ("engine/power" if choice.drive_force_n<=tyre else "tyre traction"))
    return AccelerationCapability(a,choice.gear_number,choice.engine_rpm,choice.drive_force_n,tyre,drag,roll,loads.front_n,loads.rear_n,reason)

def braking_capability(speed_mps, curvature_1pm, bike, numerical=NumericalConfig()):
    m=bike.motorcycle.mass_kg; drag,roll=_resistance(speed_mps,bike)
    upper=stoppie_deceleration_mps2(bike.environment.gravity_mps2,bike.motorcycle.wheelbase_m,bike.motorcycle.cg_from_rear_m,bike.motorcycle.cg_height_m)
    def data(d):
        loads,fyf,fyr=_axle(speed_mps,curvature_1pm,-d,bike)
        if loads.front_n < 0 or loads.rear_n < 0:return loads,0.,False
        cap=maximum_longitudinal_force_n(fyf,loads.front_n,bike.tyres.mu_longitudinal,bike.tyres.mu_lateral)+maximum_longitudinal_force_n(fyr,loads.rear_n,bike.tyres.mu_longitudinal,bike.tyres.mu_lateral)
        return loads,cap,max(0.,m*d-drag-roll)<=cap
    low,hi=0.,upper
    for _ in range(numerical.bisection_iterations):
        mid=(low+hi)/2
        if data(mid)[2]:low=mid
        else:hi=mid
    loads,cap,_=data(low)
    reason="stoppie" if abs(low-upper)<1e-5 else "tyre traction"
    return BrakingCapability(low,cap,drag,roll,loads.front_n,loads.rear_n,reason)
