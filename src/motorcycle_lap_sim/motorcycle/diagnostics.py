"""Command-line reporting for independent motorcycle-model checks."""

from __future__ import annotations

import argparse

from .config import load_motorcycle_config
from .engine import available_engine_torque_nm
from .forces import axle_normal_loads_n
from .limits import (effective_lateral_acceleration_mps2,
                     lean_lateral_acceleration_mps2,
                     stoppie_deceleration_mps2,
                     tyre_lateral_acceleration_mps2,
                     wheelie_acceleration_mps2)
from .powertrain import engine_speed_rpm, overall_ratio, rear_wheel_drive_force_n


def report(path: str) -> str:
    config = load_motorcycle_config(path)
    bike, env, tyres, power = (config.motorcycle, config.environment,
                                config.tyres, config.powertrain)
    static = axle_normal_loads_n(bike.mass_kg, env.gravity_mps2, bike.wheelbase_m,
                                 bike.cg_height_m, bike.cg_from_rear_m, 0.0)
    total = static.front_n + static.rear_n
    wheelie = wheelie_acceleration_mps2(env.gravity_mps2, bike.cg_from_rear_m,
                                        bike.cg_height_m)
    stoppie = stoppie_deceleration_mps2(env.gravity_mps2, bike.wheelbase_m,
                                        bike.cg_from_rear_m, bike.cg_height_m)
    tyre = tyre_lateral_acceleration_mps2(tyres.mu_lateral, env.gravity_mps2)
    lean = lean_lateral_acceleration_mps2(tyres.max_lean_angle_rad, env.gravity_mps2)
    effective = effective_lateral_acceleration_mps2(
        tyres.mu_lateral, tyres.max_lean_angle_rad, env.gravity_mps2)
    lines = [
        f"Motorcycle: {bike.name}", f"Mass: {bike.mass_kg:.3f} kg",
        f"Static front load: {static.front_n:.3f} N ({100*static.front_n/total:.2f}%)",
        f"Static rear load: {static.rear_n:.3f} N ({100*static.rear_n/total:.2f}%)",
        f"Wheelie acceleration limit: {wheelie:.3f} m/s^2 ({wheelie/env.gravity_mps2:.3f} g)",
        f"Stoppie deceleration limit: {stoppie:.3f} m/s^2 ({stoppie/env.gravity_mps2:.3f} g)",
        f"Tyre lateral acceleration cap: {tyre:.3f} m/s^2",
        f"Lean-angle lateral acceleration cap: {lean:.3f} m/s^2",
        f"Effective lateral acceleration cap: {effective:.3f} m/s^2",
    ]
    for speed, gear in ((10.0, 1), (20.0, 2)):
        ratio = overall_ratio(power, gear)
        rpm = engine_speed_rpm(speed, bike.wheel_radius_m, ratio)
        torque = available_engine_torque_nm(rpm, power)
        force = rear_wheel_drive_force_n(torque, ratio, power.driveline_efficiency,
                                         bike.wheel_radius_m)
        lines.append(f"At {speed:.1f} m/s in gear {gear}: engine {rpm:.1f} rpm, "
                     f"unconstrained drive force {force:.1f} N")
    return "\n".join(lines)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("configuration")
    args = parser.parse_args()
    print(report(args.configuration))


if __name__ == "__main__":
    main()
