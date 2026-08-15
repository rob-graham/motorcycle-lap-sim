"""Reproducible Phase 8 path-model and fixed-spline resolution diagnostics."""

import argparse
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

from motorcycle_lap_sim.motorcycle.config import load_motorcycle_config
from motorcycle_lap_sim.optimisation import (COARSE_PLANAR_CONTROL_POLICY,
    FINE_PLANAR_CONTROL_POLICY, REFERENCE_PLANAR_CONTROL_POLICY,
    PlanarOptimisationConfig, evaluate_planar_racing_line,
    generate_planar_control_stations, optimise_planar_racing_line, resample_planar_result)
from motorcycle_lap_sim.path import from_sampled_track
from motorcycle_lap_sim.speed_solver import solve_speed_profile
from motorcycle_lap_sim.track import Track, sample_track
from motorcycle_lap_sim.track.boundaries import calculate_boundaries

POLICIES = (("coarse", COARSE_PLANAR_CONTROL_POLICY),
            ("reference", REFERENCE_PLANAR_CONTROL_POLICY),
            ("fine", FINE_PLANAR_CONTROL_POLICY))


def metrics(label, result):
    path=result.sampled_path; speed=result.speed_profile
    print(f"{label}: controls={len(result.control_s_m)} zero_lap_s={result.initial_lap_time_s:.9f} "
          f"optimised_lap_s={result.best_lap_time_s:.9f} improvement_s={result.improvement_s:.9f} "
          f"evaluations={result.evaluations} sweeps={result.sweeps} length_m={path.total_length_m:.9f} "
          f"clearance_m={result.minimum_boundary_clearance_m:.9f} forward_min={result.minimum_forward_progress:.9f} "
          f"curvature_minmax={np.min(path.curvature_1pm):.9f}/{np.max(path.curvature_1pm):.9f} "
          f"max_abs_dk_dq={np.max(np.abs(speed.curvature_gradient_1pm2)):.9f} "
          f"max_abs_dk_dt={np.max(np.abs(speed.curvature_rate_1pmps)):.9f} "
          f"controls_minmax_m={np.min(result.best_controls_m):.6f}/{np.max(result.best_controls_m):.6f} "
          f"termination={result.termination_reason!r}")


def plot(track, zero, result, output):
    sampled=sample_track(track,.5); edges=calculate_boundaries(sampled)
    fig,ax=plt.subplots()
    ax.plot(edges.left_x_m,edges.left_y_m,label="left boundary")
    ax.plot(edges.right_x_m,edges.right_y_m,label="right boundary")
    ax.plot(sampled.x_m,sampled.y_m,"--",label="analytic centreline")
    ax.plot(zero.sampled_path.x_m,zero.sampled_path.y_m,label="zero-control planar")
    ax.plot(result.sampled_path.x_m,result.sampled_path.y_m,label="optimised planar")
    ax.plot(result.smooth_line.guide_x_m,result.smooth_line.guide_y_m,"o",ms=3,label="control guides")
    ax.set_aspect("equal",adjustable="box"); ax.set(xlabel="x [m]",ylabel="y [m]"); ax.legend()
    fig.savefig(output,dpi=160,bbox_inches="tight"); plt.close(fig)


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument("--max-evaluations",type=int,default=1500)
    args=parser.parse_args()
    bike=load_motorcycle_config("examples/motorcycles/r6_2017plus_reference.yaml")
    config=PlanarOptimisationConfig(max_evaluations=args.max_evaluations)
    saved={}
    for name,filename in (("oval","examples/tracks/test_oval.yaml"),
                          ("mallala","examples/tracks/mallala_reference.yaml")):
        track=Track.from_yaml(filename)
        centre=solve_speed_profile(from_sampled_track(sample_track(track,1.0)),bike)
        print(f"\n{name.upper()} analytic_centreline_lap_s={centre.lap_time_s:.9f}")
        for policy_name,policy in POLICIES:
            stations=generate_planar_control_stations(track,policy)
            zero=evaluate_planar_racing_line(np.zeros(len(stations)),track,bike,stations)
            if not zero.feasible:
                print(f"{policy_name}: controls={len(stations)} zero_feasible=false reason={zero.failure_reason}")
                continue
            print(f"{policy_name}: controls={len(stations)} zero_feasible=true "
                  f"zero_lap_s={zero.lap_time_s:.9f} length_m={zero.smooth_line.sampled_path.total_length_m:.9f} "
                  f"clearance_m={zero.smooth_line.minimum_boundary_clearance_m:.9f}")
            # Oval studies all model policies; Mallala must optimise reference.
            if name == "oval" or policy_name == "reference":
                result=optimise_planar_racing_line(track,bike,policy,config)
                metrics(policy_name,result); saved[(name,policy_name)]=(track,zero.smooth_line,result)
        key=(name,"reference")
        if key not in saved and name == "oval" and (name,"fine") in saved:
            print("reference policy is geometrically infeasible; reporting saved fine spline instead")
            key=(name,"fine")
        if key in saved:
            track,zero,result=saved[key]
            speed=result.speed_profile
            print("reference detail: "
                  f"speed_minmax_mps={np.min(speed.speed_mps):.6f}/{np.max(speed.speed_mps):.6f} "
                  f"gears={'/'.join(map(str,np.unique(speed.gear_number)))} "
                  f"rpm_minmax={np.min(speed.engine_rpm):.3f}/{np.max(speed.engine_rpm):.3f} "
                  f"lateral_max_mps2={np.max(np.abs(speed.lateral_acceleration_mps2)):.6f} "
                  f"forward_max_mps2={np.max(speed.longitudinal_acceleration_mps2):.6f} "
                  f"braking_max_mps2={-np.min(speed.longitudinal_acceleration_mps2):.6f}")
            print("fixed-spline output-resolution sensitivity (same geometry)")
            for spacing in (1.,.5,.25):
                path,profile=resample_planar_result(result,bike,spacing)
                print(f"spacing_m={spacing:.2f} length_m={path.total_length_m:.9f} lap_s={profile.lap_time_s:.9f}")
            plot(track,zero,result,Path(f"phase8_{name}_planar.png"))
    print("\nControl-policy comparisons are different path-model orders; fixed-spline comparisons are output resolution only.")
    print("A fastest policy is not automatically most accurate; material changes mean path-model sensitivity NOT CONVERGED.")


if __name__ == "__main__": main()
