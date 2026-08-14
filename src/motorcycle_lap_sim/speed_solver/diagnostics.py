"""Command-line validation of the fixed-path speed solver."""
import argparse, csv
import numpy as np
from motorcycle_lap_sim.motorcycle.config import load_motorcycle_config
from motorcycle_lap_sim.path import from_sampled_track
from motorcycle_lap_sim.track import Track, sample_track
from .solver import solve_speed_profile

def main(argv=None):
    parser=argparse.ArgumentParser(); parser.add_argument("track"); parser.add_argument("motorcycle")
    parser.add_argument("--spacing",type=float,default=1.0); parser.add_argument("--csv")
    args=parser.parse_args(argv); path=from_sampled_track(sample_track(Track.from_yaml(args.track),args.spacing))
    result=solve_speed_profile(path,load_motorcycle_config(args.motorcycle))
    print(f"Path length: {path.total_length_m:.3f} m\nLap time: {result.lap_time_s:.6f} s")
    print(f"Speed min/max/mean: {result.speed_mps.min():.3f} / {result.speed_mps.max():.3f} / {result.speed_mps.mean():.3f} m/s")
    print(f"Maximum lateral acceleration: {result.lateral_acceleration_mps2.max():.3f} m/s^2")
    print(f"Maximum forward acceleration: {result.longitudinal_acceleration_mps2.max():.3f} m/s^2")
    print(f"Maximum braking deceleration: {-result.longitudinal_acceleration_mps2.min():.3f} m/s^2")
    used=result.gear_number[result.gear_number>0]
    print(f"Gear range: {used.min() if len(used) else 0} - {used.max() if len(used) else 0}")
    print(f"RPM range: {result.engine_rpm.min():.0f} - {result.engine_rpm.max():.0f}")
    print(f"Iterations: {result.iterations}\nConverged: {result.converged}")
    if args.csv:
        with open(args.csv,"w",newline="",encoding="utf-8") as f:
            w=csv.writer(f); w.writerow("q_m x_m y_m curvature_1pm speed_mps speed_kph lateral_acceleration_mps2 longitudinal_acceleration_mps2 gear_number engine_rpm".split())
            w.writerows(zip(path.q_m,path.x_m,path.y_m,path.curvature_1pm,result.speed_mps,result.speed_mps*3.6,result.lateral_acceleration_mps2,result.longitudinal_acceleration_mps2,result.gear_number,result.engine_rpm))
if __name__ == "__main__": main()
