"""Re-evaluate fixed controls with Phase 6 disabled and at 0.8; do not optimise."""

from dataclasses import replace

import numpy as np

from motorcycle_lap_sim.motorcycle.config import HandlingConfig, load_motorcycle_config
from motorcycle_lap_sim.optimisation import PeriodicCubicParameterisation, evaluate_racing_line
from motorcycle_lap_sim.track import Track, sample_track


def main() -> None:
    track = Track.from_yaml("examples/tracks/test_oval.yaml")
    bike = load_motorcycle_config("examples/motorcycles/r6_2017plus_reference.yaml")
    controls = np.array([0.6875, -4, -4, 0.6875, 4, 4,
                         0.625, -4, -4, 0.5625, 4, 4])
    parameterisation = PeriodicCubicParameterisation(12)
    print("mode,spacing_m,lap_time_s,max_curvature_1pm,max_gradient_1pm2,max_rate_1pmps")
    for label, candidate in (("disabled", bike),
                             ("limit_0.8", replace(bike, handling=HandlingConfig(0.8)))):
        for spacing in (1.0, 0.5, 0.25):
            samples = sample_track(track, spacing)
            evaluation = evaluate_racing_line(controls, samples, candidate,
                                               parameterisation, 0.25)
            profile = evaluation.speed_profile
            print(f"{label},{spacing},{evaluation.lap_time_s:.9f},"
                  f"{np.max(np.abs(evaluation.sampled_path.curvature_1pm)):.9f},"
                  f"{np.max(np.abs(profile.curvature_gradient_1pm2)):.9f},"
                  f"{np.max(np.abs(profile.curvature_rate_1pmps)):.9f}")


if __name__ == "__main__":
    main()
