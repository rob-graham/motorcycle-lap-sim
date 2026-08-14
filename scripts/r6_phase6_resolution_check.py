"""Re-evaluate recorded 1.0 m Phase 6 optimum; do not re-optimise finer grids."""

import numpy as np

from motorcycle_lap_sim.motorcycle.config import load_motorcycle_config
from motorcycle_lap_sim.optimisation import PeriodicCubicParameterisation, evaluate_racing_line
from motorcycle_lap_sim.track import Track, sample_track


def main() -> None:
    track = Track.from_yaml("examples/tracks/test_oval.yaml")
    bike = load_motorcycle_config("examples/motorcycles/r6_2017plus_reference.yaml")
    controls = np.array([0.6875, -4, -4, 0.6875, 4, 4,
                         0.625, -4, -4, 0.5625, 4, 4])
    parameterisation = PeriodicCubicParameterisation(12)
    print("spacing_m,baseline_s,optimised_s,improvement_s,max_rate_1pmps,max_gradient_1pm2")
    for spacing in (1.0, 0.5, 0.25):
        samples = sample_track(track, spacing)
        baseline = evaluate_racing_line(np.zeros(12), samples, bike, parameterisation, 0.25)
        optimised = evaluate_racing_line(controls, samples, bike, parameterisation, 0.25)
        profile = optimised.speed_profile
        print(f"{spacing},{baseline.lap_time_s:.9f},{optimised.lap_time_s:.9f},"
              f"{baseline.lap_time_s - optimised.lap_time_s:.9f},"
              f"{np.max(np.abs(profile.curvature_rate_1pmps)):.9f},"
              f"{np.max(np.abs(profile.curvature_gradient_1pm2)):.9f}")


if __name__ == "__main__":
    main()
