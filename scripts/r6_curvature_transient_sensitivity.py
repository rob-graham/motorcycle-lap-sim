"""Re-evaluate one reproducible R6 test-oval line at transient limits.

The recorded Phase 6 1.0 m optimised controls are only re-evaluated at 0.25 m,
not re-optimised for each limit.  This diagnostic
does not establish that any tested value is physically correct.
"""

from dataclasses import replace
import numpy as np

from motorcycle_lap_sim.motorcycle.config import HandlingConfig, load_motorcycle_config
from motorcycle_lap_sim.optimisation import PeriodicCubicParameterisation, evaluate_racing_line
from motorcycle_lap_sim.track import Track, sample_track


def main() -> None:
    track = Track.from_yaml("examples/tracks/test_oval.yaml")
    bike = load_motorcycle_config("examples/motorcycles/r6_2017plus_reference.yaml")
    samples = sample_track(track, 0.25)
    parameterisation = PeriodicCubicParameterisation(12)
    controls = np.array([0.6875, -4, -4, 0.6875, 4, 4,
                         0.625, -4, -4, 0.5625, 4, 4])
    print("case,lap_time_s,max_abs_curvature_rate_1pmps,evaluation")
    for value in (None, 0.4, 0.8, 1.6):
        candidate = replace(bike, handling=None if value is None else HandlingConfig(value))
        result = evaluate_racing_line(controls, samples, candidate, parameterisation, 0.25)
        maximum = np.max(np.abs(result.speed_profile.curvature_rate_1pmps))
        print(f"{'disabled' if value is None else value},{result.lap_time_s:.9f},{maximum:.9f},re-evaluated")


if __name__ == "__main__":
    main()
