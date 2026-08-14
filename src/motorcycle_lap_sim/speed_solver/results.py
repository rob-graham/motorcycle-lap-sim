from dataclasses import dataclass
import numpy as np
from numpy.typing import NDArray

@dataclass(frozen=True)
class SpeedProfileResult:
    q_m: NDArray[np.float64]; speed_mps: NDArray[np.float64]
    speed_limit_lateral_mps: NDArray[np.float64]; speed_limit_powertrain_mps: NDArray[np.float64]
    lateral_acceleration_mps2: NDArray[np.float64]; longitudinal_acceleration_mps2: NDArray[np.float64]
    gear_number: NDArray[np.int64]; engine_rpm: NDArray[np.float64]
    lap_time_s: float; iterations: int; converged: bool
