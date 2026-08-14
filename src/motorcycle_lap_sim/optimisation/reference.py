"""Recorded deterministic reference candidates used by reproducible diagnostics."""

import numpy as np

PHASE5_DETERMINISTIC_LOCAL_REFERENCE = np.array(
    [0.6875, -4.0, -4.0, 0.6875, 4.0, 4.0,
     0.625, -4.0, -4.0, 0.5625, 4.0, 4.0], dtype=float)
PHASE5_DETERMINISTIC_LOCAL_REFERENCE.setflags(write=False)
