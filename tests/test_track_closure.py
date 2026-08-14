import math
from pathlib import Path

import pytest

from motorcycle_lap_sim.track.track import Track

OVAL = Path(__file__).parents[1] / "examples/tracks/test_oval.yaml"


def test_analytic_oval_closes() -> None:
    track = Track.from_yaml(OVAL)
    diagnostic = track.closure_diagnostic()
    assert track.total_length_m == pytest.approx(200.0 + 60.0 * math.pi)
    assert diagnostic.position_error_m < 1e-12
    assert abs(diagnostic.heading_error_rad) < 1e-12
    assert diagnostic.passes(1e-10, 1e-10)


def test_closure_is_diagnostic_not_forced() -> None:
    from motorcycle_lap_sim.track.primitives import Pose, Straight
    track = Track((Straight(1.0),), Pose(0, 0, 0), closed=True)
    diagnostic = track.closure_diagnostic()
    assert diagnostic.x_error_m == pytest.approx(1.0)
    assert not diagnostic.passes(0.01, 0.01)
