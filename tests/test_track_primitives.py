import math

import pytest

from motorcycle_lap_sim.track.primitives import CircularArc, Pose, Straight


def test_straight_end_pose() -> None:
    end = Straight(10.0).end_pose(Pose(1.0, 2.0, math.pi / 2))
    assert end.x_m == pytest.approx(1.0)
    assert end.y_m == pytest.approx(12.0)
    assert end.heading_rad == pytest.approx(math.pi / 2)


def test_positive_quarter_circle_turns_left() -> None:
    arc = CircularArc(10.0, math.pi / 2)
    end = arc.end_pose(Pose(0.0, 0.0, 0.0))
    assert (end.x_m, end.y_m, end.heading_rad) == pytest.approx((10.0, 10.0, math.pi / 2))
    assert arc.curvature_1pm == pytest.approx(0.1)


def test_negative_quarter_circle_turns_right() -> None:
    arc = CircularArc(10.0, -math.pi / 2)
    end = arc.end_pose(Pose(0.0, 0.0, 0.0))
    assert (end.x_m, end.y_m, end.heading_rad) == pytest.approx((10.0, -10.0, -math.pi / 2))
    assert arc.curvature_1pm == pytest.approx(-0.1)
