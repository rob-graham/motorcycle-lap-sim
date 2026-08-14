"""Analytic centreline primitives using metres and radians.

Heading is counter-clockwise from +x. Positive arc angle/curvature turns left;
negative turns right.
"""

from dataclasses import dataclass
from typing import Protocol
import math


@dataclass(frozen=True)
class Pose:
    x_m: float
    y_m: float
    heading_rad: float


class CentrelinePrimitive(Protocol):
    @property
    def length_m(self) -> float: ...
    def pose_at(self, start: Pose, s_m: float) -> Pose: ...
    @property
    def curvature_1pm(self) -> float: ...


@dataclass(frozen=True)
class Straight:
    length_m: float

    def __post_init__(self) -> None:
        if not math.isfinite(self.length_m) or self.length_m <= 0:
            raise ValueError("straight length must be finite and positive")

    @property
    def curvature_1pm(self) -> float:
        return 0.0

    def pose_at(self, start: Pose, s_m: float) -> Pose:
        _validate_distance(s_m, self.length_m)
        return Pose(start.x_m + s_m * math.cos(start.heading_rad),
                    start.y_m + s_m * math.sin(start.heading_rad),
                    start.heading_rad)

    def end_pose(self, start: Pose) -> Pose:
        return self.pose_at(start, self.length_m)


@dataclass(frozen=True)
class CircularArc:
    radius_m: float
    turn_angle_rad: float

    def __post_init__(self) -> None:
        if not math.isfinite(self.radius_m) or self.radius_m <= 0:
            raise ValueError("arc radius must be finite and positive")
        if not math.isfinite(self.turn_angle_rad) or self.turn_angle_rad == 0:
            raise ValueError("arc turn angle must be finite and non-zero")

    @property
    def length_m(self) -> float:
        return self.radius_m * abs(self.turn_angle_rad)

    @property
    def curvature_1pm(self) -> float:
        return math.copysign(1.0 / self.radius_m, self.turn_angle_rad)

    def pose_at(self, start: Pose, s_m: float) -> Pose:
        _validate_distance(s_m, self.length_m)
        delta = self.curvature_1pm * s_m
        heading = start.heading_rad + delta
        x = start.x_m + (math.sin(heading) - math.sin(start.heading_rad)) / self.curvature_1pm
        y = start.y_m - (math.cos(heading) - math.cos(start.heading_rad)) / self.curvature_1pm
        return Pose(x, y, heading)

    def end_pose(self, start: Pose) -> Pose:
        return self.pose_at(start, self.length_m)


def _validate_distance(s_m: float, length_m: float) -> None:
    tolerance = 1e-12 * max(1.0, length_m)
    if not math.isfinite(s_m) or s_m < -tolerance or s_m > length_m + tolerance:
        raise ValueError(f"distance must be within [0, {length_m}]")
