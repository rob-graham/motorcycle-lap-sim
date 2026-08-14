"""Composition, loading, and closure diagnostics for tracks."""

from dataclasses import dataclass
import math
from pathlib import Path
from typing import Any, Sequence

import numpy as np
import yaml

from .primitives import CentrelinePrimitive, CircularArc, Pose, Straight


def wrap_angle_rad(angle_rad: float) -> float:
    """Return the signed equivalent angle in [-pi, pi)."""
    return (angle_rad + math.pi) % (2.0 * math.pi) - math.pi


@dataclass(frozen=True)
class ClosureDiagnostic:
    x_error_m: float
    y_error_m: float
    position_error_m: float
    heading_error_rad: float

    def passes(self, position_tolerance_m: float, heading_tolerance_rad: float) -> bool:
        if position_tolerance_m < 0 or heading_tolerance_rad < 0:
            raise ValueError("closure tolerances must be non-negative")
        return (self.position_error_m <= position_tolerance_m
                and abs(self.heading_error_rad) <= heading_tolerance_rad)


@dataclass(frozen=True)
class Track:
    primitives: tuple[CentrelinePrimitive, ...]
    start_pose: Pose = Pose(0.0, 0.0, 0.0)
    width_left_m: float = 4.0
    width_right_m: float = 4.0
    closed: bool = False

    def __post_init__(self) -> None:
        object.__setattr__(self, "primitives", tuple(self.primitives))
        if not self.primitives:
            raise ValueError("track requires at least one primitive")
        if not math.isfinite(self.width_left_m) or self.width_left_m <= 0:
            raise ValueError("left width must be finite and positive")
        if not math.isfinite(self.width_right_m) or self.width_right_m <= 0:
            raise ValueError("right width must be finite and positive")

    @property
    def total_length_m(self) -> float:
        return math.fsum(p.length_m for p in self.primitives)

    @property
    def primitive_start_s_m(self) -> np.ndarray:
        return np.concatenate(([0.0], np.cumsum([p.length_m for p in self.primitives])))

    @property
    def end_pose(self) -> Pose:
        pose = self.start_pose
        for primitive in self.primitives:
            pose = primitive.end_pose(pose)
        return pose

    def closure_diagnostic(self) -> ClosureDiagnostic:
        end = self.end_pose
        dx = end.x_m - self.start_pose.x_m
        dy = end.y_m - self.start_pose.y_m
        return ClosureDiagnostic(dx, dy, math.hypot(dx, dy),
                                 wrap_angle_rad(end.heading_rad - self.start_pose.heading_rad))

    @classmethod
    def from_yaml(cls, path: str | Path) -> "Track":
        with Path(path).open(encoding="utf-8") as stream:
            data: dict[str, Any] = yaml.safe_load(stream)
        start = data.get("start", {})
        primitives: list[CentrelinePrimitive] = []
        for item in data["primitives"]:
            kind = item["type"]
            if kind == "straight":
                primitives.append(Straight(float(item["length_m"])))
            elif kind == "circular_arc":
                primitives.append(CircularArc(float(item["radius_m"]), float(item["turn_angle_rad"])))
            else:
                raise ValueError(f"unknown track primitive type: {kind!r}")
        return cls(tuple(primitives), Pose(float(start.get("x_m", 0.0)),
                                           float(start.get("y_m", 0.0)),
                                           float(start.get("heading_rad", 0.0))),
                   float(data["width_left_m"]), float(data["width_right_m"]),
                   bool(data.get("closed", False)))
