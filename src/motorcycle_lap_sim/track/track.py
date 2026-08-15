"""Composition, loading, and closure diagnostics for tracks."""

from dataclasses import dataclass
import math
from pathlib import Path
from typing import Any

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
    primitive_width_left_m: tuple[float | None, ...] | None = None
    primitive_width_right_m: tuple[float | None, ...] | None = None

    def __post_init__(self) -> None:
        object.__setattr__(self, "primitives", tuple(self.primitives))
        if not self.primitives:
            raise ValueError("track requires at least one primitive")
        left_default = _positive_width(self.width_left_m, "left width")
        right_default = _positive_width(self.width_right_m, "right width")
        object.__setattr__(self, "width_left_m", left_default)
        object.__setattr__(self, "width_right_m", right_default)
        object.__setattr__(self, "primitive_width_left_m", self._resolve_widths(
            self.primitive_width_left_m, left_default, "left"))
        object.__setattr__(self, "primitive_width_right_m", self._resolve_widths(
            self.primitive_width_right_m, right_default, "right"))

    def _resolve_widths(self, values: tuple[float | None, ...] | None,
                        default: float, side: str) -> tuple[float, ...]:
        supplied = (None,) * len(self.primitives) if values is None else tuple(values)
        if len(supplied) != len(self.primitives):
            raise ValueError(f"primitive {side} widths must match primitive count")
        return tuple(default if value is None else _positive_width(
            value, f"primitive {side} width") for value in supplied)

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
        widths_left: list[float | None] = []
        widths_right: list[float | None] = []
        for item in data["primitives"]:
            kind = item["type"]
            if kind == "straight":
                primitives.append(Straight(float(item["length_m"])))
            elif kind == "circular_arc":
                primitives.append(CircularArc(float(item["radius_m"]), float(item["turn_angle_rad"])))
            else:
                raise ValueError(f"unknown track primitive type: {kind!r}")
            widths_left.append(item.get("width_left_m"))
            widths_right.append(item.get("width_right_m"))
        return cls(tuple(primitives), Pose(float(start.get("x_m", 0.0)),
                                           float(start.get("y_m", 0.0)),
                                           float(start.get("heading_rad", 0.0))),
                   float(data["width_left_m"]), float(data["width_right_m"]),
                   bool(data.get("closed", False)), tuple(widths_left), tuple(widths_right))


def _positive_width(value: Any, label: str) -> float:
    """Validate a track half-width without accepting booleans as numbers."""
    if isinstance(value, bool):
        raise ValueError(f"{label} must be numeric, finite, and positive")
    try:
        result = float(value)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{label} must be numeric, finite, and positive") from exc
    if not math.isfinite(result) or result <= 0:
        raise ValueError(f"{label} must be numeric, finite, and positive")
    return result
