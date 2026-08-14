"""Deterministic engine torque-curve evaluation."""

from __future__ import annotations

from .config import PowertrainConfig


def available_engine_torque_nm(rpm: float, powertrain: PowertrainConfig) -> float:
    """Return linearly interpolated torque, or zero outside idle/rev limit."""
    if rpm < powertrain.idle_rpm or rpm > powertrain.rev_limit_rpm:
        return 0.0
    points = powertrain.torque_curve
    for left, right in zip(points, points[1:]):
        if rpm <= right.rpm:
            fraction = (rpm - left.rpm) / (right.rpm - left.rpm)
            return left.torque_nm + fraction * (right.torque_nm - left.torque_nm)
    raise RuntimeError("validated torque curve unexpectedly does not cover RPM")
