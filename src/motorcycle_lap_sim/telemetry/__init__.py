"""Measured telemetry import and two-dimensional validation helpers."""

from .aim_excel import load_aim_workbook
from .aim_quality import load_aim_gps_quality
from .model import TelemetryLap, TelemetrySession, lap_slices
from .quality import GPSQualitySeries, gps_quality_mask, require_time_alignment
from .registration import RigidRegistrationResult, fit_rigid_registration

__all__ = [
    "TelemetryLap",
    "TelemetrySession",
    "lap_slices",
    "load_aim_workbook",
    "GPSQualitySeries",
    "load_aim_gps_quality",
    "gps_quality_mask",
    "require_time_alignment",
    "RigidRegistrationResult",
    "fit_rigid_registration",
]
