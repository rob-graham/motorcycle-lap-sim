"""Measured telemetry import and two-dimensional validation helpers."""

from .aim_excel import load_aim_workbook
from .aim_quality import load_aim_gps_quality
from .model import TelemetryLap, TelemetrySession, lap_slices
from .peer import PeerTrajectoryDeviation, peer_trajectory_deviation
from .quality import GPSQualitySeries, gps_quality_mask, require_time_alignment
from .registration import RigidRegistrationResult, fit_rigid_registration
from .repeatability import (
    ChainageProgressDiagnostics,
    CrossLapEnvelope,
    chainage_progress_diagnostics,
    cross_lap_envelope,
    unwrap_closed_chainage,
)
from .speed_comparison import (
    SpeedComparisonSummary,
    SpeedEnvelopeComparison,
    compare_speed_envelope,
    periodic_interpolate,
    summarize_speed_comparison,
    uniform_closed_parameter_grid,
)

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
    "PeerTrajectoryDeviation",
    "peer_trajectory_deviation",
    "ChainageProgressDiagnostics",
    "CrossLapEnvelope",
    "unwrap_closed_chainage",
    "chainage_progress_diagnostics",
    "cross_lap_envelope",
    "SpeedEnvelopeComparison",
    "SpeedComparisonSummary",
    "uniform_closed_parameter_grid",
    "periodic_interpolate",
    "compare_speed_envelope",
    "summarize_speed_comparison",
]
