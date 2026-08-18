"""Versioned simulator-to-run-off data contract and departure candidates.

The objects in this module transfer simulator-derived facts downstream.  They
intentionally stop before any run-off deceleration, surface, barrier, impact or
risk calculation.  Candidate departure seeds are traceable interpretations of
reviewed simulation events and are not safety criteria or claims about the
worst credible departure condition.
"""

from dataclasses import dataclass
import math
from types import MappingProxyType
from typing import Mapping, Sequence

import numpy as np


RUNOFF_INTERFACE_VERSION = "0.1.0"

_REQUIRED_TRAJECTORY_FIELDS = (
    "sample_index",
    "track_s_m",
    "path_q_m",
    "bike_x_m",
    "bike_y_m",
    "left_boundary_x_m",
    "left_boundary_y_m",
    "right_boundary_x_m",
    "right_boundary_y_m",
    "speed_mps",
    "longitudinal_acceleration_mps2",
    "lateral_acceleration_signed_mps2",
    "path_curvature_1pm",
    "roll_angle_rad",
    "roll_rate_model_radps",
)

_OPTIONAL_TRAJECTORY_FIELDS = (
    "gear",
    "rpm",
    "roll_rate_limited",
    "lateral_grip_limited",
    "powertrain_speed_limited",
    "wheelie_limited",
    "stoppie_limited",
    "traction_limited",
    "engine_power_limited",
    "longitudinal_limit_reason",
)

_EVENT_TO_SEED = {
    "local_max_speed": (
        "missed_braking_candidate",
        "candidate high-speed approach state before the model-derived braking onset",
    ),
    "braking_onset": (
        "upright_overrun_candidate",
        "candidate departure state at the model-derived start of sustained braking",
    ),
    "turn_in": (
        "entry_lowside_turn_in_candidate",
        "candidate fallen-rider departure state at the model-derived turn-in landmark",
    ),
    "geometric_apex": (
        "entry_lowside_apex_candidate",
        "candidate fallen-rider departure state at the geometric-apex landmark",
    ),
    "positive_drive_pickup": (
        "exit_highside_candidate",
        "candidate exit departure state at the model-derived positive-drive pickup landmark",
    ),
}


@dataclass(frozen=True)
class DepartureSeed:
    """One traceable simulator state proposed for later run-off analysis."""

    seed_id: str
    corner: str
    seed_type: str
    sample_index: int
    track_s_m: float
    path_q_m: float
    x_m: float
    y_m: float
    heading_rad: float
    speed_mps: float
    longitudinal_acceleration_mps2: float
    lateral_acceleration_mps2: float
    curvature_1pm: float
    lean_angle_rad: float
    roll_rate_radps: float
    source_event_type: str
    source_rule: str
    confidence: str
    interpretation: str


@dataclass(frozen=True)
class RunoffInputPackage:
    """Immutable simulator-side hand-off for a separate run-off package.

    ``track_s_m`` is the reference-track parameter used to construct the
    racing line. ``path_q_m`` is distance along the solved racing line.  The
    current coordinate frame is local Cartesian metres; a global CRS/world
    transform is deliberately not fabricated when georeferencing is absent.
    """

    interface_version: str
    coordinate_frame: str
    chainage_definition: str
    trajectory: Mapping[str, np.ndarray]
    departure_seeds: tuple[DepartureSeed, ...]
    scenario_metadata: Mapping[str, str]
    warnings: tuple[str, ...]


def _finite_1d(values, name, *, allow_object=False):
    array = np.asarray(values)
    if array.ndim != 1:
        raise ValueError(f"{name} must be one-dimensional")
    if not allow_object:
        array = np.asarray(array, dtype=float)
        if not np.all(np.isfinite(array)):
            raise ValueError(f"{name} must contain only finite values")
    return array


def _validated_trajectory(columns):
    missing = [name for name in _REQUIRED_TRAJECTORY_FIELDS if name not in columns]
    if missing:
        raise ValueError(f"run-off interface trajectory is missing fields: {missing}")

    lengths = {len(np.asarray(columns[name])) for name in _REQUIRED_TRAJECTORY_FIELDS}
    if len(lengths) != 1:
        raise ValueError("run-off interface trajectory fields must have identical lengths")
    count = lengths.pop()
    if count < 3:
        raise ValueError("run-off interface requires at least three trajectory samples")

    result = {}
    for name in _REQUIRED_TRAJECTORY_FIELDS:
        values = _finite_1d(columns[name], name)
        if len(values) != count:
            raise ValueError("run-off interface trajectory fields must have identical lengths")
        copied = np.asarray(values).copy()
        copied.setflags(write=False)
        result[name] = copied

    sample_index = np.asarray(result["sample_index"], dtype=float)
    if not np.all(sample_index == np.arange(count, dtype=float)):
        raise ValueError("sample_index must be contiguous from zero")
    for name in ("track_s_m", "path_q_m"):
        values = np.asarray(result[name], dtype=float)
        if values[0] != 0.0 or np.any(np.diff(values) <= 0.0):
            raise ValueError(f"{name} must start at zero and increase strictly")
    if np.any(np.asarray(result["speed_mps"], dtype=float) < 0.0):
        raise ValueError("speed_mps must be non-negative")

    for name in _OPTIONAL_TRAJECTORY_FIELDS:
        if name not in columns:
            continue
        raw = np.asarray(columns[name])
        if raw.ndim != 1 or len(raw) != count:
            raise ValueError(f"optional trajectory field {name} must match trajectory length")
        if raw.dtype.kind in "biufc":
            values = np.asarray(raw, dtype=float)
            if not np.all(np.isfinite(values)):
                raise ValueError(f"optional trajectory field {name} must be finite")
            copied = values.copy()
        else:
            copied = raw.astype(object, copy=True)
        copied.setflags(write=False)
        result[name] = copied

    return result


def derive_closed_path_heading_rad(x_m, y_m):
    """Return wrapped path heading from periodic central differences.

    A central chord is used so the calculation is deterministic and has no
    dependency on optimiser control points or a track-centreline tangent.  A
    repeated local chord is rejected rather than silently choosing an arbitrary
    direction.
    """
    x = _finite_1d(x_m, "x_m")
    y = _finite_1d(y_m, "y_m")
    if x.shape != y.shape or len(x) < 3:
        raise ValueError("closed-path heading requires equal x/y arrays with at least three samples")
    dx = np.roll(x, -1) - np.roll(x, 1)
    dy = np.roll(y, -1) - np.roll(y, 1)
    chord = np.hypot(dx, dy)
    if np.any(chord <= 1e-12):
        raise ValueError("closed-path heading is undefined at a repeated/degenerate local chord")
    heading = np.arctan2(dy, dx)
    heading.setflags(write=False)
    return heading


def _event_value(event, name):
    if isinstance(event, Mapping):
        if name not in event:
            raise ValueError(f"run-off departure event is missing field {name}")
        return event[name]
    if not hasattr(event, name):
        raise ValueError(f"run-off departure event is missing field {name}")
    return getattr(event, name)


def _validate_event_matches_trajectory(event, trajectory, *, atol=1e-8):
    index = int(_event_value(event, "sample_index"))
    count = len(trajectory["sample_index"])
    if index < 0 or index >= count:
        raise ValueError("run-off departure event sample_index is outside the trajectory")

    comparisons = (
        ("track_s_m", "track_s_m"),
        ("path_q_m", "path_q_m"),
        ("x_m", "bike_x_m"),
        ("y_m", "bike_y_m"),
        ("speed_mps", "speed_mps"),
        ("longitudinal_acceleration_mps2", "longitudinal_acceleration_mps2"),
        ("curvature_1pm", "path_curvature_1pm"),
    )
    for event_name, trajectory_name in comparisons:
        actual = float(_event_value(event, event_name))
        expected = float(trajectory[trajectory_name][index])
        if not (math.isfinite(actual) and math.isclose(actual, expected, rel_tol=0.0, abs_tol=atol)):
            raise ValueError(
                f"event field {event_name} does not match trajectory sample {index}: "
                f"event={actual!r} trajectory={expected!r}")
    return index


def build_departure_seeds(columns, events):
    """Build traceable departure candidates from reviewed simulation events.

    Only explicitly mapped event semantics become candidates.  Capability-limit
    flags, optimiser spread and other diagnostics are intentionally excluded.
    """
    trajectory = _validated_trajectory(columns)
    heading = derive_closed_path_heading_rad(trajectory["bike_x_m"], trajectory["bike_y_m"])
    seeds = []
    counters = {}
    for event in events:
        event_type = str(_event_value(event, "event_type"))
        if event_type not in _EVENT_TO_SEED:
            continue
        index = _validate_event_matches_trajectory(event, trajectory)
        corner = str(_event_value(event, "corner"))
        seed_type, interpretation = _EVENT_TO_SEED[event_type]
        key = (corner, seed_type)
        counters[key] = counters.get(key, 0) + 1
        seed_id = f"{corner}_{seed_type}_{counters[key]}"
        lean_deg = float(_event_value(event, "lean_angle_deg"))
        roll_rate = float(_event_value(event, "roll_rate_radps"))
        if not math.isfinite(roll_rate):
            roll_rate = float(trajectory["roll_rate_model_radps"][index])
        seeds.append(DepartureSeed(
            seed_id=seed_id,
            corner=corner,
            seed_type=seed_type,
            sample_index=index,
            track_s_m=float(trajectory["track_s_m"][index]),
            path_q_m=float(trajectory["path_q_m"][index]),
            x_m=float(trajectory["bike_x_m"][index]),
            y_m=float(trajectory["bike_y_m"][index]),
            heading_rad=float(heading[index]),
            speed_mps=float(trajectory["speed_mps"][index]),
            longitudinal_acceleration_mps2=float(
                trajectory["longitudinal_acceleration_mps2"][index]),
            lateral_acceleration_mps2=float(
                trajectory["lateral_acceleration_signed_mps2"][index]),
            curvature_1pm=float(trajectory["path_curvature_1pm"][index]),
            lean_angle_rad=math.radians(lean_deg),
            roll_rate_radps=roll_rate,
            source_event_type=event_type,
            source_rule=str(_event_value(event, "source_rule")),
            confidence=str(_event_value(event, "confidence")),
            interpretation=interpretation,
        ))
    return tuple(seeds)


def build_runoff_input_package(
        columns, events, *, scenario_metadata, warnings=()):
    """Create the current local-coordinate simulator-to-run-off hand-off."""
    trajectory = _validated_trajectory(columns)
    heading = derive_closed_path_heading_rad(trajectory["bike_x_m"], trajectory["bike_y_m"])
    heading_copy = np.asarray(heading).copy()
    heading_copy.setflags(write=False)
    trajectory = dict(trajectory)
    trajectory["path_heading_rad"] = heading_copy

    metadata = {str(key): str(value) for key, value in dict(scenario_metadata).items()}
    required_metadata = ("scenario_id", "simulator_commit", "track_id")
    missing = [name for name in required_metadata if not metadata.get(name)]
    if missing:
        raise ValueError(f"run-off interface scenario metadata is missing: {missing}")

    seeds = build_departure_seeds(trajectory, events)
    warning_values = tuple(str(value) for value in warnings)
    return RunoffInputPackage(
        interface_version=RUNOFF_INTERFACE_VERSION,
        coordinate_frame="local_cartesian_m",
        chainage_definition=(
            "track_s_m=reference-track parameter used to construct the racing line; "
            "path_q_m=arc length along the solved racing line"),
        trajectory=MappingProxyType(trajectory),
        departure_seeds=seeds,
        scenario_metadata=MappingProxyType(metadata),
        warnings=warning_values,
    )
