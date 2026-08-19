"""Versioned simulator-to-run-off data contract and departure candidates.

The objects in this module transfer simulator-derived facts downstream. They
intentionally stop before any run-off deceleration, surface, barrier, impact or
risk calculation. Candidate departure seeds are traceable interpretations of
supported simulation events and are not safety criteria or claims about the
worst credible departure condition.
"""

from dataclasses import dataclass
import math
from types import MappingProxyType
from typing import Mapping

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
    "corner_exit": (
        "entry_lowside_corner_exit_candidate",
        "candidate fallen-rider departure state at the model-derived corner-exit landmark",
    ),
    "positive_drive_pickup": (
        "exit_highside_candidate",
        "candidate exit departure state at the model-derived positive-drive pickup landmark",
    ),
}

_ALLOWED_CONFIDENCE = {"high", "medium", "low"}
_REQUIRED_METADATA_FIELDS = ("scenario_id", "simulator_commit", "track_id", "event_set_id")


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
    """Simulator-side hand-off for a separate run-off package.

    ``track_s_m`` is the reference-track parameter used to construct the
    racing line. ``path_q_m`` is distance along the solved racing line. The
    current coordinate frame is local Cartesian metres; a global CRS/world
    transform is deliberately not fabricated when georeferencing is absent.

    Numeric/string trajectory arrays are defensive copies backed by immutable
    bytes, so consumers cannot re-enable NumPy writes on the exported views.
    """

    interface_version: str
    coordinate_frame: str
    chainage_definition: str
    track_length_m: float
    path_length_m: float
    sampling_convention: str
    trajectory: Mapping[str, np.ndarray]
    departure_seeds: tuple[DepartureSeed, ...]
    scenario_metadata: Mapping[str, str]
    warnings: tuple[str, ...]


def _finite_1d(values, name):
    array = np.asarray(values, dtype=float)
    if array.ndim != 1:
        raise ValueError(f"{name} must be one-dimensional")
    if not np.all(np.isfinite(array)):
        raise ValueError(f"{name} must contain only finite values")
    return array


def _immutable_array(values):
    """Return an immutable-byte-backed NumPy view of one contiguous array."""
    array = np.ascontiguousarray(values)
    raw = array.tobytes()
    result = np.frombuffer(raw, dtype=array.dtype).reshape(array.shape)
    if result.flags.writeable:
        raise RuntimeError("immutable trajectory array unexpectedly remains writeable")
    return result


def _validated_trajectory(columns):
    missing = [name for name in _REQUIRED_TRAJECTORY_FIELDS if name not in columns]
    if missing:
        raise ValueError(f"run-off interface trajectory is missing fields: {missing}")

    raw_required = {name: np.asarray(columns[name]) for name in _REQUIRED_TRAJECTORY_FIELDS}
    if any(values.ndim != 1 for values in raw_required.values()):
        raise ValueError("run-off interface trajectory fields must be one-dimensional")
    lengths = {len(values) for values in raw_required.values()}
    if len(lengths) != 1:
        raise ValueError("run-off interface trajectory fields must have identical lengths")
    count = lengths.pop()
    if count < 3:
        raise ValueError("run-off interface requires at least three trajectory samples")

    sample_values = _finite_1d(columns["sample_index"], "sample_index")
    if not np.all(sample_values == np.arange(count, dtype=float)):
        raise ValueError("sample_index must be contiguous integer values from zero")

    result = {"sample_index": _immutable_array(np.arange(count, dtype=np.int64))}
    for name in _REQUIRED_TRAJECTORY_FIELDS:
        if name == "sample_index":
            continue
        values = _finite_1d(columns[name], name)
        result[name] = _immutable_array(values)

    for name in ("track_s_m", "path_q_m"):
        values = result[name]
        if values[0] != 0.0 or np.any(np.diff(values) <= 0.0):
            raise ValueError(f"{name} must start at zero and increase strictly")
    if np.any(result["speed_mps"] < 0.0):
        raise ValueError("speed_mps must be non-negative")

    for name in _OPTIONAL_TRAJECTORY_FIELDS:
        if name not in columns:
            continue
        raw = np.asarray(columns[name])
        if raw.ndim != 1 or len(raw) != count:
            raise ValueError(f"optional trajectory field {name} must match trajectory length")
        if raw.dtype.kind in "biufc":
            finite_check = np.asarray(raw, dtype=complex if raw.dtype.kind == "c" else float)
            if not np.all(np.isfinite(finite_check)):
                raise ValueError(f"optional trajectory field {name} must be finite")
            result[name] = _immutable_array(raw)
        else:
            strings = np.asarray([str(value) for value in raw], dtype=str)
            result[name] = _immutable_array(strings)

    return result


def _validated_total_length(total_length_m, q_m, name):
    try:
        total = float(total_length_m)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{name} must be a finite scalar") from exc
    if not math.isfinite(total) or total <= float(q_m[-1]):
        raise ValueError(f"{name} must be finite and exceed the final omitted-endpoint chainage")
    return total


def derive_closed_path_heading_rad(x_m, y_m, path_q_m, total_length_m):
    """Return path heading using a periodic unequal-spacing derivative.

    The derivative is with respect to ``path_q_m`` and uses the standard
    three-point unequal-spacing formula at each sample. The start/finish
    derivative uses the supplied closed-path total length to form the wrapped
    previous/next spacing. This avoids the spacing bias of an unweighted
    ``i-1`` to ``i+1`` chord.
    """
    x = _finite_1d(x_m, "x_m")
    y = _finite_1d(y_m, "y_m")
    q = _finite_1d(path_q_m, "path_q_m")
    if x.shape != y.shape or x.shape != q.shape or len(x) < 3:
        raise ValueError(
            "closed-path heading requires equal x/y/q arrays with at least three samples")
    if q[0] != 0.0 or np.any(np.diff(q) <= 0.0):
        raise ValueError("path_q_m must start at zero and increase strictly")
    total = _validated_total_length(total_length_m, q, "path_length_m")

    h_prev = q - np.roll(q, 1)
    h_prev[0] = total - q[-1]
    h_next = np.roll(q, -1) - q
    h_next[-1] = total - q[-1]
    if np.any(h_prev <= 0.0) or np.any(h_next <= 0.0):
        raise ValueError("closed-path heading requires positive wrapped path spacing")

    previous = np.roll(np.column_stack((x, y)), 1, axis=0)
    current = np.column_stack((x, y))
    following = np.roll(current, -1, axis=0)
    a = -h_next / (h_prev * (h_prev + h_next))
    b = (h_next - h_prev) / (h_prev * h_next)
    c = h_prev / (h_next * (h_prev + h_next))
    derivative = a[:, None] * previous + b[:, None] * current + c[:, None] * following
    magnitude = np.hypot(derivative[:, 0], derivative[:, 1])
    if np.any(magnitude <= 1e-12):
        raise ValueError("closed-path heading is undefined at a repeated/degenerate local path sample")
    heading = np.arctan2(derivative[:, 1], derivative[:, 0])
    return _immutable_array(heading)


def _event_value(event, name):
    if isinstance(event, Mapping):
        if name not in event:
            raise ValueError(f"run-off departure event is missing field {name}")
        return event[name]
    if not hasattr(event, name):
        raise ValueError(f"run-off departure event is missing field {name}")
    return getattr(event, name)


def _event_real_scalar(event, name, *, allow_nan=False):
    value = _event_value(event, name)
    if isinstance(value, (bool, np.bool_, str, bytes)):
        raise ValueError(f"run-off departure event field {name} must be a numeric scalar")
    array = np.asarray(value)
    if array.ndim != 0:
        raise ValueError(f"run-off departure event field {name} must be a scalar")
    try:
        result = float(array)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"run-off departure event field {name} must be numeric") from exc
    if math.isnan(result) and allow_nan:
        return result
    if not math.isfinite(result):
        raise ValueError(f"run-off departure event field {name} must be finite")
    return result


def _event_index(event, count):
    value = _event_real_scalar(event, "sample_index")
    if value != math.trunc(value):
        raise ValueError("run-off departure event sample_index must be integer-valued")
    index = int(value)
    if index < 0 or index >= count:
        raise ValueError("run-off departure event sample_index is outside the trajectory")
    return index


def _event_text(event, name):
    value = _event_value(event, name)
    if not isinstance(value, str) or not value.strip():
        raise ValueError(f"run-off departure event field {name} must be a non-empty string")
    return value.strip()


def _compare_event_field(event, event_name, expected, *, atol=1e-8):
    actual = _event_real_scalar(event, event_name)
    if not math.isclose(actual, float(expected), rel_tol=0.0, abs_tol=atol):
        raise ValueError(
            f"event field {event_name} does not match trajectory sample: "
            f"event={actual!r} trajectory={float(expected)!r}")


def _validate_event_matches_trajectory(event, trajectory, *, atol=1e-8):
    index = _event_index(event, len(trajectory["sample_index"]))
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
        _compare_event_field(event, event_name, trajectory[trajectory_name][index], atol=atol)

    event_lean_deg = _event_real_scalar(event, "lean_angle_deg")
    expected_lean_deg = math.degrees(float(trajectory["roll_angle_rad"][index]))
    if not math.isclose(event_lean_deg, expected_lean_deg, rel_tol=0.0, abs_tol=atol):
        raise ValueError(
            "event field lean_angle_deg does not match trajectory sample: "
            f"event={event_lean_deg!r} trajectory={expected_lean_deg!r}")

    event_roll_rate = _event_real_scalar(event, "roll_rate_radps", allow_nan=True)
    if math.isfinite(event_roll_rate):
        expected_roll_rate = float(trajectory["roll_rate_model_radps"][index])
        if not math.isclose(event_roll_rate, expected_roll_rate, rel_tol=0.0, abs_tol=atol):
            raise ValueError(
                "event field roll_rate_radps does not match trajectory sample: "
                f"event={event_roll_rate!r} trajectory={expected_roll_rate!r}")
    return index


def _validated_metadata(scenario_metadata):
    """Validate provenance metadata without coercing identities to strings."""
    if not isinstance(scenario_metadata, Mapping):
        raise ValueError("run-off interface scenario metadata must be a mapping")

    raw = dict(scenario_metadata)
    invalid_keys = [key for key in raw if not isinstance(key, str) or not key.strip()]
    if invalid_keys:
        raise ValueError("run-off interface scenario metadata keys must be non-empty strings")

    invalid_required = [
        name for name in _REQUIRED_METADATA_FIELDS
        if name not in raw or not isinstance(raw[name], str) or not raw[name].strip()
    ]
    if invalid_required:
        raise ValueError(
            "run-off interface required scenario metadata must be non-empty strings: "
            f"{invalid_required}")

    invalid_values = [
        key for key, value in raw.items()
        if not isinstance(value, str) or not value.strip()
    ]
    if invalid_values:
        raise ValueError(
            "run-off interface scenario metadata values must be non-empty strings: "
            f"{invalid_values}")
    return raw


def _build_departure_seeds(trajectory, events, heading):
    """Build seeds from one validated trajectory and its derived heading."""
    seeds = []
    counters = {}
    for event in events:
        event_type = _event_text(event, "event_type")
        if event_type not in _EVENT_TO_SEED:
            continue
        index = _validate_event_matches_trajectory(event, trajectory)
        corner = _event_text(event, "corner")
        source_rule = _event_text(event, "source_rule")
        confidence = _event_text(event, "confidence").lower()
        if confidence not in _ALLOWED_CONFIDENCE:
            raise ValueError(
                f"run-off departure event confidence must be one of {sorted(_ALLOWED_CONFIDENCE)}")

        seed_type, interpretation = _EVENT_TO_SEED[event_type]
        key = (corner, seed_type)
        counters[key] = counters.get(key, 0) + 1
        seed_id = f"{corner}_{seed_type}_{counters[key]}"
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
            lean_angle_rad=float(trajectory["roll_angle_rad"][index]),
            roll_rate_radps=float(trajectory["roll_rate_model_radps"][index]),
            source_event_type=event_type,
            source_rule=source_rule,
            confidence=confidence,
            interpretation=interpretation,
        ))
    return tuple(seeds)


def build_departure_seeds(columns, events, *, path_length_m):
    """Build traceable departure candidates from supported simulation events.

    Physical seed state is always taken from the validated trajectory. Event
    copies are used only for index/provenance and stale-pair validation. A NaN
    event roll-rate copy is permitted to mean that the event did not retain a
    roll-rate value; the required trajectory roll-rate remains authoritative.
    """
    trajectory = _validated_trajectory(columns)
    path_length = _validated_total_length(
        path_length_m, trajectory["path_q_m"], "path_length_m")
    heading = derive_closed_path_heading_rad(
        trajectory["bike_x_m"], trajectory["bike_y_m"],
        trajectory["path_q_m"], path_length)
    return _build_departure_seeds(trajectory, events, heading)


def build_runoff_input_package(
        columns, events, *, track_length_m, path_length_m, scenario_metadata, warnings=()):
    """Create the current local-coordinate simulator-to-run-off hand-off."""
    trajectory = _validated_trajectory(columns)
    track_length = _validated_total_length(
        track_length_m, trajectory["track_s_m"], "track_length_m")
    path_length = _validated_total_length(
        path_length_m, trajectory["path_q_m"], "path_length_m")
    heading = derive_closed_path_heading_rad(
        trajectory["bike_x_m"], trajectory["bike_y_m"], trajectory["path_q_m"], path_length)
    trajectory = dict(trajectory)
    trajectory["heading_rad"] = heading
    # Retain the original descriptive field as an additive compatibility alias.
    trajectory["path_heading_rad"] = heading

    metadata = _validated_metadata(scenario_metadata)

    seeds = _build_departure_seeds(trajectory, events, heading)
    warning_values = tuple(str(value) for value in warnings)
    return RunoffInputPackage(
        interface_version=RUNOFF_INTERFACE_VERSION,
        coordinate_frame="local_cartesian_m",
        chainage_definition=(
            "track_s_m=reference-track parameter used to construct the racing line; "
            "path_q_m=arc length along the solved racing line"),
        track_length_m=track_length,
        path_length_m=path_length,
        sampling_convention="closed loop; duplicated endpoint omitted",
        trajectory=MappingProxyType(trajectory),
        departure_seeds=seeds,
        scenario_metadata=MappingProxyType(metadata),
        warnings=warning_values,
    )
