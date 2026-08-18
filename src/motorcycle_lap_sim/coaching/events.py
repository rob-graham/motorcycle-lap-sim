"""Deterministic rider-facing event extraction from one solved closed lap.

Phase 12A intentionally derives coaching landmarks from simulation state.  It
is not a run-off departure model and does not define the later simulator to
run-off interchange contract.
"""

from dataclasses import dataclass
import math

import numpy as np


@dataclass(frozen=True)
class EventDetectionConfig:
    """Replaceable thresholds for the first coaching-event implementation."""

    corner_lean_on_deg: float = 6.0
    corner_lean_off_deg: float = 4.0
    minimum_corner_length_m: float = 18.0
    merge_same_direction_gap_m: float = 35.0
    brake_strong_mps2: float = -1.5
    brake_onset_mps2: float = -0.35
    brake_release_mps2: float = -0.20
    positive_drive_mps2: float = 0.35
    positive_drive_hold_m: float = 4.0
    brake_release_hold_m: float = 4.0
    meaningful_braking_hold_m: float = 4.0
    turn_build_fraction: float = 0.15
    exit_clearance_recovery_fraction: float = 0.50
    exit_unwind_fraction: float = 0.50
    exit_hold_m: float = 4.0
    landmark_min_clearance_change_m: float = 0.25

    def __post_init__(self) -> None:
        values = (
            self.corner_lean_on_deg,
            self.corner_lean_off_deg,
            self.minimum_corner_length_m,
            self.merge_same_direction_gap_m,
            self.positive_drive_hold_m,
            self.brake_release_hold_m,
            self.meaningful_braking_hold_m,
            self.turn_build_fraction,
            self.exit_clearance_recovery_fraction,
            self.exit_unwind_fraction,
            self.exit_hold_m,
            self.landmark_min_clearance_change_m,
        )
        if not all(math.isfinite(value) and value >= 0.0 for value in values):
            raise ValueError("event geometry thresholds must be finite and non-negative")
        if self.corner_lean_off_deg > self.corner_lean_on_deg:
            raise ValueError("corner lean-off threshold must not exceed lean-on threshold")
        if not all(math.isfinite(value) for value in (
                self.brake_strong_mps2, self.brake_onset_mps2,
                self.brake_release_mps2, self.positive_drive_mps2)):
            raise ValueError("event acceleration thresholds must be finite")
        if self.brake_strong_mps2 >= self.brake_onset_mps2:
            raise ValueError("strong-braking threshold must be more negative than onset threshold")
        if self.brake_onset_mps2 >= self.brake_release_mps2:
            raise ValueError("brake-release threshold must be less negative than onset threshold")
        if self.positive_drive_mps2 <= 0.0:
            raise ValueError("positive-drive threshold must be positive")
        for name in ("turn_build_fraction", "exit_clearance_recovery_fraction",
                     "exit_unwind_fraction"):
            if getattr(self, name) > 1.0:
                raise ValueError(f"{name} must not exceed one")


@dataclass(frozen=True)
class CoachingEvent:
    corner: str
    event_type: str
    sample_index: int
    track_s_m: float
    path_q_m: float
    x_m: float
    y_m: float
    speed_mps: float
    longitudinal_acceleration_mps2: float
    curvature_1pm: float
    lean_angle_deg: float
    roll_rate_radps: float
    gear: int | None
    rpm: float | None
    source_rule: str
    confidence: str
    display_on_map: bool = True


_REQUIRED_KEYS = (
    "track_s_m",
    "path_q_m",
    "bike_x_m",
    "bike_y_m",
    "speed_mps",
    "longitudinal_acceleration_mps2",
    "path_curvature_1pm",
    "roll_angle_deg",
    "left_boundary_x_m",
    "left_boundary_y_m",
    "right_boundary_x_m",
    "right_boundary_y_m",
)


def _arrays(columns):
    missing = [name for name in _REQUIRED_KEYS if name not in columns]
    if missing:
        raise ValueError(f"coaching event input is missing fields: {missing}")
    result = {name: np.asarray(columns[name]) for name in columns}
    lengths = {len(np.asarray(result[name])) for name in _REQUIRED_KEYS}
    if len(lengths) != 1:
        raise ValueError("coaching event arrays must have identical lengths")
    count = lengths.pop()
    if count < 3:
        raise ValueError("coaching event extraction requires at least three samples")
    for name in _REQUIRED_KEYS:
        values = np.asarray(result[name], dtype=float)
        if values.ndim != 1 or not np.all(np.isfinite(values)):
            raise ValueError(f"coaching event field {name} must be a finite one-dimensional array")
        result[name] = values
    if result["track_s_m"][0] != 0.0 or np.any(np.diff(result["track_s_m"]) <= 0.0):
        raise ValueError("track_s_m must start at zero and increase strictly")
    if result["path_q_m"][0] != 0.0 or np.any(np.diff(result["path_q_m"]) <= 0.0):
        raise ValueError("path_q_m must start at zero and increase strictly")
    return result, count


def _true_regions(mask):
    mask = np.asarray(mask, dtype=bool)
    changes = np.diff(np.r_[False, mask, False].astype(np.int8))
    starts = np.flatnonzero(changes == 1)
    ends = np.flatnonzero(changes == -1) - 1
    return [(int(start), int(end)) for start, end in zip(starts, ends)]


def _region_length(path_q_m, start, end):
    if end <= start:
        return 0.0
    return float(path_q_m[end] - path_q_m[start])


def _expand_regions(strong_regions, weak_mask):
    count = len(weak_mask)
    expanded = []
    for start, end in strong_regions:
        while start > 0 and weak_mask[start - 1]:
            start -= 1
        while end + 1 < count and weak_mask[end + 1]:
            end += 1
        if not expanded or (start, end) != expanded[-1]:
            expanded.append((start, end))
    return expanded


def _region_turn_sign(lean_deg, start, end):
    values = lean_deg[start:end + 1]
    index = int(np.argmax(np.abs(values)))
    value = float(values[index])
    return 1 if value >= 0.0 else -1


def _merge_same_direction_regions(regions, path_q_m, lean_deg, max_gap_m):
    if not regions:
        return []
    merged = [regions[0]]
    for start, end in regions[1:]:
        old_start, old_end = merged[-1]
        gap = float(path_q_m[start] - path_q_m[old_end])
        same_direction = (
            _region_turn_sign(lean_deg, old_start, old_end)
            == _region_turn_sign(lean_deg, start, end)
        )
        if same_direction and gap <= max_gap_m:
            merged[-1] = (old_start, end)
        else:
            merged.append((start, end))
    return merged


def detect_corner_regions(columns, config=EventDetectionConfig()):
    """Return sustained lean regions, merged across short same-direction gaps."""
    arrays, _ = _arrays(columns)
    lean = arrays["roll_angle_deg"]
    path_q = arrays["path_q_m"]
    strong = np.abs(lean) >= config.corner_lean_on_deg
    weak = np.abs(lean) >= config.corner_lean_off_deg
    regions = _expand_regions(_true_regions(strong), weak)
    regions = [
        region for region in regions
        if _region_length(path_q, *region) >= config.minimum_corner_length_m
    ]
    return _merge_same_direction_regions(
        regions, path_q, lean, config.merge_same_direction_gap_m)


def _validated_corner_regions(regions, count):
    validated = []
    previous_end = -1
    for region in regions:
        if len(region) != 2:
            raise ValueError("corner regions must be (start_index, end_index) pairs")
        start, end = int(region[0]), int(region[1])
        if start < 0 or end < start or end >= count:
            raise ValueError("corner region indices are outside the solved trajectory")
        if start <= previous_end:
            raise ValueError("corner regions must be ordered and non-overlapping")
        validated.append((start, end))
        previous_end = end
    return validated


def _event(arrays, index, corner, event_type, source_rule, confidence, *, display=True):
    gear_values = arrays.get("gear")
    rpm_values = arrays.get("rpm")
    roll_rate_values = arrays.get("roll_rate_model_radps")
    gear = None
    if gear_values is not None:
        raw = float(np.asarray(gear_values)[index])
        if math.isfinite(raw) and raw > 0:
            gear = int(round(raw))
    rpm = None
    if rpm_values is not None:
        raw = float(np.asarray(rpm_values)[index])
        if math.isfinite(raw):
            rpm = raw
    roll_rate = math.nan
    if roll_rate_values is not None:
        raw = float(np.asarray(roll_rate_values)[index])
        if math.isfinite(raw):
            roll_rate = raw
    return CoachingEvent(
        corner=corner,
        event_type=event_type,
        sample_index=int(index),
        track_s_m=float(arrays["track_s_m"][index]),
        path_q_m=float(arrays["path_q_m"][index]),
        x_m=float(arrays["bike_x_m"][index]),
        y_m=float(arrays["bike_y_m"][index]),
        speed_mps=float(arrays["speed_mps"][index]),
        longitudinal_acceleration_mps2=float(arrays["longitudinal_acceleration_mps2"][index]),
        curvature_1pm=float(arrays["path_curvature_1pm"][index]),
        lean_angle_deg=float(arrays["roll_angle_deg"][index]),
        roll_rate_radps=roll_rate,
        gear=gear,
        rpm=rpm,
        source_rule=source_rule,
        confidence=confidence,
        display_on_map=display,
    )


def _held(mask, path_q, index, end, hold_m):
    final = int(np.searchsorted(path_q, path_q[index] + hold_m, side="left"))
    return final <= end and bool(np.all(mask[index:final + 1]))


def _braking_indices(acceleration, path_q, search_start, search_end, config):
    if search_end < search_start:
        return None
    window = acceleration[search_start:search_end + 1]
    strong_relative = int(np.argmin(window))
    strong = search_start + strong_relative
    if acceleration[strong] > config.brake_strong_mps2:
        return None
    onset = strong
    while onset > search_start and acceleration[onset - 1] <= config.brake_release_mps2:
        onset -= 1
    while onset < strong and acceleration[onset] > config.brake_onset_mps2:
        onset += 1
    # REL is the final sustained departure from the derived braking regime,
    # not the first threshold recovery after the single strongest sample.
    meaningful = acceleration <= config.brake_onset_mps2
    last_pulse = strong
    for index in range(strong, search_end + 1):
        if meaningful[index]:
            last_pulse = index
    release = None
    recovered = acceleration >= config.brake_release_mps2
    for index in range(last_pulse + 1, search_end + 1):
        if recovered[index] and _held(
                recovered, path_q, index, search_end, config.brake_release_hold_m):
            release = index
            break
    return onset, strong, release


def _first_sustained_positive_drive(acceleration, path_q, start, end, threshold, hold_m,
                                    braking_threshold, braking_hold_m):
    if end < start:
        return None
    # A pickup is a transition, not simply the first positive sample in an
    # arbitrarily bounded search interval.  Include start-1 -> start so that a
    # pickup exactly at VMIN is observable, while an already-positive approach
    # still cannot fabricate an event.
    for index in range(max(1, start), end + 1):
        if acceleration[index - 1] >= threshold or acceleration[index] < threshold:
            continue
        target = path_q[index] + hold_m
        final = int(np.searchsorted(path_q, target, side="left"))
        if final > end:
            continue
        if not np.all(acceleration[index:final + 1] >= threshold):
            continue
        # Reject temporary positive patches followed by another sustained
        # meaningful braking phase before the next approach boundary.
        braking = acceleration <= braking_threshold
        if any(_held(braking, path_q, later, end, braking_hold_m)
               for later in range(final + 1, end + 1) if braking[later]):
            continue
        return index
    return None


def _inside_clearance(arrays, start, end, turn_sign):
    """Euclidean bike clearance to the sampled physical inside boundary."""
    side = "left" if turn_sign > 0 else "right"
    dx = arrays["bike_x_m"][start:end + 1] - arrays[f"{side}_boundary_x_m"][start:end + 1]
    dy = arrays["bike_y_m"][start:end + 1] - arrays[f"{side}_boundary_y_m"][start:end + 1]
    return np.hypot(dx, dy), side


def _rider_landmarks(clearance, lean, curvature, path_q, start, apex, speed_apex,
                     maximum_curvature, end, turn_sign, config):
    """Find dominant corner-direction roll-in and substantial track-out."""
    build_end = max(start, min(maximum_curvature, apex))
    signed_lean = turn_sign * lean
    signed_curve = turn_sign * curvature
    lean_peak = max(0.0, float(np.max(signed_lean[start:build_end + 1])))
    curve_peak = max(0.0, float(np.max(signed_curve[start:build_end + 1])))
    lean_level = config.turn_build_fraction * lean_peak
    curve_level = config.turn_build_fraction * curve_peak
    candidates = [index for index in range(start + 1, build_end + 1)
                  if ((signed_lean[index] >= lean_level and
                       signed_lean[index] > signed_lean[index - 1]) or
                      (signed_curve[index] >= curve_level and
                       signed_curve[index] > signed_curve[index - 1]))]
    if candidates and (lean_peak > 0.0 or curve_peak > 0.0):
        turn_in = candidates[0]
        turn_rule = "onset of dominant corner-direction demanded-lean/curvature build"
        turn_confidence = "high"
    else:
        signal = np.maximum(signed_lean[start:build_end + 1] / max(lean_peak, 1e-12),
                            signed_curve[start:build_end + 1] / max(curve_peak, 1e-12))
        turn_in = start + int(np.argmax(signal))
        turn_rule = "bounded signed-curvature/demanded-lean build fallback"
        turn_confidence = "medium"

    completion = max(apex, speed_apex, maximum_curvature)
    peak_clearance = float(np.max(clearance[completion - start:]))
    clearance_gain = max(0.0, peak_clearance - float(clearance[apex - start]))
    clearance_required = max(config.landmark_min_clearance_change_m,
                             config.exit_clearance_recovery_fraction * clearance_gain)
    peak_lean = max(0.0, float(np.max(signed_lean[start:end + 1])))
    peak_curve = max(0.0, float(np.max(signed_curve[start:end + 1])))
    condition = np.zeros(len(path_q), dtype=bool)
    for index in range(completion, end + 1):
        recovered = clearance[index - start] >= clearance[apex - start] + clearance_required
        unwound = (signed_lean[index] <= config.exit_unwind_fraction * peak_lean and
                   signed_curve[index] <= config.exit_unwind_fraction * peak_curve)
        condition[index] = recovered and unwound
    corner_exit = None
    for index in range(completion, end + 1):
        if condition[index] and _held(condition, path_q, index, end, config.exit_hold_m):
            corner_exit = index
            break
    if corner_exit is not None:
        exit_rule = "substantial post-apex clearance recovery with demanded-lean/curvature unwind"
        exit_confidence = "high"
    else:
        corner_exit = end
        exit_rule = "nominal-corner-end fallback after APEX/VMIN/K-MAX completion"
        exit_confidence = "medium"
    return turn_in, corner_exit, turn_rule, exit_rule, turn_confidence, exit_confidence


def extract_coaching_events(
        columns, config=EventDetectionConfig(), *, corner_regions=None,
        expected_corner_count=None):
    """Extract ordered rider-facing landmarks from a solved representative lap.

    Detection is deliberately rule based and deterministic.  The result is a
    set of coaching landmarks for visual review, not a safety prescription.
    A caller may supply already-reviewed corner regions when a case-specific
    track segmentation rule is more appropriate than the generic lean detector.
    """
    arrays, count = _arrays(columns)
    if corner_regions is None:
        regions = detect_corner_regions(arrays, config)
    else:
        regions = _validated_corner_regions(corner_regions, count)
    if expected_corner_count is not None and len(regions) != expected_corner_count:
        raise ValueError(
            f"detected {len(regions)} corner regions; expected {expected_corner_count}")

    acceleration = arrays["longitudinal_acceleration_mps2"]
    speed = arrays["speed_mps"]
    curvature = arrays["path_curvature_1pm"]
    lean = arrays["roll_angle_deg"]
    path_q = arrays["path_q_m"]
    events = []

    previous_exit = 0
    for number, (start, end) in enumerate(regions, start=1):
        corner = f"T{number}"
        local = slice(start, end + 1)
        turn_sign = _region_turn_sign(lean, start, end)
        inside_clearance, inside_side = _inside_clearance(arrays, start, end, turn_sign)
        geometric_apex = start + int(np.argmin(inside_clearance))
        maximum_curvature = start + int(np.argmax(np.abs(curvature[local])))
        speed_apex = start + int(np.argmin(speed[local]))
        max_lean = start + int(np.argmax(np.abs(lean[local])))
        (turn_in, corner_exit, turn_rule, exit_rule,
         turn_confidence, exit_confidence) = _rider_landmarks(
             inside_clearance, lean, curvature, path_q, start, geometric_apex,
             speed_apex, maximum_curvature, end, turn_sign, config)

        # The accepted corner envelope is required to distinguish an early
        # recovery from a later braking pulse; stopping at VMIN made REL a
        # first-crossing artefact.
        search_end = end
        braking = _braking_indices(
            acceleration, path_q, previous_exit, search_end, config)
        if braking is not None:
            brake_onset, max_braking, brake_release = braking
            max_speed = previous_exit + int(np.argmax(speed[previous_exit:brake_onset + 1]))
            events.append(_event(
                arrays, max_speed, corner, "local_max_speed",
                "maximum speed between previous corner exit and braking onset", "high"))
            events.append(_event(
                arrays, brake_onset, corner, "braking_onset",
                "start of sustained deceleration preceding a strong-braking sample", "high"))
            events.append(_event(
                arrays, max_braking, corner, "maximum_braking",
                "minimum longitudinal acceleration in the approach/corner window", "high",
                display=False))
            if brake_release is not None:
                events.append(_event(
                    arrays, brake_release, corner, "brake_release",
                    "final sustained acceleration-derived departure from the braking regime",
                    "medium"))
        else:
            max_speed = previous_exit + int(np.argmax(speed[previous_exit:start + 1]))
            events.append(_event(
                arrays, max_speed, corner, "local_max_speed",
                "maximum speed between previous corner exit and turn-in; no strong braking detected",
                "medium"))

        events.append(_event(
            arrays, turn_in, corner, "turn_in", turn_rule, turn_confidence))
        events.append(_event(
            arrays, geometric_apex, corner, "geometric_apex",
            f"minimum racing-line clearance to the inside physical {inside_side} track edge "
            "within the nominal corner", "high"))
        if maximum_curvature != geometric_apex:
            events.append(_event(
                arrays, maximum_curvature, corner, "maximum_curvature",
                "maximum absolute racing-line curvature within the nominal corner", "high",
                display=False))
        if speed_apex != geometric_apex:
            events.append(_event(
                arrays, speed_apex, corner, "speed_apex",
                "minimum speed within the corner region", "high", display=False))
        if max_lean not in (geometric_apex, speed_apex):
            events.append(_event(
                arrays, max_lean, corner, "maximum_lean",
                "maximum absolute demanded lean within the corner region", "high", display=False))

        next_start = regions[number][0] if number < len(regions) else count - 1
        drive_end = next_start
        # The next sustained braking approach is a boundary for the current
        # corner's drive regime, not evidence that its pickup was false.
        braking_mask = acceleration <= config.brake_onset_mps2
        for candidate in range(end + 1, next_start + 1):
            if braking_mask[candidate] and _held(
                    braking_mask, path_q, candidate, next_start,
                    config.meaningful_braking_hold_m):
                drive_end = candidate - 1
                break
        pickup = _first_sustained_positive_drive(
            acceleration, path_q, speed_apex, drive_end,
            config.positive_drive_mps2, config.positive_drive_hold_m,
            config.brake_onset_mps2, config.meaningful_braking_hold_m)
        if pickup is not None:
            events.append(_event(
                arrays, pickup, corner, "positive_drive_pickup",
                "below-to-above transition into final sustained positive-drive regime "
                "after VMIN/final braking; derived from longitudinal acceleration", "medium"))
        events.append(_event(
            arrays, corner_exit, corner, "corner_exit", exit_rule, exit_confidence))
        previous_exit = corner_exit

    # Direction-change landmarks are useful rider cues but remain secondary on
    # the map.  Only create one where consecutive corners lean in opposite
    # directions and a real interval exists between them.
    for number in range(len(regions) - 1):
        start_a, end_a = regions[number]
        start_b, end_b = regions[number + 1]
        sign_a = _region_turn_sign(lean, start_a, end_a)
        sign_b = _region_turn_sign(lean, start_b, end_b)
        if sign_a == sign_b or start_b <= end_a:
            continue
        transition = end_a + int(np.argmin(np.abs(lean[end_a:start_b + 1])))
        events.append(_event(
            arrays, transition, f"T{number + 1}-T{number + 2}", "roll_transition",
            "minimum absolute demanded lean between opposite-direction corner regions",
            "medium", display=False))

    gear_values = arrays.get("gear")
    if gear_values is not None:
        gear = np.asarray(gear_values, dtype=float)
        for index in np.flatnonzero(np.diff(gear) != 0.0) + 1:
            if not (math.isfinite(float(gear[index - 1])) and math.isfinite(float(gear[index]))):
                continue
            nearest = "lap"
            if regions:
                distances = [
                    min(abs(index - start), abs(index - end)) for start, end in regions]
                nearest = f"T{int(np.argmin(distances)) + 1}"
            events.append(_event(
                arrays, int(index), nearest, "gear_shift",
                "solver gear number changes between adjacent samples", "high", display=False))

    priority = {
        "local_max_speed": 0,
        "braking_onset": 1,
        "maximum_braking": 2,
        "brake_release": 3,
        "turn_in": 4,
        "geometric_apex": 5,
        "maximum_curvature": 6,
        "speed_apex": 7,
        "maximum_lean": 8,
        "positive_drive_pickup": 9,
        "corner_exit": 10,
        "roll_transition": 11,
        "gear_shift": 12,
    }
    return sorted(events, key=lambda event: (
        event.track_s_m, priority.get(event.event_type, 99), event.corner))
