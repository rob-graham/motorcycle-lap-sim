"""Export one frozen-line Mallala R6 simulation lap for engineering sanity checks.

The CSV pairs every racing-line sample with the corresponding nominal track
centreline station and physical left/right boundaries.  A constant finite roll
rate may be selected explicitly; omitting it exports the frozen unconstrained
Phase 9 baseline.  The export is diagnostic only and does not calibrate a
motorcycle or rider parameter.
"""

import argparse
import csv
from dataclasses import replace
import importlib.util
import math
from pathlib import Path

import numpy as np

from motorcycle_lap_sim.motorcycle.config import HandlingConfig, load_motorcycle_config
from motorcycle_lap_sim.motorcycle.roll import demanded_lean_rad
from motorcycle_lap_sim.speed_solver import (
    braking_capability,
    forward_acceleration_capability,
    solve_speed_profile,
)
from motorcycle_lap_sim.track import sample_track_stations


LONGITUDINAL_BINDING_ATOL_MPS2 = 1e-5
LONGITUDINAL_BINDING_RTOL = 1e-7
SPEED_BINDING_ATOL_MPS = 1e-6
SPEED_BINDING_RTOL = 1e-8

CSV_FIELDS = (
    "sample_index",
    "elapsed_time_s",
    "track_s_m",
    "path_q_m",
    "track_center_x_m",
    "track_center_y_m",
    "left_boundary_x_m",
    "left_boundary_y_m",
    "right_boundary_x_m",
    "right_boundary_y_m",
    "bike_x_m",
    "bike_y_m",
    "bike_lateral_offset_m",
    "bike_tangent_offset_m",
    "left_boundary_clearance_m",
    "right_boundary_clearance_m",
    "path_curvature_1pm",
    "speed_mps",
    "speed_kph",
    "longitudinal_acceleration_mps2",
    "lateral_acceleration_signed_mps2",
    "lateral_acceleration_abs_mps2",
    "roll_angle_rad",
    "roll_angle_deg",
    "roll_rate_model_radps",
    "roll_rate_model_degps",
    "roll_rate_ceiling_mps",
    "roll_rate_limited",
    "lateral_grip_limited",
    "powertrain_speed_limited",
    "wheelie_limited",
    "stoppie_limited",
    "traction_limited",
    "drive_traction_limited",
    "braking_traction_limited",
    "engine_power_limited",
    "longitudinal_limit_reason",
    "forward_capability_mps2",
    "forward_capability_reason",
    "braking_capability_mps2",
    "braking_capability_reason",
    "gear",
    "rpm",
)


def _load_phase9_module():
    path = Path(__file__).resolve().with_name("r6_phase9_baseline_check.py")
    spec = importlib.util.spec_from_file_location(
        "r6_phase9_baseline_for_trajectory_export", path)
    if spec is None or spec.loader is None:
        raise ImportError(f"cannot load Phase 9 baseline module from {path}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


phase9 = _load_phase9_module()


def _positive_float(text):
    value = float(text)
    if not math.isfinite(value) or value <= 0:
        raise argparse.ArgumentTypeError("value must be finite and positive")
    return value


def build_parser():
    parser = argparse.ArgumentParser()
    parser.add_argument("output_csv", type=Path)
    parser.add_argument(
        "--max-roll-rate-radps", type=_positive_float,
        help=("optional constant Level-1 maximum roll rate; omit for the "
              "unconstrained frozen baseline"),
    )
    parser.add_argument(
        "--spacing-m", type=float, choices=phase9.OUTPUT_SPACINGS_M, default=1.0,
        help="frozen Phase 9 output spacing to export",
    )
    return parser


def _isclose(actual, capability):
    return bool(np.isclose(
        actual, capability,
        rtol=LONGITUDINAL_BINDING_RTOL,
        atol=LONGITUDINAL_BINDING_ATOL_MPS2,
    ))


def longitudinal_limit_diagnostics(path, result, bike):
    """Classify the actual constant-acceleration segment limit at each sample.

    ``longitudinal_acceleration_mps2[i]`` describes segment ``i -> i+1``.
    Forward capability is therefore evaluated at sample ``i`` exactly as in the
    forward solver pass.  Braking capability is evaluated at sample ``i+1``
    exactly as in the backward pass.  A physical reason is flagged only when
    the final segment acceleration is actually at that capability; otherwise
    the segment is labelled ``profile/other`` because another local or
    propagated speed constraint is controlling it.
    """
    count = len(path.q_m)
    reason = np.full(count, "profile/other", dtype=object)
    forward_value = np.empty(count)
    forward_reason = np.empty(count, dtype=object)
    braking_value = np.empty(count)
    braking_reason = np.empty(count, dtype=object)
    wheelie = np.zeros(count, dtype=bool)
    stoppie = np.zeros(count, dtype=bool)
    drive_traction = np.zeros(count, dtype=bool)
    braking_traction = np.zeros(count, dtype=bool)
    engine_power = np.zeros(count, dtype=bool)

    for index in range(count):
        following = (index + 1) % count
        forward = forward_acceleration_capability(
            float(result.speed_mps[index]), float(path.curvature_1pm[index]), bike)
        braking = braking_capability(
            float(result.speed_mps[following]),
            float(path.curvature_1pm[following]), bike)
        forward_value[index] = forward.acceleration_mps2
        forward_reason[index] = forward.limiting_reason
        braking_value[index] = braking.deceleration_mps2
        braking_reason[index] = braking.limiting_reason

        actual = float(result.longitudinal_acceleration_mps2[index])
        if actual > LONGITUDINAL_BINDING_ATOL_MPS2 and _isclose(
                actual, forward.acceleration_mps2):
            reason[index] = forward.limiting_reason
            wheelie[index] = forward.limiting_reason == "wheelie"
            drive_traction[index] = forward.limiting_reason == "tyre traction"
            engine_power[index] = forward.limiting_reason == "engine/power"
        elif actual < -LONGITUDINAL_BINDING_ATOL_MPS2 and _isclose(
                -actual, braking.deceleration_mps2):
            reason[index] = braking.limiting_reason
            stoppie[index] = braking.limiting_reason == "stoppie"
            braking_traction[index] = braking.limiting_reason == "tyre traction"
        elif abs(actual) <= LONGITUDINAL_BINDING_ATOL_MPS2:
            reason[index] = "steady/profile"

    traction = drive_traction | braking_traction
    return {
        "reason": reason,
        "forward_value": forward_value,
        "forward_reason": forward_reason,
        "braking_value": braking_value,
        "braking_reason": braking_reason,
        "wheelie": wheelie,
        "stoppie": stoppie,
        "traction": traction,
        "drive_traction": drive_traction,
        "braking_traction": braking_traction,
        "engine_power": engine_power,
    }


def trajectory_columns(track, smooth, result, bike):
    """Return one equal-length array for every CSV field."""
    path = smooth.sampled_path
    count = len(path.q_m)
    if len(result.speed_mps) != count:
        raise ValueError("speed profile and racing-line path sample counts differ")

    # PeriodicPlanarSpline.sampled_path uses this exact centreline-s parameter
    # grid.  Reconstructing it here preserves the nominal track coordinate used
    # by the racing-line geometry without a nearest-point projection.
    track_s = np.arange(count, dtype=float) * track.total_length_m / count
    track_sample = sample_track_stations(track, track_s)
    spline_x, spline_y, *_ = smooth.spline.evaluate(track_s)
    if (not np.allclose(spline_x, path.x_m, rtol=0.0, atol=1e-10)
            or not np.allclose(spline_y, path.y_m, rtol=0.0, atol=1e-10)):
        raise RuntimeError("reconstructed track-s parameter grid does not match sampled racing line")

    left_x = track_sample.x_m + track_sample.width_left_m * track_sample.normal_x
    left_y = track_sample.y_m + track_sample.width_left_m * track_sample.normal_y
    right_x = track_sample.x_m - track_sample.width_right_m * track_sample.normal_x
    right_y = track_sample.y_m - track_sample.width_right_m * track_sample.normal_y

    delta_x = path.x_m - track_sample.x_m
    delta_y = path.y_m - track_sample.y_m
    lateral_offset = delta_x * track_sample.normal_x + delta_y * track_sample.normal_y
    tangent_offset = delta_x * track_sample.tangent_x + delta_y * track_sample.tangent_y
    left_clearance = track_sample.width_left_m - lateral_offset
    right_clearance = track_sample.width_right_m + lateral_offset

    following_speed = np.roll(result.speed_mps, -1)
    segment_time = 2.0 * path.segment_lengths_m / (result.speed_mps + following_speed)
    elapsed = np.r_[0.0, np.cumsum(segment_time[:-1])]
    if not math.isclose(float(np.sum(segment_time)), result.lap_time_s,
                        rel_tol=0.0, abs_tol=1e-9):
        raise RuntimeError("trajectory segment times do not reproduce solver lap time")

    lean = demanded_lean_rad(
        result.speed_mps, path.curvature_1pm,
        gravity_mps2=bike.environment.gravity_mps2)
    signed_lateral = result.speed_mps ** 2 * path.curvature_1pm

    finite_roll = np.isfinite(result.speed_limit_roll_rate_mps)
    roll_binding = finite_roll & np.isclose(
        result.speed_mps, result.speed_limit_roll_rate_mps,
        rtol=SPEED_BINDING_RTOL, atol=SPEED_BINDING_ATOL_MPS)
    lateral_binding = np.isfinite(result.speed_limit_lateral_mps) & np.isclose(
        result.speed_mps, result.speed_limit_lateral_mps,
        rtol=SPEED_BINDING_RTOL, atol=SPEED_BINDING_ATOL_MPS)
    powertrain_binding = np.isfinite(result.speed_limit_powertrain_mps) & np.isclose(
        result.speed_mps, result.speed_limit_powertrain_mps,
        rtol=SPEED_BINDING_RTOL, atol=SPEED_BINDING_ATOL_MPS)

    longitudinal = longitudinal_limit_diagnostics(path, result, bike)
    return {
        "sample_index": np.arange(count, dtype=int),
        "elapsed_time_s": elapsed,
        "track_s_m": track_s,
        "path_q_m": path.q_m,
        "track_center_x_m": track_sample.x_m,
        "track_center_y_m": track_sample.y_m,
        "left_boundary_x_m": left_x,
        "left_boundary_y_m": left_y,
        "right_boundary_x_m": right_x,
        "right_boundary_y_m": right_y,
        "bike_x_m": path.x_m,
        "bike_y_m": path.y_m,
        "bike_lateral_offset_m": lateral_offset,
        "bike_tangent_offset_m": tangent_offset,
        "left_boundary_clearance_m": left_clearance,
        "right_boundary_clearance_m": right_clearance,
        "path_curvature_1pm": path.curvature_1pm,
        "speed_mps": result.speed_mps,
        "speed_kph": result.speed_mps * 3.6,
        "longitudinal_acceleration_mps2": result.longitudinal_acceleration_mps2,
        "lateral_acceleration_signed_mps2": signed_lateral,
        "lateral_acceleration_abs_mps2": result.lateral_acceleration_mps2,
        "roll_angle_rad": lean,
        "roll_angle_deg": np.degrees(lean),
        # This is the Level-1 curvature-transition term that the new constraint
        # actually limits; longitudinal-acceleration contribution is omitted by
        # design and is therefore not silently labelled as full IMU roll rate.
        "roll_rate_model_radps": result.demanded_roll_rate_radps,
        "roll_rate_model_degps": np.degrees(result.demanded_roll_rate_radps),
        "roll_rate_ceiling_mps": result.speed_limit_roll_rate_mps,
        "roll_rate_limited": roll_binding.astype(int),
        "lateral_grip_limited": lateral_binding.astype(int),
        "powertrain_speed_limited": powertrain_binding.astype(int),
        "wheelie_limited": longitudinal["wheelie"].astype(int),
        "stoppie_limited": longitudinal["stoppie"].astype(int),
        "traction_limited": longitudinal["traction"].astype(int),
        "drive_traction_limited": longitudinal["drive_traction"].astype(int),
        "braking_traction_limited": longitudinal["braking_traction"].astype(int),
        "engine_power_limited": longitudinal["engine_power"].astype(int),
        "longitudinal_limit_reason": longitudinal["reason"],
        "forward_capability_mps2": longitudinal["forward_value"],
        "forward_capability_reason": longitudinal["forward_reason"],
        "braking_capability_mps2": longitudinal["braking_value"],
        "braking_capability_reason": longitudinal["braking_reason"],
        "gear": result.gear_number,
        "rpm": result.engine_rpm,
    }


def write_trajectory_csv(path, columns):
    missing = [field for field in CSV_FIELDS if field not in columns]
    if missing:
        raise ValueError(f"trajectory export is missing fields: {missing}")
    lengths = {len(np.asarray(columns[field])) for field in CSV_FIELDS}
    if len(lengths) != 1:
        raise ValueError("trajectory export columns must have identical lengths")

    output = Path(path)
    output.parent.mkdir(parents=True, exist_ok=True)
    with output.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow(CSV_FIELDS)
        for values in zip(*(columns[field] for field in CSV_FIELDS)):
            writer.writerow(values)


def main(argv=None):
    args = build_parser().parse_args(argv)
    track, _, _, evaluations = phase9.evaluate_baseline(speed_backend="python")
    phase9.verify_default_regression(
        phase9.sha256_file(phase9.DEFAULT_CONTROLS),
        phase9.sha256_file(phase9.DEFAULT_TRACK),
        phase9.sha256_file(phase9.DEFAULT_MOTORCYCLE),
        evaluations,
    )

    matching = [
        evaluation for spacing, evaluation in zip(phase9.OUTPUT_SPACINGS_M, evaluations)
        if spacing == args.spacing_m
    ]
    if len(matching) != 1:
        raise RuntimeError(f"no unique frozen evaluation for spacing {args.spacing_m}")
    evaluation = matching[0]
    if evaluation.smooth_line is None or evaluation.speed_profile is None:
        raise RuntimeError("frozen Phase 9 evaluation did not return required trajectory artifacts")

    bike = load_motorcycle_config(phase9.DEFAULT_MOTORCYCLE)
    if bike.handling is not None:
        raise RuntimeError("canonical R6 baseline unexpectedly has handling enabled")

    if args.max_roll_rate_radps is None:
        result = evaluation.speed_profile
        case = "unconstrained"
    else:
        bike = replace(
            bike, handling=HandlingConfig(max_roll_rate_radps=args.max_roll_rate_radps))
        result = solve_speed_profile(evaluation.smooth_line.sampled_path, bike)
        case = f"roll_{args.max_roll_rate_radps:.6g}_radps"

    columns = trajectory_columns(track, evaluation.smooth_line, result, bike)
    write_trajectory_csv(args.output_csv, columns)

    print(f"case={case}")
    print(f"spacing_m={args.spacing_m:.6f}")
    print(f"lap_s={result.lap_time_s:.9f}")
    print(f"samples={len(result.speed_mps)}")
    print(f"output_csv={args.output_csv}")
    print("roll_rate_note=roll_rate_model_* is the Level-1 curvature-transition term; it is not a full IMU roll-rate prediction")
    print("limit_flag_note=wheelie/stoppie/traction flags are true only when the final segment acceleration is actually at that solver capability")


if __name__ == "__main__":
    main()
