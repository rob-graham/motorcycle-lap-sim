"""Decompose steady-lean roll-rate demand into curvature and speed-change terms.

The active Level-1 finite-roll constraint uses only the curvature-transition
term.  This diagnostic adds the analytic contribution caused by longitudinal
speed change and compares their sum with a finite-difference derivative of the
solved steady-lean demand.  It is post-processing only and changes no solver or
model parameter.
"""

import argparse
import importlib.util
import math
from dataclasses import replace
from pathlib import Path

import numpy as np

from motorcycle_lap_sim.motorcycle.config import HandlingConfig, load_motorcycle_config
from motorcycle_lap_sim.motorcycle.roll import demanded_roll_rate_radps
from motorcycle_lap_sim.optimisation import (
    REFERENCE_PLANAR_CONTROL_POLICY,
    generate_planar_control_stations,
    planar_control_bounds,
)
from motorcycle_lap_sim.track import Track


def _load_sibling(filename, module_name):
    path = Path(__file__).resolve().with_name(filename)
    spec = importlib.util.spec_from_file_location(module_name, path)
    if spec is None or spec.loader is None:
        raise ImportError(f"cannot load sibling script from {path}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


spatial = _load_sibling(
    "r6_phase9f_spatial_comparison.py", "r6_phase9f_for_roll_decomposition")
sector = _load_sibling(
    "r6_phase9g_sector_diagnostics.py", "r6_phase9g_for_roll_decomposition")
phase8 = spatial.phase8
phase9 = spatial.phase9


def _positive_float(text):
    value = float(text)
    if not math.isfinite(value) or value <= 0.0:
        raise argparse.ArgumentTypeError("value must be finite and positive")
    return value


def build_parser():
    parser = argparse.ArgumentParser()
    parser.add_argument("roll_aware_controls_csv", type=Path)
    parser.add_argument("--max-roll-rate-radps", type=_positive_float, default=0.8)
    parser.add_argument(
        "--sim-spacing-m", type=float, choices=spatial.SIM_SPACINGS_M, default=0.25)
    parser.add_argument(
        "--sector-length-m", type=_positive_float, default=sector.DEFAULT_SECTOR_LENGTH_M)
    return parser


def periodic_centered_derivative(distance_m, values):
    distance = np.asarray(distance_m, dtype=float)
    data = np.asarray(values, dtype=float)
    if distance.ndim != 1 or len(distance) < 3 or data.shape != distance.shape:
        raise ValueError("distance and values must be equal 1D arrays with at least three samples")
    if (not np.all(np.isfinite(distance)) or not np.all(np.isfinite(data))
            or np.any(np.diff(distance) <= 0.0)):
        raise ValueError("distance must increase and derivative inputs must be finite")
    spacing = np.diff(distance)
    nominal = float(np.median(spacing))
    if not math.isfinite(nominal) or nominal <= 0.0:
        raise ValueError("path spacing must be finite and positive")
    lap_length = float(distance[-1] + nominal - distance[0])
    previous_s = np.roll(distance, 1)
    next_s = np.roll(distance, -1)
    previous_s[0] -= lap_length
    next_s[-1] += lap_length
    return (np.roll(data, -1) - np.roll(data, 1)) / (next_s - previous_s)


def speed_change_roll_rate_radps(
        distance_m, speed_mps, curvature_1pm, *, gravity_mps2=9.80665):
    distance = np.asarray(distance_m, dtype=float)
    speed = np.asarray(speed_mps, dtype=float)
    curvature = np.asarray(curvature_1pm, dtype=float)
    if speed.shape != distance.shape or curvature.shape != distance.shape:
        raise ValueError("distance, speed, and curvature must have identical shapes")
    if not math.isfinite(gravity_mps2) or gravity_mps2 <= 0.0:
        raise ValueError("gravity must be finite and positive")
    if np.any(speed < 0.0) or not np.all(np.isfinite(speed)) or not np.all(np.isfinite(curvature)):
        raise ValueError("speed must be finite/non-negative and curvature finite")
    dv_ds = periodic_centered_derivative(distance, speed)
    longitudinal_acceleration = speed * dv_ds
    lean_ratio = speed * speed * curvature / gravity_mps2
    return (
        2.0 * speed * longitudinal_acceleration * curvature / gravity_mps2
        / (1.0 + lean_ratio * lean_ratio)
    )


def centered_curvature_roll_rate_radps(
        distance_m, speed_mps, curvature_1pm, *, gravity_mps2=9.80665):
    distance = np.asarray(distance_m, dtype=float)
    speed = np.asarray(speed_mps, dtype=float)
    curvature = np.asarray(curvature_1pm, dtype=float)
    if speed.shape != distance.shape or curvature.shape != distance.shape:
        raise ValueError("distance, speed, and curvature must have identical shapes")
    if not math.isfinite(gravity_mps2) or gravity_mps2 <= 0.0:
        raise ValueError("gravity must be finite and positive")
    if np.any(speed < 0.0) or not np.all(np.isfinite(speed)) or not np.all(np.isfinite(curvature)):
        raise ValueError("speed must be finite/non-negative and curvature finite")
    dkappa_ds = periodic_centered_derivative(distance, curvature)
    lean_ratio = speed * speed * curvature / gravity_mps2
    return (
        speed ** 3 * dkappa_ds / gravity_mps2
        / (1.0 + lean_ratio * lean_ratio)
    )


def _case_arrays(evaluation, gravity_mps2):
    if evaluation.smooth_line is None or evaluation.speed_profile is None:
        raise RuntimeError("evaluation did not return required trajectory artifacts")
    path = evaluation.smooth_line.sampled_path
    result = evaluation.speed_profile
    full = demanded_roll_rate_radps(
        path.q_m, result.speed_mps, result.demanded_lean_rad, closed=True)
    level1 = np.asarray(result.demanded_roll_rate_radps, dtype=float)
    speed_term = speed_change_roll_rate_radps(
        path.q_m, result.speed_mps, path.curvature_1pm,
        gravity_mps2=gravity_mps2,
    )
    centered_curvature = centered_curvature_roll_rate_radps(
        path.q_m, result.speed_mps, path.curvature_1pm,
        gravity_mps2=gravity_mps2,
    )
    return {
        "q_m": np.asarray(path.q_m, dtype=float),
        "speed_mps": np.asarray(result.speed_mps, dtype=float),
        "curvature_1pm": np.asarray(path.curvature_1pm, dtype=float),
        "full": full,
        "level1": level1,
        "speed_term": speed_term,
        "combined_active": level1 + speed_term,
        "centered_curvature": centered_curvature,
        "combined_centered": centered_curvature + speed_term,
        "roll_binding": (
            np.isfinite(result.speed_limit_roll_rate_mps)
            & np.isclose(
                result.speed_mps, result.speed_limit_roll_rate_mps,
                rtol=spatial.SPEED_BINDING_RTOL,
                atol=spatial.SPEED_BINDING_ATOL_MPS,
            )
        ),
    }


def _rms(values):
    data = np.asarray(values, dtype=float)
    return float(np.sqrt(np.mean(data * data)))


def _sector_rows(track, arrays, limit, sector_length_m):
    starts, ends = sector._sector_edges(track.total_length_m, sector_length_m)
    count = len(arrays["q_m"])
    track_s = np.arange(count, dtype=float) * track.total_length_m / count
    indices = sector._sector_indices(
        track_s, track.total_length_m, sector_length_m, len(starts))
    full_exceed = np.abs(arrays["full"]) > limit + 1e-9
    binding = arrays["roll_binding"]
    missed = full_exceed & ~binding
    combined_exceed = np.abs(arrays["combined_active"]) > limit + 1e-9
    centered_exceed = np.abs(arrays["combined_centered"]) > limit + 1e-9

    rows = []
    for index, (start, end) in enumerate(zip(starts, ends)):
        mask = indices == index
        missed_count = int(np.count_nonzero(mask & missed))
        rows.append({
            "sector_index": index,
            "start_m": float(start),
            "end_m": float(end),
            "samples": int(np.count_nonzero(mask)),
            "missed": missed_count,
            "missed_explained_active": int(np.count_nonzero(mask & missed & combined_exceed)),
            "missed_explained_centered": int(np.count_nonzero(mask & missed & centered_exceed)),
            "mean_abs_speed_term": float(np.mean(np.abs(arrays["speed_term"][mask]))),
            "peak_abs_speed_term": float(np.max(np.abs(arrays["speed_term"][mask]))),
            "rms_active_residual": _rms(
                arrays["full"][mask] - arrays["combined_active"][mask]),
            "rms_centered_residual": _rms(
                arrays["full"][mask] - arrays["combined_centered"][mask]),
        })
    return rows


def _print_case(label, arrays, rows, limit):
    full_exceed = np.abs(arrays["full"]) > limit + 1e-9
    binding = arrays["roll_binding"]
    missed = full_exceed & ~binding
    combined_active_exceed = np.abs(arrays["combined_active"]) > limit + 1e-9
    combined_centered_exceed = np.abs(arrays["combined_centered"]) > limit + 1e-9

    print(f"{label}_full_exceed_samples={np.count_nonzero(full_exceed)}/{len(full_exceed)}")
    print(f"{label}_level1_binding_samples={np.count_nonzero(binding)}/{len(binding)}")
    print(f"{label}_full_exceed_without_level1_binding_samples={np.count_nonzero(missed)}/{len(missed)}")
    print(
        f"{label}_missed_exceedances_explained_by_level1_plus_speed_term="
        f"{np.count_nonzero(missed & combined_active_exceed)}/{np.count_nonzero(missed)}")
    print(
        f"{label}_missed_exceedances_explained_by_centered_curvature_plus_speed_term="
        f"{np.count_nonzero(missed & combined_centered_exceed)}/{np.count_nonzero(missed)}")
    print(f"{label}_maximum_abs_speed_change_term_radps={np.max(np.abs(arrays['speed_term'])):.9f}")
    print(f"{label}_rms_speed_change_term_radps={_rms(arrays['speed_term']):.9f}")
    print(
        f"{label}_rms_full_minus_level1_plus_speed_term_radps="
        f"{_rms(arrays['full'] - arrays['combined_active']):.9f}")
    print(
        f"{label}_rms_full_minus_centered_components_radps="
        f"{_rms(arrays['full'] - arrays['combined_centered']):.9f}")

    ranked = sorted(
        rows,
        key=lambda row: (row["missed"], row["mean_abs_speed_term"]),
        reverse=True,
    )
    print(f"{label}_largest_omitted_speed_change_sectors:")
    for row in ranked[:6]:
        explained = row["missed_explained_active"]
        denominator = row["missed"]
        print(
            f"  sector={row['sector_index']:02d} "
            f"chainage_m={row['start_m']:.1f}:{row['end_m']:.1f} "
            f"missed_full_exceed={denominator}/{row['samples']} "
            f"explained_by_level1_plus_speed={explained}/{denominator} "
            f"explained_by_centered_components={row['missed_explained_centered']}/{denominator} "
            f"mean_abs_speed_term_radps={row['mean_abs_speed_term']:.6f} "
            f"peak_abs_speed_term_radps={row['peak_abs_speed_term']:.6f} "
            f"rms_active_residual_radps={row['rms_active_residual']:.6f} "
            f"rms_centered_residual_radps={row['rms_centered_residual']:.6f}")


def main(argv=None):
    args = build_parser().parse_args(argv)
    spatial._require_canonical_inputs()
    track = Track.from_yaml(phase9.DEFAULT_TRACK)
    base_bike = load_motorcycle_config(phase9.DEFAULT_MOTORCYCLE)
    if base_bike.handling is not None:
        raise RuntimeError("canonical R6 baseline unexpectedly has handling enabled")
    roll_bike = replace(
        base_bike, handling=HandlingConfig(max_roll_rate_radps=args.max_roll_rate_radps))

    stations = generate_planar_control_stations(track, REFERENCE_PLANAR_CONTROL_POLICY)
    lower, upper = planar_control_bounds(track, stations, phase9.BOUNDARY_MARGIN_M)
    frozen_controls = phase9.load_frozen_controls(
        phase9.DEFAULT_CONTROLS, stations, lower, upper)
    roll_aware_controls = phase8.load_initial_controls_csv(
        args.roll_aware_controls_csv, stations, lower, upper)

    evaluations = {
        "frozen_roll": spatial._evaluate(
            track, roll_bike, stations, frozen_controls, args.sim_spacing_m),
        "roll_aware": spatial._evaluate(
            track, roll_bike, stations, roll_aware_controls, args.sim_spacing_m),
    }

    print(f"roll_aware_controls_csv={args.roll_aware_controls_csv}")
    print(f"roll_aware_controls_sha256={spatial._sha256_file(args.roll_aware_controls_csv)}")
    print(f"max_roll_rate_radps={args.max_roll_rate_radps:.9f}")
    print(f"sim_spacing_m={args.sim_spacing_m:.2f}")
    print(f"sector_length_m={args.sector_length_m:.3f}")
    print("diagnostic_note=speed-change and centered-curvature terms are analytic post-processing diagnostics; the active solver remains unchanged")

    for label, evaluation in evaluations.items():
        arrays = _case_arrays(
            evaluation, base_bike.environment.gravity_mps2)
        rows = _sector_rows(
            track, arrays, args.max_roll_rate_radps, args.sector_length_m)
        _print_case(label, arrays, rows, args.max_roll_rate_radps)

    print("interpretation_note=if Level-1 plus the speed-change term explains most full-rate exceedances missed by Level-1 and the centered-component residual is small, the discrepancy is physical rather than primarily a differentiation artifact")
    print("calibration_note=no model parameter is changed or fitted by this command")


if __name__ == "__main__":
    main()
