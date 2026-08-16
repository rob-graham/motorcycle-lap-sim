"""Compare Level-1 and full steady-lean roll-rate demand for Mallala R6 cases.

The active finite-roll constraint uses only the curvature-transition term.  This
post-processing diagnostic differentiates the solved steady-lean demand along
the actual speed profile, so braking/acceleration effects enter through the
speed variation without changing the solver or calibrating any parameter.
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
    "r6_phase9f_spatial_comparison.py", "r6_phase9f_for_roll_components")
sector = _load_sibling(
    "r6_phase9g_sector_diagnostics.py", "r6_phase9g_for_roll_components")
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


def _case_diagnostics(track, evaluation, max_roll_rate_radps, sector_length_m):
    if evaluation.smooth_line is None or evaluation.speed_profile is None:
        raise RuntimeError("evaluation did not return required trajectory artifacts")
    path = evaluation.smooth_line.sampled_path
    result = evaluation.speed_profile
    full_rate = demanded_roll_rate_radps(
        path.q_m, result.speed_mps, result.demanded_lean_rad, closed=True)
    level1 = np.asarray(result.demanded_roll_rate_radps, dtype=float)
    finite_ceiling = np.isfinite(result.speed_limit_roll_rate_mps)
    level1_binding = finite_ceiling & np.isclose(
        result.speed_mps, result.speed_limit_roll_rate_mps,
        rtol=spatial.SPEED_BINDING_RTOL, atol=spatial.SPEED_BINDING_ATOL_MPS)
    tolerance = 1e-9
    full_exceed = np.abs(full_rate) > max_roll_rate_radps + tolerance
    full_below_at_binding = level1_binding & (np.abs(full_rate) < max_roll_rate_radps - tolerance)
    full_exceed_without_binding = full_exceed & ~level1_binding

    count = len(result.speed_mps)
    track_s = np.arange(count, dtype=float) * track.total_length_m / count
    starts, ends = sector._sector_edges(track.total_length_m, sector_length_m)
    indices = sector._sector_indices(
        track_s, track.total_length_m, sector_length_m, len(starts))

    rows = []
    for index, (start, end) in enumerate(zip(starts, ends)):
        mask = indices == index
        binding_count = int(np.count_nonzero(mask & level1_binding))
        full_exceed_count = int(np.count_nonzero(mask & full_exceed))
        rows.append({
            "sector_index": index,
            "start_m": float(start),
            "end_m": float(end),
            "samples": int(np.count_nonzero(mask)),
            "level1_binding": binding_count,
            "full_exceed": full_exceed_count,
            "full_exceed_without_binding": int(np.count_nonzero(mask & full_exceed_without_binding)),
            "full_below_at_binding": int(np.count_nonzero(mask & full_below_at_binding)),
            "level1_peak": float(np.max(np.abs(level1[mask]))),
            "full_peak": float(np.max(np.abs(full_rate[mask]))),
            "mean_abs_full_minus_level1": float(
                np.mean(np.abs(full_rate[mask]) - np.abs(level1[mask]))),
            "mean_abs_full_at_binding": (
                float(np.mean(np.abs(full_rate[mask & level1_binding])))
                if binding_count else math.nan),
        })
    return rows, level1_binding, full_exceed, full_below_at_binding, full_exceed_without_binding, full_rate


def _print_case(label, rows, binding, full_exceed, full_below_at_binding,
                full_exceed_without_binding, full_rate, limit):
    print(f"{label}_level1_binding_samples={np.count_nonzero(binding)}/{len(binding)}")
    print(f"{label}_full_steady_lean_rate_exceed_samples={np.count_nonzero(full_exceed)}/{len(full_exceed)}")
    print(f"{label}_full_exceed_without_level1_binding_samples={np.count_nonzero(full_exceed_without_binding)}/{len(binding)}")
    print(f"{label}_level1_binding_but_full_below_limit_samples={np.count_nonzero(full_below_at_binding)}/{np.count_nonzero(binding)}")
    print(f"{label}_maximum_abs_full_steady_lean_rate_radps={np.max(np.abs(full_rate)):.9f}")
    print(f"{label}_roll_limit_radps={limit:.9f}")

    ranked = sorted(
        rows,
        key=lambda row: max(row["full_exceed_without_binding"], row["full_below_at_binding"]),
        reverse=True,
    )
    print(f"{label}_largest_level1_vs_full_discrepancy_sectors:")
    for row in ranked[:6]:
        print(
            f"  sector={row['sector_index']:02d} "
            f"chainage_m={row['start_m']:.1f}:{row['end_m']:.1f} "
            f"level1_binding={row['level1_binding']}/{row['samples']} "
            f"full_exceed={row['full_exceed']}/{row['samples']} "
            f"full_exceed_without_binding={row['full_exceed_without_binding']} "
            f"binding_but_full_below={row['full_below_at_binding']} "
            f"level1_peak_radps={row['level1_peak']:.6f} "
            f"full_peak_radps={row['full_peak']:.6f} "
            f"mean_abs_full_minus_level1_radps={row['mean_abs_full_minus_level1']:.6f} "
            f"mean_abs_full_at_binding_radps="
            f"{row['mean_abs_full_at_binding']:.6f}" if math.isfinite(row["mean_abs_full_at_binding"])
            else
            f"  sector={row['sector_index']:02d} chainage_m={row['start_m']:.1f}:{row['end_m']:.1f} "
            f"level1_binding={row['level1_binding']}/{row['samples']} full_exceed={row['full_exceed']}/{row['samples']} "
            f"full_exceed_without_binding={row['full_exceed_without_binding']} binding_but_full_below={row['full_below_at_binding']} "
            f"level1_peak_radps={row['level1_peak']:.6f} full_peak_radps={row['full_peak']:.6f} "
            f"mean_abs_full_minus_level1_radps={row['mean_abs_full_minus_level1']:.6f} "
            "mean_abs_full_at_binding_radps=not_available")


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
    print("diagnostic_note=full steady-lean roll rate is post-processing only; it includes actual speed variation and does not change the active Level-1 constraint")

    for label, evaluation in evaluations.items():
        diagnostics = _case_diagnostics(
            track, evaluation, args.max_roll_rate_radps, args.sector_length_m)
        _print_case(label, *diagnostics, args.max_roll_rate_radps)

    print("interpretation_note=full exceedance without Level-1 binding suggests omitted speed-change effects can increase demanded lean rate; Level-1 binding with full rate below the limit suggests the current curvature-only constraint is locally conservative")
    print("calibration_note=no model parameter is changed or fitted by this command")


if __name__ == "__main__":
    main()
