"""Profile steady-state Phase 11 candidate-evaluation runtime without changing numerics.

The diagnostic separates smooth-path geometry construction (including dense
corridor validation and sampled-path construction) from the fixed-path speed
solve. It also times the ordinary full candidate evaluation so the split can be
checked against the production call path. When the Numba backend is selected,
the diagnostic additionally separates its Python/setup work, compiled
propagation kernel, and post-processing work.
"""

import argparse
from dataclasses import replace
import importlib.util
import math
from pathlib import Path
import statistics
import time

import numpy as np

from motorcycle_lap_sim.motorcycle.config import HandlingConfig, load_motorcycle_config
from motorcycle_lap_sim.optimisation import (
    REFERENCE_PLANAR_CONTROL_POLICY,
    evaluate_planar_racing_line,
    generate_planar_control_stations,
    planar_control_bounds,
)
from motorcycle_lap_sim.racing_line import build_smooth_racing_line_path
from motorcycle_lap_sim.speed_solver import SolverConfig, solve_speed_profile
from motorcycle_lap_sim.track import Track


def _load_sibling(filename, module_name):
    path = Path(__file__).resolve().with_name(filename)
    spec = importlib.util.spec_from_file_location(module_name, path)
    if spec is None or spec.loader is None:
        raise ImportError(f"cannot load sibling script from {path}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


phase8 = _load_sibling("r6_phase8_planar_optimisation_check.py", "phase8_runtime_profile")
phase9 = _load_sibling("r6_phase9_baseline_check.py", "phase9_runtime_profile")
phase9f = _load_sibling("r6_phase9f_roll_aware_optimisation.py", "phase9f_runtime_profile")


def _positive_float(text):
    value = float(text)
    if not math.isfinite(value) or value <= 0.0:
        raise argparse.ArgumentTypeError("value must be finite and positive")
    return value


def _nonnegative_float(text):
    value = float(text)
    if not math.isfinite(value) or value < 0.0:
        raise argparse.ArgumentTypeError("value must be finite and non-negative")
    return value


def _positive_int(text):
    value = int(text)
    if value <= 0:
        raise argparse.ArgumentTypeError("value must be positive")
    return value


def build_parser():
    parser = argparse.ArgumentParser()
    parser.add_argument("controls_csv", type=Path)
    parser.add_argument("--margin-m", type=_nonnegative_float, default=0.25)
    parser.add_argument("--sample-spacing-m", type=_positive_float, default=1.0)
    parser.add_argument("--boundary-check-spacing-m", type=_positive_float, default=0.125)
    parser.add_argument("--max-roll-rate-radps", type=_positive_float, default=0.8)
    parser.add_argument("--repeats", type=_positive_int, default=5)
    parser.add_argument("--speed-backend", choices=("python", "numba", "both"), default="both")
    return parser


def _timed_samples(function, repeats):
    samples = []
    result = None
    for _ in range(repeats):
        started = time.perf_counter()
        result = function()
        samples.append(time.perf_counter() - started)
    return result, tuple(samples)


def timing_summary(samples):
    values = tuple(float(value) for value in samples)
    if not values or any(not math.isfinite(value) or value < 0.0 for value in values):
        raise ValueError("timing samples must be non-empty, finite, and non-negative")
    return {
        "minimum_s": min(values),
        "median_s": statistics.median(values),
        "maximum_s": max(values),
    }


def sampled_path_count(sampled_path):
    """Return the number of fixed-path samples using the SampledPath public coordinate."""
    return len(sampled_path.q_m)


def _print_timing(label, samples):
    summary = timing_summary(samples)
    print(f"{label}_minimum_s={summary['minimum_s']:.9f}")
    print(f"{label}_median_s={summary['median_s']:.9f}")
    print(f"{label}_maximum_s={summary['maximum_s']:.9f}")
    return summary


def _numba_module():
    from motorcycle_lap_sim.speed_solver import numba_backend
    return numba_backend


def _prepare_numba_speed(path, bike, numba_backend):
    """Reproduce the Numba solver work before its compiled propagation kernel."""
    count = len(path.q_m)
    lateral = np.array([
        numba_backend.lateral_speed_limit_mps(k, bike) for k in path.curvature_1pm
    ])
    power = np.full(count, numba_backend.maximum_rev_limited_speed_mps(bike))
    gradient = numba_backend.curvature_gradient_1pm2(path)
    handling = bike.handling
    curvature_limit = (
        np.full(count, np.inf)
        if handling is None or handling.max_path_curvature_rate_1pmps is None
        else numba_backend.curvature_transient_speed_limit_mps(
            path, handling.max_path_curvature_rate_1pmps)
    )
    pre_roll_cap = np.minimum(np.minimum(lateral, power), curvature_limit)
    roll_limit = (
        np.full(count, np.inf)
        if handling is None or handling.max_roll_rate_radps is None
        else numba_backend.roll_rate_speed_limit_mps(
            path.curvature_1pm,
            gradient,
            pre_roll_cap,
            handling.max_roll_rate_radps,
            gravity_mps2=bike.environment.gravity_mps2,
        )
    )
    initial = np.minimum(pre_roll_cap, roll_limit)
    parameters = numba_backend._parameters(bike)
    return {
        "lateral": lateral,
        "power": power,
        "gradient": gradient,
        "curvature_limit": curvature_limit,
        "roll_limit": roll_limit,
        "initial": initial,
        "parameters": parameters,
        "curvature": np.asarray(path.curvature_1pm),
        "segment_lengths": np.asarray(path.segment_lengths_m),
    }


def _propagate_numba_speed(prepared, config, numba_backend):
    return numba_backend._propagate(
        prepared["curvature"],
        prepared["segment_lengths"],
        prepared["initial"],
        config.speed_tolerance_mps,
        config.max_iterations,
        prepared["parameters"],
    )


def _postprocess_numba_speed(path, bike, prepared, propagated, numba_backend):
    """Reproduce the Numba solver work after its compiled propagation kernel."""
    speed, iteration, converged = propagated
    if not converged:
        raise RuntimeError("profiled Numba propagation did not converge")
    following = np.roll(speed, -1)
    longitudinal = (
        (following ** 2 - speed ** 2) / (2 * prepared["segment_lengths"])
    )
    lateral_acceleration = speed ** 2 * np.abs(path.curvature_1pm)
    gears = np.empty(len(speed), dtype=int)
    rpms = np.empty(len(speed))
    parameters = prepared["parameters"]
    for index, value in enumerate(speed):
        gears[index], rpms[index], _ = numba_backend._best_gear(
            value,
            *parameters[5:6],
            parameters[11],
            parameters[12],
            parameters[13],
            parameters[14],
            parameters[15],
            parameters[16],
            parameters[17],
            parameters[18],
        )
    curvature_rate = speed * prepared["gradient"]
    lean = numba_backend.demanded_lean_rad(
        speed,
        path.curvature_1pm,
        gravity_mps2=bike.environment.gravity_mps2,
    )
    roll_rate = numba_backend.curvature_transition_roll_rate_radps(
        speed,
        path.curvature_1pm,
        prepared["gradient"],
        gravity_mps2=bike.environment.gravity_mps2,
    )
    arrays = [
        speed,
        prepared["lateral"],
        prepared["power"],
        prepared["gradient"],
        curvature_rate,
        prepared["curvature_limit"],
        prepared["roll_limit"],
        lean,
        roll_rate,
        lateral_acceleration,
        longitudinal,
        gears,
        rpms,
    ]
    for array in arrays:
        array.setflags(write=False)
    return numba_backend.SpeedProfileResult(
        path.q_m,
        speed,
        prepared["lateral"],
        prepared["power"],
        prepared["gradient"],
        curvature_rate,
        prepared["curvature_limit"],
        prepared["roll_limit"],
        lean,
        roll_rate,
        lateral_acceleration,
        longitudinal,
        gears,
        rpms,
        numba_backend.lap_time_seconds(path, speed),
        iteration,
        True,
    )


def _profile_numba_components(path, bike, repeats, reference_lap_time_s):
    """Measure the Numba backend's setup, propagation and post-processing stages."""
    numba_backend = _numba_module()
    config = SolverConfig()
    prepared = _prepare_numba_speed(path, bike, numba_backend)
    propagated = _propagate_numba_speed(prepared, config, numba_backend)
    reconstructed = _postprocess_numba_speed(
        path, bike, prepared, propagated, numba_backend)
    difference = abs(reconstructed.lap_time_s - reference_lap_time_s)
    if not math.isfinite(difference) or difference > 1e-9:
        raise RuntimeError(
            "Numba profiling reconstruction disagrees with production solver: "
            f"lap difference={difference:.12g} s"
        )

    _, setup_samples = _timed_samples(
        lambda: _prepare_numba_speed(path, bike, numba_backend), repeats)
    setup_summary = _print_timing("speed_numba_setup", setup_samples)

    prepared = _prepare_numba_speed(path, bike, numba_backend)
    _, propagation_samples = _timed_samples(
        lambda: _propagate_numba_speed(prepared, config, numba_backend), repeats)
    propagation_summary = _print_timing(
        "speed_numba_propagation", propagation_samples)

    propagated = _propagate_numba_speed(prepared, config, numba_backend)
    _, post_samples = _timed_samples(
        lambda: _postprocess_numba_speed(
            path, bike, prepared, propagated, numba_backend),
        repeats,
    )
    post_summary = _print_timing("speed_numba_postprocess", post_samples)
    return setup_summary, propagation_summary, post_summary


def main(argv=None):
    args = build_parser().parse_args(argv)
    phase9f._require_canonical_inputs()

    track = Track.from_yaml(phase9.DEFAULT_TRACK)
    base_bike = load_motorcycle_config(phase9.DEFAULT_MOTORCYCLE)
    if base_bike.handling is not None:
        raise RuntimeError("canonical R6 baseline unexpectedly has handling enabled")
    bike = replace(
        base_bike,
        handling=HandlingConfig(max_roll_rate_radps=args.max_roll_rate_radps),
    )

    stations = generate_planar_control_stations(track, REFERENCE_PLANAR_CONTROL_POLICY)
    lower, upper = planar_control_bounds(track, stations, args.margin_m)
    controls = phase8.load_initial_controls_csv(args.controls_csv, stations, lower, upper)

    geometry = lambda: build_smooth_racing_line_path(
        track,
        controls,
        guide_s_m=stations,
        sample_spacing_m=args.sample_spacing_m,
        boundary_margin_m=args.margin_m,
        boundary_check_spacing_m=args.boundary_check_spacing_m,
    )

    # One untimed construction confirms feasibility and provides the fixed path
    # used for speed-only timing. Repeated geometry timings rebuild everything.
    smooth = geometry()
    print(f"controls_csv={args.controls_csv}")
    print(f"controls_sha256={phase9.sha256_file(args.controls_csv)}")
    print(f"control_count={len(stations)}")
    print(f"margin_m={args.margin_m:.9f}")
    print(f"sample_spacing_m={args.sample_spacing_m:.9f}")
    print(f"boundary_check_spacing_m={args.boundary_check_spacing_m:.9f}")
    print(f"max_roll_rate_radps={args.max_roll_rate_radps:.9f}")
    print(f"repeats={args.repeats}")
    print(f"speed_backend={args.speed_backend}")
    print(f"sample_count={sampled_path_count(smooth.sampled_path)}")
    print(f"boundary_check_count={len(smooth.evaluated_track_s_m)}")
    print(f"path_length_m={smooth.sampled_path.total_length_m:.9f}")
    print(f"minimum_boundary_clearance_m={smooth.minimum_boundary_clearance_m:.9f}")

    _, geometry_samples = _timed_samples(geometry, args.repeats)
    geometry_summary = _print_timing("geometry", geometry_samples)

    backends = ("python", "numba") if args.speed_backend == "both" else (args.speed_backend,)
    for backend in backends:
        if backend == "python":
            solver = solve_speed_profile
        else:
            solver = _numba_module().solve_speed_profile_numba
        # Warm the selected backend before measuring steady-state speed solves.
        warm = solver(smooth.sampled_path, bike)
        _, speed_samples = _timed_samples(
            lambda solver=solver: solver(smooth.sampled_path, bike), args.repeats)
        speed_summary = _print_timing(f"speed_{backend}", speed_samples)

        if backend == "numba":
            setup, propagation, post = _profile_numba_components(
                smooth.sampled_path, bike, args.repeats, warm.lap_time_s)
            component_sum = (
                setup["median_s"] + propagation["median_s"] + post["median_s"]
            )
            print(f"speed_numba_component_sum_median_s={component_sum:.9f}")
            if speed_summary["median_s"] > 0.0:
                print(
                    "speed_numba_setup_fraction="
                    f"{setup['median_s'] / speed_summary['median_s']:.6f}")
                print(
                    "speed_numba_propagation_fraction="
                    f"{propagation['median_s'] / speed_summary['median_s']:.6f}")
                print(
                    "speed_numba_postprocess_fraction="
                    f"{post['median_s'] / speed_summary['median_s']:.6f}")

        # Warm the ordinary full evaluator as well. This includes backend lookup,
        # complete geometry construction and the speed solve used by optimisation.
        warm_full = evaluate_planar_racing_line(
            controls, track, bike, stations,
            sample_spacing_m=args.sample_spacing_m,
            boundary_margin_m=args.margin_m,
            boundary_check_spacing_m=args.boundary_check_spacing_m,
            speed_backend=backend,
        )
        if not warm_full.feasible:
            raise RuntimeError(f"warmed full evaluation is infeasible: {warm_full.failure_reason}")
        full, full_samples = _timed_samples(
            lambda backend=backend: evaluate_planar_racing_line(
                controls, track, bike, stations,
                sample_spacing_m=args.sample_spacing_m,
                boundary_margin_m=args.margin_m,
                boundary_check_spacing_m=args.boundary_check_spacing_m,
                speed_backend=backend,
            ),
            args.repeats,
        )
        if not full.feasible:
            raise RuntimeError(f"timed full evaluation is infeasible: {full.failure_reason}")
        full_summary = _print_timing(f"full_{backend}", full_samples)
        estimated_geometry_fraction = (
            geometry_summary["median_s"] / full_summary["median_s"]
            if full_summary["median_s"] > 0.0 else math.nan
        )
        estimated_speed_fraction = (
            speed_summary["median_s"] / full_summary["median_s"]
            if full_summary["median_s"] > 0.0 else math.nan
        )
        print(f"{backend}_lap_time_s={warm.lap_time_s:.9f}")
        print(f"{backend}_estimated_geometry_fraction={estimated_geometry_fraction:.6f}")
        print(f"{backend}_estimated_speed_fraction={estimated_speed_fraction:.6f}")
        print(f"{backend}_full_evaluations_per_second={1.0 / full_summary['median_s']:.6f}")


if __name__ == "__main__":
    main()
