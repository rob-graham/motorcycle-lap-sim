"""User-facing command line interface for motorcycle-lap-sim."""

import argparse
from importlib.metadata import version
import math
from pathlib import Path

from motorcycle_lap_sim.motorcycle import load_motorcycle_config
from motorcycle_lap_sim.optimisation import (
    COARSE_PLANAR_CONTROL_POLICY, FINE_PLANAR_CONTROL_POLICY,
    REFERENCE_PLANAR_CONTROL_POLICY, PlanarOptimisationConfig,
    generate_planar_control_stations, load_planar_controls_csv,
    optimise_planar_racing_line, planar_control_bounds, write_planar_controls_csv)
from motorcycle_lap_sim.runoff import retained_export
from motorcycle_lap_sim.track import Track


_PLANAR_POLICIES = {
    "coarse": COARSE_PLANAR_CONTROL_POLICY,
    "reference": REFERENCE_PLANAR_CONTROL_POLICY,
    "fine": FINE_PLANAR_CONTROL_POLICY,
}
_PLANAR_DEFAULTS = PlanarOptimisationConfig()


def _positive_float(value):
    parsed = float(value)
    if not math.isfinite(parsed) or parsed <= 0:
        raise argparse.ArgumentTypeError("must be finite and positive")
    return parsed


def _nonnegative_float(value):
    parsed = float(value)
    if not math.isfinite(parsed) or parsed < 0:
        raise argparse.ArgumentTypeError("must be finite and non-negative")
    return parsed


def _step_reduction(value):
    parsed = float(value)
    if not math.isfinite(parsed) or not 0 < parsed < 1:
        raise argparse.ArgumentTypeError("must be finite and between zero and one")
    return parsed


def _positive_int(value):
    parsed = int(value)
    if parsed <= 0:
        raise argparse.ArgumentTypeError("must be a positive integer")
    return parsed


def _repository_root():
    return Path(__file__).resolve().parents[2]


def _default_georeference_path():
    return (_repository_root() / "examples" / "tracks"
            / "mallala_reference.georeference.json")


def build_parser():
    parser = argparse.ArgumentParser(
        prog="motorcycle-lap-sim",
        description="Motorcycle lap simulation engineering workflows.",
        epilog=("Optimise any supported track and motorcycle: motorcycle-lap-sim "
                "optimise TRACK.yaml MOTORCYCLE.yaml\n"
                "Additional engineering diagnostics remain available as separate commands."),
    )
    parser.add_argument(
        "--version", action="version",
        version=f"%(prog)s {version('motorcycle-lap-sim')}")
    commands = parser.add_subparsers(dest="command", required=True)

    optimise = commands.add_parser(
        "optimise", help="Run deterministic local direct-planar optimisation")
    optimise.add_argument("track", type=Path, metavar="TRACK.yaml")
    optimise.add_argument("motorcycle", type=Path, metavar="MOTORCYCLE.yaml")
    optimise.add_argument("--output", type=Path, metavar="CONTROLS.csv")
    optimise.add_argument("--policy", choices=tuple(_PLANAR_POLICIES), default="reference")
    advanced_optimise = optimise.add_argument_group("advanced optimisation options")
    advanced_optimise.add_argument(
        "--restart-controls", "--initial-controls-csv", dest="initial_controls_csv",
        type=Path, metavar="CONTROLS.csv",
        help=("warm-start from a prior controls CSV with the exact same track, policy, "
              "and boundary-margin layout; search counters and step are not restored"))
    advanced_optimise.add_argument("--initial-step-m", type=_positive_float,
                                   default=_PLANAR_DEFAULTS.initial_step_m)
    advanced_optimise.add_argument("--minimum-step-m", type=_positive_float,
                                   default=_PLANAR_DEFAULTS.minimum_step_m)
    advanced_optimise.add_argument("--step-reduction", type=_step_reduction,
                                   default=_PLANAR_DEFAULTS.step_reduction)
    advanced_optimise.add_argument(
        "--lap-time-improvement-tolerance-s", type=_nonnegative_float,
        default=_PLANAR_DEFAULTS.lap_time_improvement_tolerance_s)
    advanced_optimise.add_argument("--max-sweeps", type=_positive_int,
                                   default=_PLANAR_DEFAULTS.max_sweeps)
    advanced_optimise.add_argument("--max-evaluations", type=_positive_int,
                                   default=_PLANAR_DEFAULTS.max_evaluations)
    advanced_optimise.add_argument("--boundary-margin-m", type=_nonnegative_float,
                                   default=_PLANAR_DEFAULTS.boundary_margin_m)
    advanced_optimise.add_argument("--boundary-check-spacing-m", type=_positive_float,
                                   default=_PLANAR_DEFAULTS.boundary_check_spacing_m)
    advanced_optimise.add_argument("--optimisation-sample-spacing-m", type=_positive_float,
                                   default=_PLANAR_DEFAULTS.optimisation_sample_spacing_m)
    advanced_optimise.add_argument("--workers", type=_positive_int,
                                   default=_PLANAR_DEFAULTS.parallel_workers)
    advanced_optimise.add_argument("--speed-backend", choices=("python", "numba"),
                                   default=_PLANAR_DEFAULTS.speed_backend)
    optimise.set_defaults(handler=_run_optimise)

    export = commands.add_parser("export", help="Export simulator results")
    export_commands = export.add_subparsers(dest="export_command", required=True)
    runoff = export_commands.add_parser(
        "runoff", help="Write the retained Mallala LOWSIDE producer bundle")
    runoff.add_argument("representative_controls_csv", type=Path, metavar="CONTROLS.csv")
    runoff.add_argument(
        "--output", type=Path, dest="output_dir", metavar="DIR",
        help="output directory (default: runoff-bundle beside CONTROLS.csv)")
    georeference = runoff.add_mutually_exclusive_group()
    georeference.add_argument(
        "--georeference-json", type=Path, metavar="PATH",
        help="override the committed Mallala georeference")
    georeference.add_argument(
        "--no-georeference", action="store_true",
        help="intentionally create a non-georeferenced bundle")

    advanced = runoff.add_argument_group("advanced retained-case options")
    advanced.add_argument("--delete-index", type=int,
                          default=retained_export.phase12a.DEFAULT_DELETE_INDEX)
    advanced.add_argument("--margin-m", type=retained_export.phase12a._nonnegative_float,
                          default=retained_export.phase12a.DEFAULT_MARGIN_M)
    advanced.add_argument(
        "--max-roll-rate-radps", type=retained_export.phase12a._positive_float,
        default=retained_export.phase12a.DEFAULT_MAX_ROLL_RATE_RADPS)
    advanced.add_argument("--spacing-m", type=retained_export.phase12a._positive_float,
                          default=retained_export.phase12a.DEFAULT_SPACING_M)
    advanced.add_argument(
        "--boundary-check-spacing-m", type=retained_export.phase12a._positive_float,
        default=retained_export.phase12a.DEFAULT_BOUNDARY_CHECK_SPACING_M)
    advanced.add_argument("--expected-lap-s", type=retained_export.phase12a._positive_float,
                          default=retained_export.phase12a.DEFAULT_EXPECTED_LAP_S)
    advanced.add_argument(
        "--lap-tolerance-s", type=retained_export.phase12a._nonnegative_float,
        default=retained_export.phase12a.DEFAULT_LAP_TOLERANCE_S)
    runoff.set_defaults(handler=_run_runoff_export)
    return parser


def _run_optimise(args, parser):
    track_file = args.track.expanduser().resolve()
    motorcycle_file = args.motorcycle.expanduser().resolve()
    if not track_file.is_file():
        parser.error(f"track file does not exist: {track_file}")
    if not motorcycle_file.is_file():
        parser.error(f"motorcycle file does not exist: {motorcycle_file}")
    restart_file = (None if args.initial_controls_csv is None
                    else args.initial_controls_csv.expanduser().resolve())
    if restart_file is not None and not restart_file.is_file():
        parser.error(f"restart controls file does not exist: {restart_file}")
    output = ((Path.cwd() / f"{track_file.stem}_controls.csv").resolve()
              if args.output is None else args.output.expanduser().resolve())
    config = PlanarOptimisationConfig(
        initial_step_m=args.initial_step_m, minimum_step_m=args.minimum_step_m,
        step_reduction=args.step_reduction,
        lap_time_improvement_tolerance_s=args.lap_time_improvement_tolerance_s,
        max_sweeps=args.max_sweeps, max_evaluations=args.max_evaluations,
        boundary_margin_m=args.boundary_margin_m,
        boundary_check_spacing_m=args.boundary_check_spacing_m,
        optimisation_sample_spacing_m=args.optimisation_sample_spacing_m,
        parallel_workers=args.workers, speed_backend=args.speed_backend)
    track = Track.from_yaml(track_file)
    motorcycle = load_motorcycle_config(motorcycle_file)
    policy = _PLANAR_POLICIES[args.policy]
    initial_controls = None
    if restart_file is not None:
        stations = generate_planar_control_stations(track, policy)
        lower, upper = planar_control_bounds(track, stations, config.boundary_margin_m)
        try:
            initial_controls = load_planar_controls_csv(
                restart_file, stations, lower, upper)
        except ValueError as error:
            parser.error(f"restart controls are incompatible: {error}")
    result = optimise_planar_racing_line(
        track, motorcycle, policy, config, initial_controls_m=initial_controls)
    write_planar_controls_csv(output, result)
    print("Deterministic LOCAL optimisation complete; this is not proof of a global fastest line.")
    print(f"Track: {track_file}")
    print(f"Motorcycle: {motorcycle_file}")
    print(f"Controls output: {output}")
    print(f"Control policy: {args.policy}")
    print(f"Restart controls: {restart_file or 'none (zero-control start)'}")
    print(f"Control count: {len(result.control_s_m)}")
    print(f"Speed backend: {config.speed_backend}; workers: {config.parallel_workers}")
    print(f"Initial lap time: {result.initial_lap_time_s:.9f} s")
    print(f"Optimised lap time: {result.best_lap_time_s:.9f} s")
    print(f"Improvement: {result.improvement_s:.9f} s")
    print(f"Evaluations: {result.evaluations}; sweeps: {result.sweeps}")
    print(f"Final step: {result.final_step_m:.9f} m")
    print(f"Termination reason: {result.termination_reason}")
    print(f"Minimum boundary clearance: {result.minimum_boundary_clearance_m:.9f} m")
    return 0


def _run_runoff_export(args, parser):
    controls = args.representative_controls_csv.expanduser().resolve()
    if not controls.is_file():
        parser.error(f"controls file does not exist: {controls}")
    args.representative_controls_csv = controls
    args.output_dir = ((controls.parent / "runoff-bundle") if args.output_dir is None
                       else args.output_dir.expanduser().resolve())
    if args.no_georeference:
        args.georeference_json = None
    else:
        georeference = (args.georeference_json.expanduser().resolve()
                        if args.georeference_json is not None
                        else _default_georeference_path())
        if not georeference.is_file():
            parser.error(f"georeference JSON does not exist: {georeference}")
        args.georeference_json = georeference

    print(f"Controls: {args.representative_controls_csv}")
    print(f"Output: {args.output_dir}")
    print(f"Georeference: {args.georeference_json or 'disabled'}")
    retained_export.run_export(args)
    print(f"Run-off bundle written: {args.output_dir}")
    return 0


def main(argv=None):
    parser = build_parser()
    args = parser.parse_args(argv)
    return args.handler(args, parser)
