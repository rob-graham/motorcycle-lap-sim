"""User-facing command line interface for motorcycle-lap-sim."""

import argparse
from importlib.metadata import version
from pathlib import Path

from motorcycle_lap_sim.runoff import retained_export


def _repository_root():
    return Path(__file__).resolve().parents[2]


def _default_georeference_path():
    return (_repository_root() / "examples" / "tracks"
            / "mallala_reference.georeference.json")


def build_parser():
    parser = argparse.ArgumentParser(
        prog="motorcycle-lap-sim",
        description="Motorcycle lap simulation engineering workflows.",
        epilog=("Current user-facing workflow: export runoff. Example: "
                "motorcycle-lap-sim export runoff PATH_TO_CONTROLS.csv\n"
                "Additional engineering diagnostics remain available as separate commands."),
    )
    parser.add_argument(
        "--version", action="version",
        version=f"%(prog)s {version('motorcycle-lap-sim')}")
    commands = parser.add_subparsers(dest="command", required=True)

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
    result = retained_export.run_export(args)
    print(f"Run-off bundle written: {args.output_dir}")
    return result


def main(argv=None):
    parser = build_parser()
    args = parser.parse_args(argv)
    return args.handler(args, parser)
