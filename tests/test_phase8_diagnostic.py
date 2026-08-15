"""Lightweight tests for the Phase 8 diagnostic command-line selection helpers."""

from pathlib import Path
import runpy


SCRIPT = runpy.run_path(Path(__file__).parents[1] / "scripts/r6_phase8_planar_optimisation_check.py")
build_parser = SCRIPT["build_parser"]
selected_policies = SCRIPT["selected_policies"]
selected_tracks = SCRIPT["selected_tracks"]


def test_targeted_cli_selection():
    args = build_parser().parse_args(("--track", "oval", "--policy", "fine",
                                      "--max-evaluations", "25"))

    assert args.track == "oval"
    assert args.policy == "fine"
    assert args.max_evaluations == 25
    assert [name for name, _ in selected_tracks(args.track)] == ["oval"]
    assert [name for name, _ in selected_policies(args.policy)] == ["fine"]


def test_complete_diagnostic_is_default():
    args = build_parser().parse_args(())

    assert [name for name, _ in selected_tracks(args.track)] == ["oval", "mallala"]
    assert [name for name, _ in selected_policies(args.policy)] == [
        "coarse", "reference", "fine",
    ]
