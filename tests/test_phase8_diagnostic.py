"""Lightweight tests for the Phase 8 diagnostic command-line selection helpers."""

from pathlib import Path
import runpy

import numpy as np


SCRIPT = runpy.run_path(Path(__file__).parents[1] / "scripts/r6_phase8_planar_optimisation_check.py")
build_parser = SCRIPT["build_parser"]
selected_policies = SCRIPT["selected_policies"]
selected_tracks = SCRIPT["selected_tracks"]
half_lap_symmetry_differences = SCRIPT["half_lap_symmetry_differences"]
optimisation_config = SCRIPT["optimisation_config"]


def test_targeted_cli_selection():
    args = build_parser().parse_args(("--track", "oval", "--policy", "fine",
                                      "--max-evaluations", "25", "--max-sweeps", "100",
                                      "--workers", "2"))

    assert args.track == "oval"
    assert args.policy == "fine"
    assert args.max_evaluations == 25
    assert args.max_sweeps == 100
    assert optimisation_config(args).max_sweeps == 100
    assert optimisation_config(args).max_evaluations == 25
    assert optimisation_config(args).parallel_workers == 2
    assert [name for name, _ in selected_tracks(args.track)] == ["oval"]
    assert [name for name, _ in selected_policies(args.policy)] == ["fine"]


def test_complete_diagnostic_is_default():
    args = build_parser().parse_args(())

    assert [name for name, _ in selected_tracks(args.track)] == ["oval", "mallala"]
    assert [name for name, _ in selected_policies(args.policy)] == [
        "coarse", "reference", "fine",
    ]
    assert args.workers == 1


def test_half_lap_symmetry_differences_require_repeated_even_station_layout():
    stations = np.array([0.0, 2.0, 5.0, 10.0, 12.0, 15.0])
    controls = np.array([1.0, 2.0, 4.0, 0.5, 3.0, 2.0])

    assert np.array_equal(half_lap_symmetry_differences(stations, controls, 20.0),
                          [0.5, -1.0, 2.0])
    assert half_lap_symmetry_differences(stations[:-1], controls[:-1], 20.0) is None
    assert half_lap_symmetry_differences(stations + [0, 0, 0, 0, 0, 0.1],
                                         controls, 20.0) is None
