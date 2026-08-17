import importlib.util
from pathlib import Path

import numpy as np
import pytest


SCRIPT = Path(__file__).resolve().parents[1] / "scripts" / "r6_phase11_relocated_basis_reoptimisation.py"
spec = importlib.util.spec_from_file_location("phase11_relocated_basis_reopt_test", SCRIPT)
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


def test_parser_defaults():
    args = module.build_parser().parse_args(["controls.csv", "output"])
    assert args.relocate_index == 27
    assert args.relocate_shift_m == 5.0
    assert args.minimum_station_gap_m == 5.0
    assert args.margin_m == 0.25
    assert args.max_roll_rate_radps == 0.8
    assert args.initial_step_m == 0.125
    assert args.minimum_step_m == 0.0625
    assert args.max_sweeps == 12
    assert args.max_evaluations == 4000
    assert args.workers == 1
    assert args.optimisation_spacing_m == 1.0
    assert args.common_spacing_m == 0.125
    assert args.boundary_check_spacing_m == 0.125
    assert args.plot_dpi == 400


def test_relocated_basis_moves_only_requested_station():
    stations = np.array([0.0, 20.0, 40.0, 60.0])
    moved = module.relocated_basis(stations, 2, 5.0, 80.0, 5.0)
    np.testing.assert_allclose(moved, [0.0, 20.0, 45.0, 60.0])
    np.testing.assert_allclose(stations, [0.0, 20.0, 40.0, 60.0])


def test_relocated_basis_rejects_neighbour_crossing():
    stations = np.array([0.0, 20.0, 40.0, 60.0])
    with pytest.raises(ValueError, match="bounded neighbour interval"):
        module.relocated_basis(stations, 2, 16.0, 80.0, 5.0)


def test_relocation_shift_must_be_nonzero_in_main(tmp_path):
    with pytest.raises(ValueError, match="finite and non-zero"):
        module.main(["missing.csv", str(tmp_path), "--relocate-shift-m", "0"])


def test_compact_discards_heavy_candidate_results():
    evaluation = module.PlanarObjectiveEvaluation(
        True, 71.25, smooth_line=object(), speed_profile=object())

    compact = module._compact(evaluation)

    assert compact.feasible
    assert compact.lap_time_s == 71.25
    assert compact.smooth_line is None
    assert compact.speed_profile is None
