import importlib.util
from pathlib import Path


SCRIPT = Path(__file__).resolve().parents[1] / "scripts" / "r6_phase11_margin_aware_restart.py"
spec = importlib.util.spec_from_file_location("phase11_margin_restart", SCRIPT)
phase11 = importlib.util.module_from_spec(spec)
assert spec.loader is not None
spec.loader.exec_module(phase11)


def test_margin_controls_filename_is_stable_and_matches_initial_run_output():
    assert phase11.margin_controls_filename(0.25) == "margin_0.250m_final_controls.csv"
    assert phase11.margin_controls_filename(0.5) == "margin_0.500m_final_controls.csv"


def test_restart_parser_defaults_to_smaller_coordinate_step():
    args = phase11.build_parser().parse_args([
        "reviewed.csv",
        "restart_dir",
        "output_dir",
    ])

    assert args.initial_step_m == 0.25
    assert args.margins_m == phase11.phase11.DEFAULT_MARGINS_M
    assert args.common_spacing_m == 0.125
    assert args.boundary_check_spacing_m == 0.125
