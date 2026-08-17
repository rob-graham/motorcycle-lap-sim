import importlib.util
from pathlib import Path

import pytest


SCRIPT = Path(__file__).resolve().parents[1] / "scripts" / "r6_phase11_runtime_profile.py"
spec = importlib.util.spec_from_file_location("phase11_runtime_profile", SCRIPT)
phase11 = importlib.util.module_from_spec(spec)
assert spec.loader is not None
spec.loader.exec_module(phase11)


def test_timing_summary_reports_minimum_median_and_maximum():
    summary = phase11.timing_summary((0.3, 0.1, 0.2, 0.4, 0.5))

    assert summary == {
        "minimum_s": 0.1,
        "median_s": 0.3,
        "maximum_s": 0.5,
    }


def test_timing_summary_rejects_invalid_samples():
    with pytest.raises(ValueError, match="non-empty"):
        phase11.timing_summary(())
    with pytest.raises(ValueError, match="finite"):
        phase11.timing_summary((0.1, float("nan")))
    with pytest.raises(ValueError, match="non-negative"):
        phase11.timing_summary((0.1, -0.2))


def test_parser_defaults_match_phase11_runtime_case():
    args = phase11.build_parser().parse_args(["controls.csv"])

    assert args.margin_m == 0.25
    assert args.sample_spacing_m == 1.0
    assert args.boundary_check_spacing_m == 0.125
    assert args.max_roll_rate_radps == 0.8
    assert args.repeats == 5
    assert args.speed_backend == "both"
