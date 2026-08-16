import importlib.util
from pathlib import Path

import pytest


SCRIPT_PATH = Path(__file__).parents[1] / "scripts" / "r6_phase10_speed_validation.py"
SPEC = importlib.util.spec_from_file_location("r6_phase10_speed_validation", SCRIPT_PATH)
MODULE = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
SPEC.loader.exec_module(MODULE)


def test_expected_complete_bin_guard_accepts_reviewed_coverage():
    MODULE._require_expected_complete_bins(219, 256, 219)


def test_expected_complete_bin_guard_rejects_stale_envelope():
    with pytest.raises(RuntimeError, match="may be stale"):
        MODULE._require_expected_complete_bins(256, 256, 219)


def test_expected_complete_bin_guard_rejects_impossible_expectation():
    with pytest.raises(ValueError, match="between zero and comparison bin count"):
        MODULE._require_expected_complete_bins(219, 256, 257)


def test_expected_complete_bin_guard_is_optional():
    MODULE._require_expected_complete_bins(256, 256, None)
