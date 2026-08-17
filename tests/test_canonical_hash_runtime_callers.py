import importlib.util
from pathlib import Path


def _load_script(filename, module_name):
    path = Path("scripts") / filename
    spec = importlib.util.spec_from_file_location(module_name, path.resolve())
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_phase9_hash_compatibility_name_keeps_canonical_text_semantics():
    phase9 = _load_script("r6_phase9_baseline_check.py", "phase9_hash_compat_test")

    assert phase9.sha256_file is phase9.canonical_text_sha256
    assert phase9.sha256_file(phase9.DEFAULT_CONTROLS) == phase9.EXPECTED_CONTROLS_SHA256
    assert phase9.sha256_file(phase9.DEFAULT_TRACK) == phase9.EXPECTED_TRACK_SHA256
    assert phase9.sha256_file(phase9.DEFAULT_MOTORCYCLE) == phase9.EXPECTED_MOTORCYCLE_SHA256


def test_phase9f_canonical_input_runtime_check_executes():
    phase9f = _load_script(
        "r6_phase9f_roll_aware_optimisation.py", "phase9f_hash_runtime_test")

    phase9f._require_canonical_inputs()


def test_spatial_canonical_input_runtime_check_executes():
    spatial = _load_script(
        "r6_phase9f_spatial_comparison.py", "phase9f_spatial_hash_runtime_test")

    spatial._require_canonical_inputs()
