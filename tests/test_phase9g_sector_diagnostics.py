import importlib.util
from pathlib import Path

import numpy as np
import pytest


def _load_script():
    path = Path("scripts/r6_phase9g_sector_diagnostics.py").resolve()
    spec = importlib.util.spec_from_file_location("r6_phase9g_sector_diagnostics_test", path)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_sector_edges_cover_track_without_overlap():
    diagnostic = _load_script()
    starts, ends = diagnostic._sector_edges(250.5, 100.0)

    assert np.allclose(starts, [0.0, 100.0, 200.0])
    assert np.allclose(ends, [100.0, 200.0, 250.5])
    assert ends[-1] == pytest.approx(250.5)
    assert np.allclose(starts[1:], ends[:-1])


def test_sector_indices_are_periodic_and_keep_final_partial_sector():
    diagnostic = _load_script()
    values = np.array([0.0, 99.9, 100.0, 249.9, 250.4, 250.5, 251.0])
    indices = diagnostic._sector_indices(values, 250.5, 100.0, 3)

    assert np.array_equal(indices, [0, 0, 1, 2, 2, 0, 0])


def test_require_sector_sum_accepts_exact_accounting_and_rejects_mismatch():
    diagnostic = _load_script()
    rows = [
        {"delta_s": 0.1},
        {"delta_s": -0.025},
        {"delta_s": 0.225},
    ]

    assert diagnostic._require_sector_sum(rows, "delta_s", 0.3, "test") == pytest.approx(0.3)
    with pytest.raises(RuntimeError, match="does not match whole-lap value"):
        diagnostic._require_sector_sum(rows, "delta_s", 0.31, "test")
