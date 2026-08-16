import importlib.util
from pathlib import Path

import numpy as np
import pytest


def _load_script():
    path = Path("scripts/r6_phase11c_latent_basin_search.py").resolve()
    spec = importlib.util.spec_from_file_location("r6_phase11c_latent_basin_search_test", path)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_latent_to_reference_controls_zero_maps_to_zero():
    diagnostic = _load_script()
    reference_s = np.linspace(0.0, 900.0, 10, endpoint=False)
    controls = diagnostic.latent_to_reference_controls(
        np.zeros(6), reference_s, 1000.0,
        np.full(10, -3.0), np.full(10, 4.0))

    assert np.array_equal(controls, np.zeros(10))


def test_latent_to_reference_controls_is_periodic_smooth_and_bounded():
    diagnostic = _load_script()
    reference_s = np.array([0.0, 50.0, 100.0, 950.0])
    latent = np.array([1.0, 0.0, -1.0, 0.0])
    lower = np.full(4, -0.4)
    upper = np.full(4, 0.6)

    controls = diagnostic.latent_to_reference_controls(
        latent, reference_s, 1000.0, lower, upper)

    assert np.all(np.isfinite(controls))
    assert np.all(controls >= lower)
    assert np.all(controls <= upper)
    assert abs(controls[0] - controls[-1]) < 0.3


def test_latent_to_reference_controls_rejects_bad_shapes_and_nonfinite_values():
    diagnostic = _load_script()
    reference_s = np.array([0.0, 100.0])
    lower = np.array([-1.0, -1.0])
    upper = np.array([1.0, 1.0])

    with pytest.raises(ValueError, match="at least four"):
        diagnostic.latent_to_reference_controls(
            np.zeros(3), reference_s, 1000.0, lower, upper)
    with pytest.raises(ValueError, match="finite"):
        diagnostic.latent_to_reference_controls(
            np.array([0.0, 0.0, np.nan, 0.0]), reference_s, 1000.0, lower, upper)
    with pytest.raises(ValueError, match="matching 1D arrays"):
        diagnostic.latent_to_reference_controls(
            np.zeros(4), reference_s, 1000.0, np.array([-1.0]), upper)
