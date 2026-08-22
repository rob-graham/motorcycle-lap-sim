"""Portable input/output helpers for direct-planar physical controls."""

import csv
from pathlib import Path

import numpy as np


CONTROLS_CSV_HEADER = (
    "index", "control_s_m", "best_offset_m", "lower_bound_m", "upper_bound_m")


def write_planar_controls_csv(path, result):
    """Write an optimisation result in the strict physical-control format."""
    output = Path(path)
    with output.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow(CONTROLS_CSV_HEADER)
        rows = zip(result.control_s_m, result.best_controls_m,
                   result.lower_bounds_m, result.upper_bounds_m)
        for index, values in enumerate(rows):
            writer.writerow((index, *values))


def load_planar_controls_csv(path, control_s_m, lower_bounds_m, upper_bounds_m):
    """Load a strict same-layout warm-start controls CSV.

    The saved control stations and bounds must match the current generated
    layout exactly within a small floating-point tolerance. Values are never
    reordered, projected, or clipped.
    """
    required = CONTROLS_CSV_HEADER
    path = Path(path)
    try:
        with path.open(newline="", encoding="utf-8") as stream:
            reader = csv.DictReader(stream)
            fieldnames = tuple(reader.fieldnames or ())
            missing = [field for field in required if field not in fieldnames]
            if missing:
                raise ValueError(
                    "controls CSV is missing required columns: " + ", ".join(missing))
            rows = list(reader)
    except OSError as error:
        raise ValueError(f"controls CSV {path} cannot be read: {error}") from error

    stations = np.asarray(control_s_m, dtype=float)
    lower = np.asarray(lower_bounds_m, dtype=float)
    upper = np.asarray(upper_bounds_m, dtype=float)
    if stations.ndim != 1 or lower.shape != stations.shape or upper.shape != stations.shape:
        raise ValueError("current control stations and bounds must be matching 1D arrays")
    if len(rows) != len(stations):
        raise ValueError(
            f"controls CSV row count {len(rows)} does not match generated "
            f"control-station count {len(stations)}")

    parsed = np.empty((len(stations), 4), dtype=float)
    for expected_index, row in enumerate(rows):
        try:
            index = int(row["index"])
            if str(index) != row["index"].strip():
                raise ValueError
        except (TypeError, ValueError) as error:
            raise ValueError(
                f"controls CSV row {expected_index + 2} has an invalid index") from error
        if index != expected_index:
            raise ValueError(
                f"controls CSV index {index} at row {expected_index + 2} does not "
                f"match expected sequential index {expected_index}")
        try:
            parsed[expected_index] = [float(row[field]) for field in required[1:]]
        except (TypeError, ValueError) as error:
            raise ValueError(
                f"controls CSV row {expected_index + 2} contains a non-numeric value") from error
        if not np.all(np.isfinite(parsed[expected_index])):
            raise ValueError(
                f"controls CSV row {expected_index + 2} contains a non-finite numeric value")

    saved_stations, controls, saved_lower, saved_upper = parsed.T
    tolerance = dict(rtol=0.0, atol=1e-9)
    if not np.allclose(saved_stations, stations, **tolerance):
        raise ValueError("controls CSV control_s_m does not match generated control stations")
    if not np.allclose(saved_lower, lower, **tolerance):
        raise ValueError("controls CSV lower_bound_m does not match current control bounds")
    if not np.allclose(saved_upper, upper, **tolerance):
        raise ValueError("controls CSV upper_bound_m does not match current control bounds")
    outside = (controls < lower) | (controls > upper)
    if np.any(outside):
        index = int(np.flatnonzero(outside)[0])
        raise ValueError(
            f"controls CSV best_offset_m at index {index} is outside current bounds")
    return controls.copy()
