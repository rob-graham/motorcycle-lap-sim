"""Portable input/output helpers for direct-planar physical controls."""

import csv
from pathlib import Path


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

