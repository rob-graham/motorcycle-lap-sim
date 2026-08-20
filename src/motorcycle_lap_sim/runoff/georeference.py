"""Validated rigid local-to-projected georeferencing extension."""

from dataclasses import asdict, dataclass
import json
import math
from pathlib import Path

import numpy as np


GEOREFERENCE_EXTENSION_VERSION = "0.1.0"
GEOREFERENCE_SCHEMA = "motorcycle-lap-sim-georeference/1"


@dataclass(frozen=True)
class Georeference:
    """Rigid planar mapping; rotation is finite, in radians, and not normalised."""

    schema: str
    horizontal_crs: str
    origin_projected_x_m: float
    origin_projected_y_m: float
    rotation_rad_ccw: float
    source: str
    status: str
    source_sha256: str | None = None
    derivation: str | None = None


def _required_string(document, name):
    value = document.get(name)
    if not isinstance(value, str) or not value.strip():
        raise ValueError(f"{name} must be a non-empty string")
    return value


def _finite_number(document, name):
    value = document.get(name)
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{name} must be a finite numeric scalar")
    value = float(value)
    if not math.isfinite(value):
        raise ValueError(f"{name} must be a finite numeric scalar")
    return value


def parse_georeference(document):
    """Validate a decoded georeference document and return its immutable value."""
    if not isinstance(document, dict):
        raise ValueError("georeference document must be an object")
    permitted = {
        "schema", "horizontal_crs", "origin_projected_x_m", "origin_projected_y_m",
        "rotation_rad_ccw", "source", "status", "source_sha256", "derivation",
    }
    unexpected = sorted(set(document) - permitted)
    if unexpected:
        raise ValueError(f"unexpected georeference fields: {unexpected}")
    schema = _required_string(document, "schema")
    if schema != GEOREFERENCE_SCHEMA:
        raise ValueError(f"schema must equal {GEOREFERENCE_SCHEMA!r}")
    optional = {}
    for name in ("source_sha256", "derivation"):
        if name in document:
            optional[name] = _required_string(document, name)
    return Georeference(
        schema=schema,
        horizontal_crs=_required_string(document, "horizontal_crs"),
        origin_projected_x_m=_finite_number(document, "origin_projected_x_m"),
        origin_projected_y_m=_finite_number(document, "origin_projected_y_m"),
        rotation_rad_ccw=_finite_number(document, "rotation_rad_ccw"),
        source=_required_string(document, "source"),
        status=_required_string(document, "status"),
        **optional,
    )


def load_georeference(path):
    """Load JSON fail-closed (including rejection of JSON NaN and Infinity)."""
    def reject_constant(value):
        raise ValueError(f"non-finite JSON number is not permitted: {value}")
    with Path(path).open(encoding="utf-8") as stream:
        return parse_georeference(json.load(stream, parse_constant=reject_constant))


def georeference_json_bytes(georeference):
    """Return deterministic UTF-8 JSON with a single LF terminator."""
    if not isinstance(georeference, Georeference):
        raise TypeError("georeference must be a Georeference")
    document = {key: value for key, value in asdict(georeference).items() if value is not None}
    # Revalidate so manually constructed values cannot bypass bundle validation.
    parse_georeference(document)
    return (json.dumps(document, sort_keys=True, indent=2, ensure_ascii=False,
                       allow_nan=False) + "\n").encode("utf-8")


def local_xy_to_projected_xy(georeference, local_x, local_y):
    """Apply the documented translation and counter-clockwise rotation."""
    if not isinstance(georeference, Georeference):
        raise TypeError("georeference must be a Georeference")
    x = np.asarray(local_x, dtype=float)
    y = np.asarray(local_y, dtype=float)
    if x.shape != y.shape or not np.all(np.isfinite(x)) or not np.all(np.isfinite(y)):
        raise ValueError("local_x and local_y must have matching shapes and finite values")
    cosine = math.cos(georeference.rotation_rad_ccw)
    sine = math.sin(georeference.rotation_rad_ccw)
    projected_x = georeference.origin_projected_x_m + cosine * x - sine * y
    projected_y = georeference.origin_projected_y_m + sine * x + cosine * y
    if x.ndim == 0:
        return float(projected_x), float(projected_y)
    return projected_x, projected_y
