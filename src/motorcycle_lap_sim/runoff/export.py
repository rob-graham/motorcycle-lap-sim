"""Deterministic, language-neutral serialization of run-off input packages."""

from collections import Counter
import csv
from dataclasses import fields
import hashlib
import io
import json
import math
from pathlib import Path

import numpy as np

from .interface import DepartureSeed, RunoffInputPackage
from .georeference import (
    GEOREFERENCE_EXTENSION_VERSION, GEOREFERENCE_SCHEMA, Georeference,
    georeference_json_bytes,
)


RUNOFF_BUNDLE_VERSION = "1.0.0"
TRAJECTORY_FILENAME = "trajectory.csv"
DEPARTURE_SEEDS_FILENAME = "departure_seeds.csv"
MANIFEST_FILENAME = "manifest.json"
GEOREFERENCE_FILENAME = "georeference.json"

DEPARTURE_SEED_FIELDS = tuple(field.name for field in fields(DepartureSeed))


def _csv_bytes(fieldnames, rows):
    stream = io.StringIO(newline="")
    writer = csv.DictWriter(
        stream, fieldnames=fieldnames, lineterminator="\n", extrasaction="raise")
    writer.writeheader()
    writer.writerows(rows)
    return stream.getvalue().encode("utf-8")


def _scalar(value, name):
    if isinstance(value, np.generic):
        value = value.item()
    if isinstance(value, float) and not math.isfinite(value):
        raise ValueError(f"{name} contains a non-finite value")
    if isinstance(value, complex):
        raise ValueError(f"{name} contains an unsupported complex value")
    if not isinstance(value, (str, int, float, bool)):
        raise ValueError(f"{name} contains an unsupported value type")
    return value


def _sha256(data):
    return hashlib.sha256(data).hexdigest()


def serialize_runoff_bundle(package, georeference=None):
    """Return byte-stable bundle files, optionally with a georeference extension."""
    if not isinstance(package, RunoffInputPackage):
        raise TypeError("package must be a RunoffInputPackage")
    if not package.interface_version:
        raise ValueError("run-off interface version is required")
    trajectory_fields = tuple(package.trajectory.keys())
    if not trajectory_fields:
        raise ValueError("trajectory must contain fields")
    lengths = {len(np.asarray(package.trajectory[name])) for name in trajectory_fields}
    if len(lengths) != 1:
        raise ValueError("trajectory fields must have identical lengths")
    row_count = lengths.pop()
    trajectory_rows = []
    for index in range(row_count):
        trajectory_rows.append({
            name: _scalar(package.trajectory[name][index], f"trajectory.{name}")
            for name in trajectory_fields
        })
    trajectory_bytes = _csv_bytes(trajectory_fields, trajectory_rows)

    seed_rows = []
    for seed in package.departure_seeds:
        seed_rows.append({
            name: _scalar(getattr(seed, name), f"departure_seed.{name}")
            for name in DEPARTURE_SEED_FIELDS
        })
    seed_bytes = _csv_bytes(DEPARTURE_SEED_FIELDS, seed_rows)

    metadata = {
        str(key): _scalar(value, f"scenario_metadata.{key}")
        for key, value in package.scenario_metadata.items()
    }
    if any(not key or not isinstance(value, str) or not value for key, value in metadata.items()):
        raise ValueError("scenario metadata keys and values must be non-empty strings")
    warnings = [_scalar(value, "warnings") for value in package.warnings]
    if any(not isinstance(value, str) for value in warnings):
        raise ValueError("warnings must be strings")
    track_length = _scalar(package.track_length_m, "track_length_m")
    path_length = _scalar(package.path_length_m, "path_length_m")
    source_counts = Counter(seed.source_event_type for seed in package.departure_seeds)
    seed_counts = Counter(seed.seed_type for seed in package.departure_seeds)
    manifest = {
        "bundle_version": RUNOFF_BUNDLE_VERSION,
        "runoff_interface_version": package.interface_version,
        "coordinate_frame": package.coordinate_frame,
        "chainage_definition": package.chainage_definition,
        "sampling_convention": package.sampling_convention,
        "track_length_m": track_length,
        "path_length_m": path_length,
        "scenario_metadata": metadata,
        "warnings": warnings,
        "trajectory": {
            "filename": TRAJECTORY_FILENAME,
            "sha256": _sha256(trajectory_bytes),
            "fields": list(trajectory_fields),
            "row_count": row_count,
        },
        "departure_seeds": {
            "filename": DEPARTURE_SEEDS_FILENAME,
            "sha256": _sha256(seed_bytes),
            "fields": list(DEPARTURE_SEED_FIELDS),
            "count": len(seed_rows),
            "counts_by_seed_type": dict(sorted(seed_counts.items())),
            "counts_by_source_event_type": dict(sorted(source_counts.items())),
        },
    }
    georeference_bytes = None
    if georeference is not None:
        if not isinstance(georeference, Georeference):
            raise TypeError("georeference must be a Georeference or None")
        georeference_bytes = georeference_json_bytes(georeference)
        manifest["extensions"] = {
            "georeference": {
                "extension_version": GEOREFERENCE_EXTENSION_VERSION,
                "schema": GEOREFERENCE_SCHEMA,
                "filename": GEOREFERENCE_FILENAME,
                "sha256": _sha256(georeference_bytes),
            }
        }
    manifest_bytes = (json.dumps(
        manifest, sort_keys=True, indent=2, ensure_ascii=False, allow_nan=False) + "\n").encode("utf-8")
    result = {
        MANIFEST_FILENAME: manifest_bytes,
        TRAJECTORY_FILENAME: trajectory_bytes,
        DEPARTURE_SEEDS_FILENAME: seed_bytes,
    }
    if georeference_bytes is not None:
        result[GEOREFERENCE_FILENAME] = georeference_bytes
    return result


def write_runoff_bundle(package, output_dir, georeference=None):
    """Write a deterministic directory bundle and return paths and hashes."""
    files = serialize_runoff_bundle(package, georeference)
    output = Path(output_dir)
    output.mkdir(parents=True, exist_ok=True)
    result = {}
    for filename, data in files.items():
        path = output / filename
        path.write_bytes(data)
        result[filename] = {"path": path, "sha256": _sha256(data)}
    return result
