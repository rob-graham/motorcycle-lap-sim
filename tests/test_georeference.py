import csv
import hashlib
import json
import math
from pathlib import Path

import numpy as np
import pytest

from motorcycle_lap_sim.runoff import (
    GEOREFERENCE_SCHEMA, Georeference, georeference_json_bytes,
    load_georeference, local_xy_to_projected_xy, parse_georeference,
)
from motorcycle_lap_sim.track import Track, sample_track

ROOT = Path(__file__).parents[1]
SIDECAR = ROOT / "examples/tracks/mallala_reference.georeference.json"
CONTROLS = ROOT / "examples/tracks/mallala_reference_control_points_epsg7854.csv"
TRACK = ROOT / "examples/tracks/mallala_reference.yaml"


def document(**changes):
    value = {
        "schema": GEOREFERENCE_SCHEMA, "horizontal_crs": "EPSG:1234",
        "origin_projected_x_m": 10, "origin_projected_y_m": 20,
        "rotation_rad_ccw": 0, "source": "fixture", "status": "TEST",
    }
    value.update(changes)
    return value


def test_parse_and_deterministic_serialization():
    value = parse_georeference(document(source_sha256="abc", derivation="known"))
    assert value.horizontal_crs == "EPSG:1234"
    assert georeference_json_bytes(value) == georeference_json_bytes(value)
    assert georeference_json_bytes(value).endswith(b"\n")
    assert hashlib.sha256(georeference_json_bytes(value)).hexdigest() == "b324e4c87c9593fe9edc52a074d7af45c51dd93cbd026daab8728139a8626101"


@pytest.mark.parametrize("bad", [
    [], None, "text", 1,
    document(schema="wrong"),
    {key: value for key, value in document().items() if key != "source"},
    document(horizontal_crs=""), document(source=[]), document(status={}),
    document(origin_projected_x_m=True), document(origin_projected_y_m=[]),
    document(rotation_rad_ccw="0"), document(rotation_rad_ccw=math.nan),
    document(rotation_rad_ccw=math.inf), document(source_sha256=[]),
    document(scale=1),
])
def test_invalid_documents_fail_closed(bad):
    with pytest.raises(ValueError):
        parse_georeference(bad)


def test_loader_rejects_json_nonfinite(tmp_path):
    path = tmp_path / "bad.json"
    path.write_text(json.dumps(document()).replace('"rotation_rad_ccw": 0',
                                                    '"rotation_rad_ccw": NaN'))
    with pytest.raises(ValueError, match="non-finite"):
        load_georeference(path)


def test_rigid_transform_zero_ninety_arbitrary_and_multiple_points():
    zero = parse_georeference(document())
    assert local_xy_to_projected_xy(zero, 2, 3) == pytest.approx((12, 23))
    ninety = parse_georeference(document(rotation_rad_ccw=math.pi / 2))
    assert local_xy_to_projected_xy(ninety, 2, 3) == pytest.approx((7, 22))
    angle = 0.37
    arbitrary = parse_georeference(document(rotation_rad_ccw=angle))
    x, y = local_xy_to_projected_xy(arbitrary, np.array([0, 2]), np.array([0, -4]))
    assert x == pytest.approx([10, 10 + 2 * math.cos(angle) + 4 * math.sin(angle)])
    assert y == pytest.approx([20, 20 + 2 * math.sin(angle) - 4 * math.cos(angle)])


def _point_to_closed_polyline_distance(point, x, y):
    starts = np.column_stack((x, y))
    ends = np.roll(starts, -1, axis=0)
    delta = ends - starts
    relative = point - starts
    fraction = np.clip(np.sum(relative * delta, axis=1) / np.sum(delta * delta, axis=1), 0, 1)
    nearest = starts + fraction[:, None] * delta
    return float(np.min(np.linalg.norm(nearest - point, axis=1)))


def test_canonical_mallala_georeference_against_34_source_observations():
    georeference = load_georeference(SIDECAR)
    assert georeference.horizontal_crs == "EPSG:7854"
    assert local_xy_to_projected_xy(georeference, 0, 0) == pytest.approx(
        (270778.883038345084060, 6188979.605572527274489), abs=1e-9)
    with CONTROLS.open(newline="", encoding="utf-8") as stream:
        rows = list(csv.DictReader(stream))
    direction = math.atan2(float(rows[1]["northing_m"]) - float(rows[0]["northing_m"]),
                           float(rows[1]["easting_m"]) - float(rows[0]["easting_m"]))
    assert direction == pytest.approx(georeference.rotation_rad_ccw, abs=1e-14)
    samples = sample_track(Track.from_yaml(TRACK), 0.05)
    x, y = local_xy_to_projected_xy(georeference, samples.x_m, samples.y_m)
    distances = np.array([_point_to_closed_polyline_distance(
        np.array([float(row["easting_m"]), float(row["northing_m"])]), x, y) for row in rows])
    assert len(distances) == 34
    assert np.median(distances) <= 0.8
    assert np.sqrt(np.mean(distances ** 2)) <= 0.9
    assert np.max(distances) <= 1.7


def test_contract_snapshot_identifies_exact_extension():
    contract = json.loads((ROOT / "contracts/georeference_0.1.0.json").read_text())
    assert contract["extension_version"] == "0.1.0"
    assert contract["schema"] == GEOREFERENCE_SCHEMA
    assert contract["transform"]["positive_rotation"] == "counter-clockwise"
    document_fields = {field["name"]: field
                       for field in contract["georeference_document"]["fields"]}
    assert contract["georeference_document"]["additional_fields_permitted"] is False
    assert document_fields["origin_projected_x_m"]["unit"] == "m"
    assert document_fields["rotation_rad_ccw"]["type"] == "finite number"
    assert document_fields["source_sha256"]["required"] is False
    manifest = contract["manifest_extension"]
    assert manifest["location"] == "manifest.json.extensions.georeference"
    assert manifest["additional_fields_permitted"] is False
    manifest_fields = {field["name"]: field for field in manifest["fields"]}
    assert manifest_fields["filename"]["exact_value"] == "georeference.json"
    assert "exact georeference.json bytes" in manifest_fields["sha256"]["meaning"]
