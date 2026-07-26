import pytest
import yaml

from agt_teach_repeat.path_io import (
    atomic_write_json,
    load_reference_path,
    sha256_file,
    write_reference_paths,
)
from agt_teach_repeat.path_types import PathPose, TeachRepeatError


def poses():
    return (
        PathPose(1, 0.0, 0.0, frame_id="map"),
        PathPose(2, 1.0, 0.0, frame_id="map"),
    )


def test_reference_round_trip_and_schema_rejection(tmp_path):
    paths = write_reference_paths(tmp_path, "route_01", poses())
    loaded = load_reference_path(paths["yaml"], expected_demo_id="route_01")
    assert loaded == poses()
    document = yaml.safe_load(paths["yaml"].read_text(encoding="utf-8"))
    document["schema_version"] = 99
    paths["yaml"].write_text(yaml.safe_dump(document), encoding="utf-8")
    with pytest.raises(TeachRepeatError, match="unsupported"):
        load_reference_path(paths["yaml"])


def test_atomic_json_replaces_content_and_rejects_non_finite(tmp_path):
    output = tmp_path / "result.json"
    atomic_write_json(output, {"value": 1.0})
    first_hash = sha256_file(output)
    atomic_write_json(output, {"value": 2.0})
    assert sha256_file(output) != first_hash
    assert not list(tmp_path.glob(".*.tmp"))
    with pytest.raises(TeachRepeatError, match="NaN or Inf"):
        atomic_write_json(output, {"value": float("inf")})
    assert "2.0" in output.read_text(encoding="utf-8")
