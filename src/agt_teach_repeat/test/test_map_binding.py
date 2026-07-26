import pytest

from agt_teach_repeat.path_io import (
    atomic_write_yaml,
    load_manifest,
    sha256_file,
    verify_manifest_bindings,
)
from agt_teach_repeat.path_types import PathPose, TransformSE2
from agt_teach_repeat.path_io import write_reference_paths


def make_manifest(tmp_path):
    processed = tmp_path / "processed"
    reference = write_reference_paths(
        processed,
        "route_01",
        (
            PathPose(1, 0.0, 0.0, frame_id="map"),
            PathPose(2, 1.0, 0.0, frame_id="map"),
        ),
    )["yaml"]
    map_yaml = tmp_path / "map.yaml"
    map_yaml.write_text("image: map.pgm\nresolution: 0.1\norigin: [0, 0, 0]\n", encoding="utf-8")
    pcd = tmp_path / "map.pcd"
    pcd.write_bytes(b"pcd")
    record = tmp_path / "record.yaml"
    record.write_text("state: ready\nmap_file: map.pcd\n", encoding="utf-8")
    manifest = {
        "schema_version": 1,
        "demo_id": "route_01",
        "source": {
            "bag_path": "bag",
            "bag_sha256": "sha256:" + "0" * 64,
            "odometry_topic": "/agt/mapping/odometry",
        },
        "map": {
            "map_id": "map_01",
            "map_yaml": str(map_yaml),
            "map_yaml_sha256": sha256_file(map_yaml),
            "localization_pcd": str(pcd),
            "localization_pcd_sha256": sha256_file(pcd),
            "processing_record": str(record),
            "processing_record_sha256": sha256_file(record),
        },
        "platform": {"profile": "profiles/platforms/bunker.yaml"},
        "frames": {
            "source_frame": "odom",
            "execution_frame": "map",
            "map_from_teach_odom": {"x": 0, "y": 0, "z": 0, "yaw": 0},
        },
        "processing": {},
        "execution": {},
        "assets": {
            "reference_path": "processed/reference_path.yaml",
            "reference_path_sha256": sha256_file(reference),
        },
    }
    path = tmp_path / "manifest.yaml"
    atomic_write_yaml(path, manifest)
    return path, map_yaml


def test_map_and_path_hash_mismatch_is_fail_closed(tmp_path):
    path, map_yaml = make_manifest(tmp_path)
    manifest_path, manifest = load_manifest(path)
    assert verify_manifest_bindings(manifest_path, manifest)["valid"] is True
    map_yaml.write_text("changed", encoding="utf-8")
    result = verify_manifest_bindings(manifest_path, manifest)
    assert result["valid"] is False
    assert "map_yaml_hash_mismatch" in result["errors"]


def test_se2_map_binding_preserves_z_and_rotates_yaw():
    pose = PathPose(1, 1.0, 0.0, 0.4)
    transformed = TransformSE2(x=2.0, y=3.0, z=0.5, yaw=1.5707963267948966).apply(pose)
    assert transformed.x == pytest.approx(2.0)
    assert transformed.y == pytest.approx(4.0)
    assert transformed.z == pytest.approx(0.9)
    assert transformed.yaw == pytest.approx(1.5707963267948966)
