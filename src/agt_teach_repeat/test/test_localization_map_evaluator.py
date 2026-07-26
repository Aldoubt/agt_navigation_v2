from agt_teach_repeat.localization_map_evaluator import (
    check_map_integrity,
    evaluate_localization,
    pcd_point_count,
)
from agt_teach_repeat.path_io import (
    atomic_write_yaml,
    sha256_file,
    write_reference_paths,
)
from agt_teach_repeat.path_types import PathPose


THRESHOLDS = {
    "minimum_attempts": 3,
    "minimum_success_rate": 0.8,
    "maximum_ambiguous_results": 0,
    "maximum_wrong_accepts": 0,
    "maximum_tracking_lost_count": 0,
    "maximum_median_relocalization_time_s": 5.0,
}


def make_manifest(tmp_path):
    image = tmp_path / "map.pgm"
    image.write_bytes(b"P5\n1 1\n255\n\xfe")
    map_yaml = tmp_path / "map.yaml"
    map_yaml.write_text("image: map.pgm\nresolution: 0.1\norigin: [0, 0, 0]\n", encoding="utf-8")
    pcd = tmp_path / "map.pcd"
    pcd.write_text(
        "# .PCD v0.7\n"
        "FIELDS x y z\n"
        "SIZE 4 4 4\n"
        "TYPE F F F\n"
        "COUNT 1 1 1\n"
        "WIDTH 3\n"
        "HEIGHT 1\n"
        "POINTS 3\n"
        "DATA ascii\n"
        "0 0 0\n1 0 0\n2 0 0\n",
        encoding="ascii",
    )
    record = tmp_path / "record.yaml"
    record.write_text(
        f"state: ready\nmap_file: map.pcd\npcd_sha256: {sha256_file(pcd)}\n",
        encoding="utf-8",
    )
    reference = write_reference_paths(
        tmp_path / "processed",
        "route_01",
        (
            PathPose(1, 0.0, 0.0, frame_id="map"),
            PathPose(2, 1.0, 0.0, frame_id="map"),
        ),
    )["yaml"]
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
    return path, pcd


def test_map_integrity_checks_pcd_record_hash_and_minimum_points(tmp_path):
    manifest, pcd = make_manifest(tmp_path)
    assert pcd_point_count(pcd) == 3
    assert check_map_integrity(manifest, minimum_pcd_points=3)["valid"] is True
    result = check_map_integrity(manifest, minimum_pcd_points=4)
    assert result["valid"] is False
    assert "pcd_point_count_too_small" in result["errors"]
    (tmp_path / "map.pgm").write_bytes(b"P5\n1 1\n255\n")
    assert check_map_integrity(manifest, minimum_pcd_points=3)["valid"] is False


def test_relocalization_grade_requires_multiple_attempts_and_preserves_nulls():
    one = evaluate_localization(
        [{"success": True, "state": "TRACKING", "runtime_ms": 100.0, "fitness_score": 0.01}],
        THRESHOLDS,
    )
    assert one["grade"] == "OFFLINE_ONLY"
    assert one["samples"][0]["overlap_ratio"] is None
    samples = [
        {"success": True, "state": "TRACKING", "runtime_ms": 1000.0, "ambiguous_result": False},
        {"success": True, "state": "TRACKING", "runtime_ms": 2000.0, "ambiguous_result": False},
        {"success": True, "state": "TRACKING", "runtime_ms": 3000.0, "ambiguous_result": False},
    ]
    assert evaluate_localization(samples, THRESHOLDS)["grade"] == "FIELD_CANDIDATE"
    assert (
        evaluate_localization(samples, THRESHOLDS, field_validated=True)["grade"]
        == "FIELD_VALIDATED"
    )
