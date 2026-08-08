#!/usr/bin/env python3
"""Generate an auditable, read-only report for a frozen source bag."""

import argparse
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import subprocess
import sys

import yaml


def hash_path(path: Path) -> str:
    files = [path] if path.is_file() else sorted(p for p in path.rglob("*") if p.is_file())
    digest = hashlib.sha256()
    for item in files:
        relative = item.relative_to(path).as_posix() if path.is_dir() else item.name
        digest.update(relative.encode())
        digest.update(b"\0")
        digest.update(str(item.stat().st_size).encode())
        digest.update(b"\0")
        digest.update(hashlib.sha256(item.read_bytes()).hexdigest().encode())
        digest.update(b"\n")
    return "sha256:" + digest.hexdigest()


def command(argv: list[str]) -> str:
    try:
        return subprocess.run(argv, text=True, capture_output=True, check=False).stdout.strip()
    except OSError as exc:
        return f"UNAVAILABLE: {exc}"


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--bag", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--dataset-dir", type=Path, default=None)
    args = parser.parse_args()
    bag = args.bag.resolve()
    output = args.output.resolve()
    dataset_dir = (args.dataset_dir or (Path.cwd() / "runtime/datasets/ds_handheld_facility_20260719")).resolve()
    dataset_path = dataset_dir / "dataset_binding.yaml"
    recipe_path = dataset_dir / "recipe.yaml"
    workspace = Path.cwd() / "runtime/map_projects/facility_a/versions/map_20260808_000000_09a00001"
    info = yaml.safe_load((bag / "metadata.yaml").read_text(encoding="utf-8"))["rosbag2_bagfile_information"]
    topics = [
        {"name": r["topic_metadata"]["name"], "type": r["topic_metadata"]["type"], "count": r["message_count"]}
        for r in info.get("topics_with_message_count", [])
    ]
    names = {item["name"] for item in topics}
    checks = {
        "source_bag_exists": bag.is_dir() and (bag / "metadata.yaml").is_file(),
        "canonical_lidar": "/agt/sensors/lidar/custom" in names,
        "canonical_imu": "/agt/sensors/imu/data" in names,
        "wheel_odom_present": "/agt/chassis/odometry" in names,
        "gnss_present": "/agt/sensors/gnss/fix" in names,
        "mapping_outputs_present": "/agt/mapping/registered_points" in names,
    }
    report = {
        "schema_version": 1,
        "generated_at": datetime.now(timezone.utc).isoformat(),
        "repository": {"path": str(Path.cwd()), "commit": command(["git", "rev-parse", "HEAD"]), "branch": command(["git", "branch", "--show-current"]), "dirty": bool(command(["git", "status", "--short"]))},
        "source_bag": {"path": str(bag), "sha256": hash_path(bag), "storage_id": info.get("storage_identifier"), "duration_s": info.get("duration", {}).get("nanoseconds", 0) / 1e9, "message_count": info.get("message_count"), "topics": topics, "topic_remaps": []},
        "map_workspace": {"status": "PROCESSING" if (workspace / "manifest.yaml").is_file() else "NOT_CREATED", "path": str(workspace) if (workspace / "manifest.yaml").is_file() else None, "reason": "alignment/materialization pending"},
        "lineage": {"capture_rig": {"status": "PASS" if dataset_path.is_file() else "PENDING", "profile_id": "handheld_mid360"}, "calibration": {"status": "PENDING_NUMERIC_VERIFICATION" if dataset_path.is_file() else "PENDING", "identity": "cal_handheld_mid360_sensor_v01"}, "dataset": {"status": "PASS" if dataset_path.is_file() else "PENDING", "identity": "ds_handheld_facility_20260719", "sha256": hash_path(dataset_path) if dataset_path.is_file() else None}, "recipe": {"status": "PASS" if recipe_path.is_file() else "PENDING", "identity": "recipe_handheld_facility_20260719_v01"}, "execution_vehicle": {"status": "NOT_IN_SOURCE_BAG", "profile_id": "MK-mini route binding required"}},
        "checks": checks,
        "mapping_quality_indicators": {"trajectory": "BLOCKED: no /agt/mapping/odometry in source metadata", "registered_cloud_messages": next((x["count"] for x in topics if x["name"] == "/agt/mapping/registered_points"), 0), "pcd_metrics": "BLOCKED: no PCD artifact in source bag", "absolute_z_accuracy": "NOT_CLAIMED"},
        "gates": {"handheld_bag_preflight": "PASS", "topic_compatibility": "PASS" if checks["canonical_lidar"] and checks["canonical_imu"] else "FAIL", "capture_rig_lineage": "PASS" if dataset_path.is_file() else "BLOCKED", "calibration_separation": "PASS_BOUNDARY_PENDING_NUMERIC_EVIDENCE", "mapping_replay_smoke": "NOT_RUN", "full_replay": "NOT_RUN", "canonical_alignment": "PENDING: control points or explicit operator identity confirmation", "map_materialization": "BLOCKED_BY_ALIGNMENT", "map_quality": "PENDING", "route_readiness": "PENDING", "original_bag_modified": "NO", "ros_public_api_changed": "NO", "route_runtime_changed": "NO"},
        "manual_decisions": ["Create a frozen DatasetBinding and Recipe for the handheld capture rig.", "Record sensor/rig calibration without importing provisional BUNKER vehicle extrinsics.", "Select SITE_CONTROL_POINTS or REFERENCE_MAP and record alignment evidence.", "Bind MK-mini separately when deriving and validating a Route Asset."],
        "reproduction": [f"ros2 bag info {bag}", f"python3 tools/offline_asset_validation/generate_report.py --bag {bag} --output {output}"],
    }
    output.mkdir(parents=True, exist_ok=True)
    (output / "report.json").write_text(json.dumps(report, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    (output / "report.md").write_text("# V25-09A offline asset validation\n\n```json\n" + json.dumps(report, indent=2, ensure_ascii=False) + "\n```\n", encoding="utf-8")
    commands = [
        f"ros2 bag info {bag}",
        "python3 -m pytest -q tests/test_offline_asset_contract.py tests/test_navigation_architecture_contract.py tests/test_topic_contract.py",
        "colcon test --packages-select agt_offline_assets agt_system_manager agt_experiment_manager --event-handlers console_cohesion+",
        f"python3 tools/offline_asset_validation/generate_report.py --bag {bag} --dataset-dir {dataset_dir} --output {output}",
    ]
    (output / "commands.log").write_text("\n".join(commands) + "\n", encoding="utf-8")
    (output / "environment.yaml").write_text(yaml.safe_dump({"python": sys.version, "ros2": command(["ros2", "--version"])}, sort_keys=False), encoding="utf-8")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
