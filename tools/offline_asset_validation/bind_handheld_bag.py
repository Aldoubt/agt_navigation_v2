#!/usr/bin/env python3
"""Create immutable lineage artifacts for a handheld MID360 bag.

The command only binds evidence. It does not claim vehicle calibration or map
alignment, and it never modifies the source bag.
"""

import argparse
from datetime import datetime, timezone
import os
from pathlib import Path
import subprocess
import yaml

from agt_offline_assets import sha256_file, sha256_path_bundle


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--bag", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--site-id", default="facility_a")
    parser.add_argument("--epoch-id", default="20260719_handheld")
    parser.add_argument("--capture-rig-profile", default="profiles/capture_rigs/handheld_mid360.yaml")
    parser.add_argument("--calibration", default="runtime/calibrations/handheld_mid360_sensor_v01.yaml")
    args = parser.parse_args()
    root = Path.cwd()
    bag = args.bag.expanduser().resolve()
    rig = (root / args.capture_rig_profile).resolve() if not Path(args.capture_rig_profile).is_absolute() else Path(args.capture_rig_profile)
    calibration = (root / args.calibration).resolve() if not Path(args.calibration).is_absolute() else Path(args.calibration)
    if not bag.is_dir() or not (bag / "metadata.yaml").is_file():
        raise SystemExit("bag must be a rosbag2 directory containing metadata.yaml")
    for path in (rig, calibration):
        if not path.is_file():
            raise SystemExit(f"missing lineage artifact: {path}")
    commit = subprocess.run(["git", "rev-parse", "HEAD"], capture_output=True, text=True, check=True).stdout.strip()
    output_root = args.output.expanduser().resolve()
    try:
        bag_reference = Path(os.path.relpath(bag, output_root)).as_posix()
    except ValueError:
        bag_reference = str(bag)
    binding = {
        "schema_version": 1,
        "dataset_id": "ds_handheld_facility_20260719",
        "site_id": args.site_id,
        "epoch_id": args.epoch_id,
        "purpose": "OPERATIONAL",
        "source_role": "MAP_CAPTURE",
        "bag": {"path": bag_reference, "sha256": sha256_path_bundle(bag)},
        "capture_rig": {"profile_id": "handheld_mid360", "profile_sha256": sha256_file(rig)},
        "platform": {"profile_id": "handheld_mid360", "profile_sha256": sha256_file(rig)},
        "calibration": {
            "calibration_id": "cal_handheld_mid360_sensor_v01",
            "calibration_sha256": sha256_file(calibration),
            "scope": "SENSOR_CAPTURE_RIG_ONLY",
        },
        "execution_vehicle": {},
        "replay_topic_remaps": {},
        "created_at": datetime.now(timezone.utc).isoformat(timespec="seconds"),
    }
    args.output.mkdir(parents=True, exist_ok=True)
    dataset_path = args.output / "dataset_binding.yaml"
    dataset_path.write_text(yaml.safe_dump(binding, sort_keys=False, allow_unicode=True), encoding="utf-8")
    recipe = {
        "schema_version": 1,
        "recipe_id": "recipe_handheld_facility_20260719_v01",
        "source_dataset_id": binding["dataset_id"],
        "source_dataset_sha256": sha256_file(dataset_path),
        "calibration_id": binding["calibration"]["calibration_id"],
        "calibration_sha256": binding["calibration"]["calibration_sha256"],
        "platform_profile": "handheld_mid360",
        "platform_profile_sha256": binding["capture_rig"]["profile_sha256"],
        "capture_rig": {"profile_id": "handheld_mid360", "profile_sha256": binding["capture_rig"]["profile_sha256"]},
        "repository_commit": commit,
        "repository_dirty_at_binding": bool(subprocess.run(["git", "status", "--short"], capture_output=True, text=True, check=True).stdout.strip()),
        "random_seed": 0,
        "mapping": {"backend": "fast_livo2", "input_topics": ["/agt/sensors/lidar/custom", "/agt/sensors/imu/data"]},
        "alignment": {"mode": "SITE_CONTROL_POINTS", "status": "PENDING", "identity_requires_manual_confirmation": True},
        "cleaning": {"pipeline": [], "record_required": True},
        "products": {"localization_prior": True, "navigation_occupancy": True, "semantic_base": True},
    }
    (args.output / "recipe.yaml").write_text(yaml.safe_dump(recipe, sort_keys=False, allow_unicode=True), encoding="utf-8")
    print(yaml.safe_dump({"dataset": str(dataset_path), "recipe": str(args.output / "recipe.yaml"), "bag_sha256": binding["bag"]["sha256"], "repository_commit": commit}, sort_keys=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
