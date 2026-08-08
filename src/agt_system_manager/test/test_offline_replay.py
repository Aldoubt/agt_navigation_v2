from pathlib import Path

import pytest
import yaml

from agt_offline_assets import AssetContractError, create_map_workspace, sha256_file, sha256_path_bundle
from agt_system_manager.offline_replay import MAPPING_INPUT_TOPICS, prepare_offline_replay_plan


ROOT = Path(__file__).resolve().parents[3]
PLATFORM = ROOT / "profiles" / "platforms" / "bunker.yaml"


def _workspace(tmp_path: Path):
    bag = tmp_path / "bag"
    bag.mkdir()
    (bag / "metadata.yaml").write_text(
        "rosbag2_bagfile_information: {}\n", encoding="utf-8"
    )
    (bag / "data_0.db3").write_bytes(b"offline-replay-source")

    calibration = tmp_path / "calibration.yaml"
    calibration.write_text("calibration_id: cal_replay\n", encoding="utf-8")
    dataset = tmp_path / "dataset.yaml"
    dataset.write_text(
        yaml.safe_dump(
            {
                "schema_version": 1,
                "dataset_id": "ds_replay",
                "site_id": "greenhouse_replay",
                "epoch_id": "2026-08-08-replay",
                "purpose": "OPERATIONAL",
                "bag": {"path": str(bag), "sha256": sha256_path_bundle(bag)},
                "platform": {
                    "profile_id": "bunker",
                    "profile_sha256": sha256_file(PLATFORM),
                },
                "calibration": {
                    "calibration_id": "cal_replay",
                    "calibration_sha256": sha256_file(calibration),
                },
            },
            sort_keys=False,
        ),
        encoding="utf-8",
    )
    recipe = tmp_path / "recipe.yaml"
    recipe.write_text(
        yaml.safe_dump(
            {
                "schema_version": 1,
                "recipe_id": "recipe_replay",
                "source_dataset_id": "ds_replay",
                "source_dataset_sha256": sha256_file(dataset),
                "calibration_id": "cal_replay",
                "calibration_sha256": sha256_file(calibration),
                "platform_profile": "bunker",
                "platform_profile_sha256": sha256_file(PLATFORM),
                "repository_commit": "deadbeef",
                "random_seed": 0,
                "mapping": {
                    "backend": "fast_livo2",
                    "config_sha256": "sha256:" + "2" * 64,
                },
                "trajectory_optimization": {"backend": "none"},
                "alignment": {"mode": "SITE_CONTROL_POINTS"},
                "cleaning": {"pipeline": []},
                "products": {
                    "localization_prior": True,
                    "navigation_occupancy": True,
                    "semantic_base": True,
                },
            },
            sort_keys=False,
        ),
        encoding="utf-8",
    )
    site_frame = tmp_path / "site_frame.yaml"
    site_frame.write_text(
        yaml.safe_dump(
            {"schema_version": 1, "site_id": "greenhouse_replay", "frame_id": "map"}
        ),
        encoding="utf-8",
    )
    alignment = tmp_path / "alignment.yaml"
    alignment.write_text(
        yaml.safe_dump(
            {
                "schema_version": 1,
                "site_id": "greenhouse_replay",
                "epoch_id": "2026-08-08-replay",
                "map_frame": "map",
                "source_frame": "mapping_session",
                "method": "SITE_CONTROL_POINTS",
                "transform": {
                    "translation": [0.0, 0.0, 0.0],
                    "quaternion_xyzw": [0.0, 0.0, 0.0, 1.0],
                },
            },
            sort_keys=False,
        ),
        encoding="utf-8",
    )
    workspace = create_map_workspace(
        tmp_path / "maps",
        map_id="greenhouse_replay",
        map_version_id="map_20260808_120000_1234abcd",
        dataset_binding_path=dataset,
        recipe_path=recipe,
        site_frame_path=site_frame,
        alignment_path=alignment,
        platform_profile_path=PLATFORM,
        calibration_path=calibration,
    )
    return workspace, bag


def test_offline_replay_plan_is_mapping_only_and_deterministic(tmp_path):
    workspace, bag = _workspace(tmp_path)
    plan = prepare_offline_replay_plan(
        workspace.manifest_path,
        source_bag=bag,
        platform_profile=PLATFORM,
        playback_rate=0.5,
    )
    assert plan.map_id == "greenhouse_replay"
    assert plan.source_bag_sha256 == sha256_path_bundle(bag)
    assert plan.start_arguments["start_sensor"] == "false"
    assert plan.start_arguments["start_chassis"] == "false"
    assert plan.start_arguments["start_rviz"] == "false"
    assert plan.start_arguments["use_sim_time"] == "true"
    assert plan.playback_command[:6] == (
        "ros2", "bag", "play", "--clock", "--rate", "0.5"
    )
    assert plan.playback_command[-len(MAPPING_INPUT_TOPICS):] == MAPPING_INPUT_TOPICS
    assert "/agt/mapping/registered_points" not in MAPPING_INPUT_TOPICS


def test_offline_replay_rejects_wrong_bag_before_starting_ros(tmp_path):
    workspace, _ = _workspace(tmp_path)
    wrong = tmp_path / "wrong_bag"
    wrong.mkdir()
    (wrong / "metadata.yaml").write_text(
        "rosbag2_bagfile_information: {}\n", encoding="utf-8"
    )
    (wrong / "data_0.db3").write_bytes(b"different")
    with pytest.raises(AssetContractError) as error:
        prepare_offline_replay_plan(
            workspace.manifest_path,
            source_bag=wrong,
            platform_profile=PLATFORM,
        )
    assert error.value.code == "offline_replay_bag_hash_mismatch"


def test_offline_replay_rejects_wrong_platform_before_starting_ros(tmp_path):
    workspace, bag = _workspace(tmp_path)
    wrong_profile = tmp_path / "wrong_platform.yaml"
    wrong_profile.write_text("platform:\n  name: wrong\n", encoding="utf-8")
    with pytest.raises(AssetContractError) as error:
        prepare_offline_replay_plan(
            workspace.manifest_path,
            source_bag=bag,
            platform_profile=wrong_profile,
        )
    assert error.value.code == "offline_replay_platform_mismatch"
