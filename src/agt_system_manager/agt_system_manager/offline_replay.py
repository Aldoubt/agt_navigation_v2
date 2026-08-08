"""Preflight and deterministic command planning for bag-driven map derivation."""

from dataclasses import dataclass
from pathlib import Path
from typing import Any

import yaml

from agt_offline_assets import (
    AssetContractError,
    DatasetBinding,
    sha256_file,
    sha256_path_bundle,
    validate_map_workspace,
)


MAPPING_INPUT_TOPICS = (
    "/clock",
    "/tf_static",
    "/agt/sensors/lidar/custom",
    "/agt/sensors/imu/data",
)


@dataclass(frozen=True)
class OfflineReplayPlan:
    workspace_manifest: Path
    workspace_root: Path
    map_id: str
    source_bag: Path
    source_bag_sha256: str
    platform_profile: Path
    start_arguments: dict[str, str]
    playback_command: tuple[str, ...]
    replay_log: Path


def _yaml(path: Path) -> dict[str, Any]:
    try:
        value = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
    except (OSError, UnicodeError, yaml.YAMLError) as exc:
        raise AssetContractError("offline_replay_yaml_invalid", f"unreadable YAML: {path}") from exc
    if not isinstance(value, dict):
        raise AssetContractError("offline_replay_yaml_invalid", f"YAML must be a mapping: {path}")
    return value


def prepare_offline_replay_plan(
    workspace_manifest: str | Path,
    *,
    source_bag: str | Path,
    platform_profile: str | Path,
    playback_rate: float = 1.0,
    user_config_path: str | Path | None = None,
) -> OfflineReplayPlan:
    """Validate immutable inputs before any mapping/playback process is started."""
    manifest_path = Path(workspace_manifest).expanduser().resolve()
    compliance = validate_map_workspace(manifest_path)
    if not compliance.valid:
        raise AssetContractError(
            "offline_replay_workspace_invalid",
            "PROCESSING workspace failed compliance audit: " + ", ".join(compliance.errors),
        )
    manifest = _yaml(manifest_path)
    if str(manifest.get("state", "")).upper() not in {"PROCESSING", "DRAFT"}:
        raise AssetContractError(
            "offline_replay_workspace_state_invalid",
            "offline replay requires a DRAFT or PROCESSING map workspace",
        )
    if "mapping_session_handoff" in (manifest.get("assets") or {}):
        raise AssetContractError(
            "offline_replay_already_ingested",
            "workspace already contains mapping-session evidence",
        )

    root = manifest_path.parent
    source_block = manifest.get("source") or {}
    dataset_path = root / str(source_block.get("dataset_binding", ""))
    dataset = DatasetBinding.from_file(dataset_path)

    bag = Path(source_bag).expanduser().resolve()
    if not bag.is_dir() or not (bag / "metadata.yaml").is_file():
        raise AssetContractError(
            "offline_replay_bag_invalid", "source bag must be a complete rosbag2 directory"
        )
    actual_bag_hash = sha256_path_bundle(bag)
    if actual_bag_hash != dataset.bag_sha256:
        raise AssetContractError(
            "offline_replay_bag_hash_mismatch",
            "source bag bytes differ from the Dataset binding",
        )

    profile = Path(platform_profile).expanduser().resolve()
    if not profile.is_file():
        raise AssetContractError(
            "offline_replay_platform_missing", "platform profile is missing"
        )
    if sha256_file(profile) != str(manifest.get("platform_profile_sha256", "")):
        raise AssetContractError(
            "offline_replay_platform_mismatch",
            "platform profile differs from the map workspace lineage",
        )

    try:
        rate = float(playback_rate)
    except (TypeError, ValueError) as exc:
        raise AssetContractError(
            "offline_replay_rate_invalid", "playback rate must be numeric"
        ) from exc
    if not 0.1 <= rate <= 4.0:
        raise AssetContractError(
            "offline_replay_rate_invalid", "playback rate must be between 0.1 and 4.0"
        )

    arguments = {
        "platform_profile": str(profile),
        "start_sensor": "false",
        "start_chassis": "false",
        "start_chassis_monitor": "false",
        "start_rviz": "false",
        "start_mapping_gui": "false",
        "use_sim_time": "true",
    }
    if user_config_path is not None:
        config = Path(user_config_path).expanduser().resolve()
        if not config.is_file():
            raise AssetContractError(
                "offline_replay_config_missing", "user_config_path is missing"
            )
        arguments["user_config_path"] = str(config)

    command = (
        "ros2",
        "bag",
        "play",
        "--clock",
        "--rate",
        f"{rate:g}",
        str(bag),
        "--topics",
        *MAPPING_INPUT_TOPICS,
    )
    return OfflineReplayPlan(
        workspace_manifest=manifest_path,
        workspace_root=root,
        map_id=str(manifest.get("map_id", "")),
        source_bag=bag,
        source_bag_sha256=actual_bag_hash,
        platform_profile=profile,
        start_arguments=arguments,
        playback_command=command,
        replay_log=root / "processing" / "source_replay.log",
    )
