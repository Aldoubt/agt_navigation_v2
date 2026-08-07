"""Shared, fail-closed task dispatch gate."""

from dataclasses import dataclass, field
from typing import Any


@dataclass
class ReadinessInputs:
    active_mode: str = "IDLE"
    map_id: str = ""
    map_version_id: str = ""
    map_ready: bool = False
    navigation_map_valid: bool = False
    localization_pcd_valid: bool = False
    active_map_hash: str = ""
    localization_map_id: str = ""
    localization_map_hash: str = ""
    localization_state: str = "UNINITIALIZED"
    pose_valid: bool = False
    localization_accepted: bool = False
    status_stale: bool = True
    emergency_stop: bool = True
    chassis_connected: bool = False
    safety_allows_navigation: bool = False
    nav2_active: bool = False
    map_to_odom_fresh: bool = False
    odom_to_base_fresh: bool = False
    base_to_lidar_fresh: bool = False
    tf_chain_fresh: bool = False  # derived compatibility field
    task_valid: bool = False
    sensor_input_ready: bool = True
    health_revision: int = 0
    warnings: list[tuple[str, str]] = field(default_factory=list)


@dataclass(frozen=True)
class ReadinessResult:
    ready: bool
    active_mode: str
    map_id: str
    map_version_id: str
    localization_state: str
    health_revision: int
    blocker_codes: list[str]
    blocker_messages: list[str]
    warning_codes: list[str]
    warning_messages: list[str]


def evaluate_task_readiness(inputs: ReadinessInputs, *, gate_profile: int = 0) -> ReadinessResult:
    blockers: list[tuple[str, str]] = []

    def require(condition: bool, code: str, message: str) -> None:
        if not condition:
            blockers.append((code, message))

    if gate_profile not in (0, 1):
        return ReadinessResult(
            ready=False, active_mode=inputs.active_mode, map_id=inputs.map_id,
            map_version_id=inputs.map_version_id, localization_state=inputs.localization_state,
            health_revision=inputs.health_revision, blocker_codes=["INVALID_READINESS_PROFILE"],
            blocker_messages=["readiness gate profile is not supported"], warning_codes=[], warning_messages=[])

    require(inputs.active_mode == "NAVIGATION", "MODE_NOT_NAVIGATION", "active mode is not NAVIGATION")
    require(bool(inputs.map_id and inputs.map_version_id), "MAP_ID_MISSING", "active map identity is incomplete")
    require(inputs.map_ready, "MAP_NOT_READY", "active map version is not READY")
    require(inputs.navigation_map_valid, "NAV_MAP_INVALID", "navigation PGM/YAML is missing or hash-invalid")
    require(inputs.localization_pcd_valid, "LOCALIZATION_PCD_INVALID", "localization PCD is missing or hash-invalid")
    require(
        bool(inputs.active_map_hash)
        and inputs.localization_map_id == inputs.map_id
        and inputs.localization_map_hash == inputs.active_map_hash,
        "LOCALIZATION_MAP_MISMATCH",
        "localization map identity does not match the active map",
    )
    if gate_profile == 0:
        require(inputs.localization_state == "TRACKING", "LOCALIZATION_NOT_TRACKING", "localization is not TRACKING")
        require(inputs.pose_valid, "POSE_INVALID", "localization pose_valid is false")
        require(inputs.localization_accepted, "LOCALIZATION_NOT_ACCEPTED", "localization quality gate is not accepted")
        require(not inputs.status_stale, "LOCALIZATION_STATUS_STALE", "localization status is stale")
    require(not inputs.emergency_stop, "EMERGENCY_STOP", "emergency stop is active")
    require(inputs.chassis_connected, "CHASSIS_DISCONNECTED", "chassis is not connected")
    require(inputs.safety_allows_navigation, "SAFETY_NOT_READY", "agt_safety does not allow navigation input")
    require(inputs.nav2_active, "NAV2_NOT_ACTIVE", "required Nav2 lifecycle nodes are not active")
    require(inputs.odom_to_base_fresh, "ODOM_TO_BASE_TF_NOT_FRESH", "odom -> base_footprint TF is missing or stale")
    require(inputs.base_to_lidar_fresh, "BASE_TO_LIDAR_TF_NOT_FRESH", "base_link -> lidar_link TF is missing or stale")
    if gate_profile == 0:
        require(inputs.map_to_odom_fresh, "MAP_TO_ODOM_TF_NOT_FRESH", "map -> odom TF is missing or stale")
    require(inputs.task_valid, "TASK_INVALID", "task has not passed the existing validator")
    require(inputs.sensor_input_ready, "SENSOR_INPUT_UNHEALTHY", "required sensor input health evidence is not ready")

    return ReadinessResult(
        ready=not blockers,
        active_mode=inputs.active_mode,
        map_id=inputs.map_id,
        map_version_id=inputs.map_version_id,
        localization_state=inputs.localization_state,
        health_revision=inputs.health_revision,
        blocker_codes=[code for code, _ in blockers],
        blocker_messages=[message for _, message in blockers],
        warning_codes=[code for code, _ in inputs.warnings],
        warning_messages=[message for _, message in inputs.warnings],
    )
