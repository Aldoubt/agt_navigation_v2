"""Pure snapshot helpers used by the RobotState ROS adapter."""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any, Mapping


MODE_VALUES = {
    "IDLE": 1,
    "SENSOR_ONLY": 2,
    "MAPPING": 3,
    "LOCALIZATION_DEBUG": 4,
    "NAVIGATION": 5,
    "ERROR": 6,
}


def mode_value(name: str) -> int:
    return MODE_VALUES.get(str(name).upper(), 0)


def load_process_status(path: str | Path) -> tuple[int, int, str]:
    target = Path(path)
    if not target.is_file():
        return 0, 0, ""
    try:
        value = json.loads(target.read_text(encoding="utf-8"))
    except (OSError, UnicodeError, json.JSONDecodeError):
        return 0, 0, ""
    if not isinstance(value, list):
        return 0, 0, ""
    valid = [item for item in value if isinstance(item, dict)]
    running = [item for item in valid if item.get("returncode") is None]
    profiles = sorted(
        {str(item.get("profile", "")) for item in running if item.get("profile")}
    )
    return len(valid), len(running), ",".join(profiles)


def diagnostic_values(message: Any, name: str) -> Mapping[str, str] | None:
    for status in getattr(message, "status", []):
        if getattr(status, "name", "") != name:
            continue
        return {
            str(item.key): str(item.value).strip().lower()
            for item in getattr(status, "values", [])
        }
    return None


def uint8_value(value: Any, default: int = 0) -> int:
    if isinstance(value, (bytes, bytearray)):
        return int(value[0]) if value else default
    try:
        return int(value)
    except (TypeError, ValueError):
        return default


def parse_safety_status(message: Any) -> dict[str, bool]:
    values = diagnostic_values(message, "agt_safety/tracked_controller")
    required = {"motion_enabled", "emergency_stop", "estop_latched", "navigation_ready"}
    if values is None or not required.issubset(values):
        return {
            "known": False,
            "motion_enabled": False,
            "emergency_stop": False,
            "estop_latched": False,
            "navigation_ready": False,
        }
    parsed = {key: values[key] == "true" for key in required}
    return {
        "known": True,
        "motion_enabled": parsed["motion_enabled"],
        "emergency_stop": parsed["emergency_stop"],
        "estop_latched": parsed["estop_latched"],
        "navigation_ready": (
            parsed["navigation_ready"]
            and parsed["motion_enabled"]
            and not parsed["emergency_stop"]
            and not parsed["estop_latched"]
        ),
    }


def parse_chassis_status(message: Any) -> dict[str, Any]:
    values = diagnostic_values(message, "agt_chassis/bunker")
    statuses = [
        item
        for item in getattr(message, "status", [])
        if getattr(item, "name", "") == "agt_chassis/bunker"
    ]
    if values is None or not statuses:
        return {"known": False, "connected": False, "control_mode": 0}
    connected = uint8_value(getattr(statuses[0], "level", 2), 2) < 2
    try:
        raw_mode = int(values.get("control_mode", "-1"))
    except ValueError:
        raw_mode = -1
    # BUNKER reports remote/command control as 0/1. Unknown vendor values stay UNKNOWN.
    control_mode = {0: 1, 1: 2}.get(raw_mode, 0)
    return {"known": True, "connected": connected, "control_mode": control_mode}


def nav2_state_from_health(health: Any, active_mode: str) -> int:
    if str(active_mode).upper() != "NAVIGATION":
        return 1
    for component in getattr(health, "components", []):
        if getattr(component, "component_id", "") != "nav2":
            continue
        if (
            getattr(component, "present", False)
            and not getattr(component, "missing_nodes", [])
            and not getattr(component, "lifecycle_failures", [])
            and uint8_value(getattr(component, "state", 3), 3) in {1, 2}
        ):
            return 2
        return 3
    return 0
