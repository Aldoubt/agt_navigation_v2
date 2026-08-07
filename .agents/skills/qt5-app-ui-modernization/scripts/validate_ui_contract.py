#!/usr/bin/env python3
"""Validate the replaceable Qt shell, profiles, and vendored-fork contract."""

from __future__ import annotations

import json
import os
import re
import subprocess
import sys
from pathlib import Path


SKILL_DIR = Path(__file__).resolve().parents[1]
REPO_ROOT = Path(__file__).resolve().parents[4]
QT_ROOT = REPO_ROOT / "third_party" / "ros_qt5_gui_app"
CONFIG_ROOT = REPO_ROOT / "src" / "agt_ui_bridge" / "config"


def load_json(path: Path, errors: list[str]) -> dict:
    try:
        return json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as error:
        errors.append(f"{path.relative_to(REPO_ROOT)}: {error}")
        return {}


def require(condition: bool, message: str, errors: list[str]) -> None:
    if not condition:
        errors.append(message)


def validate_skill(errors: list[str]) -> None:
    text = (SKILL_DIR / "SKILL.md").read_text(encoding="utf-8")
    require(text.startswith("---\n"), "SKILL.md: missing YAML frontmatter", errors)
    require("name: qt5-app-ui-modernization" in text,
            "SKILL.md: incorrect name", errors)
    required_description = (
        "description: Use when redesigning, polishing, restructuring, theming, "
        "or replacing the Qt5 Widgets frontend"
    )
    require(required_description in text,
            "SKILL.md: trigger description is incomplete", errors)
    for relative in (
        "agents/openai.yaml",
        "references/agt-ui-contracts.md",
        "references/design-system.md",
        "references/qt5-widgets-patterns.md",
    ):
        require((SKILL_DIR / relative).is_file(), f"skill: missing {relative}", errors)


def validate_themes(errors: list[str]) -> None:
    required_tokens = {
        "background", "surface", "text", "mutedText", "border", "accent",
        "success", "warning", "danger",
    }
    for theme_id in ("agt-light", "agt-dark"):
        directory = QT_ROOT / "resources" / "themes" / theme_id
        manifest = load_json(directory / "theme.json", errors)
        qss = directory / "theme.qss"
        require(qss.is_file(), f"theme {theme_id}: missing theme.qss", errors)
        require(manifest.get("id") == theme_id,
                f"theme {theme_id}: manifest id mismatch", errors)
        tokens = manifest.get("tokens", {})
        require(required_tokens <= set(tokens),
                f"theme {theme_id}: missing required tokens", errors)
        if qss.is_file():
            stylesheet = qss.read_text(encoding="utf-8")
            for token in required_tokens:
                require(f"@{token}" in stylesheet,
                        f"theme {theme_id}: QSS does not consume @{token}", errors)


def validate_profiles(errors: list[str]) -> None:
    common = {
        "UiThemeId": "agt-light",
        "UiLayoutId": "control-center-v1",
        "UiDensity": "comfortable",
        "ShowAdvancedDiagnostics": "false",
    }
    capabilities = {
        "mapping": ("true", "false", "false", "false"),
        "candidate": ("false", "false", "false", "false"),
        "navigation": ("false", "true", "true", "false"),
        "offline": ("false", "false", "false", "false"),
        "teach": ("false", "false", "false", "true"),
    }
    configs: dict[str, dict] = {}
    for name, expected in capabilities.items():
        data = load_json(CONFIG_ROOT / f"ros_qt5_gui_{name}.json", errors)
        configs[name] = data
        keys = data.get("key_value", {})
        for key, value in common.items():
            require(keys.get(key) == value,
                    f"profile {name}: expected {key}={value}", errors)
        actual = tuple(keys.get(key) for key in (
            "EnableMappingSessionControl", "EnableRelocalization",
            "EnableMapManager", "EnableBagManager",
        ))
        require(actual == expected,
                f"profile {name}: manager capabilities {actual} != {expected}", errors)
        require(keys.get("EnableDebugGoalPose") == "false",
                f"profile {name}: debug goal must default false", errors)

    for name in ("navigation", "offline", "teach"):
        keys = configs[name].get("key_value", {})
        require(keys.get("EnableBaseMapEditing") == "false",
                f"profile {name}: READY raster editing must be disabled", errors)
        require(keys.get("EnableBaseMapSaveAs") == "false",
                f"profile {name}: READY save-as must be disabled", errors)
    for name in ("offline", "teach"):
        require(configs[name].get("key_value", {}).get("EnableTaskExecution") == "false",
                f"profile {name}: task execution must be disabled", errors)
    candidate = configs["candidate"].get("key_value", {})
    require(candidate.get("EnableMapOpen") == "false",
            "profile candidate: map open must be disabled", errors)
    require(candidate.get("EnableBaseMapSaveAs") == "false",
            "profile candidate: save-as must be disabled", errors)

    forbidden = {"/agt/chassis/cmd_vel", "/agt/safety/cmd_vel"}
    for path in CONFIG_ROOT.glob("ros_qt5_gui_*.json"):
        data = load_json(path, errors)
        topics = {item.get("topic") for item in data.get("display_config", [])}
        require(not (topics & forbidden),
                f"{path.name}: contains forbidden command topic", errors)


def validate_qt_sources(errors: list[str]) -> None:
    channel_path = QT_ROOT / "src" / "channel" / "ros2" / "rclcomm.cpp"
    channel = channel_path.read_text(encoding="utf-8")
    for endpoint in (
        "/agt/system/robot_state", "/agt/missions/execute",
        "/agt/mapping/manage_session", "/agt/localization/relocalize",
        "/agt/maps/list", "/agt/maps/manage", "/agt/data/bags/list",
        "/agt/data/bags/manage",
    ):
        require(endpoint in channel, f"ROS2 channel: missing {endpoint}", errors)
    for forbidden in ("/agt/chassis/cmd_vel", "/agt/safety/cmd_vel",
                      "std::system(", "ros2 launch"):
        require(forbidden not in channel,
                f"ROS2 channel: forbidden text {forbidden}", errors)
    require('GET_CONFIG_VALUE("EnableDebugGoalPose", "false")' in channel,
            "ROS2 channel: /goal_pose is not fail-closed", errors)
    require(
        'SET_DEFAULT_TOPIC_NAME(MSG_ID_SET_ROBOT_SPEED, "/agt/cmd_vel_manual")'
        in channel,
        "ROS2 channel: manual velocity default must use /agt/cmd_vel_manual",
        errors,
    )
    for source in (QT_ROOT / "src" / "app" / "ui").rglob("*.cpp"):
        text = source.read_text(encoding="utf-8")
        require("QStringLiteral(\"" not in text or not re.search(
            r'QStringLiteral\("[^"\n]*[\u4e00-\u9fff]', text),
            f"{source.relative_to(QT_ROOT)}: Chinese text bypasses UiLanguage", errors)


def validate_pin(errors: list[str]) -> None:
    marker_path = QT_ROOT / ".agt-fork-commit"
    require(marker_path.is_file(), "vendored Qt snapshot: missing .agt-fork-commit", errors)
    if not marker_path.is_file():
        return
    marker = marker_path.read_text(encoding="ascii").strip()
    require(bool(re.fullmatch(r"[0-9a-f]{40}", marker)),
            "vendored Qt snapshot: pin is not a full lowercase SHA", errors)
    readme = (REPO_ROOT / "third_party" / "README.md").read_text(encoding="utf-8")
    require(marker in readme, "third_party/README.md: Qt SHA differs from marker", errors)
    require((QT_ROOT / "LICENSE").is_file(), "vendored Qt snapshot: GPL LICENSE missing", errors)

    fork = os.environ.get("AGT_QT_FORK", "")
    if fork:
        fork_path = Path(fork).resolve()
        try:
            head = subprocess.check_output(
                ["git", "-C", str(fork_path), "rev-parse", "HEAD"], text=True
            ).strip()
        except (OSError, subprocess.CalledProcessError) as error:
            errors.append(f"AGT_QT_FORK: cannot read HEAD: {error}")
        else:
            require(head == marker, "AGT_QT_FORK HEAD differs from vendored pin", errors)


def main() -> int:
    errors: list[str] = []
    validate_skill(errors)
    validate_themes(errors)
    validate_profiles(errors)
    validate_qt_sources(errors)
    validate_pin(errors)
    if errors:
        for error in errors:
            print(f"ERROR: {error}", file=sys.stderr)
        return 1
    print("Qt5 UI contract validation passed")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
