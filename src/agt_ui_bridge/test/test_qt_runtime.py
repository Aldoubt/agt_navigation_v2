import json
from pathlib import Path
import sys

from PIL import Image
import pytest
import yaml

PACKAGE_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_ROOT))

from agt_ui_bridge.qt_runtime import (  # noqa: E402
    QtRuntimeError,
    prepare_runtime_config,
    task_library_runtime_keys,
    validate_nav2_map,
    validate_topology_for_map,
)


def _map(tmp_path):
    image = tmp_path / "map.pgm"
    Image.new("L", (20, 10), 255).save(image)
    yaml_path = tmp_path / "map.yaml"
    yaml_path.write_text(
        yaml.safe_dump(
            {
                "image": image.name,
                "resolution": 0.5,
                "origin": [-2.0, -1.0, 0.0],
                "negate": 0,
                "occupied_thresh": 0.65,
                "free_thresh": 0.196,
                "mode": "trinary",
            }
        ),
        encoding="utf-8",
    )
    return yaml_path


def _config(path, map_path=""):
    path.write_text(json.dumps({"map_config": {"path": map_path}}), encoding="utf-8")


def test_valid_nav2_map_is_accepted(tmp_path):
    yaml_path, geometry = validate_nav2_map(_map(tmp_path))
    assert yaml_path == tmp_path / "map.yaml"
    assert (geometry["width"], geometry["height"]) == (20, 10)


def test_non_nav2_yaml_is_rejected(tmp_path):
    path = tmp_path / "parameters.yaml"
    path.write_text("node: {ros__parameters: {value: 1}}\n", encoding="utf-8")
    with pytest.raises(QtRuntimeError, match="requires"):
        validate_nav2_map(path)


def test_bad_persisted_map_is_cleared_before_gui_starts(tmp_path):
    config = tmp_path / "config.json"
    template = tmp_path / "template.json"
    _config(config, "/missing/map")
    _config(template)
    warnings = prepare_runtime_config(config, template)
    assert json.loads(config.read_text())["map_config"]["path"] == ""
    assert warnings and "cleared unsafe" in warnings[0]


def test_explicit_bad_map_fails_closed(tmp_path):
    config = tmp_path / "config.json"
    template = tmp_path / "template.json"
    _config(config)
    _config(template)
    with pytest.raises(QtRuntimeError):
        prepare_runtime_config(config, template, requested_map="/missing/map.yaml")


def test_explicit_map_is_persisted_without_extension_for_vendor(tmp_path):
    config = tmp_path / "config.json"
    template = tmp_path / "template.json"
    _config(config)
    _config(template)
    map_yaml = _map(tmp_path)
    assert prepare_runtime_config(config, template, requested_map=map_yaml) == []
    assert json.loads(config.read_text())["map_config"]["path"] == str(
        map_yaml.with_suffix("")
    )


def test_missing_profile_contract_keys_are_merged_without_overwriting_user_values(
    tmp_path,
):
    config = tmp_path / "config.json"
    template = tmp_path / "template.json"
    config.write_text(
        json.dumps(
            {
                "map_config": {"path": ""},
                "key_value": {
                    "FixedFrameId": "custom_map",
                    "TaskLineCheckStepRatio": "0.5",
                },
                "display_config": [
                    {"display_name": "kGlobalCostMap", "visible": True},
                    {"display_name": "kOccupancyMap", "visible": True},
                ],
            }
        ),
        encoding="utf-8",
    )
    template.write_text(
        json.dumps(
            {
                "map_config": {"path": ""},
                "key_value": {
                    "FixedFrameId": "map",
                    "EnableTaskExecution": "true",
                    "EnableCostmapDisplay": "false",
                },
            }
        ),
        encoding="utf-8",
    )

    prepare_runtime_config(config, template)

    keys = json.loads(config.read_text(encoding="utf-8"))["key_value"]
    assert keys == {
        "FixedFrameId": "custom_map",
        "EnableTaskExecution": "true",
        "EnableCostmapDisplay": "false",
    }
    displays = json.loads(config.read_text(encoding="utf-8"))["display_config"]
    assert displays == [
        {"display_name": "kGlobalCostMap", "visible": False},
        {"display_name": "kOccupancyMap", "visible": True},
    ]


def test_stale_topology_is_reported(tmp_path):
    map_yaml = _map(tmp_path)
    _, geometry = validate_nav2_map(map_yaml)
    map_yaml.with_suffix(".topology").write_text(
        json.dumps({"points": [{"name": "old", "x": 100.0, "y": 100.0}]}),
        encoding="utf-8",
    )
    warnings = validate_topology_for_map(map_yaml, geometry)
    assert warnings == ["topology point 0 lies outside the selected map"]


def test_task_library_yaml_supplies_versioned_runtime_defaults(tmp_path):
    task_config = tmp_path / "task_library.yaml"
    task_config.write_text(
        yaml.safe_dump(
            {
                "task_library": {
                    "enabled": True,
                    "maximum_points": 123,
                    "maximum_loops": 7,
                    "unknown_cell_policy": "warn",
                    "autosave_enabled": False,
                    "autosave_interval_s": 12,
                    "backup_count": 3,
                }
            }
        ),
        encoding="utf-8",
    )
    keys = task_library_runtime_keys(task_config)
    assert keys["TaskMaximumPoints"] == "123"
    assert keys["TaskUnknownCellPolicy"] == "warn"
    assert keys["TaskAutosaveEnabled"] == "false"

    config = tmp_path / "config.json"
    template = tmp_path / "template.json"
    _config(config)
    _config(template)
    prepare_runtime_config(
        config,
        template,
        runtime_maps_root=tmp_path / "maps",
        task_library_config=task_config,
    )
    runtime = json.loads(config.read_text(encoding="utf-8"))["key_value"]
    assert runtime["TaskMaximumLoops"] == "7"
    assert runtime["TaskLibraryRoot"] == str((tmp_path / "maps").resolve())


def test_profile_owned_capabilities_replace_stale_persisted_values(tmp_path):
    config = tmp_path / "config.json"
    template = tmp_path / "template.json"
    config.write_text(
        json.dumps(
            {
                "map_config": {"path": ""},
                "key_value": {
                    "EnableBaseMapEditing": "true",
                    "EnableMapOpen": "true",
                    "TaskLibraryEnabled": "true",
                },
            }
        ),
        encoding="utf-8",
    )
    template.write_text(
        json.dumps(
            {
                "map_config": {"path": ""},
                "key_value": {
                    "EnableBaseMapEditing": "false",
                    "EnableMapOpen": "false",
                    "TaskLibraryEnabled": "false",
                },
            }
        ),
        encoding="utf-8",
    )

    prepare_runtime_config(config, template)

    keys = json.loads(config.read_text(encoding="utf-8"))["key_value"]
    assert keys["EnableBaseMapEditing"] == "false"
    assert keys["EnableMapOpen"] == "false"
    assert keys["TaskLibraryEnabled"] == "false"
