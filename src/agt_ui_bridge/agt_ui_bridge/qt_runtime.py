"""Fail-safe preparation for the unmodified ros_qt5_gui_app runtime."""

import json
import math
from pathlib import Path

import yaml

from .map_transform import load_grayscale_map_image


class QtRuntimeError(ValueError):
    pass


def task_library_runtime_keys(path):
    try:
        document = yaml.safe_load(Path(path).read_text(encoding="utf-8"))
    except (OSError, UnicodeError, yaml.YAMLError) as exc:
        raise QtRuntimeError(f"cannot parse task library config: {exc}") from exc
    settings = document.get("task_library") if isinstance(document, dict) else None
    if not isinstance(settings, dict):
        raise QtRuntimeError("task library config must contain task_library settings")
    required = {
        "enabled",
        "maximum_points",
        "maximum_loops",
        "unknown_cell_policy",
        "autosave_enabled",
        "autosave_interval_s",
        "backup_count",
    }
    missing = sorted(required - settings.keys())
    if missing:
        raise QtRuntimeError(f"task library config is missing {missing[0]}")
    if not isinstance(settings["enabled"], bool) or not isinstance(
        settings["autosave_enabled"], bool
    ):
        raise QtRuntimeError("task library enabled settings must be boolean")
    integers = ("maximum_points", "maximum_loops", "autosave_interval_s")
    if any(
        isinstance(settings[key], bool)
        or not isinstance(settings[key], int)
        or settings[key] <= 0
        for key in integers
    ):
        raise QtRuntimeError("task library limits and autosave interval must be positive integers")
    backup_count = settings["backup_count"]
    if isinstance(backup_count, bool) or not isinstance(backup_count, int) or backup_count < 0:
        raise QtRuntimeError("task library backup_count must be a non-negative integer")
    policy = settings["unknown_cell_policy"]
    if policy not in {"reject", "warn", "allow"}:
        raise QtRuntimeError("task library unknown_cell_policy is invalid")
    def boolean_text(value):
        return "true" if value else "false"
    return {
        "TaskLibraryEnabled": boolean_text(settings["enabled"]),
        "TaskMaximumPoints": str(settings["maximum_points"]),
        "TaskMaximumLoops": str(settings["maximum_loops"]),
        "TaskUnknownCellPolicy": policy,
        "TaskAutosaveEnabled": boolean_text(settings["autosave_enabled"]),
        "TaskAutosaveIntervalS": str(settings["autosave_interval_s"]),
        "TaskBackupCount": str(backup_count),
    }


def validate_nav2_map(path):
    yaml_path = Path(path).expanduser()
    if not yaml_path.suffix:
        yaml_path = yaml_path.with_suffix(".yaml")
    elif yaml_path.suffix.lower() != ".yaml":
        raise QtRuntimeError(
            "the unmodified Qt frontend supports navigation maps with a .yaml suffix only"
        )
    yaml_path = yaml_path.resolve()
    if not yaml_path.is_file():
        raise QtRuntimeError(f"map YAML does not exist: {yaml_path}")
    try:
        document = yaml.safe_load(yaml_path.read_text(encoding="utf-8"))
    except (OSError, UnicodeError, yaml.YAMLError) as exc:
        raise QtRuntimeError(f"cannot parse map YAML: {exc}") from exc
    if not isinstance(document, dict):
        raise QtRuntimeError("map YAML must contain a mapping")

    try:
        resolution = float(document["resolution"])
        origin = [float(value) for value in document["origin"]]
        image_value = document["image"]
        negate = int(document["negate"])
        occupied = float(document["occupied_thresh"])
        free = float(document["free_thresh"])
    except (KeyError, TypeError, ValueError) as exc:
        raise QtRuntimeError(
            "map YAML requires image, resolution, origin, negate, "
            "occupied_thresh and free_thresh"
        ) from exc
    if not isinstance(image_value, str) or not image_value:
        raise QtRuntimeError("map image must be a non-empty path")
    if resolution <= 0.0 or not math.isfinite(resolution):
        raise QtRuntimeError("map resolution must be finite and positive")
    if len(origin) != 3 or not all(math.isfinite(value) for value in origin):
        raise QtRuntimeError("map origin must contain three finite values")
    if negate not in {0, 1} or not (0.0 <= free < occupied <= 1.0):
        raise QtRuntimeError("map thresholds or negate value are invalid")

    image_path = Path(image_value).expanduser()
    if not image_path.is_absolute():
        image_path = yaml_path.parent / image_path
    image_path = image_path.resolve()
    if not image_path.is_file():
        raise QtRuntimeError(f"map image does not exist: {image_path}")
    try:
        image = load_grayscale_map_image(image_path)
        width, height = image.size
    except (OSError, ValueError) as exc:
        raise QtRuntimeError(f"map image is unreadable: {exc}") from exc
    if width <= 0 or height <= 0:
        raise QtRuntimeError("map image has invalid dimensions")
    return yaml_path, {
        "resolution": resolution,
        "origin": origin,
        "width": width,
        "height": height,
    }


def validate_topology_for_map(map_yaml, geometry):
    topology_path = Path(map_yaml).with_suffix(".topology")
    if not topology_path.exists():
        return []
    try:
        document = json.loads(topology_path.read_text(encoding="utf-8"))
        points = document.get("points", [])
    except (OSError, UnicodeError, json.JSONDecodeError, AttributeError) as exc:
        return [f"topology file is invalid: {exc}"]
    warnings = []
    map_name = document.get("map_name")
    if map_name and map_name != Path(map_yaml).stem:
        warnings.append(
            f"topology map_name '{map_name}' does not match selected map "
            f"'{Path(map_yaml).stem}'"
        )
    origin_x, origin_y, origin_yaw = geometry["origin"]
    cos_yaw, sin_yaw = math.cos(origin_yaw), math.sin(origin_yaw)
    max_x = geometry["width"] * geometry["resolution"]
    max_y = geometry["height"] * geometry["resolution"]
    for index, point in enumerate(points):
        try:
            dx = float(point["x"]) - origin_x
            dy = float(point["y"]) - origin_y
            local_x = cos_yaw * dx + sin_yaw * dy
            local_y = -sin_yaw * dx + cos_yaw * dy
            valid = 0.0 <= local_x < max_x and 0.0 <= local_y < max_y
        except (KeyError, TypeError, ValueError):
            valid = False
        if not valid:
            warnings.append(f"topology point {index} lies outside the selected map")
    return warnings


def prepare_runtime_config(
    config_path,
    template_path,
    requested_map=None,
    runtime_maps_root=None,
    task_library_config=None,
):
    config_path = Path(config_path)
    template_path = Path(template_path)
    template = json.loads(template_path.read_text(encoding="utf-8"))
    try:
        config = json.loads(config_path.read_text(encoding="utf-8"))
    except (OSError, UnicodeError, json.JSONDecodeError):
        config = template

    # Capability and frame keys are versioned profile contract. Add newly
    # introduced keys without overwriting operator-persisted values.
    runtime_keys = config.setdefault("key_value", {})
    runtime_keys.pop("TaskLineCheckStepRatio", None)
    defaults = dict(template.get("key_value", {}))
    if task_library_config:
        task_library_enabled = defaults.get("TaskLibraryEnabled", "true")
        defaults.update(task_library_runtime_keys(task_library_config))
        defaults["TaskLibraryEnabled"] = task_library_enabled
    profile_owned_keys = {
        "EnableTaskExecution",
        "EnableCostmapDisplay",
        "EnableOfflinePlanningPreview",
        "EnableManualControl",
        "EnableBaseMapEditing",
        "EnableBaseMapSaveAs",
        "EnableMapOpen",
        "EnableLegacyTopologyTasks",
        "TaskLibraryEnabled",
    }
    for key, value in defaults.items():
        if key in profile_owned_keys:
            runtime_keys[key] = value
        else:
            runtime_keys.setdefault(key, value)
    if runtime_maps_root:
        runtime_keys["TaskLibraryRoot"] = str(Path(runtime_maps_root).expanduser().resolve())
    if runtime_keys.get("EnableCostmapDisplay", "false") != "true":
        for display in config.get("display_config", []):
            if display.get("display_name") in {
                "kGlobalCostMap",
                "kLocalCostMap",
            }:
                display["visible"] = False

    map_value = requested_map or config.get("map_config", {}).get("path", "")
    warnings = []
    if map_value:
        try:
            map_yaml, geometry = validate_nav2_map(map_value)
        except QtRuntimeError as exc:
            if requested_map:
                raise
            warnings.append(f"cleared unsafe persisted map path: {exc}")
            config.setdefault("map_config", {})["path"] = ""
        else:
            config.setdefault("map_config", {})["path"] = str(map_yaml.with_suffix(""))
            warnings.extend(validate_topology_for_map(map_yaml, geometry))
    config_path.write_text(
        json.dumps(config, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
    )
    return warnings
