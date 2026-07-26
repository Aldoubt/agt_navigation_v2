"""Deterministic direction and turn annotations for a processed teach path."""

from dataclasses import dataclass
import bisect
import json
import math
from pathlib import Path

from .path_types import TeachRepeatError, wrap_angle


ANNOTATION_SCHEMA_VERSION = 1
EVENT_TYPES = {
    "START",
    "END",
    "TURN_LEFT",
    "TURN_RIGHT",
    "U_TURN_LEFT",
    "U_TURN_RIGHT",
    "IN_PLACE_LEFT",
    "IN_PLACE_RIGHT",
}


@dataclass(frozen=True)
class RouteAnnotationConfig:
    direction_spacing_m: float = 4.0
    turn_window_m: float = 2.5
    u_turn_window_m: float = 7.5
    minimum_turn_angle_rad: float = math.radians(35.0)
    u_turn_angle_rad: float = math.radians(135.0)
    event_cluster_gap_m: float = 0.75
    event_separation_m: float = 2.0
    in_place_max_translation_m: float = 0.05
    in_place_min_yaw_change_rad: float = math.radians(30.0)

    def __post_init__(self):
        positive = (
            self.direction_spacing_m,
            self.turn_window_m,
            self.u_turn_window_m,
            self.minimum_turn_angle_rad,
            self.u_turn_angle_rad,
            self.event_cluster_gap_m,
            self.event_separation_m,
            self.in_place_max_translation_m,
            self.in_place_min_yaw_change_rad,
        )
        if not all(math.isfinite(value) and value > 0.0 for value in positive):
            raise TeachRepeatError(
                "invalid_route_annotation_config",
                "route annotation thresholds must be positive and finite",
            )
        if self.minimum_turn_angle_rad >= self.u_turn_angle_rad:
            raise TeachRepeatError(
                "invalid_route_annotation_config",
                "turn threshold must be lower than U-turn threshold",
            )

    def to_dict(self):
        return {
            "direction_spacing_m": self.direction_spacing_m,
            "turn_window_m": self.turn_window_m,
            "u_turn_window_m": self.u_turn_window_m,
            "minimum_turn_angle_rad": self.minimum_turn_angle_rad,
            "u_turn_angle_rad": self.u_turn_angle_rad,
            "event_cluster_gap_m": self.event_cluster_gap_m,
            "event_separation_m": self.event_separation_m,
            "in_place_max_translation_m": self.in_place_max_translation_m,
            "in_place_min_yaw_change_rad": self.in_place_min_yaw_change_rad,
        }


def _cumulative_lengths(poses):
    output = [0.0]
    for first, second in zip(poses, poses[1:]):
        output.append(output[-1] + math.hypot(second.x - first.x, second.y - first.y))
    return output


def _unwrapped_yaws(poses):
    output = [poses[0].yaw]
    for first, second in zip(poses, poses[1:]):
        output.append(output[-1] + wrap_angle(second.yaw - first.yaw))
    return output


def _index_at_distance(cumulative, distance):
    return min(
        len(cumulative) - 1,
        max(0, bisect.bisect_left(cumulative, distance)),
    )


def _annotation_pose(pose):
    return {
        "x": float(f"{pose.x:.12g}"),
        "y": float(f"{pose.y:.12g}"),
        "yaw": float(f"{pose.yaw:.12g}"),
    }


def _event(event_type, index, distance, pose, heading_change=0.0):
    return {
        "type": event_type,
        "path_index": int(index),
        "distance_m": float(f"{distance:.12g}"),
        "heading_change_rad": float(f"{heading_change:.12g}"),
        **_annotation_pose(pose),
    }


def _in_place_events(poses, cumulative, config):
    runs = []
    current = []
    for index, (first, second) in enumerate(zip(poses, poses[1:])):
        translation = math.hypot(second.x - first.x, second.y - first.y)
        yaw_change = wrap_angle(second.yaw - first.yaw)
        if (
            translation <= config.in_place_max_translation_m
            and abs(yaw_change) >= config.in_place_min_yaw_change_rad
        ):
            current.append((index, yaw_change))
        elif current:
            runs.append(current)
            current = []
    if current:
        runs.append(current)

    output = []
    for run in runs:
        total_change = sum(item[1] for item in run)
        selected_index = run[len(run) // 2][0]
        event_type = "IN_PLACE_LEFT" if total_change > 0.0 else "IN_PLACE_RIGHT"
        output.append(
            _event(
                event_type,
                selected_index,
                cumulative[selected_index],
                poses[selected_index],
                total_change,
            )
        )
    return output


def _moving_turn_candidates(poses, cumulative, yaws, config):
    total = cumulative[-1]
    candidates = []
    for index, distance in enumerate(cumulative):
        if index == 0 or index == len(cumulative) - 1:
            continue
        before = _index_at_distance(
            cumulative, max(0.0, distance - config.turn_window_m)
        )
        after = _index_at_distance(
            cumulative, min(total, distance + config.turn_window_m)
        )
        if before >= index or after <= index:
            continue
        heading_change = yaws[after] - yaws[before]
        u_turn_before = _index_at_distance(
            cumulative, max(0.0, distance - config.u_turn_window_m)
        )
        u_turn_after = _index_at_distance(
            cumulative, min(total, distance + config.u_turn_window_m)
        )
        u_turn_change = yaws[u_turn_after] - yaws[u_turn_before]
        if abs(u_turn_change) >= config.u_turn_angle_rad:
            candidates.append((index, distance, u_turn_change))
        elif abs(heading_change) >= config.minimum_turn_angle_rad:
            candidates.append((index, distance, heading_change))
    return candidates


def _cluster_turn_candidates(candidates, config):
    if not candidates:
        return []
    groups = [[candidates[0]]]
    for candidate in candidates[1:]:
        previous = groups[-1][-1]
        same_direction = candidate[2] * previous[2] > 0.0
        close = candidate[1] - previous[1] <= config.event_cluster_gap_m
        if same_direction and close:
            groups[-1].append(candidate)
        else:
            groups.append([candidate])
    selected = []
    for group in groups:
        center_distance = 0.5 * (group[0][1] + group[-1][1])
        selected.append(
            min(
                group,
                key=lambda item: (-abs(item[2]), abs(item[1] - center_distance), item[0]),
            )
        )
    return selected


def classify_route(poses, config=None):
    poses = tuple(pose.normalized() for pose in poses)
    config = config or RouteAnnotationConfig()
    if len(poses) < 2:
        raise TeachRepeatError("path_too_short", "route annotations require two poses")
    if any(pose.frame_id != "map" for pose in poses):
        raise TeachRepeatError("invalid_path_frame", "route annotations require map frame")
    cumulative = _cumulative_lengths(poses)
    if cumulative[-1] <= 0.0:
        raise TeachRepeatError("zero_length_path", "route annotations require translation")
    yaws = _unwrapped_yaws(poses)

    directions = []
    target = config.direction_spacing_m
    while target < cumulative[-1]:
        index = _index_at_distance(cumulative, target)
        directions.append(
            {
                "path_index": index,
                "distance_m": float(f"{cumulative[index]:.12g}"),
                **_annotation_pose(poses[index]),
            }
        )
        target += config.direction_spacing_m

    in_place = _in_place_events(poses, cumulative, config)
    turn_events = []
    for index, distance, heading_change in _cluster_turn_candidates(
        _moving_turn_candidates(poses, cumulative, yaws, config), config
    ):
        if any(
            abs(distance - event["distance_m"]) < config.event_separation_m
            for event in in_place
        ):
            continue
        if abs(heading_change) >= config.u_turn_angle_rad:
            event_type = "U_TURN_LEFT" if heading_change > 0.0 else "U_TURN_RIGHT"
        else:
            event_type = "TURN_LEFT" if heading_change > 0.0 else "TURN_RIGHT"
        turn_events.append(
            _event(
                event_type,
                index,
                distance,
                poses[index],
                heading_change,
            )
        )

    events = [
        _event("START", 0, 0.0, poses[0]),
        *sorted(in_place + turn_events, key=lambda item: (item["distance_m"], item["type"])),
        _event("END", len(poses) - 1, cumulative[-1], poses[-1]),
    ]
    for index, event in enumerate(events):
        event["event_id"] = f"event_{index:04d}"
    return {"directions": directions, "events": events}


def route_annotation_document(
    demo_id,
    poses,
    reference_path_sha256,
    config=None,
):
    config = config or RouteAnnotationConfig()
    classified = classify_route(poses, config)
    return {
        "schema_version": ANNOTATION_SCHEMA_VERSION,
        "demo_id": str(demo_id),
        "frame_id": "map",
        "reference_path_sha256": str(reference_path_sha256),
        "config": config.to_dict(),
        **classified,
    }


def load_route_annotations(
    path,
    *,
    expected_demo_id=None,
    expected_reference_path_sha256=None,
):
    path = Path(path)
    try:
        document = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise TeachRepeatError(
            "route_annotations_unreadable",
            f"route annotations are unreadable: {exc}",
        ) from exc
    if not isinstance(document, dict) or document.get("schema_version") != 1:
        raise TeachRepeatError(
            "route_annotations_schema_mismatch",
            "unsupported route annotation schema",
        )
    if expected_demo_id and document.get("demo_id") != expected_demo_id:
        raise TeachRepeatError(
            "route_annotations_demo_mismatch",
            "route annotation demo_id does not match manifest",
        )
    if (
        expected_reference_path_sha256
        and document.get("reference_path_sha256")
        != expected_reference_path_sha256
    ):
        raise TeachRepeatError(
            "route_annotations_path_mismatch",
            "route annotations do not match the reference path",
        )
    if document.get("frame_id") != "map":
        raise TeachRepeatError(
            "invalid_path_frame", "route annotations must use map frame"
        )
    directions = document.get("directions")
    events = document.get("events")
    if not isinstance(directions, list) or not isinstance(events, list):
        raise TeachRepeatError(
            "route_annotations_invalid", "directions and events must be arrays"
        )
    for event in events:
        if not isinstance(event, dict) or event.get("type") not in EVENT_TYPES:
            raise TeachRepeatError(
                "route_annotations_invalid", "route event type is invalid"
            )
    numeric_fields = ("x", "y", "yaw", "distance_m")
    for entry in directions + events:
        try:
            values = tuple(float(entry[field]) for field in numeric_fields)
        except (KeyError, TypeError, ValueError) as exc:
            raise TeachRepeatError(
                "route_annotations_invalid", "route annotation values are invalid"
            ) from exc
        if not all(math.isfinite(value) for value in values):
            raise TeachRepeatError(
                "route_annotations_invalid", "route annotation values must be finite"
            )
    return document
