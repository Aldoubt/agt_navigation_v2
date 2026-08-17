from __future__ import annotations

import json
import math
from pathlib import Path
from typing import Mapping, Sequence


def _pose_triplet(value) -> list[float]:
    if len(value) != 3:
        raise ValueError("coverage pose must be [x, y, yaw]")
    out = [float(value[0]), float(value[1]), float(value[2])]
    if not all(math.isfinite(v) for v in out):
        raise ValueError("coverage pose must be finite")
    return out


def components_from_path_semantics(
    poses: Sequence[Sequence[float]],
    semantics: Mapping,
    *,
    swath_to_row: Mapping[str, str] | None = None,
) -> list[dict]:
    raw = [_pose_triplet(p) for p in poses]
    if len(raw) < 2:
        raise ValueError("coverage raw path requires at least two poses")
    segments = semantics.get("raw_segments") if isinstance(semantics, Mapping) else None
    if not isinstance(segments, list) or not segments:
        raise ValueError("coverage path semantics must contain raw_segments")
    swath_to_row = dict(swath_to_row or {})
    expected_interval = 0
    output: list[dict] = []
    for item in segments:
        if not isinstance(item, Mapping):
            raise ValueError("coverage raw segment must be a mapping")
        start = int(item["start_index"])
        end = int(item["end_index"])
        if start != expected_interval or end < start or end >= len(raw) - 1:
            raise ValueError("coverage raw_segments must exactly and contiguously cover path intervals")
        component_type = str(item.get("component_type", ""))
        if component_type not in {"SWATH", "CONNECTION"}:
            raise ValueError(f"unsupported coverage component_type {component_type!r}")
        component_id = str(item.get("component_id", ""))
        swath_id = str(item.get("swath_id", ""))
        if component_type == "SWATH":
            if not swath_id:
                raise ValueError("SWATH segment missing swath_id")
            semantic_ref = swath_to_row.get(swath_id, swath_id)
        else:
            semantic_ref = component_id
        output.append({
            "segment_type": component_type,
            "semantic_ref": semantic_ref,
            "source_component_id": component_id,
            "source_swath_id": swath_id,
            "points": [list(p) for p in raw[start:end + 2]],
        })
        expected_interval = end + 1
    if expected_interval != len(raw) - 1:
        raise ValueError("coverage raw_segments leave unclassified path intervals")
    return output


def _row_features(path: Path) -> list[tuple[str, tuple[float, float], tuple[float, float]]]:
    document = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(document, dict) or document.get("type") != "FeatureCollection":
        raise ValueError("semantic map must be a GeoJSON FeatureCollection")
    rows = []
    for feature in document.get("features", []):
        if not isinstance(feature, dict):
            continue
        props = feature.get("properties") or {}
        if props.get("enabled", True) is False:
            continue
        if str(props.get("feature_type", "")) not in {"row_centerline", "access_lane"}:
            continue
        geometry = feature.get("geometry") or {}
        if geometry.get("type") != "LineString":
            continue
        coords = geometry.get("coordinates") or []
        if len(coords) < 2:
            continue
        feature_id = str(props.get("id", feature.get("id", "")))
        if not feature_id:
            raise ValueError("enabled semantic row/access lane missing id")
        start = (float(coords[0][0]), float(coords[0][1]))
        end = (float(coords[-1][0]), float(coords[-1][1]))
        rows.append((feature_id, start, end))
    if not rows:
        raise ValueError("semantic map contains no enabled row_centerline/access_lane features")
    return rows


def _endpoint_error(a0, a1, b0, b1) -> float:
    direct = max(math.dist(a0, b0), math.dist(a1, b1))
    reverse = max(math.dist(a0, b1), math.dist(a1, b0))
    return min(direct, reverse)


def match_swaths_to_semantic_rows(
    poses: Sequence[Sequence[float]],
    semantics: Mapping,
    semantic_map_path: Path | str,
    *,
    max_endpoint_error_m: float = 0.35,
) -> dict[str, str]:
    raw = [_pose_triplet(p) for p in poses]
    rows = _row_features(Path(semantic_map_path))
    segments = semantics.get("raw_segments") if isinstance(semantics, Mapping) else None
    if not isinstance(segments, list):
        raise ValueError("coverage path semantics missing raw_segments")
    swaths = []
    for item in segments:
        if str(item.get("component_type", "")) != "SWATH":
            continue
        start = int(item["start_index"])
        end = int(item["end_index"])
        if start < 0 or end < start or end + 1 >= len(raw):
            raise ValueError("SWATH segment range is outside the raw path")
        swath_id = str(item.get("swath_id", ""))
        if not swath_id:
            raise ValueError("SWATH segment missing swath_id")
        swaths.append((swath_id, (raw[start][0], raw[start][1]), (raw[end + 1][0], raw[end + 1][1])))
    if not swaths:
        raise ValueError("coverage path semantics contains no SWATH segments")

    candidates = []
    for swath_id, first, last in swaths:
        for row_id, row_first, row_last in rows:
            candidates.append((_endpoint_error(first, last, row_first, row_last), swath_id, row_id))
    candidates.sort()
    mapping: dict[str, str] = {}
    used_rows: set[str] = set()
    for error, swath_id, row_id in candidates:
        if error > max_endpoint_error_m:
            continue
        if swath_id in mapping or row_id in used_rows:
            continue
        mapping[swath_id] = row_id
        used_rows.add(row_id)
    missing = sorted({item[0] for item in swaths} - set(mapping))
    if missing:
        raise ValueError(
            f"could not match coverage swaths to accepted semantic rows within {max_endpoint_error_m:.3f} m: {missing}"
        )
    return mapping


def collect_coverage_components_live(
    semantic_map_path: Path | str,
    *,
    timeout_s: float = 60.0,
    max_endpoint_error_m: float = 0.35,
) -> tuple[list[dict], float]:
    """Trigger the existing coverage stack and return validated benchmark components."""
    import time

    try:
        import rclpy
        from diagnostic_msgs.msg import DiagnosticArray
        from nav_msgs.msg import Path as NavPath
        from rclpy.node import Node
        from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
        from std_msgs.msg import String
        from std_srvs.srv import Trigger
        from agt_coverage_planning.path_semantics import Pose2D, parse_path_semantics
    except ImportError as exc:
        raise RuntimeError(
            "live Fields2Cover benchmark requires ROS2 + agt_coverage_planning runtime dependencies"
        ) from exc

    semantic_path = Path(semantic_map_path).expanduser().resolve()
    rclpy.init()
    node = Node("agt_route_benchmark_coverage_client")
    qos = QoSProfile(depth=1)
    qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
    qos.reliability = ReliabilityPolicy.RELIABLE
    state = {"raw": None, "semantics": None, "status": None, "status_values": {}, "status_count": 0}

    def raw_callback(msg):
        state["raw"] = msg

    def semantics_callback(msg):
        try:
            state["semantics"] = json.loads(msg.data)
        except (TypeError, ValueError):
            state["semantics"] = None

    def status_callback(msg):
        if not msg.status:
            return
        status = msg.status[0]
        if status.name != "agt_coverage_request_adapter":
            return
        state["status"] = status.message
        state["status_values"] = {item.key: item.value for item in status.values}
        state["status_count"] += 1

    node.create_subscription(NavPath, "/agt/coverage/path_raw", raw_callback, qos)
    node.create_subscription(String, "/agt/coverage/path_semantics", semantics_callback, qos)
    node.create_subscription(DiagnosticArray, "/agt/coverage/status", status_callback, qos)
    trigger = node.create_client(Trigger, "/agt/coverage/plan")
    deadline = time.monotonic() + float(timeout_s)
    try:
        while time.monotonic() < deadline and not trigger.wait_for_service(timeout_sec=0.25):
            rclpy.spin_once(node, timeout_sec=0.05)
        if not trigger.service_is_ready():
            raise RuntimeError("coverage plan service did not become ready")

        accepted = False
        status_count_before_accept = state["status_count"]
        last_message = ""
        while time.monotonic() < deadline and not accepted:
            future = trigger.call_async(Trigger.Request())
            while time.monotonic() < deadline and not future.done():
                rclpy.spin_once(node, timeout_sec=0.05)
            if not future.done() or future.result() is None:
                break
            response = future.result()
            accepted = bool(response.success)
            last_message = str(response.message)
            if not accepted:
                rclpy.spin_once(node, timeout_sec=0.25)
        if not accepted:
            raise RuntimeError(f"coverage planning request was not accepted: {last_message}")

        final_state = None
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            if state["status_count"] <= status_count_before_accept:
                continue
            if state["status"] in {"SUCCEEDED", "FAILED", "REJECTED"}:
                final_state = state["status"]
                break
        if final_state != "SUCCEEDED":
            values = state["status_values"]
            raise RuntimeError(
                f"coverage planning failed: state={final_state or state['status']} "
                f"error_code={values.get('error_code', '')} detail={values.get('detail', '')}"
            )

        settle_deadline = min(deadline, time.monotonic() + 2.0)
        while time.monotonic() < settle_deadline and (state["raw"] is None or state["semantics"] is None):
            rclpy.spin_once(node, timeout_sec=0.05)
        if state["raw"] is None or state["semantics"] is None:
            raise RuntimeError("coverage succeeded but raw path/path semantics were not received")

        raw_msg = state["raw"]
        if raw_msg.header.frame_id != "map":
            raise RuntimeError(f"coverage raw path frame must be map, got {raw_msg.header.frame_id!r}")
        poses = []
        semantic_poses = []
        for stamped in raw_msg.poses:
            q = stamped.pose.orientation
            yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
            x = float(stamped.pose.position.x)
            y = float(stamped.pose.position.y)
            poses.append((x, y, yaw))
            semantic_poses.append(Pose2D(x, y, yaw))
        parse_path_semantics(state["semantics"], semantic_poses, frame_id="map")
        mapping = match_swaths_to_semantic_rows(
            poses,
            state["semantics"],
            semantic_path,
            max_endpoint_error_m=max_endpoint_error_m,
        )
        components = components_from_path_semantics(poses, state["semantics"], swath_to_row=mapping)
        planning_time = float(state["status_values"].get("planning_time", 0.0) or 0.0)
        return components, planning_time
    finally:
        node.destroy_node()
        rclpy.shutdown()
