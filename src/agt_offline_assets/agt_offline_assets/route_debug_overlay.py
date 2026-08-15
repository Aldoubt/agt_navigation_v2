"""Render-only GeoJSON generation for Route Debug 2D."""

from __future__ import annotations

from dataclasses import dataclass
import json
import math
from pathlib import Path
from typing import Any, Mapping

import numpy as np

from .forward_connector import ForwardConnectorSample, _dubins_candidates, _sample_candidate
from .forward_connector_navigation_gate import _preview_local_footprint, _transform_polygon
from .navigation_map_derivation import FREE, OCCUPIED, UNKNOWN
from .route_debug_dataset import RouteDebugDataset
from .vehicle_safe_lane import _normalize, _offset_candidates, _resample_polyline
from .vehicle_safe_lane_occupancy_sources import _candidate_key, _fully_free, _pose_mask

ROUTE_DEBUG_OVERLAY_SCHEMA = "agt_route_debug_overlay/v1"


@dataclass(frozen=True)
class RouteDebugOverlayConfig:
    forward_sample_step_m: float = 0.05
    collision_sample_spacing_m: float = 0.10
    lateral_search_step_m: float = 0.05
    maximum_lateral_shift_m: float = 0.50
    preview_footprint_padding_m: float = 0.05


def _feature(feature_id, *, layer_group, layer_key, feature_kind, frame_id, status,
             source_asset, source_id, source_field, geometry, inspector=None,
             is_failure=False, **extra):
    props = {
        "feature_id": feature_id, "layer_group": layer_group, "layer_key": layer_key,
        "feature_kind": feature_kind, "frame_id": frame_id, "status": status,
        "source_asset": source_asset, "source_id": source_id, "source_field": source_field,
        "is_failure": bool(is_failure), "inspector": dict(inspector or {}),
    }
    props.update(extra)
    return {"type": "Feature", "properties": props, "geometry": dict(geometry)}


def _line(points):
    return {"type": "LineString", "coordinates": [[float(p[0]), float(p[1])] for p in points]}


def _point(x, y):
    return {"type": "Point", "coordinates": [float(x), float(y)]}


def _polygon(points):
    ring = [[float(p[0]), float(p[1])] for p in points]
    if ring and ring[0] != ring[-1]:
        ring.append(list(ring[0]))
    return {"type": "Polygon", "coordinates": [ring]}


def _failure_status(status):
    if not status:
        return False
    text = str(status).upper()
    return not any(token in text for token in ("ACCEPTED", "READY", "FREE", "KEEP_FORWARD", "ELIGIBLE"))


def _connector_inspector(connector):
    return {
        "RELATED": {"connector_id": connector.connector_id, "from_aisle_id": connector.from_aisle_id,
                    "to_aisle_id": connector.to_aisle_id, "turn_zone_id": connector.turn_zone_id},
        "COVERAGE": {"request": "present" if connector.request is not None else "missing"},
        "FORWARD": {"status": connector.forward_status, "backend": connector.forward_backend,
                    "path_type": connector.forward_path_type},
        "R5.6": {"zone_fit_status": connector.zone_fit_status,
                 "forward_audit_status": connector.forward_audit_status},
        "R6A": {"decision": connector.r6a_decision},
        "R6B": {"status": connector.r6b_status, "backend": connector.r6b_backend,
                "path_length_m": connector.r6b_path_length_m,
                "forward_distance_m": connector.r6b_forward_distance_m,
                "reverse_distance_m": connector.r6b_reverse_distance_m,
                "cusp_count": connector.r6b_cusp_count,
                "search_expansions": connector.r6b_search_expansions,
                "goal_position_error_m": connector.r6b_goal_position_error_m,
                "goal_yaw_error_rad": connector.r6b_goal_yaw_error_rad},
    }


def _add_structure(dataset, features):
    for region in dataset.no_go_regions:
        features.append(_feature(
            f"no-go:{region.region_id}", layer_group="base", layer_key="base.no_go",
            feature_kind="NO_GO", frame_id=dataset.frame_id, status="NO_GO",
            source_asset=region.source_asset, source_id=region.region_id,
            source_field="overrides[].polygon_xy", geometry=_polygon(region.polygon_xy),
            inspector={"NAVIGATION / VEHICLE FEASIBILITY": {
                "semantic_state": "NO_GO",
                "note": "semantic operator exclusion; separate from physical OCCUPIED"},
                "SOURCE": {"asset": region.source_asset}}, is_failure=True))

    for record in dataset.aisles:
        aisle = record.aisle
        inspector = {
            "STRUCTURE": {"aisle_id": aisle.aisle_id, "kind": aisle.kind, "pair_kind": aisle.pair_kind,
                          "length_m": aisle.length_m, "geometric_width_m": aisle.geometric_width_m,
                          "minimum_required_width_m": aisle.minimum_required_width_m,
                          "diagnostic_status": aisle.diagnostic_status},
            "COVERAGE": {"sequence": None if record.traversal is None else record.traversal.sequence,
                         "graph_orientation": None if record.traversal is None else record.traversal.graph_orientation,
                         "motion_direction": None if record.traversal is None else record.traversal.motion_direction,
                         "rejection": None if record.rejection is None else record.rejection.reason},
            "VEHICLE": {"lane_status": None if record.vehicle_lane is None else record.vehicle_lane.status,
                        "lane_coverage_fraction": None if record.vehicle_lane is None else record.vehicle_lane.coverage_fraction},
            "NAVIGATION / VEHICLE FEASIBILITY": {
                "classification": None if record.diagnostic is None else record.diagnostic.classification,
                "reference_free_fraction": None if record.diagnostic is None else record.diagnostic.reference_free_fraction,
                "any_lateral_footprint_free_fraction": None if record.diagnostic is None else record.diagnostic.any_lateral_footprint_free_fraction,
                "occupancy_source": None if record.occupancy_source is None else record.occupancy_source.classification},
            "SOURCE": {"asset": "aisle_graph.yaml"},
        }
        features.append(_feature(
            f"aisle:{aisle.aisle_id}", layer_group="structure", layer_key="structure.aisles",
            feature_kind="AISLE_CENTERLINE", frame_id=dataset.frame_id, status=aisle.diagnostic_status,
            source_asset="aisle_graph.yaml", source_id=aisle.aisle_id,
            source_field="aisles[].centerline_xyz", geometry=_line(aisle.centerline_xyz),
            inspector=inspector, is_failure=record.rejection is not None))
        if record.vehicle_lane is not None and record.vehicle_lane.centerline_xyz:
            lane = record.vehicle_lane
            features.append(_feature(
                f"vehicle-lane:{aisle.aisle_id}", layer_group="structure", layer_key="structure.vehicle_safe_lane",
                feature_kind="VEHICLE_SAFE_LANE", frame_id=dataset.frame_id, status=lane.status,
                source_asset="vehicle_safe_lane.yaml", source_id=aisle.aisle_id,
                source_field="aisles[].centerline_xyz", geometry=_line(lane.centerline_xyz),
                inspector=inspector, is_failure=lane.status != "VEHICLE_SAFE_LANE_READY"))

    if dataset.turn_zones is not None:
        for zone in dataset.turn_zones.zones:
            features.append(_feature(
                f"turn-zone:{zone.zone_id}", layer_group="structure", layer_key="structure.turn_zones",
                feature_kind="TURN_ZONE", frame_id=dataset.frame_id,
                status="ALLOW_TURN" if zone.allow_turn else "TURN_DISABLED",
                source_asset="turn_zones.yaml", source_id=zone.zone_id, source_field="zones[].polygon_xy",
                geometry=_polygon(zone.polygon_xy),
                inspector={"STRUCTURE": {"zone_id": zone.zone_id, "side": zone.side,
                    "free_fraction": zone.free_fraction, "allow_turn": zone.allow_turn,
                    "allow_reverse": zone.allow_reverse}, "SOURCE": {"asset": "turn_zones.yaml"}},
                is_failure=not zone.allow_turn))


def _add_coverage(dataset, features):
    for record in dataset.aisles:
        traversal = record.traversal
        if traversal is None:
            continue
        points = record.aisle.centerline_xyz
        if traversal.graph_orientation == "AGAINST_ROW_DIRECTION":
            points = tuple(reversed(points))
        features.append(_feature(
            f"coverage:{traversal.sequence:03d}:{record.aisle_id}", layer_group="coverage",
            layer_key="coverage.order", feature_kind="COVERAGE_TRAVERSAL",
            frame_id=dataset.frame_id, status="FORWARD", source_asset="coverage_order.yaml",
            source_id=record.aisle_id, source_field="traversals[]", geometry=_line(points),
            inspector={"COVERAGE": {"sequence": traversal.sequence, "aisle_id": traversal.aisle_id,
                "motion_direction": traversal.motion_direction, "graph_orientation": traversal.graph_orientation,
                "entry_side": traversal.entry_side, "exit_side": traversal.exit_side,
                "geometric_width_m": traversal.geometric_width_m, "required_width_m": traversal.required_width_m},
                "SOURCE": {"asset": "coverage_order.yaml"}},
            sequence=traversal.sequence, motion_direction=traversal.motion_direction))
    for request in dataset.connector_requests:
        features.append(_feature(
            f"request:{request.connector_id}", layer_group="coverage", layer_key="coverage.requests",
            feature_kind="CONNECTOR_REQUEST", frame_id=dataset.frame_id, status="REQUESTED",
            source_asset="coverage_order.yaml", source_id=request.connector_id,
            source_field="connector_requests[]", geometry=_line((request.start_pose, request.goal_pose)),
            inspector={"COVERAGE": {"connector_id": request.connector_id,
                "from_aisle_id": request.from_aisle_id, "to_aisle_id": request.to_aisle_id,
                "turn_zone_id": request.turn_zone_id, "side": request.side},
                "SOURCE": {"asset": "coverage_order.yaml"}}))


def _add_forward_candidates(dataset, features, cfg):
    vehicle = dataset.vehicle_profile
    if vehicle is None or not vehicle.planning_preview_ready:
        return
    radius = float(vehicle.minimum_turning_radius_m)
    for connector in dataset.connectors:
        if connector.request is None or not connector.forward_candidates:
            continue
        candidates = {kind: lengths for kind, lengths in _dubins_candidates(
            connector.request.start_pose, connector.request.goal_pose, radius)}
        for candidate in connector.forward_candidates:
            lengths = candidates.get(candidate.path_type)
            if lengths is None:
                continue
            samples, _ = _sample_candidate(connector.request.start_pose, connector.request.goal_pose,
                candidate.path_type, lengths, radius, cfg.forward_sample_step_m)
            if len(samples) < 2:
                continue
            inspector = _connector_inspector(connector)
            inspector["FORWARD CANDIDATE"] = {
                "path_type": candidate.path_type, "length_m": candidate.length_m,
                "local_headland_candidate": candidate.local_headland_candidate,
                "preview_footprint_free": candidate.preview_footprint_free,
                "max_required_zone_extension_m": candidate.max_required_zone_extension_m,
                "derived_geometry_method": "REPLAY_ANALYTIC_DUBINS_FROM_FROZEN_REQUEST_AND_PROFILE"}
            features.append(_feature(
                f"forward-candidate:{connector.connector_id}:{candidate.path_type}",
                layer_group="motion", layer_key="motion.forward_candidates",
                feature_kind="FORWARD_CANDIDATE", frame_id=dataset.frame_id,
                status="PREVIEW_FOOTPRINT_FREE" if candidate.preview_footprint_free else "PREVIEW_REJECTED",
                source_asset="forward_connector_candidate_audit.yaml", source_id=connector.connector_id,
                source_field="connectors[].candidates[]",
                geometry=_line([(sample.x, sample.y) for sample in samples]), inspector=inspector,
                is_failure=not candidate.preview_footprint_free, path_type=candidate.path_type,
                local_headland_candidate=candidate.local_headland_candidate,
                preview_footprint_free=candidate.preview_footprint_free,
                derived_geometry_method="REPLAY_ANALYTIC_DUBINS_FROM_FROZEN_REQUEST_AND_PROFILE"))


def _split_motion(samples):
    if not samples:
        return []
    groups = []; current = [samples[0]]; direction = samples[0].motion_direction
    for sample in samples[1:]:
        if sample.motion_direction != direction:
            current.append(sample); groups.append((direction, tuple(current)))
            current = [current[-1], sample]; direction = sample.motion_direction
        else:
            current.append(sample)
    groups.append((direction, tuple(current)))
    return groups


def _add_motion(dataset, features):
    for connector in dataset.connectors:
        inspector = _connector_inspector(connector)
        if len(connector.forward_samples) >= 2:
            features.append(_feature(
                f"forward-selected:{connector.connector_id}", layer_group="motion",
                layer_key="motion.forward_selected", feature_kind="FORWARD_SELECTED",
                frame_id=dataset.frame_id, status=connector.forward_status or "FORWARD",
                source_asset="forward_connectors.yaml", source_id=connector.connector_id,
                source_field="connectors[].samples",
                geometry=_line([(s.x, s.y) for s in connector.forward_samples]),
                inspector=inspector, is_failure=_failure_status(connector.forward_status),
                motion_direction="FORWARD"))
        for index, (direction, samples) in enumerate(_split_motion(connector.r6b_samples)):
            if len(samples) < 2:
                continue
            features.append(_feature(
                f"r6-motion:{connector.connector_id}:{index:02d}:{direction.lower()}",
                layer_group="motion",
                layer_key="motion.reverse" if direction == "REVERSE" else "motion.forward_selected",
                feature_kind="R6_MOTION_SEGMENT", frame_id=dataset.frame_id,
                status=connector.r6b_status or direction,
                source_asset="reverse_primitive_connectors.yaml", source_id=connector.connector_id,
                source_field="connectors[].samples", geometry=_line([(s.x, s.y) for s in samples]),
                inspector=inspector, is_failure=_failure_status(connector.r6b_status),
                motion_direction=direction, segment_index=index, backend=connector.r6b_backend))
        for index, sample in enumerate(connector.r6b_samples):
            if sample.is_cusp:
                features.append(_feature(
                    f"cusp:{connector.connector_id}:{index:03d}", layer_group="motion",
                    layer_key="motion.reverse", feature_kind="CUSP", frame_id=dataset.frame_id,
                    status="CUSP", source_asset="reverse_primitive_connectors.yaml",
                    source_id=connector.connector_id, source_field="connectors[].samples[].is_cusp",
                    geometry=_point(sample.x, sample.y), inspector=inspector,
                    motion_direction=sample.motion_direction, segment_index=sample.segment_index))
        failure_status = connector.r6b_status or connector.forward_audit_status or connector.forward_status
        if connector.request is not None and _failure_status(failure_status):
            x = 0.5 * (connector.request.start_pose[0] + connector.request.goal_pose[0])
            y = 0.5 * (connector.request.start_pose[1] + connector.request.goal_pose[1])
            features.append(_feature(
                f"failed:{connector.connector_id}", layer_group="diagnostics",
                layer_key="diagnostics.failed", feature_kind="FAILED_CONNECTOR",
                frame_id=dataset.frame_id, status=failure_status or "FAILED",
                source_asset="route-production frozen assets", source_id=connector.connector_id,
                source_field="status", geometry=_point(x, y), inspector=inspector, is_failure=True))


def _add_collisions(dataset, features, cfg):
    navigation = dataset.navigation; vehicle = dataset.vehicle_profile
    masks = dataset.occupancy_source_masks; graph = dataset.aisle_graph
    if navigation is None or vehicle is None or graph is None or not vehicle.planning_preview_ready:
        return
    direction = _normalize(graph.row_direction_xy); perpendicular = np.array([-direction[1], direction[0]], dtype=np.float64)
    yaw = math.atan2(float(direction[1]), float(direction[0]))
    local_footprint = _preview_local_footprint(vehicle, cfg.preview_footprint_padding_m)
    for record in dataset.aisles:
        aisle = record.aisle; samples, _ = _resample_polyline(aisle, cfg.collision_sample_spacing_m)
        required_width = float(vehicle.navigation_width_m + 2.0 * cfg.preview_footprint_padding_m)
        surplus = float(aisle.geometric_width_m) - required_width
        if surplus < -1.0e-9:
            continue
        allowed = min(float(cfg.maximum_lateral_shift_m), 0.5 * max(0.0, surplus))
        offsets = _offset_candidates(allowed, cfg.lateral_search_step_m)
        for station_index, sample in enumerate(samples):
            candidates = []; free_pose = False
            for offset in offsets:
                x = float(sample[0] + offset * perpendicular[0]); y = float(sample[1] + offset * perpendicular[1])
                mask, coverage = _pose_mask(x, y, yaw, navigation, local_footprint)
                candidates.append((float(offset), x, y, mask, float(coverage)))
                if _fully_free(mask, coverage, navigation):
                    free_pose = True; break
            if free_pose or not candidates:
                continue
            offset, x, y, mask, coverage = min(candidates,
                key=lambda item: _candidate_key(item[3], item[4], navigation, item[0]))
            cells = navigation.occupancy[mask]
            occupied = int(np.count_nonzero(cells == OCCUPIED)); unknown = int(np.count_nonzero(cells == UNKNOWN)); free = int(np.count_nonzero(cells == FREE))
            counts = {"RAW_OBSTACLE_DIRECT": 0, "GEOMETRY_DIRECT": 0, "PADDING_ONLY": 0,
                      "UNEXPLAINED_OCCUPIED": occupied if masks is None else 0,
                      "UNKNOWN": unknown, "OUT_OF_GRID": 0 if coverage >= 1.0 - 1.0e-9 else 1}
            if masks is not None:
                counts["RAW_OBSTACLE_DIRECT"] = int(np.count_nonzero(mask & masks.raw_obstacle_direct))
                counts["GEOMETRY_DIRECT"] = int(np.count_nonzero(mask & masks.geometry_direct))
                counts["PADDING_ONLY"] = int(np.count_nonzero(mask & masks.padding_only))
                counts["UNEXPLAINED_OCCUPIED"] = int(np.count_nonzero(mask & masks.unexplained_occupied))
            ranking = ("RAW_OBSTACLE_DIRECT", "GEOMETRY_DIRECT", "PADDING_ONLY",
                       "UNEXPLAINED_OCCUPIED", "UNKNOWN", "OUT_OF_GRID")
            dominant = max(ranking, key=lambda name: (counts[name], -ranking.index(name)))
            probe = ForwardConnectorSample(x=x, y=y, z=float(sample[2]), yaw=yaw)
            footprint = _transform_polygon(local_footprint, probe)
            inspector = {
                "STRUCTURE": {"aisle_id": aisle.aisle_id, "station_index": station_index,
                    "geometric_width_m": aisle.geometric_width_m, "required_preview_width_m": required_width,
                    "allowed_lateral_shift_m": allowed, "selected_lateral_offset_m": offset},
                "CONFLICT": {"dominant_source": dominant, "free_count": free, "occupied_count": occupied,
                    "unknown_count": unknown, "grid_coverage_fraction": coverage, **counts},
                "SOURCE": {"asset": "navigation_map.yaml",
                    "source_exactness": None if masks is None else masks.source_exactness},
            }
            features.append(_feature(
                f"collision:{aisle.aisle_id}:{station_index:04d}", layer_group="diagnostics",
                layer_key="diagnostics.conflicts", feature_kind="COLLISION_STATION",
                frame_id=dataset.frame_id, status="NO_FULLY_FREE_BOUNDED_LATERAL_POSE",
                source_asset="navigation_map.yaml", source_id=aisle.aisle_id, source_field="occupancy",
                geometry=_point(x, y), inspector=inspector, is_failure=True, dominant_source=dominant,
                free_count=free, occupied_count=occupied, unknown_count=unknown,
                grid_coverage_fraction=coverage, source_counts=counts,
                footprint_polygon_xy=[[float(px), float(py)] for px, py in footprint]))


def build_route_debug_overlay(dataset: RouteDebugDataset, config: RouteDebugOverlayConfig | None = None) -> dict[str, Any]:
    """Build GeoJSON strictly for visualization; no route decision is changed."""
    cfg = config or RouteDebugOverlayConfig(); features = []
    _add_structure(dataset, features); _add_coverage(dataset, features)
    _add_forward_candidates(dataset, features, cfg); _add_motion(dataset, features)
    _add_collisions(dataset, features, cfg)
    return {"type": "FeatureCollection", "agt_schema": ROUTE_DEBUG_OVERLAY_SCHEMA,
            "frame_id": dataset.frame_id, "validation_scope": "DEBUG_RENDER_ONLY", "features": features}


def write_route_debug_overlay(overlay: Mapping[str, Any], path: str | Path, *, overwrite: bool = False) -> Path:
    output = Path(path).expanduser()
    if output.exists() and not overwrite:
        raise FileExistsError(f"route debug overlay already exists: {output}")
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(dict(overlay), ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    return output
