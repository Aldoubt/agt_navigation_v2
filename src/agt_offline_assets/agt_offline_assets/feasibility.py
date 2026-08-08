"""Validate a versioned Route Asset with the existing full-footprint geometry core."""

from dataclasses import dataclass
import json
import math
from pathlib import Path

from shapely.geometry import Polygon
from shapely.ops import unary_union

from agt_coverage_planning.path_validator import Pose2D, ValidatorConfig, validate_path
from agt_ui_bridge.platform_profile import load_platform_profile
from agt_ui_bridge.semantic_io import load_semantic_map

from .contracts import AssetContractError, RoutePolicy, load_yaml_mapping, sha256_file
from .grid_io import load_nav2_grid
from .route_asset import load_route_csv, write_route_manifest


@dataclass(frozen=True)
class FeasibilityResult:
    passed: bool
    report: dict
    geometry_result: object


def validate_route_asset(
    route_dir: str | Path,
    *,
    map_manifest_path: str | Path,
    platform_profile_path: str | Path,
    write_outputs: bool = True,
    maximum_preview_footprints: int = 250,
) -> FeasibilityResult:
    """Validate and, when requested, atomically finalize one Route revision.

    A READY route is immutable. All feasibility evidence and preview output are
    generated before the final route.yaml promotion write.
    """
    route_dir = Path(route_dir).expanduser().resolve()
    route_manifest_path = route_dir / "route.yaml"
    route_manifest = load_yaml_mapping(route_manifest_path)
    if write_outputs and str(route_manifest.get("status", "")).upper() == "READY":
        raise AssetContractError(
            "ready_route_immutable",
            "READY route revisions are immutable; create a new revision for any change",
        )
    map_manifest_path = Path(map_manifest_path).expanduser().resolve()
    map_manifest = load_yaml_mapping(map_manifest_path)
    if str(map_manifest.get("state", "")).upper() != "READY":
        raise AssetContractError("route_map_not_ready", "route feasibility requires a READY map")
    platform_path = Path(platform_profile_path).expanduser().resolve()
    platform = load_platform_profile(platform_path)

    _validate_bindings(route_dir, route_manifest, map_manifest_path, map_manifest, platform_path, platform)
    policy_path = route_dir / str(route_manifest["policy_binding"]["path"])
    policy = RoutePolicy.from_file(policy_path)
    route_csv = route_dir / "route.csv"
    samples = load_route_csv(route_csv)
    semantic_path = (route_dir / str(route_manifest["semantic_binding"]["path"])).resolve()
    semantic_map = load_semantic_map(semantic_path)
    enabled_semantic_ids = {str(item.id) for item in semantic_map.features if item.enabled}

    grid = load_nav2_grid(map_manifest_path.parent / "navigation" / "map.yaml")
    poses = [Pose2D(sample.x, sample.y, sample.yaw) for sample in samples]
    validator = ValidatorConfig(
        unknown_space_policy="free" if policy.unknown_space_allowed else "collision",
        outside_costmap_is_collision=True,
    )
    footprint = tuple(tuple(point) for point in platform["footprint"])
    geometry = validate_path(
        poses,
        "map",
        grid,
        footprint,
        float(platform["min_turning_radius"]),
        validator,
    )

    errors = list(geometry.report.error_codes)
    warnings = []
    if geometry.report.minimum_clearance + 1e-9 < policy.minimum_clearance_m:
        errors.append("minimum_clearance_violation")
    if not policy.allow_reverse and any(sample.direction == "R" for sample in samples):
        errors.append("reverse_not_allowed")
    for previous, current in zip(samples, samples[1:]):
        if previous.direction != current.direction and previous.segment_id == current.segment_id:
            errors.append("direction_change_inside_segment")
            break

    semantic_refs = {sample.semantic_ref for sample in samples if sample.semantic_ref}
    derived_semantic_refs = sorted(
        ref for ref in semantic_refs if ref.startswith("preview_aisle_")
    )
    unknown_semantic_refs = sorted({
        ref for ref in semantic_refs
        if ref != "<connector>"
        and not ref.startswith("preview_aisle_")
        and ref not in enabled_semantic_ids
    })
    if unknown_semantic_refs:
        errors.append("semantic_reference_missing")
    if derived_semantic_refs:
        warnings.append("derived_aisle_reference_from_frozen_semantic_map")

    semantic_invalid_samples = _semantic_footprint_violations(
        semantic_map, geometry.samples, footprint
    )
    if semantic_invalid_samples:
        errors.append("semantic_footprint_violation")
    if "<connector>" in semantic_refs:
        warnings.append("straight_connector_candidate_requires_planner_review")

    length_m = 0.0
    reverse_length_m = 0.0
    direction_changes = 0
    for previous, current in zip(samples, samples[1:]):
        distance = math.hypot(current.x - previous.x, current.y - previous.y)
        length_m += distance
        if current.direction == "R":
            reverse_length_m += distance
        if previous.direction != current.direction:
            direction_changes += 1

    errors = sorted(set(errors))
    passed = not errors
    invalid_segments = sorted(set(geometry.report.invalid_segment_indices) | {
        int(item.segment_index) for item in semantic_invalid_samples
    })
    report = {
        "status": "PASS" if passed else "FAIL",
        "route_id": str(route_manifest.get("route_id", "")),
        "revision": int(route_manifest.get("revision", 0)),
        "checks": {
            "map_binding": "PASS",
            "semantic_binding": "PASS" if not unknown_semantic_refs else "FAIL",
            "semantic_free_space": "PASS" if not semantic_invalid_samples else "FAIL",
            "vehicle_binding": "PASS",
            "policy_binding": "PASS",
            "full_footprint_sweep": "PASS" if geometry.report.collision_pose_count == 0 else "FAIL",
            "kinematics": "PASS" if "minimum_turning_radius_violation" not in errors else "FAIL",
            "minimum_clearance": "PASS" if "minimum_clearance_violation" not in errors else "FAIL",
            "direction_semantics": "PASS" if not any(code in errors for code in (
                "reverse_not_allowed", "direction_change_inside_segment"
            )) else "FAIL",
        },
        "metrics": {
            "length_m": round(length_m, 6),
            "reverse_length_m": round(reverse_length_m, 6),
            "min_clearance_m": float(geometry.report.minimum_clearance),
            "maximum_curvature": float(geometry.report.maximum_curvature),
            "direction_changes": direction_changes,
            "footprint_collision_count": int(geometry.report.collision_pose_count),
            "semantic_footprint_violation_count": len(semantic_invalid_samples),
            "derived_semantic_reference_count": len(derived_semantic_refs),
            "curvature_violation_count": int(
                "minimum_turning_radius_violation" in geometry.report.error_codes
            ),
            "unknown_intersection_count": int(geometry.report.unknown_collision_pose_count),
            "out_of_bounds_pose_count": int(geometry.report.out_of_bounds_pose_count),
            "sample_count": int(geometry.report.sample_count),
        },
        "derived_semantic_refs": derived_semantic_refs,
        "invalid_segment_indices": invalid_segments,
        "errors": errors,
        "warnings": warnings,
    }

    result = FeasibilityResult(passed, report, geometry)
    if write_outputs:
        report_path = route_dir / "feasibility_report.json"
        report_path.write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")

        from .preview import write_route_preview

        preview_path = write_route_preview(
            route_dir,
            platform_profile_path=platform_path,
            feasibility_result=result,
            maximum_footprints=maximum_preview_footprints,
            extra_invalid_samples=semantic_invalid_samples,
        )
        updated = dict(route_manifest)
        updated["route_csv_sha256"] = sha256_file(route_csv)
        updated["feasibility_report_sha256"] = sha256_file(report_path)
        updated["preview_sha256"] = sha256_file(preview_path)
        updated["status"] = "READY" if passed else "INVALID"
        write_route_manifest(route_manifest_path, updated)
    return result


def _validate_bindings(
    route_dir: Path,
    route_manifest: dict,
    map_manifest_path: Path,
    map_manifest: dict,
    platform_path: Path,
    platform: dict,
) -> None:
    if str(route_manifest.get("frame_id", "")) != "map":
        raise AssetContractError("route_frame_invalid", "Route Asset frame_id must be map")
    map_binding = route_manifest.get("map_binding") or {}
    if str(map_binding.get("map_id", "")) != str(map_manifest.get("map_id", "")):
        raise AssetContractError("route_map_id_mismatch", "route map_id differs from map manifest")
    if str(map_binding.get("map_version_id", "")) != str(map_manifest.get("map_version_id", "")):
        raise AssetContractError("route_map_version_mismatch", "route map_version_id differs from map manifest")
    if str(map_binding.get("manifest_sha256", "")) != sha256_file(map_manifest_path):
        raise AssetContractError("route_map_hash_mismatch", "route map manifest hash mismatch")

    semantic_binding = route_manifest.get("semantic_binding") or {}
    semantic_path = (route_dir / str(semantic_binding.get("path", ""))).resolve()
    if not semantic_path.is_file() or sha256_file(semantic_path) != str(semantic_binding.get("sha256", "")):
        raise AssetContractError("route_semantic_hash_mismatch", "route semantic binding is invalid")
    coverage_path = (route_dir / str(semantic_binding.get("coverage_path", ""))).resolve()
    if not coverage_path.is_file() or sha256_file(coverage_path) != str(semantic_binding.get("coverage_sha256", "")):
        raise AssetContractError("route_coverage_hash_mismatch", "route coverage binding is invalid")

    policy_binding = route_manifest.get("policy_binding") or {}
    policy_path = (route_dir / str(policy_binding.get("path", ""))).resolve()
    if not policy_path.is_file() or sha256_file(policy_path) != str(policy_binding.get("sha256", "")):
        raise AssetContractError("route_policy_hash_mismatch", "route policy binding is invalid")

    vehicle = route_manifest.get("vehicle_binding") or {}
    if str(vehicle.get("platform_id", "")) != str(platform["name"]):
        raise AssetContractError("route_vehicle_id_mismatch", "route vehicle id differs from selected platform")
    if str(vehicle.get("platform_profile_sha256", "")) != sha256_file(platform_path):
        raise AssetContractError("route_vehicle_hash_mismatch", "route vehicle profile hash mismatch")

    route_csv = route_dir / "route.csv"
    if not route_csv.is_file() or str(route_manifest.get("route_csv_sha256", "")) != sha256_file(route_csv):
        raise AssetContractError("route_csv_hash_mismatch", "route CSV hash mismatch")


def _semantic_footprint_violations(semantic_map, samples, footprint):
    fields = []
    forbidden = []
    for feature in semantic_map.features:
        if not feature.enabled or feature.geometry_type != "Polygon":
            continue
        polygon = Polygon(feature.coordinates[0])
        if feature.feature_type == "field_boundary":
            fields.append(polygon)
        elif feature.feature_type in {"exclusion_zone", "keepout_zone"}:
            forbidden.append(polygon)
    if not fields:
        raise AssetContractError(
            "semantic_field_missing", "semantic feasibility requires an enabled field_boundary"
        )
    allowed = unary_union(fields)
    if forbidden:
        allowed = allowed.difference(unary_union(forbidden))

    invalid = []
    for sample in samples:
        polygon = Polygon(_transform_footprint(footprint, sample.pose))
        if not allowed.covers(polygon):
            invalid.append(sample)
    return invalid


def _transform_footprint(footprint, pose):
    cosine = math.cos(pose.yaw)
    sine = math.sin(pose.yaw)
    return [
        (
            pose.x + cosine * x - sine * y,
            pose.y + sine * x + cosine * y,
        )
        for x, y in footprint
    ]
