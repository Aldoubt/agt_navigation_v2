"""Validate a versioned Route Asset with the existing full-footprint geometry core."""

from dataclasses import dataclass
import json
import math
from pathlib import Path

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
) -> FeasibilityResult:
    route_dir = Path(route_dir).expanduser().resolve()
    route_manifest_path = route_dir / "route.yaml"
    route_manifest = load_yaml_mapping(route_manifest_path)
    map_manifest_path = Path(map_manifest_path).expanduser().resolve()
    map_manifest = load_yaml_mapping(map_manifest_path)
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
    geometry = validate_path(
        poses,
        "map",
        grid,
        tuple(tuple(point) for point in platform["footprint"]),
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
    unknown_semantic_refs = sorted({
        sample.semantic_ref for sample in samples
        if sample.semantic_ref and sample.semantic_ref != "<connector>"
        and sample.semantic_ref not in enabled_semantic_ids
    })
    if unknown_semantic_refs:
        errors.append("semantic_reference_missing")
    if any(sample.semantic_ref == "<connector>" for sample in samples):
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
    report = {
        "status": "PASS" if passed else "FAIL",
        "route_id": str(route_manifest.get("route_id", "")),
        "revision": int(route_manifest.get("revision", 0)),
        "checks": {
            "map_binding": "PASS",
            "semantic_binding": "PASS" if not unknown_semantic_refs else "FAIL",
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
            "curvature_violation_count": int(
                "minimum_turning_radius_violation" in geometry.report.error_codes
            ),
            "unknown_intersection_count": int(geometry.report.unknown_collision_pose_count),
            "out_of_bounds_pose_count": int(geometry.report.out_of_bounds_pose_count),
            "sample_count": int(geometry.report.sample_count),
        },
        "invalid_segment_indices": list(geometry.report.invalid_segment_indices),
        "errors": errors,
        "warnings": warnings,
    }

    if write_outputs:
        report_path = route_dir / "feasibility_report.json"
        report_path.write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
        updated = dict(route_manifest)
        updated["route_csv_sha256"] = sha256_file(route_csv)
        updated["feasibility_report_sha256"] = sha256_file(report_path)
        updated["status"] = "READY" if passed else "INVALID"
        write_route_manifest(route_manifest_path, updated)
    return FeasibilityResult(passed, report, geometry)


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
