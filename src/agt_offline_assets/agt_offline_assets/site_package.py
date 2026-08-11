"""Versioned Site Package root binding for V25-12A.

A Site Package does not redefine map-version or Route Asset internals. It binds
already accepted project assets by stable identity so runtime deployment can select
one auditable site configuration without relying on directory recency.
"""

from dataclasses import dataclass
from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path
import re
from typing import Any, Iterable, Mapping
from uuid import uuid4

import yaml

from .contracts import AssetContractError, load_yaml_mapping, sha256_file
from .map_validation import validate_map_workspace
from .workspace import compute_map_content_sha256


_SITE_PACKAGE_ID_RE = re.compile(r"^sitepkg_[0-9]{8}_[0-9]{6}_[0-9a-fA-F]{8}$")
_SAFE_ID_RE = re.compile(r"^[A-Za-z0-9][A-Za-z0-9_.-]*$")
_SITE_CONTENT_KEYS = (
    "schema_version",
    "site_package_schema",
    "site_id",
    "site_package_id",
    "frame_id",
    "map_binding",
    "calibration_binding",
    "localization_binding",
    "navigation_binding",
    "semantic_binding",
    "vehicle_binding",
    "routes",
    "localization_prior",
    "benchmark_bindings",
)


def _now() -> str:
    return datetime.now(timezone.utc).isoformat(timespec="seconds")


def generate_site_package_id() -> str:
    return datetime.now(timezone.utc).strftime("sitepkg_%Y%m%d_%H%M%S_") + uuid4().hex[:8]


def compute_site_package_content_sha256(manifest: Mapping[str, Any]) -> str:
    stable = {
        key: manifest[key]
        for key in _SITE_CONTENT_KEYS
        if key in manifest
    }
    payload = json.dumps(
        stable,
        sort_keys=True,
        separators=(",", ":"),
        ensure_ascii=False,
        allow_nan=False,
    ).encode("utf-8")
    return "sha256:" + hashlib.sha256(payload).hexdigest()


def _atomic_yaml(path: Path, value: Mapping[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(path.name + ".tmp")
    temporary.write_text(
        yaml.safe_dump(dict(value), sort_keys=False, allow_unicode=True),
        encoding="utf-8",
    )
    os.replace(temporary, path)


def _require_safe_id(value: Any, *, field: str, code: str) -> str:
    text = str(value or "").strip()
    if not _SAFE_ID_RE.fullmatch(text):
        raise AssetContractError(code, f"{field} must use letters, digits, '.', '_' or '-' only")
    return text


def _platform_identity(path: str | Path) -> tuple[str, str]:
    profile_path = Path(path).expanduser().resolve()
    profile = load_yaml_mapping(profile_path)
    platform = profile.get("platform")
    if not isinstance(platform, Mapping):
        raise AssetContractError(
            "site_platform_profile_invalid", "platform profile requires a platform mapping"
        )
    platform_id = _require_safe_id(
        platform.get("name"), field="platform.name", code="site_platform_id_invalid"
    )
    return platform_id, sha256_file(profile_path)


def _required_map_asset(manifest: Mapping[str, Any], asset_id: str) -> Mapping[str, Any]:
    assets = manifest.get("assets")
    if not isinstance(assets, Mapping):
        raise AssetContractError("site_map_assets_invalid", "READY map assets table is invalid")
    record = assets.get(asset_id)
    if not isinstance(record, Mapping) or not record.get("sha256"):
        raise AssetContractError(
            "site_map_asset_missing", f"READY map does not freeze required asset: {asset_id}"
        )
    return record


def _optional_map_asset(manifest: Mapping[str, Any], asset_id: str) -> Mapping[str, Any] | None:
    assets = manifest.get("assets")
    if not isinstance(assets, Mapping):
        return None
    record = assets.get(asset_id)
    return record if isinstance(record, Mapping) and record.get("sha256") else None


def _canonical_map_manifest(maps_root: str | Path, binding: Mapping[str, Any]) -> Path:
    root = Path(maps_root).expanduser().resolve()
    map_id = _require_safe_id(
        binding.get("map_id"), field="map_binding.map_id", code="site_map_id_invalid"
    )
    version_id = _require_safe_id(
        binding.get("map_version_id"),
        field="map_binding.map_version_id",
        code="site_map_version_invalid",
    )
    candidate = (root / map_id / "versions" / version_id / "manifest.yaml").resolve()
    try:
        candidate.relative_to(root)
    except ValueError as exc:
        raise AssetContractError("site_map_path_escape", "resolved map manifest escapes maps root") from exc
    return candidate


def _canonical_route_manifest(map_root: Path, route_id: str, revision: int) -> Path:
    route_id = _require_safe_id(
        route_id, field="routes[].route_id", code="site_route_id_invalid"
    )
    if int(revision) <= 0:
        raise AssetContractError("site_route_revision_invalid", "route revision must be positive")
    root = map_root.resolve()
    candidate = (root / "routes" / route_id / str(int(revision)) / "route.yaml").resolve()
    try:
        candidate.relative_to(root)
    except ValueError as exc:
        raise AssetContractError("site_route_path_escape", "resolved route manifest escapes map root") from exc
    return candidate


def _validate_ready_route(
    route_manifest_path: Path,
    *,
    map_manifest: Mapping[str, Any],
    platform_id: str,
    platform_hash: str,
) -> dict[str, Any]:
    if not route_manifest_path.is_file():
        raise AssetContractError(
            "site_route_manifest_missing", f"route manifest is missing: {route_manifest_path}"
        )
    route = load_yaml_mapping(route_manifest_path)
    route_id = _require_safe_id(
        route.get("route_id"), field="route.route_id", code="site_route_id_invalid"
    )
    revision = int(route.get("revision", 0))
    if revision <= 0:
        raise AssetContractError("site_route_revision_invalid", "route revision must be positive")
    if str(route.get("status", "")).upper() != "READY":
        raise AssetContractError("site_route_not_ready", "Site Package routes must be READY")
    if str(route.get("frame_id", "")) != "map":
        raise AssetContractError("site_route_frame_invalid", "Route frame_id must be map")

    map_binding = route.get("map_binding") or {}
    expected_map = {
        "map_id": str(map_manifest.get("map_id", "")),
        "map_version_id": str(map_manifest.get("map_version_id", "")),
        "map_content_sha256": str(map_manifest.get("map_content_sha256", "")),
    }
    for key, expected in expected_map.items():
        if str(map_binding.get(key, "")) != expected:
            raise AssetContractError(
                "site_route_map_binding_mismatch",
                f"READY route {route_id}:{revision} {key} differs from selected map",
            )

    vehicle = route.get("vehicle_binding") or {}
    if str(vehicle.get("platform_id", "")) != platform_id:
        raise AssetContractError(
            "site_route_vehicle_id_mismatch",
            f"READY route {route_id}:{revision} targets a different vehicle profile",
        )
    if str(vehicle.get("platform_profile_sha256", "")) != platform_hash:
        raise AssetContractError(
            "site_route_vehicle_hash_mismatch",
            f"READY route {route_id}:{revision} vehicle profile hash differs from Site Package",
        )

    route_root = route_manifest_path.parent
    child_hashes = (
        ("route.csv", route.get("route_csv_sha256"), "site_route_csv_hash_mismatch"),
        (
            str((route.get("policy_binding") or {}).get("path", "policy.yaml")),
            (route.get("policy_binding") or {}).get("sha256"),
            "site_route_policy_hash_mismatch",
        ),
        (
            "feasibility_report.json",
            route.get("feasibility_report_sha256"),
            "site_route_feasibility_hash_mismatch",
        ),
        ("preview.geojson", route.get("preview_sha256"), "site_route_preview_hash_mismatch"),
    )
    for relative_text, expected_hash, code in child_hashes:
        relative = Path(str(relative_text))
        if not relative.parts or relative.is_absolute() or ".." in relative.parts:
            raise AssetContractError("site_route_child_path_invalid", "route child path is invalid")
        child = (route_root / relative).resolve()
        try:
            child.relative_to(route_root.resolve())
        except ValueError as exc:
            raise AssetContractError("site_route_child_path_escape", "route child escapes route root") from exc
        if not child.is_file() or not expected_hash or sha256_file(child) != str(expected_hash):
            raise AssetContractError(code, f"READY route child hash mismatch: {relative}")

    feasibility = json.loads((route_root / "feasibility_report.json").read_text(encoding="utf-8"))
    if str(feasibility.get("status", "")).upper() != "PASS":
        raise AssetContractError(
            "site_route_feasibility_not_pass", "READY route feasibility report must be PASS"
        )

    return {
        "route_id": route_id,
        "revision": revision,
        "route_yaml_sha256": sha256_file(route_manifest_path),
    }


@dataclass(frozen=True)
class SitePackageComplianceResult:
    valid: bool
    site_id: str
    site_package_id: str
    state: str
    checks: Mapping[str, bool]
    errors: tuple[str, ...]
    warnings: tuple[str, ...]

    def to_dict(self) -> dict[str, Any]:
        return {
            "valid": self.valid,
            "site_id": self.site_id,
            "site_package_id": self.site_package_id,
            "state": self.state,
            "checks": dict(self.checks),
            "errors": list(self.errors),
            "warnings": list(self.warnings),
        }


def create_site_package(
    sites_root: str | Path,
    *,
    map_manifest_path: str | Path,
    platform_profile_path: str | Path,
    route_dirs: Iterable[str | Path] = (),
    site_package_id: str | None = None,
) -> Path:
    """Create a DRAFT Site Package that binds existing READY map/route assets."""
    map_manifest_path = Path(map_manifest_path).expanduser().resolve()
    map_compliance = validate_map_workspace(map_manifest_path)
    if not map_compliance.valid or map_compliance.state != "READY":
        raise AssetContractError(
            "site_map_not_ready",
            "Site Package requires a compliant READY map: " + ",".join(map_compliance.errors),
        )
    map_manifest = load_yaml_mapping(map_manifest_path)
    expected_map_hash = compute_map_content_sha256(map_manifest)
    if str(map_manifest.get("map_content_sha256", "")) != expected_map_hash:
        raise AssetContractError(
            "site_map_content_identity_invalid", "selected map content identity is invalid"
        )
    site_id = _require_safe_id(
        map_manifest.get("site_id"), field="map.site_id", code="site_id_invalid"
    )
    if str(map_manifest.get("frame_id", "")) != "map":
        raise AssetContractError("site_map_frame_invalid", "selected READY map frame_id must be map")

    platform_id, platform_hash = _platform_identity(platform_profile_path)
    package_id = site_package_id or generate_site_package_id()
    if not _SITE_PACKAGE_ID_RE.fullmatch(package_id):
        raise AssetContractError(
            "site_package_id_invalid", "site_package_id must use sitepkg_YYYYMMDD_HHMMSS_<8 hex>"
        )

    package_root = (
        Path(sites_root).expanduser().resolve()
        / site_id
        / "packages"
        / package_id
    )
    if package_root.exists():
        raise AssetContractError(
            "site_package_exists", f"Site Package already exists: {package_root}"
        )

    localization_pcd = _required_map_asset(map_manifest, "localization_pcd")
    processing_record = _required_map_asset(map_manifest, "processing_record")
    navigation_yaml = _required_map_asset(map_manifest, "navigation_yaml")
    navigation_pgm = _required_map_asset(map_manifest, "navigation_pgm")
    semantic_map = _optional_map_asset(map_manifest, "semantic_map")
    semantic_coverage = _optional_map_asset(map_manifest, "semantic_coverage")
    if semantic_map is not None and semantic_coverage is None:
        raise AssetContractError(
            "site_semantic_coverage_missing",
            "selected READY map has semantic_map without semantic_coverage",
        )

    route_bindings = []
    seen_routes: set[tuple[str, int]] = set()
    for route_dir_value in route_dirs:
        supplied_manifest = Path(route_dir_value).expanduser().resolve() / "route.yaml"
        route = load_yaml_mapping(supplied_manifest)
        route_id = _require_safe_id(
            route.get("route_id"), field="route.route_id", code="site_route_id_invalid"
        )
        revision = int(route.get("revision", 0))
        canonical = _canonical_route_manifest(map_manifest_path.parent, route_id, revision)
        if supplied_manifest != canonical:
            raise AssetContractError(
                "site_route_not_canonical",
                "route must be selected from the chosen map version's canonical routes directory",
            )
        key = (route_id, revision)
        if key in seen_routes:
            raise AssetContractError("site_route_duplicate", f"duplicate route binding: {key}")
        seen_routes.add(key)
        route_bindings.append(
            _validate_ready_route(
                canonical,
                map_manifest=map_manifest,
                platform_id=platform_id,
                platform_hash=platform_hash,
            )
        )
    route_bindings.sort(key=lambda item: (item["route_id"], item["revision"]))

    calibration = map_manifest.get("calibration") or {}
    manifest: dict[str, Any] = {
        "schema_version": 1,
        "site_package_schema": "agt_site_package/v1",
        "site_id": site_id,
        "site_package_id": package_id,
        "state": "DRAFT",
        "created_at": _now(),
        "frame_id": "map",
        "map_binding": {
            "map_id": str(map_manifest.get("map_id", "")),
            "map_version_id": str(map_manifest.get("map_version_id", "")),
            "map_content_sha256": expected_map_hash,
        },
        "calibration_binding": {
            "calibration_id": str(calibration.get("calibration_id", "")),
            "sha256": str(calibration.get("sha256", "")),
        },
        "localization_binding": {
            "map_asset_id": "localization_pcd",
            "map_sha256": str(localization_pcd["sha256"]),
            "processing_asset_id": "processing_record",
            "processing_sha256": str(processing_record["sha256"]),
        },
        "navigation_binding": {
            "yaml_asset_id": "navigation_yaml",
            "yaml_sha256": str(navigation_yaml["sha256"]),
            "pgm_asset_id": "navigation_pgm",
            "pgm_sha256": str(navigation_pgm["sha256"]),
        },
        "vehicle_binding": {
            "platform_id": platform_id,
            "platform_profile_sha256": platform_hash,
        },
        "routes": route_bindings,
        "benchmark_bindings": [],
        "notes": "V25-12A root binding over existing READY map/route assets; no runtime ROS interface.",
    }
    if semantic_map is not None:
        manifest["semantic_binding"] = {
            "map_asset_id": "semantic_map",
            "map_sha256": str(semantic_map["sha256"]),
            "coverage_asset_id": "semantic_coverage",
            "coverage_sha256": str(semantic_coverage["sha256"]),
        }

    manifest["site_package_content_sha256"] = compute_site_package_content_sha256(manifest)
    manifest_path = package_root / "manifest.yaml"
    _atomic_yaml(manifest_path, manifest)
    return manifest_path


def _check_equal(
    checks: dict[str, bool],
    errors: list[str],
    name: str,
    actual: Any,
    expected: Any,
    code: str,
) -> None:
    passed = actual == expected
    checks[name] = bool(passed)
    if not passed:
        errors.append(code)


def validate_site_package(
    manifest_path: str | Path,
    *,
    maps_root: str | Path,
    platform_profile_path: str | Path,
) -> SitePackageComplianceResult:
    """Read-only validation of Site Package identity and all declared child bindings."""
    manifest_path = Path(manifest_path).expanduser().resolve()
    try:
        manifest = load_yaml_mapping(manifest_path)
    except (OSError, ValueError, yaml.YAMLError) as exc:
        return SitePackageComplianceResult(
            False, "", "", "", {}, (f"site_manifest_unreadable:{exc}",), ()
        )

    site_id = str(manifest.get("site_id", ""))
    package_id = str(manifest.get("site_package_id", ""))
    state = str(manifest.get("state", "")).upper()
    checks: dict[str, bool] = {}
    errors: list[str] = []
    warnings: list[str] = []

    checks["schema_valid"] = (
        manifest.get("schema_version") == 1
        and manifest.get("site_package_schema") == "agt_site_package/v1"
    )
    if not checks["schema_valid"]:
        errors.append("site_schema_invalid")
    checks["site_id_valid"] = bool(_SAFE_ID_RE.fullmatch(site_id))
    if not checks["site_id_valid"]:
        errors.append("site_id_invalid")
    checks["package_id_valid"] = bool(_SITE_PACKAGE_ID_RE.fullmatch(package_id))
    if not checks["package_id_valid"]:
        errors.append("site_package_id_invalid")
    checks["frame_valid"] = str(manifest.get("frame_id", "")) == "map"
    if not checks["frame_valid"]:
        errors.append("site_frame_invalid")
    checks["state_valid"] = state in {"DRAFT", "READY", "INVALID", "ARCHIVED"}
    if not checks["state_valid"]:
        errors.append("site_state_invalid")

    expected_content = compute_site_package_content_sha256(manifest)
    _check_equal(
        checks,
        errors,
        "content_identity_valid",
        str(manifest.get("site_package_content_sha256", "")),
        expected_content,
        "site_content_identity_mismatch",
    )

    map_manifest: dict[str, Any] = {}
    map_manifest_path: Path | None = None
    try:
        map_binding = manifest.get("map_binding")
        if not isinstance(map_binding, Mapping):
            raise AssetContractError("site_map_binding_missing", "map_binding mapping is required")
        map_manifest_path = _canonical_map_manifest(maps_root, map_binding)
        compliance = validate_map_workspace(map_manifest_path)
        checks["map_ready_and_compliant"] = compliance.valid and compliance.state == "READY"
        if not checks["map_ready_and_compliant"]:
            errors.append("site_map_not_ready")
        map_manifest = load_yaml_mapping(map_manifest_path)
        _check_equal(checks, errors, "map_site_matches", str(map_manifest.get("site_id", "")), site_id, "site_map_site_mismatch")
        _check_equal(checks, errors, "map_id_matches", str(map_manifest.get("map_id", "")), str(map_binding.get("map_id", "")), "site_map_id_mismatch")
        _check_equal(checks, errors, "map_version_matches", str(map_manifest.get("map_version_id", "")), str(map_binding.get("map_version_id", "")), "site_map_version_mismatch")
        _check_equal(checks, errors, "map_content_matches", str(map_manifest.get("map_content_sha256", "")), str(map_binding.get("map_content_sha256", "")), "site_map_hash_mismatch")
    except (AssetContractError, OSError, ValueError, yaml.YAMLError) as exc:
        checks["map_ready_and_compliant"] = False
        errors.append(getattr(exc, "code", "site_map_validation_error"))

    try:
        platform_id, platform_hash = _platform_identity(platform_profile_path)
        vehicle = manifest.get("vehicle_binding")
        if not isinstance(vehicle, Mapping):
            raise AssetContractError("site_vehicle_binding_missing", "vehicle_binding mapping is required")
        _check_equal(checks, errors, "vehicle_id_matches", platform_id, str(vehicle.get("platform_id", "")), "site_vehicle_id_mismatch")
        _check_equal(checks, errors, "vehicle_hash_matches", platform_hash, str(vehicle.get("platform_profile_sha256", "")), "site_vehicle_hash_mismatch")
    except (AssetContractError, OSError, ValueError, yaml.YAMLError) as exc:
        platform_id, platform_hash = "", ""
        checks["vehicle_hash_matches"] = False
        errors.append(getattr(exc, "code", "site_vehicle_validation_error"))

    if map_manifest:
        try:
            calibration = map_manifest.get("calibration") or {}
            package_calibration = manifest.get("calibration_binding") or {}
            _check_equal(checks, errors, "calibration_id_matches", str(package_calibration.get("calibration_id", "")), str(calibration.get("calibration_id", "")), "site_calibration_id_mismatch")
            _check_equal(checks, errors, "calibration_hash_matches", str(package_calibration.get("sha256", "")), str(calibration.get("sha256", "")), "site_calibration_hash_mismatch")

            localization = manifest.get("localization_binding") or {}
            loc_pcd = _required_map_asset(map_manifest, "localization_pcd")
            loc_processing = _required_map_asset(map_manifest, "processing_record")
            _check_equal(checks, errors, "localization_map_matches", str(localization.get("map_sha256", "")), str(loc_pcd.get("sha256", "")), "site_localization_map_hash_mismatch")
            _check_equal(checks, errors, "localization_processing_matches", str(localization.get("processing_sha256", "")), str(loc_processing.get("sha256", "")), "site_localization_processing_hash_mismatch")

            navigation = manifest.get("navigation_binding") or {}
            nav_yaml = _required_map_asset(map_manifest, "navigation_yaml")
            nav_pgm = _required_map_asset(map_manifest, "navigation_pgm")
            _check_equal(checks, errors, "navigation_yaml_matches", str(navigation.get("yaml_sha256", "")), str(nav_yaml.get("sha256", "")), "site_navigation_yaml_hash_mismatch")
            _check_equal(checks, errors, "navigation_pgm_matches", str(navigation.get("pgm_sha256", "")), str(nav_pgm.get("sha256", "")), "site_navigation_pgm_hash_mismatch")

            package_semantic = manifest.get("semantic_binding")
            map_semantic = _optional_map_asset(map_manifest, "semantic_map")
            map_coverage = _optional_map_asset(map_manifest, "semantic_coverage")
            if map_semantic is None:
                checks["semantic_binding_matches"] = package_semantic is None
                if package_semantic is not None:
                    errors.append("site_semantic_unexpected")
                warnings.append("site_package_has_no_semantic_product")
            else:
                if not isinstance(package_semantic, Mapping) or map_coverage is None:
                    checks["semantic_binding_matches"] = False
                    errors.append("site_semantic_binding_missing")
                else:
                    semantic_ok = (
                        str(package_semantic.get("map_sha256", "")) == str(map_semantic.get("sha256", ""))
                        and str(package_semantic.get("coverage_sha256", "")) == str(map_coverage.get("sha256", ""))
                    )
                    checks["semantic_binding_matches"] = semantic_ok
                    if not semantic_ok:
                        errors.append("site_semantic_hash_mismatch")
        except AssetContractError as exc:
            errors.append(exc.code)

    routes = manifest.get("routes")
    if not isinstance(routes, list):
        checks["routes_valid"] = False
        errors.append("site_routes_invalid")
        routes = []
    else:
        routes_valid = True
        seen: set[tuple[str, int]] = set()
        if map_manifest_path is not None and map_manifest:
            for binding in routes:
                try:
                    if not isinstance(binding, Mapping):
                        raise AssetContractError("site_route_binding_invalid", "route binding must be a mapping")
                    route_id = _require_safe_id(binding.get("route_id"), field="routes[].route_id", code="site_route_id_invalid")
                    revision = int(binding.get("revision", 0))
                    key = (route_id, revision)
                    if key in seen:
                        raise AssetContractError("site_route_duplicate", f"duplicate route binding: {key}")
                    seen.add(key)
                    route_manifest_path = _canonical_route_manifest(map_manifest_path.parent, route_id, revision)
                    actual_binding = _validate_ready_route(
                        route_manifest_path,
                        map_manifest=map_manifest,
                        platform_id=platform_id,
                        platform_hash=platform_hash,
                    )
                    if actual_binding != dict(binding):
                        raise AssetContractError("site_route_hash_mismatch", f"route binding changed: {key}")
                except (AssetContractError, OSError, ValueError, yaml.YAMLError, json.JSONDecodeError) as exc:
                    routes_valid = False
                    errors.append(getattr(exc, "code", "site_route_validation_error"))
        elif routes:
            routes_valid = False
        checks["routes_valid"] = routes_valid

    benchmark_bindings = manifest.get("benchmark_bindings")
    checks["benchmark_bindings_valid"] = isinstance(benchmark_bindings, list)
    if not checks["benchmark_bindings_valid"]:
        errors.append("site_benchmark_bindings_invalid")
    elif benchmark_bindings:
        warnings.append("benchmark_bindings_reserved_for_v25_12f")

    if manifest.get("localization_prior") is not None:
        warnings.append("localization_prior_reserved_for_v25_12d")

    errors = sorted(set(errors))
    warnings = sorted(set(warnings))
    return SitePackageComplianceResult(
        not errors,
        site_id,
        package_id,
        state,
        checks,
        tuple(errors),
        tuple(warnings),
    )


def refresh_site_package(
    manifest_path: str | Path,
    *,
    maps_root: str | Path,
    platform_profile_path: str | Path,
    requested_state: str | None = None,
) -> dict[str, Any]:
    """Revalidate a DRAFT Site Package and optionally promote it one-way to READY."""
    manifest_path = Path(manifest_path).expanduser().resolve()
    manifest = load_yaml_mapping(manifest_path)
    if str(manifest.get("state", "")).upper() == "READY":
        raise AssetContractError(
            "ready_site_package_immutable",
            "READY Site Package is immutable; create a new package identity for binding changes",
        )

    manifest["site_package_content_sha256"] = compute_site_package_content_sha256(manifest)
    _atomic_yaml(manifest_path, manifest)
    result = validate_site_package(
        manifest_path,
        maps_root=maps_root,
        platform_profile_path=platform_profile_path,
    )
    if not result.valid:
        raise AssetContractError(
            "site_package_not_compliant",
            "Site Package validation failed: " + ",".join(result.errors),
        )

    if requested_state is not None:
        state = str(requested_state).upper()
        if state not in {"DRAFT", "READY", "INVALID", "ARCHIVED"}:
            raise AssetContractError("site_state_invalid", f"invalid Site Package state: {state}")
        manifest["state"] = state
    manifest["site_package_content_sha256"] = compute_site_package_content_sha256(manifest)
    _atomic_yaml(manifest_path, manifest)
    return manifest
