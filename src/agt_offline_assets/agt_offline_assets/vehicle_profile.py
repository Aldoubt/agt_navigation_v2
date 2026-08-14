"""Canonical execution-vehicle profile adapter for offline route production.

This module reads ``profiles/platforms/<platform>.yaml`` as the single source of
vehicle geometry and kinematics.  It intentionally does not create a second
vehicle configuration format for the Workbench or Route Asset pipeline.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from pathlib import Path
from typing import Any, Mapping

import numpy as np
import yaml

from .contracts import AssetContractError, sha256_file


@dataclass(frozen=True)
class CanonicalVehicleProfile:
    profile_id: str
    profile_path: str
    profile_sha256: str
    kinematics: str
    footprint_frame: str
    base_frame: str
    physical_length_m: float
    physical_width_m: float
    footprint_xy: tuple[tuple[float, float], ...]
    navigation_footprint_xy: tuple[tuple[float, float], ...]
    navigation_width_m: float
    navigation_length_m: float
    wheel_base_m: float | None
    track_width_m: float | None
    wheel_diameter_m: float | None
    ground_clearance_m: float | None
    minimum_turning_radius_m: float
    minimum_turning_radius_verified: bool
    maximum_steering_angle_rad: float | None
    maximum_steering_angle_deg: float | None
    allow_in_place_rotation: bool
    max_forward_velocity_mps: float | None
    max_reverse_velocity_mps: float | None
    max_angular_velocity_rps: float | None
    manufacturer_maximum_speed_mps: float | None
    route_acceptance_enabled: bool
    preview_planning_enabled: bool
    blocked_reason: str

    @property
    def planning_preview_ready(self) -> bool:
        """Whether offline ordering/connector preview has enough kinematic truth."""
        if not self.preview_planning_enabled:
            return False
        if self.kinematics == "ackermann":
            if not self.minimum_turning_radius_verified:
                return False
            if self.minimum_turning_radius_m <= 0.0:
                return False
        return bool(self.navigation_footprint_xy)

    @property
    def route_feasibility_ready(self) -> bool:
        """Whether the profile may participate in formal vehicle-READY promotion."""
        if not self.route_acceptance_enabled:
            return False
        if self.kinematics == "ackermann" and not self.minimum_turning_radius_verified:
            return False
        return bool(self.navigation_footprint_xy)


def _mapping(value: Any, *, code: str, field: str) -> Mapping[str, Any]:
    if not isinstance(value, Mapping):
        raise AssetContractError(code, f"{field} must be a mapping")
    return value


def _positive(value: Any, *, code: str, field: str) -> float:
    numeric = float(value)
    if not math.isfinite(numeric) or numeric <= 0.0:
        raise AssetContractError(code, f"{field} must be finite and > 0")
    return numeric


def _optional_nonnegative(value: Any) -> float | None:
    if value is None:
        return None
    numeric = float(value)
    if not math.isfinite(numeric) or numeric < 0.0:
        return None
    return numeric


def _optional_positive(value: Any) -> float | None:
    if value is None:
        return None
    numeric = float(value)
    if not math.isfinite(numeric) or numeric <= 0.0:
        return None
    return numeric


def _polygon(value: Any, *, code: str, field: str) -> tuple[tuple[float, float], ...]:
    if not isinstance(value, (list, tuple)) or len(value) < 3:
        raise AssetContractError(code, f"{field} must contain at least three XY vertices")
    points: list[tuple[float, float]] = []
    for vertex in value:
        if not isinstance(vertex, (list, tuple)) or len(vertex) != 2:
            raise AssetContractError(code, f"{field} vertices must be [x, y]")
        x, y = float(vertex[0]), float(vertex[1])
        if not math.isfinite(x) or not math.isfinite(y):
            raise AssetContractError(code, f"{field} contains non-finite coordinates")
        points.append((x, y))
    return tuple(points)


def _polygon_extent(points: tuple[tuple[float, float], ...]) -> tuple[float, float]:
    arr = np.asarray(points, dtype=np.float64)
    length = float(np.max(arr[:, 0]) - np.min(arr[:, 0]))
    width = float(np.max(arr[:, 1]) - np.min(arr[:, 1]))
    return length, width


def _velocity(platform: Mapping[str, Any], limits: Mapping[str, Any], *keys: str) -> float | None:
    for key in keys:
        if key in limits:
            return _optional_nonnegative(limits.get(key))
        if key in platform:
            return _optional_nonnegative(platform.get(key))
    return None


def load_canonical_vehicle_profile(path: str | Path) -> CanonicalVehicleProfile:
    """Load one canonical platform profile and normalize route-relevant fields."""
    profile_path = Path(path).expanduser().resolve()
    if not profile_path.is_file():
        raise AssetContractError("vehicle_profile_missing", f"profile does not exist: {profile_path}")
    raw = yaml.safe_load(profile_path.read_text(encoding="utf-8")) or {}
    if not isinstance(raw, Mapping):
        raise AssetContractError("vehicle_profile_invalid", "vehicle profile must be a YAML mapping")
    platform = _mapping(raw.get("platform"), code="vehicle_platform_missing", field="platform")
    geometry = _mapping(platform.get("geometry"), code="vehicle_geometry_missing", field="platform.geometry")
    limits = platform.get("limits") or {}
    if not isinstance(limits, Mapping):
        raise AssetContractError("vehicle_limits_invalid", "platform.limits must be a mapping")

    profile_id = str(platform.get("name", "")).strip()
    if not profile_id:
        raise AssetContractError("vehicle_name_missing", "platform.name is required")
    kinematics = str(platform.get("kinematics", "")).strip()
    if kinematics not in {"tracked_differential", "differential", "ackermann"}:
        raise AssetContractError(
            "vehicle_kinematics_unsupported",
            f"unsupported platform.kinematics: {kinematics or '<empty>'}",
        )

    physical_length = _positive(geometry.get("length"), code="vehicle_length_invalid", field="geometry.length")
    physical_width = _positive(geometry.get("width"), code="vehicle_width_invalid", field="geometry.width")
    footprint = _polygon(geometry.get("footprint"), code="vehicle_footprint_invalid", field="geometry.footprint")
    navigation_raw = geometry.get("navigation_footprint") or geometry.get("footprint")
    navigation_footprint = _polygon(
        navigation_raw,
        code="vehicle_navigation_footprint_invalid",
        field="geometry.navigation_footprint",
    )
    navigation_length, navigation_width = _polygon_extent(navigation_footprint)

    wheel_base = _optional_positive(geometry.get("wheel_base") or geometry.get("wheelbase"))
    track_width = _optional_positive(
        geometry.get("track_width") or geometry.get("wheel_track") or geometry.get("effective_track_width")
    )
    wheel_diameter = _optional_positive(geometry.get("wheel_diameter"))
    if wheel_diameter is None:
        wheel_radius = _optional_positive(geometry.get("wheel_radius"))
        wheel_diameter = None if wheel_radius is None else 2.0 * wheel_radius
    ground_clearance = _optional_nonnegative(geometry.get("ground_clearance"))
    steering_rad = _optional_positive(geometry.get("max_steering_angle_rad"))
    steering_deg = _optional_positive(geometry.get("max_steering_angle_deg"))
    if steering_rad is None and steering_deg is not None:
        steering_rad = math.radians(steering_deg)
    if steering_deg is None and steering_rad is not None:
        steering_deg = math.degrees(steering_rad)

    coverage = platform.get("coverage_repair") or {}
    if not isinstance(coverage, Mapping):
        coverage = {}
    allow_in_place = bool(coverage.get("allow_in_place_rotation", False))

    if kinematics == "ackermann":
        radius_verified = bool(geometry.get("min_turning_radius_verified", False))
        radius = float(geometry.get("min_turning_radius", 0.0) or 0.0)
        if radius_verified and (not math.isfinite(radius) or radius <= 0.0):
            raise AssetContractError(
                "vehicle_turning_radius_invalid",
                "verified Ackermann minimum turning radius must be > 0",
            )
        allow_in_place = False
    else:
        # Differential/tracked vehicles are kinematically capable of zero-radius
        # body rotation; this is distinct from whether a selected controller
        # chooses to use it for a particular route.
        radius = 0.0
        radius_verified = True
        if "allow_in_place_rotation" not in coverage:
            allow_in_place = True

    route_acceptance = platform.get("route_acceptance") or {}
    if route_acceptance and not isinstance(route_acceptance, Mapping):
        raise AssetContractError("vehicle_route_acceptance_invalid", "route_acceptance must be a mapping")
    if route_acceptance:
        acceptance_enabled = bool(route_acceptance.get("enabled", True))
        preview_enabled = bool(route_acceptance.get("preview_planning_enabled", acceptance_enabled))
        blocked_reason = str(route_acceptance.get("blocked_reason", "")).strip()
    else:
        acceptance_enabled = True
        preview_enabled = True
        blocked_reason = ""
    if kinematics == "ackermann" and not radius_verified:
        acceptance_enabled = False
        preview_enabled = False
        if not blocked_reason:
            blocked_reason = "Ackermann minimum turning radius is not verified"

    manufacturer_performance = platform.get("manufacturer_performance") or {}
    if not isinstance(manufacturer_performance, Mapping):
        manufacturer_performance = {}

    return CanonicalVehicleProfile(
        profile_id=profile_id,
        profile_path=profile_path.as_posix(),
        profile_sha256=sha256_file(profile_path),
        kinematics=kinematics,
        footprint_frame=str(platform.get("footprint_frame", "base_footprint")),
        base_frame=str(platform.get("base_frame", "base_link")),
        physical_length_m=physical_length,
        physical_width_m=physical_width,
        footprint_xy=footprint,
        navigation_footprint_xy=navigation_footprint,
        navigation_width_m=navigation_width,
        navigation_length_m=navigation_length,
        wheel_base_m=wheel_base,
        track_width_m=track_width,
        wheel_diameter_m=wheel_diameter,
        ground_clearance_m=ground_clearance,
        minimum_turning_radius_m=radius,
        minimum_turning_radius_verified=radius_verified,
        maximum_steering_angle_rad=steering_rad,
        maximum_steering_angle_deg=steering_deg,
        allow_in_place_rotation=allow_in_place,
        max_forward_velocity_mps=_velocity(platform, limits, "max_forward_velocity", "max_linear_velocity"),
        max_reverse_velocity_mps=_velocity(platform, limits, "max_reverse_velocity"),
        max_angular_velocity_rps=_velocity(platform, limits, "max_angular_velocity"),
        manufacturer_maximum_speed_mps=_optional_nonnegative(
            manufacturer_performance.get("maximum_speed_mps")
        ),
        route_acceptance_enabled=acceptance_enabled,
        preview_planning_enabled=preview_enabled,
        blocked_reason=blocked_reason,
    )


def vehicle_profile_to_route_binding(profile: CanonicalVehicleProfile) -> dict[str, Any]:
    """Return hash-bound route metadata without duplicating the source profile."""
    return {
        "platform_id": profile.profile_id,
        "platform_profile_sha256": profile.profile_sha256,
        "kinematics": profile.kinematics,
        "navigation_footprint": [list(vertex) for vertex in profile.navigation_footprint_xy],
        "navigation_width_m": profile.navigation_width_m,
        "navigation_length_m": profile.navigation_length_m,
        "wheel_base_m": profile.wheel_base_m,
        "track_width_m": profile.track_width_m,
        "wheel_diameter_m": profile.wheel_diameter_m,
        "ground_clearance_m": profile.ground_clearance_m,
        "minimum_turning_radius_m": profile.minimum_turning_radius_m,
        "minimum_turning_radius_verified": profile.minimum_turning_radius_verified,
        "maximum_steering_angle_rad": profile.maximum_steering_angle_rad,
        "allow_in_place_rotation": profile.allow_in_place_rotation,
        "preview_planning_ready": profile.planning_preview_ready,
        "route_feasibility_ready": profile.route_feasibility_ready,
        "blocked_reason": profile.blocked_reason,
    }
