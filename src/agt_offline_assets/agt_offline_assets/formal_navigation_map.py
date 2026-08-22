"""Formal structure-aware Navigation Map materialization for V25 Workbench.

The formal materializer keeps raw Ground-only evidence auditable while allowing
agricultural structure to repair two narrowly defined classes of defects:

* Ground-only UNKNOWN inside a validated aisle geometry; and
* weak, locally supported sensor-obstacle cells that form a short gap along the
  crop-row direction.

Strong sensor obstacles, terrain geometry failures, crop-row structural bands,
and cells outside the frozen Site Boundary remain hard blocked.
"""

from __future__ import annotations

from dataclasses import dataclass
from math import ceil

import numpy as np

from .navigation_corridor import CorridorRefinementResult
from .navigation_map_derivation import FREE, OCCUPIED, UNKNOWN, NavigationMapResult
from .site_boundary import SiteBoundary, rasterize_site_boundary


FORMAL_NAVIGATION_MATERIALIZATION_SCHEMA = "agt_structure_aware_navigation_map/v2"


@dataclass(frozen=True)
class StructureAwareNavigationConfig:
    """Conservative policy for recovering weak occupied evidence.

    A direct sensor obstacle is considered soft only when both its absolute
    obstacle count and obstacle/point ratio remain below the configured limits.
    Soft occupied cells are not freed merely because they lie in an aisle: they
    must also have ground support and form a short longitudinal gap between
    already-free cells along the crop-row direction.
    """

    enable_soft_occupied_recovery: bool = True
    soft_obstacle_max_count: int = 4
    soft_obstacle_max_ratio: float = 0.05
    soft_recovery_min_ground_support_points: int = 2
    soft_recovery_max_gap_m: float = 0.60

    def validate(self) -> None:
        if self.soft_obstacle_max_count < 1:
            raise ValueError("soft_obstacle_max_count must be >= 1")
        if not 0.0 <= self.soft_obstacle_max_ratio <= 1.0:
            raise ValueError("soft_obstacle_max_ratio must be in [0, 1]")
        if self.soft_recovery_min_ground_support_points < 1:
            raise ValueError("soft_recovery_min_ground_support_points must be >= 1")
        if self.soft_recovery_max_gap_m < 0.0:
            raise ValueError("soft_recovery_max_gap_m must be >= 0")


@dataclass(frozen=True)
class StructureAwareNavigationResult:
    """Generated navigation grid plus explicit provenance masks."""

    navigation: NavigationMapResult
    observed_free_mask: np.ndarray
    structure_inferred_free_mask: np.ndarray
    structure_inferred_unknown_free_mask: np.ndarray
    structure_recovered_soft_occupied_mask: np.ndarray
    base_hard_occupied_mask: np.ndarray
    base_soft_occupied_mask: np.ndarray
    row_structural_blocked_mask: np.ndarray
    site_boundary_blocked_mask: np.ndarray
    unresolved_unknown_mask: np.ndarray
    schema: str = FORMAL_NAVIGATION_MATERIALIZATION_SCHEMA

    def counts(self) -> dict[str, int]:
        return {
            "observed_free": int(np.count_nonzero(self.observed_free_mask)),
            "structure_inferred_free": int(
                np.count_nonzero(self.structure_inferred_free_mask)
            ),
            "structure_inferred_unknown_free": int(
                np.count_nonzero(self.structure_inferred_unknown_free_mask)
            ),
            "structure_recovered_soft_occupied": int(
                np.count_nonzero(self.structure_recovered_soft_occupied_mask)
            ),
            "base_hard_occupied": int(
                np.count_nonzero(self.base_hard_occupied_mask)
            ),
            "base_soft_occupied": int(
                np.count_nonzero(self.base_soft_occupied_mask)
            ),
            "row_structural_blocked": int(
                np.count_nonzero(self.row_structural_blocked_mask)
            ),
            "site_boundary_blocked": int(
                np.count_nonzero(self.site_boundary_blocked_mask)
            ),
            "unresolved_unknown": int(
                np.count_nonzero(self.unresolved_unknown_mask)
            ),
        }


def _require_scipy():
    try:
        from scipy import ndimage
    except ImportError as exc:
        raise RuntimeError("formal navigation materialization requires scipy") from exc
    return ndimage


def _validate_mask_shape(mask: np.ndarray, shape: tuple[int, int], name: str) -> np.ndarray:
    values = np.asarray(mask, dtype=bool)
    if values.shape != shape:
        raise ValueError(
            f"formal navigation evidence grid shape mismatch: {name} "
            f"has {values.shape}, expected {shape}"
        )
    return values


def _normalize_direction(direction_xy) -> np.ndarray | None:
    if direction_xy is None:
        return None
    direction = np.asarray(direction_xy, dtype=np.float64).reshape(2)
    norm = float(np.linalg.norm(direction))
    if not np.isfinite(norm) or norm <= 1.0e-12:
        raise ValueError("row_direction_xy must be finite and non-zero")
    return direction / norm


def _shift_from_neighbor(mask: np.ndarray, dr: int, dc: int) -> np.ndarray:
    """Return output[r,c] = mask[r+dr,c+dc] without wraparound."""

    output = np.zeros(mask.shape, dtype=bool)
    height, width = mask.shape
    dst_r0 = max(0, -dr)
    dst_r1 = min(height, height - dr)
    dst_c0 = max(0, -dc)
    dst_c1 = min(width, width - dc)
    if dst_r0 >= dst_r1 or dst_c0 >= dst_c1:
        return output
    src_r0 = dst_r0 + dr
    src_r1 = dst_r1 + dr
    src_c0 = dst_c0 + dc
    src_c1 = dst_c1 + dc
    output[dst_r0:dst_r1, dst_c0:dst_c1] = mask[
        src_r0:src_r1, src_c0:src_c1
    ]
    return output


def _nearest_anchor_distance(
    seed_free: np.ndarray,
    allowed_path: np.ndarray,
    direction_xy: np.ndarray,
    *,
    sign: int,
    resolution_m: float,
    maximum_distance_m: float,
) -> np.ndarray:
    """Distance to the first free anchor while walking along one row direction."""

    distance = np.full(seed_free.shape, np.inf, dtype=np.float64)
    clear = np.ones(seed_free.shape, dtype=bool)
    max_steps = max(1, int(ceil(maximum_distance_m / resolution_m)) + 1)
    seen_offsets: set[tuple[int, int]] = set()
    dx, dy = float(direction_xy[0]), float(direction_xy[1])
    for step in range(1, max_steps + 1):
        dc = int(round(sign * step * dx))
        dr = int(round(sign * step * dy))
        if dr == 0 and dc == 0:
            continue
        offset = (dr, dc)
        if offset in seen_offsets:
            continue
        seen_offsets.add(offset)
        path_here = _shift_from_neighbor(allowed_path, dr, dc)
        clear &= path_here
        anchor_here = clear & _shift_from_neighbor(seed_free, dr, dc)
        first = np.isinf(distance) & anchor_here
        distance[first] = float(step) * resolution_m
    return distance


def _directional_soft_recovery(
    *,
    seed_free: np.ndarray,
    soft_candidate: np.ndarray,
    allowed_path: np.ndarray,
    row_direction_xy,
    resolution_m: float,
    maximum_gap_m: float,
) -> np.ndarray:
    direction = _normalize_direction(row_direction_xy)
    if direction is None or maximum_gap_m <= 0.0 or not np.any(soft_candidate):
        return np.zeros(seed_free.shape, dtype=bool)

    positive = _nearest_anchor_distance(
        seed_free,
        allowed_path,
        direction,
        sign=1,
        resolution_m=resolution_m,
        maximum_distance_m=maximum_gap_m,
    )
    negative = _nearest_anchor_distance(
        seed_free,
        allowed_path,
        direction,
        sign=-1,
        resolution_m=resolution_m,
        maximum_distance_m=maximum_gap_m,
    )
    # For N blocked cells, center-to-center anchor distance is (N+1)*resolution.
    # The +resolution term therefore admits a physical gap of at most max_gap_m.
    bridge = (
        np.isfinite(positive)
        & np.isfinite(negative)
        & ((positive + negative) <= maximum_gap_m + resolution_m + 1.0e-12)
    )
    return soft_candidate & bridge


def _occupied_provenance(
    navigation: NavigationMapResult,
    occupancy_source: np.ndarray,
    policy: StructureAwareNavigationConfig,
) -> tuple[np.ndarray, np.ndarray]:
    """Split Ground-only OCCUPIED into hard and potentially recoverable masks."""

    base_occupied = occupancy_source == OCCUPIED
    obstacle_count = np.asarray(navigation.obstacle_count, dtype=np.int32)
    point_count = np.asarray(navigation.point_count, dtype=np.int32)
    ground_valid = np.asarray(navigation.ground_valid, dtype=bool)
    slope = np.asarray(navigation.slope_deg, dtype=np.float64)
    step = np.asarray(navigation.step_m, dtype=np.float64)

    direct_obstacle = obstacle_count >= int(navigation.config.minimum_obstacle_points)
    obstacle_ratio = obstacle_count.astype(np.float64) / np.maximum(
        point_count.astype(np.float64), 1.0
    )
    geometry_hard = (
        ground_valid
        & (point_count > 0)
        & (
            (slope > float(navigation.config.maximum_slope_deg))
            | (step > float(navigation.config.maximum_step_m))
        )
    )
    soft_direct = (
        direct_obstacle
        & ~geometry_hard
        & (obstacle_count <= int(policy.soft_obstacle_max_count))
        & (obstacle_ratio <= float(policy.soft_obstacle_max_ratio))
    )
    hard_direct = geometry_hard | (direct_obstacle & ~soft_direct)

    padding_cells = int(
        ceil(float(navigation.config.obstacle_padding_m) / float(navigation.resolution_m))
    )
    if padding_cells > 0 and np.any(hard_direct):
        ndimage = _require_scipy()
        hard_with_padding = ndimage.maximum_filter(
            hard_direct.astype(np.uint8),
            size=2 * padding_cells + 1,
            mode="constant",
        ).astype(bool)
    else:
        hard_with_padding = hard_direct

    hard_occupied = base_occupied & hard_with_padding
    soft_occupied = base_occupied & ~hard_occupied
    return hard_occupied, soft_occupied


def materialize_structure_aware_navigation_map(
    navigation: NavigationMapResult,
    corridor: CorridorRefinementResult,
    site_boundary: SiteBoundary,
    *,
    frame_id: str = "map",
    row_direction_xy=None,
    config: StructureAwareNavigationConfig | None = None,
) -> StructureAwareNavigationResult:
    """Materialize the automatic structure-aware Generated Navigation Map.

    Precedence is deterministic and fail-closed:

    1. outside Site Boundary -> OCCUPIED
    2. hard Ground/terrain evidence -> OCCUPIED
    3. row structural band -> OCCUPIED
    4. Ground-only FREE -> FREE
    5. Ground-only UNKNOWN inside valid aisle geometry -> FREE
    6. weak sensor OCCUPIED may become FREE only when it is a short,
       ground-supported longitudinal gap inside the same aisle
    7. unrecovered soft OCCUPIED remains OCCUPIED
    8. otherwise -> UNKNOWN
    """

    policy = config or StructureAwareNavigationConfig()
    policy.validate()
    site_boundary.validate(expected_frame_id=frame_id)

    occupancy_source = np.asarray(navigation.occupancy, dtype=np.uint8)
    shape = occupancy_source.shape
    expected_shape = (int(navigation.height), int(navigation.width))
    if shape != expected_shape:
        raise ValueError(
            "formal navigation evidence grid shape mismatch: navigation occupancy "
            f"has {shape}, expected {expected_shape}"
        )

    aisle = _validate_mask_shape(
        corridor.aisle_geometric_envelope,
        shape,
        "aisle_geometric_envelope",
    )
    row_band = _validate_mask_shape(
        corridor.row_structural_band,
        shape,
        "row_structural_band",
    )
    inside_boundary = rasterize_site_boundary(
        site_boundary,
        navigation,
        expected_frame_id=frame_id,
    )
    inside_boundary = _validate_mask_shape(
        inside_boundary,
        shape,
        "site_boundary",
    )

    base_free = occupancy_source == FREE
    base_unknown = occupancy_source == UNKNOWN
    outside_boundary = ~inside_boundary
    hard_occupied, soft_occupied = _occupied_provenance(
        navigation, occupancy_source, policy
    )

    inferred_unknown = base_unknown & aisle & ~row_band & inside_boundary
    seed_free = (base_free | inferred_unknown) & aisle & ~row_band & inside_boundary
    soft_candidate = (
        soft_occupied
        & aisle
        & ~row_band
        & inside_boundary
        & np.asarray(navigation.ground_valid, dtype=bool)
        & (
            np.asarray(navigation.ground_support_count, dtype=np.int32)
            >= int(policy.soft_recovery_min_ground_support_points)
        )
    )
    allowed_path = (
        aisle
        & inside_boundary
        & ~row_band
        & ~hard_occupied
    )
    if policy.enable_soft_occupied_recovery:
        recovered_soft = _directional_soft_recovery(
            seed_free=seed_free,
            soft_candidate=soft_candidate,
            allowed_path=allowed_path,
            row_direction_xy=row_direction_xy,
            resolution_m=float(navigation.resolution_m),
            maximum_gap_m=float(policy.soft_recovery_max_gap_m),
        )
    else:
        recovered_soft = np.zeros(shape, dtype=bool)

    structure_inferred_free = inferred_unknown | recovered_soft

    generated_occupancy = np.full(shape, UNKNOWN, dtype=np.uint8)
    generated_occupancy[base_free] = FREE
    generated_occupancy[inferred_unknown | recovered_soft] = FREE
    generated_occupancy[hard_occupied | soft_occupied | row_band | outside_boundary] = OCCUPIED
    # Recovery is applied after the conservative occupied assignment, but hard
    # masks remain impossible to override because recovered_soft excludes them.
    generated_occupancy[recovered_soft] = FREE
    generated_occupancy[row_band | outside_boundary | hard_occupied] = OCCUPIED

    generated = NavigationMapResult(
        **{**navigation.__dict__, "occupancy": generated_occupancy}
    )

    return StructureAwareNavigationResult(
        navigation=generated,
        observed_free_mask=base_free & ~row_band & inside_boundary,
        structure_inferred_free_mask=structure_inferred_free,
        structure_inferred_unknown_free_mask=inferred_unknown,
        structure_recovered_soft_occupied_mask=recovered_soft,
        base_hard_occupied_mask=hard_occupied,
        base_soft_occupied_mask=soft_occupied,
        row_structural_blocked_mask=row_band,
        site_boundary_blocked_mask=outside_boundary,
        unresolved_unknown_mask=generated_occupancy == UNKNOWN,
    )
