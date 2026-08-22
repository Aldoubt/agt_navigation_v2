"""Formal structure-aware Navigation Map materialization for V25 Workbench.

This module converts conservative Ground-only occupancy evidence into the
automatic Generated map used by formal V25 map revisions.  It may resolve only
Ground-only UNKNOWN cells inside structurally valid agricultural aisle geometry.
Existing OCCUPIED evidence, crop-row structural bands, and cells outside the
frozen Site Boundary remain hard blocked.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from .navigation_corridor import CorridorRefinementResult
from .navigation_map_derivation import FREE, OCCUPIED, UNKNOWN, NavigationMapResult
from .site_boundary import SiteBoundary, rasterize_site_boundary


FORMAL_NAVIGATION_MATERIALIZATION_SCHEMA = "agt_structure_aware_navigation_map/v1"


@dataclass(frozen=True)
class StructureAwareNavigationResult:
    """Generated navigation grid plus explicit provenance masks."""

    navigation: NavigationMapResult
    observed_free_mask: np.ndarray
    structure_inferred_free_mask: np.ndarray
    base_hard_occupied_mask: np.ndarray
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
            "base_hard_occupied": int(
                np.count_nonzero(self.base_hard_occupied_mask)
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


def _validate_mask_shape(mask: np.ndarray, shape: tuple[int, int], name: str) -> np.ndarray:
    values = np.asarray(mask, dtype=bool)
    if values.shape != shape:
        raise ValueError(
            f"formal navigation evidence grid shape mismatch: {name} "
            f"has {values.shape}, expected {shape}"
        )
    return values


def materialize_structure_aware_navigation_map(
    navigation: NavigationMapResult,
    corridor: CorridorRefinementResult,
    site_boundary: SiteBoundary,
    *,
    frame_id: str = "map",
) -> StructureAwareNavigationResult:
    """Materialize the automatic structure-aware Generated Navigation Map.

    Precedence is deterministic and fail-closed:

    1. outside Site Boundary -> OCCUPIED
    2. Ground-only OCCUPIED -> OCCUPIED
    3. row structural band -> OCCUPIED
    4. Ground-only FREE -> FREE
    5. Ground-only UNKNOWN inside valid aisle geometry -> FREE
    6. otherwise -> UNKNOWN

    Agricultural structure therefore repairs missing evidence only.  It never
    automatically frees a cell that Ground-only derivation already classified as
    OCCUPIED.
    """

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
    base_occupied = occupancy_source == OCCUPIED
    base_unknown = occupancy_source == UNKNOWN
    outside_boundary = ~inside_boundary

    structure_inferred_free = (
        base_unknown
        & aisle
        & ~row_band
        & inside_boundary
    )

    generated_occupancy = np.full(shape, UNKNOWN, dtype=np.uint8)
    generated_occupancy[base_free] = FREE
    generated_occupancy[structure_inferred_free] = FREE
    generated_occupancy[base_occupied | row_band | outside_boundary] = OCCUPIED

    generated = NavigationMapResult(
        **{**navigation.__dict__, "occupancy": generated_occupancy}
    )

    return StructureAwareNavigationResult(
        navigation=generated,
        observed_free_mask=base_free & ~row_band & inside_boundary,
        structure_inferred_free_mask=structure_inferred_free,
        base_hard_occupied_mask=base_occupied,
        row_structural_blocked_mask=row_band,
        site_boundary_blocked_mask=outside_boundary,
        unresolved_unknown_mask=generated_occupancy == UNKNOWN,
    )
