"""Safe formal override replay for V25 Accepted Navigation Maps.

Formal human corrections remain ordered world-coordinate polygons.  FORCE_FREE
may correct independently evidenced automatic errors, but it may never create
FREE cells outside the frozen Site Boundary or inside the current crop-row
structural band.  FORCE_OCCUPIED remains a conservative blocking operation.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Iterable, Mapping

import numpy as np

from .navigation_corridor import CorridorRefinementResult
from .navigation_map_derivation import (
    NavigationMapResult,
    apply_navigation_overrides,
)
from .site_boundary import SiteBoundary, rasterize_site_boundary


@dataclass(frozen=True)
class FormalOverrideReplayResult:
    navigation: NavigationMapResult
    force_free_changed_cell_count: int
    force_occupied_changed_cell_count: int
    force_free_area_m2: float
    force_occupied_area_m2: float


def _validated_row_band(
    corridor: CorridorRefinementResult,
    shape: tuple[int, int],
) -> np.ndarray:
    row_band = np.asarray(corridor.row_structural_band, dtype=bool)
    if row_band.shape != shape:
        raise ValueError(
            "formal navigation override grid shape mismatch: "
            f"row_structural_band has {row_band.shape}, expected {shape}"
        )
    return row_band


def replay_formal_navigation_overrides(
    generated: NavigationMapResult,
    corridor: CorridorRefinementResult,
    site_boundary: SiteBoundary,
    overrides: Iterable[Mapping[str, object]],
    *,
    frame_id: str = "map",
) -> FormalOverrideReplayResult:
    """Replay ordered formal overrides while preserving hard geometric invariants."""

    site_boundary.validate(expected_frame_id=frame_id)
    shape = np.asarray(generated.occupancy).shape
    expected_shape = (int(generated.height), int(generated.width))
    if shape != expected_shape:
        raise ValueError(
            "formal navigation override grid shape mismatch: "
            f"navigation occupancy has {shape}, expected {expected_shape}"
        )

    row_band = _validated_row_band(corridor, shape)
    inside_boundary = rasterize_site_boundary(
        site_boundary,
        generated,
        expected_frame_id=frame_id,
    )
    if inside_boundary.shape != shape:
        raise ValueError("formal navigation override grid shape mismatch: site_boundary")

    current = generated
    force_free_changed = 0
    force_occupied_changed = 0

    for index, record in enumerate(overrides):
        mode = str(record.get("mode", "")).strip().lower()
        if mode not in {"force_free", "force_occupied"}:
            raise ValueError(
                "formal override mode must be force_free or force_occupied "
                f"at index {index}"
            )

        candidate_occupancy = apply_navigation_overrides(current, [record])
        changed = candidate_occupancy != current.occupancy

        if mode == "force_free":
            if np.any(changed & ~inside_boundary):
                raise ValueError("FORCE_FREE_SITE_BOUNDARY_CONFLICT")
            if np.any(changed & row_band):
                raise ValueError("FORCE_FREE_ROW_STRUCTURAL_CONFLICT")
            force_free_changed += int(np.count_nonzero(changed))
        else:
            force_occupied_changed += int(np.count_nonzero(changed))

        current = NavigationMapResult(
            **{**current.__dict__, "occupancy": candidate_occupancy}
        )

    cell_area_m2 = float(generated.resolution_m) ** 2
    return FormalOverrideReplayResult(
        navigation=current,
        force_free_changed_cell_count=force_free_changed,
        force_occupied_changed_cell_count=force_occupied_changed,
        force_free_area_m2=float(force_free_changed) * cell_area_m2,
        force_occupied_area_m2=float(force_occupied_changed) * cell_area_m2,
    )
