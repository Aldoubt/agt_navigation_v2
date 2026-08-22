"""Safe formal override replay for V25 Accepted Navigation Maps.

Formal human corrections remain ordered world-coordinate polygons. FORCE_FREE
may correct independently evidenced automatic errors, but it may never create
FREE cells outside the frozen Site Boundary or inside the current crop-row
structural band. FORCE_OCCUPIED remains a conservative blocking operation.

Replay also emits per-override diagnostics so the Workbench can distinguish a
recorded polygon from the cells that actually changed in Accepted.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Iterable, Mapping

import numpy as np

from .navigation_corridor import CorridorRefinementResult
from .navigation_map_derivation import (
    FREE,
    OCCUPIED,
    UNKNOWN,
    NavigationMapResult,
    _polygon_inside,
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
    override_diagnostics: tuple[dict[str, object], ...] = ()


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


def _override_mask(
    generated: NavigationMapResult,
    record: Mapping[str, object],
) -> np.ndarray:
    columns = np.arange(generated.width, dtype=np.float64)
    rows = np.arange(generated.height, dtype=np.float64)
    xx, yy = np.meshgrid(
        generated.origin_x_m + (columns + 0.5) * generated.resolution_m,
        generated.origin_y_m + (rows + 0.5) * generated.resolution_m,
    )
    return _polygon_inside(
        xx.reshape(-1),
        yy.reshape(-1),
        record.get("polygon_xy", []),
    ).reshape(generated.occupancy.shape)


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
    diagnostics: list[dict[str, object]] = []

    for index, record in enumerate(overrides):
        mode = str(record.get("mode", "")).strip().lower()
        if mode not in {"force_free", "force_occupied"}:
            raise ValueError(
                "formal override mode must be force_free or force_occupied "
                f"at index {index}"
            )

        requested = _override_mask(current, record)
        before = np.asarray(current.occupancy, dtype=np.uint8)
        target = FREE if mode == "force_free" else OCCUPIED
        changed = requested & (before != target)
        blocked_boundary = requested & ~inside_boundary if mode == "force_free" else np.zeros(shape, dtype=bool)
        blocked_row = requested & row_band if mode == "force_free" else np.zeros(shape, dtype=bool)
        blocked_boundary_count = int(np.count_nonzero(blocked_boundary))
        blocked_row_count = int(np.count_nonzero(blocked_row))

        if mode == "force_free":
            if blocked_boundary_count:
                raise ValueError(
                    "FORCE_FREE_SITE_BOUNDARY_CONFLICT: "
                    f"{blocked_boundary_count} requested cells are outside Site Boundary"
                )
            if blocked_row_count:
                raise ValueError(
                    "FORCE_FREE_ROW_STRUCTURAL_CONFLICT: "
                    f"{blocked_row_count} requested cells overlap row structural band"
                )

        candidate_occupancy = apply_navigation_overrides(current, [record])
        effective_changed = candidate_occupancy != before
        changed_count = int(np.count_nonzero(effective_changed))

        diagnostic: dict[str, object] = {
            "id": str(record.get("id", f"override_{index + 1:03d}")),
            "mode": mode,
            "requested_cell_count": int(np.count_nonzero(requested)),
            "effective_changed_cell_count": changed_count,
            "already_target_cell_count": int(np.count_nonzero(requested & (before == target))),
            "blocked_site_boundary_cell_count": blocked_boundary_count,
            "blocked_row_structural_cell_count": blocked_row_count,
            "unknown_to_free_cell_count": int(
                np.count_nonzero(effective_changed & (before == UNKNOWN) & (candidate_occupancy == FREE))
            ),
            "occupied_to_free_cell_count": int(
                np.count_nonzero(effective_changed & (before == OCCUPIED) & (candidate_occupancy == FREE))
            ),
            "free_to_occupied_cell_count": int(
                np.count_nonzero(effective_changed & (before == FREE) & (candidate_occupancy == OCCUPIED))
            ),
            "unknown_to_occupied_cell_count": int(
                np.count_nonzero(effective_changed & (before == UNKNOWN) & (candidate_occupancy == OCCUPIED))
            ),
        }
        diagnostics.append(diagnostic)

        if mode == "force_free":
            force_free_changed += changed_count
        else:
            force_occupied_changed += changed_count

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
        override_diagnostics=tuple(diagnostics),
    )
