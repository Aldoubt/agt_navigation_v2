"""Pure review diagnostics for the V25 Structure-Aware Map Workbench.

This module is intentionally read-only with respect to Navigation Map policy.
It explains *why* geometric aisles fail to become connected FREE corridors by
projecting formal hard-occupancy provenance back into each row-pair envelope.
"""

from __future__ import annotations

from typing import Any, Mapping

import numpy as np

from agt_offline_assets.formal_navigation_map import HardOccupancyProvenance
from agt_offline_assets.navigation_map_derivation import FREE, OCCUPIED, UNKNOWN


_ACCEPTED_AISLE_STATES = {"ACCEPTED", "ACCEPTED_NO_CENTERLINE"}


def _normalized_direction(structure: Any) -> np.ndarray:
    direction = np.asarray(structure.row_model.direction_xy, dtype=np.float64).reshape(2)
    norm = float(np.linalg.norm(direction))
    if not np.isfinite(norm) or norm <= 1.0e-12:
        raise ValueError("review diagnostic row direction must be finite and non-zero")
    return direction / norm


def _grid_uv(navigation: Any, structure: Any) -> tuple[np.ndarray, np.ndarray]:
    occupancy = np.asarray(navigation.occupancy)
    rows, cols = np.indices(occupancy.shape, dtype=np.float64)
    xx = float(navigation.origin_x_m) + (
        cols + 0.5
    ) * float(navigation.resolution_m)
    yy = float(navigation.origin_y_m) + (
        rows + 0.5
    ) * float(navigation.resolution_m)
    direction = _normalized_direction(structure)
    perpendicular = np.array([-direction[1], direction[0]], dtype=np.float64)
    u = xx * direction[0] + yy * direction[1]
    v = xx * perpendicular[0] + yy * perpendicular[1]
    return u, v


def _owned_mask(geometric: np.ndarray, v: np.ndarray, diagnostic: Any) -> np.ndarray:
    low = min(
        float(diagnostic.left_row_center_v_m),
        float(diagnostic.right_row_center_v_m),
    )
    high = max(
        float(diagnostic.left_row_center_v_m),
        float(diagnostic.right_row_center_v_m),
    )
    return geometric & (v >= low - 1.0e-9) & (v <= high + 1.0e-9)


def derive_aisle_hard_conflict_mask(
    corridor: Any,
    provenance: HardOccupancyProvenance,
) -> np.ndarray:
    """Return hard OCCUPIED cells that lie inside validated aisle geometry."""

    geometric = np.asarray(corridor.aisle_geometric_envelope, dtype=bool)
    hard = np.asarray(provenance.hard_after_padding_mask, dtype=bool)
    if geometric.shape != hard.shape:
        raise ValueError("aisle hard-conflict grid shape mismatch")
    return geometric & hard


def derive_connectivity_breakpoint_mask(
    navigation: Any,
    structure: Any,
    corridor: Any,
    accepted_occupancy: np.ndarray,
) -> np.ndarray:
    """Mark longitudinal aisle cross-sections that contain no FREE cell.

    This is a diagnostic, not a repair operator.  A marked slice says that the
    current Accepted raster has no traversable cell anywhere across that aisle
    at one row-direction station, which is a concrete reason end-to-end grid
    connectivity can fail.
    """

    occupancy = np.asarray(accepted_occupancy, dtype=np.uint8)
    geometric = np.asarray(corridor.aisle_geometric_envelope, dtype=bool)
    if occupancy.shape != geometric.shape:
        raise ValueError("connectivity breakpoint grid shape mismatch")
    if np.asarray(navigation.occupancy).shape != occupancy.shape:
        raise ValueError("connectivity breakpoint navigation shape mismatch")

    u, v = _grid_uv(navigation, structure)
    resolution = float(navigation.resolution_m)
    if resolution <= 0.0:
        raise ValueError("connectivity breakpoint resolution must be > 0")
    free = occupancy == FREE
    output = np.zeros(occupancy.shape, dtype=bool)

    for diagnostic in corridor.aisle_pair_diagnostics:
        if str(getattr(diagnostic, "pair_kind", "ROW_ROW")) != "ROW_ROW":
            continue
        if str(getattr(diagnostic, "status", "")) not in _ACCEPTED_AISLE_STATES:
            continue
        owned = _owned_mask(geometric, v, diagnostic)
        if not np.any(owned):
            continue
        u_min = float(np.min(u[owned]))
        bins = np.floor((u - u_min) / resolution + 1.0e-9).astype(np.int64)
        for bin_id in np.unique(bins[owned]):
            section = owned & (bins == int(bin_id))
            if np.any(section) and not np.any(section & free):
                output |= section & ~free
    return output


def build_aisle_provenance_reports(
    navigation: Any,
    structure: Any,
    corridor: Any,
    accepted_occupancy: np.ndarray,
    provenance: HardOccupancyProvenance,
    *,
    materialized: Any | None = None,
    qa: Mapping[str, object] | None = None,
) -> list[dict[str, object]]:
    """Build one auditable occupancy-cause report for every interior row pair."""

    occupancy = np.asarray(accepted_occupancy, dtype=np.uint8)
    geometric = np.asarray(corridor.aisle_geometric_envelope, dtype=bool)
    if occupancy.shape != geometric.shape:
        raise ValueError("aisle provenance grid shape mismatch")
    u, v = _grid_uv(navigation, structure)
    del u  # ownership uses the transverse row coordinate only
    breakpoints = derive_connectivity_breakpoint_mask(
        navigation,
        structure,
        corridor,
        occupancy,
    )
    qa_by_id = {
        str(item.get("aisle_id")): dict(item)
        for item in ((qa or {}).get("aisles") or [])
    }

    masks: dict[str, np.ndarray] = {
        "direct_obstacle": np.asarray(provenance.direct_obstacle_mask, dtype=bool),
        "strong_sensor_obstacle": np.asarray(
            provenance.strong_sensor_obstacle_mask, dtype=bool
        ),
        "slope_hard": np.asarray(provenance.slope_hard_mask, dtype=bool),
        "step_hard": np.asarray(provenance.step_hard_mask, dtype=bool),
        "hard_before_padding": np.asarray(
            provenance.hard_before_padding_mask, dtype=bool
        ),
        "padding_added_hard": np.asarray(
            provenance.padding_added_hard_mask, dtype=bool
        ),
        "hard_after_padding": np.asarray(
            provenance.hard_after_padding_mask, dtype=bool
        ),
        "soft_occupied": np.asarray(provenance.soft_occupied_mask, dtype=bool),
        "connectivity_breakpoint": breakpoints,
    }
    if materialized is not None:
        masks["row_structural_block"] = np.asarray(
            materialized.row_structural_blocked_mask, dtype=bool
        )
        masks["site_boundary_block"] = np.asarray(
            materialized.site_boundary_blocked_mask, dtype=bool
        )

    reports: list[dict[str, object]] = []
    for diagnostic in corridor.aisle_pair_diagnostics:
        if str(getattr(diagnostic, "pair_kind", "ROW_ROW")) != "ROW_ROW":
            continue
        pair_index = int(diagnostic.pair_index)
        aisle_id = f"aisle_{pair_index:03d}"
        owned = _owned_mask(geometric, v, diagnostic)
        total = int(np.count_nonzero(owned))
        accepted_status = str(diagnostic.status) in _ACCEPTED_AISLE_STATES
        qa_entry = qa_by_id.get(aisle_id, {})
        report: dict[str, object] = {
            "aisle_id": aisle_id,
            "pair_index": pair_index,
            "pair_kind": str(getattr(diagnostic, "pair_kind", "ROW_ROW")),
            "diagnostic_status": str(diagnostic.status),
            "accepted_geometry": bool(accepted_status and total > 0),
            "left_row_center_v_m": float(diagnostic.left_row_center_v_m),
            "right_row_center_v_m": float(diagnostic.right_row_center_v_m),
            "center_distance_m": float(diagnostic.center_distance_m),
            "geometric_available_width_m": float(
                diagnostic.geometric_available_width_m
            ),
            "minimum_required_width_m": float(diagnostic.minimum_required_width_m),
            "longitudinal_overlap_m": (
                None
                if diagnostic.longitudinal_overlap_m is None
                else float(diagnostic.longitudinal_overlap_m)
            ),
            "geometric_cell_count": total,
            "safe_cell_count_from_corridor": int(diagnostic.safe_cell_count),
            "safe_centerline_cell_count": int(diagnostic.centerline_cell_count),
            "grid_connectivity": qa_entry.get("grid_connectivity"),
        }
        if total > 0:
            report.update(
                {
                    "free_cell_count": int(np.count_nonzero(owned & (occupancy == FREE))),
                    "unknown_cell_count": int(
                        np.count_nonzero(owned & (occupancy == UNKNOWN))
                    ),
                    "occupied_cell_count": int(
                        np.count_nonzero(owned & (occupancy == OCCUPIED))
                    ),
                }
            )
        else:
            report.update(
                {
                    "free_cell_count": 0,
                    "unknown_cell_count": 0,
                    "occupied_cell_count": 0,
                }
            )
        for name, mask in masks.items():
            count = int(np.count_nonzero(owned & mask)) if total > 0 else 0
            report[f"{name}_cell_count"] = count
            report[f"{name}_fraction"] = float(count / total) if total else 0.0
        reports.append(report)
    return reports
