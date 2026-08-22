"""Planner-independent QA for formal structure-aware Navigation Maps."""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any, Iterable, Mapping

import numpy as np
from shapely.geometry import Polygon

from .formal_navigation_map import StructureAwareNavigationResult
from .formal_navigation_override import (
    FormalOverrideReplayResult,
    replay_formal_navigation_overrides,
)
from .navigation_map_derivation import FREE, OCCUPIED, UNKNOWN, NavigationMapResult
from .site_boundary import SiteBoundary, rasterize_site_boundary


FORMAL_NAVIGATION_QA_SCHEMA = "agt_formal_navigation_qa/v2"
_ACCEPTED_DIAGNOSTIC_STATES = {"ACCEPTED", "ACCEPTED_NO_CENTERLINE"}


def _require_scipy():
    try:
        from scipy import ndimage
    except ImportError as exc:
        raise RuntimeError("formal navigation QA requires scipy") from exc
    return ndimage


def _grid_xy(navigation: NavigationMapResult) -> tuple[np.ndarray, np.ndarray]:
    rows, cols = np.indices(navigation.occupancy.shape, dtype=np.float64)
    xx = float(navigation.origin_x_m) + (
        cols + 0.5
    ) * float(navigation.resolution_m)
    yy = float(navigation.origin_y_m) + (
        rows + 0.5
    ) * float(navigation.resolution_m)
    return xx, yy


def _normalized_direction(structure: Any) -> np.ndarray:
    direction = np.asarray(structure.row_model.direction_xy, dtype=np.float64).reshape(2)
    norm = float(np.linalg.norm(direction))
    if not np.isfinite(norm) or norm <= 1.0e-12:
        raise ValueError("formal navigation QA row direction must be finite and non-zero")
    return direction / norm


def _free_connects_longitudinal_ends(
    owned: np.ndarray,
    free: np.ndarray,
    u: np.ndarray,
    resolution_m: float,
) -> bool:
    if not np.any(owned):
        return False
    free_owned = owned & free
    if not np.any(free_owned):
        return False

    u_min = float(np.min(u[owned]))
    u_max = float(np.max(u[owned]))
    tolerance = 0.51 * float(resolution_m)
    starts = np.argwhere(free_owned & (u <= u_min + tolerance))
    targets = free_owned & (u >= u_max - tolerance)
    if starts.size == 0 or not np.any(targets):
        return False

    height, width = owned.shape
    visited = np.zeros(owned.shape, dtype=bool)
    stack = [(int(row), int(col)) for row, col in starts]
    for row, col in stack:
        visited[row, col] = True

    while stack:
        row, col = stack.pop()
        if targets[row, col]:
            return True
        for dr in (-1, 0, 1):
            for dc in (-1, 0, 1):
                if dr == 0 and dc == 0:
                    continue
                rr = row + dr
                cc = col + dc
                if (
                    0 <= rr < height
                    and 0 <= cc < width
                    and free_owned[rr, cc]
                    and not visited[rr, cc]
                ):
                    visited[rr, cc] = True
                    stack.append((rr, cc))
    return False


def _largest_free_component_fraction(occupancy: np.ndarray) -> float:
    free = np.asarray(occupancy, dtype=np.uint8) == FREE
    total_free = int(np.count_nonzero(free))
    if total_free == 0:
        return 0.0
    ndimage = _require_scipy()
    labels, count = ndimage.label(free, structure=np.ones((3, 3), dtype=np.uint8))
    if count <= 0:
        return 0.0
    sizes = np.bincount(labels.reshape(-1), minlength=count + 1)[1:]
    largest = int(np.max(sizes)) if sizes.size else 0
    return float(largest / total_free)


def _site_boundary_shape_diagnostics(boundary: SiteBoundary) -> dict[str, object]:
    polygon = Polygon(boundary.outer_boundary_xy)
    hull = polygon.convex_hull
    area_ratio = float(polygon.area / hull.area) if hull.area > 0.0 else 0.0
    perimeter_ratio = (
        float(polygon.length / hull.length) if hull.length > 0.0 else float("inf")
    )
    warnings: list[str] = []
    # These are review warnings, not authority failures. A greenhouse perimeter
    # can be concave, but a deeply serpentine polygon is often a corridor mask
    # accidentally authored as the outer vehicle-permitted boundary.
    if perimeter_ratio > 2.5 or area_ratio < 0.55:
        warnings.append("SITE_BOUNDARY_SHAPE_REVIEW_REQUIRED")
    if len(boundary.outer_boundary_xy) > 32:
        warnings.append("SITE_BOUNDARY_VERTEX_COMPLEXITY_REVIEW")
    return {
        "site_boundary_vertex_count": len(boundary.outer_boundary_xy),
        "site_boundary_area_m2": float(polygon.area),
        "site_boundary_perimeter_m": float(polygon.length),
        "site_boundary_convex_area_ratio": area_ratio,
        "site_boundary_convex_perimeter_ratio": perimeter_ratio,
        "site_boundary_warnings": warnings,
    }


def _aisle_reports(
    accepted_navigation: NavigationMapResult,
    structure: Any,
    corridor: Any,
) -> list[dict[str, object]]:
    shape = accepted_navigation.occupancy.shape
    geometric = np.asarray(corridor.aisle_geometric_envelope, dtype=bool)
    if geometric.shape != shape:
        raise ValueError("formal navigation QA aisle geometry shape mismatch")

    direction = _normalized_direction(structure)
    perpendicular = np.array([-direction[1], direction[0]], dtype=np.float64)
    xx, yy = _grid_xy(accepted_navigation)
    u = xx * direction[0] + yy * direction[1]
    v = xx * perpendicular[0] + yy * perpendicular[1]
    occupancy = np.asarray(accepted_navigation.occupancy, dtype=np.uint8)
    free = occupancy == FREE

    reports: list[dict[str, object]] = []
    for diagnostic in corridor.aisle_pair_diagnostics:
        if str(diagnostic.status) not in _ACCEPTED_DIAGNOSTIC_STATES:
            continue
        low = min(
            float(diagnostic.left_row_center_v_m),
            float(diagnostic.right_row_center_v_m),
        )
        high = max(
            float(diagnostic.left_row_center_v_m),
            float(diagnostic.right_row_center_v_m),
        )
        owned = geometric & (v >= low - 1.0e-9) & (v <= high + 1.0e-9)
        total = int(np.count_nonzero(owned))
        if total == 0:
            free_count = unknown_count = occupied_count = 0
            free_fraction = unknown_fraction = occupied_fraction = 0.0
            connected = False
        else:
            free_count = int(np.count_nonzero(owned & (occupancy == FREE)))
            unknown_count = int(np.count_nonzero(owned & (occupancy == UNKNOWN)))
            occupied_count = int(np.count_nonzero(owned & (occupancy == OCCUPIED)))
            free_fraction = free_count / total
            unknown_fraction = unknown_count / total
            occupied_fraction = occupied_count / total
            connected = _free_connects_longitudinal_ends(
                owned,
                free,
                u,
                accepted_navigation.resolution_m,
            )
        reports.append(
            {
                "aisle_id": f"aisle_{int(diagnostic.pair_index):03d}",
                "pair_kind": str(diagnostic.pair_kind),
                "diagnostic_status": str(diagnostic.status),
                "cell_count": total,
                "free_cell_count": free_count,
                "unknown_cell_count": unknown_count,
                "occupied_conflict_cell_count": occupied_count,
                "free_fraction": float(free_fraction),
                "unknown_fraction": float(unknown_fraction),
                "occupied_conflict_fraction": float(occupied_fraction),
                "grid_connectivity": bool(connected),
            }
        )
    return reports


def evaluate_formal_navigation_qa(
    *,
    ground_evidence: NavigationMapResult,
    materialized: StructureAwareNavigationResult,
    accepted: FormalOverrideReplayResult,
    overrides: Iterable[Mapping[str, object]],
    structure: Any,
    corridor: Any,
    site_boundary: SiteBoundary,
) -> dict[str, object]:
    """Evaluate safety invariants separately from navigation usability."""

    expected_shape = ground_evidence.occupancy.shape
    if materialized.navigation.occupancy.shape != expected_shape:
        raise ValueError("formal navigation QA materialized grid shape mismatch")
    if accepted.navigation.occupancy.shape != expected_shape:
        raise ValueError("formal navigation QA Accepted grid shape mismatch")

    site_boundary.validate(expected_frame_id="map")
    inside = rasterize_site_boundary(
        site_boundary,
        accepted.navigation,
        expected_frame_id="map",
    )
    row_band = np.asarray(corridor.row_structural_band, dtype=bool)
    if row_band.shape != expected_shape:
        raise ValueError("formal navigation QA row structural grid shape mismatch")

    accepted_occupancy = np.asarray(accepted.navigation.occupancy, dtype=np.uint8)
    outside_free = (accepted_occupancy == FREE) & ~inside
    row_free = (accepted_occupancy == FREE) & row_band
    outside_count = int(np.count_nonzero(outside_free))
    row_leak_count = int(np.count_nonzero(row_free))

    replay = replay_formal_navigation_overrides(
        materialized.navigation,
        corridor,
        site_boundary,
        overrides,
        frame_id="map",
    )
    matches_replay = bool(
        np.array_equal(replay.navigation.occupancy, accepted.navigation.occupancy)
    )

    hard_failures: list[str] = []
    if outside_count > 0:
        hard_failures.append("OUTSIDE_SITE_BOUNDARY_FREE")
    if row_leak_count > 0:
        hard_failures.append("ROW_STRUCTURAL_FREE_LEAK")
    if not matches_replay:
        hard_failures.append("ACCEPTED_REPLAY_MISMATCH")

    aisle_reports = _aisle_reports(accepted.navigation, structure, corridor)
    connected_aisles = sum(bool(item.get("grid_connectivity")) for item in aisle_reports)
    total_aisles = len(aisle_reports)
    structural_status = "PASS" if not hard_failures else "FAIL"
    if total_aisles == 0:
        usability_status = "NOT_EVALUATED"
    elif connected_aisles == total_aisles:
        usability_status = "PASS"
    else:
        usability_status = "REVIEW_REQUIRED"

    total_cells = int(accepted_occupancy.size)
    unknown_count = int(np.count_nonzero(accepted_occupancy == UNKNOWN))
    inferred_count = int(np.count_nonzero(materialized.structure_inferred_free_mask))
    recovered_soft_count = int(
        np.count_nonzero(materialized.structure_recovered_soft_occupied_mask)
    )
    boundary_shape = _site_boundary_shape_diagnostics(site_boundary)

    return {
        "schema": FORMAL_NAVIGATION_QA_SCHEMA,
        # Backwards-compatible safety status consumed by existing binders/tests.
        "status": structural_status,
        "structural_safety_status": structural_status,
        "navigation_usability_status": usability_status,
        "hard_failures": hard_failures,
        "outside_site_boundary_free_count": outside_count,
        "row_structural_band_free_leak_count": row_leak_count,
        "accepted_aisle_count": total_aisles,
        "connected_aisle_count": connected_aisles,
        "aisles": aisle_reports,
        "largest_free_component_fraction": _largest_free_component_fraction(
            accepted_occupancy
        ),
        "map_unknown_fraction": (
            float(unknown_count / total_cells) if total_cells else 0.0
        ),
        "structure_inferred_free_fraction": (
            float(inferred_count / total_cells) if total_cells else 0.0
        ),
        "structure_recovered_soft_occupied_cell_count": recovered_soft_count,
        "manual_force_free_area_m2": float(accepted.force_free_area_m2),
        "manual_force_occupied_area_m2": float(accepted.force_occupied_area_m2),
        "override_diagnostics": [dict(item) for item in accepted.override_diagnostics],
        "accepted_matches_replay": matches_replay,
        **boundary_shape,
    }


def write_formal_navigation_qa(
    report: Mapping[str, object],
    path: str | Path,
) -> Path:
    output = Path(path)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(
        json.dumps(dict(report), indent=2, sort_keys=True, ensure_ascii=False) + "\n",
        encoding="utf-8",
    )
    return output
