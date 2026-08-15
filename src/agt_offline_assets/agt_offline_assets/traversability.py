"""V25-12F rich traversability evidence and bounded aisle occlusion recovery.

The A-first policy is intentionally conservative: current OCCUPIED cells remain
blocked, semantic NO_GO remains blocked, and a manually frozen Site Boundary is
mandatory. Only short longitudinal UNKNOWN gaps inside structurally valid
agricultural aisle geometry may become INFERRED_TRAVERSABLE.
"""

from __future__ import annotations

from dataclasses import dataclass, field
import math
from pathlib import Path
from typing import Any, Iterable, Mapping

import numpy as np
import yaml

from .contracts import sha256_file
from .navigation_grid import load_navigation_grid
from .navigation_map_derivation import (
    FREE,
    OCCUPIED,
    UNKNOWN,
    NavigationMapResult,
    _write_pgm,
)
from .site_boundary import SiteBoundary, rasterize_site_boundary
from .turn_zones import _points_inside_polygon

TRAVERSABILITY_EVIDENCE_SCHEMA = "agt_traversability_evidence/v1"
TRAVERSABILITY_DERIVATION_SCHEMA = "agt_v25_12f_navigation_map_derivation/v1"

_REQUIRED_MASK_KEYS = (
    "observed_free_mask",
    "inferred_traversable_mask",
    "hard_blocked_mask",
    "sensor_obstacle_mask",
    "unknown_mask",
    "semantic_no_go_mask",
    "aisle_geometric_envelope_mask",
)


@dataclass(frozen=True)
class TraversabilityConfig:
    maximum_inferred_gap_m: float = 0.60

    def validate(self) -> None:
        if (
            not math.isfinite(float(self.maximum_inferred_gap_m))
            or float(self.maximum_inferred_gap_m) <= 0.0
        ):
            raise ValueError("maximum_inferred_gap_m must be finite and > 0")


@dataclass(frozen=True)
class TraversabilityEvidence:
    frame_id: str
    observed_free_mask: np.ndarray
    inferred_traversable_mask: np.ndarray
    hard_blocked_mask: np.ndarray
    sensor_obstacle_mask: np.ndarray
    unknown_mask: np.ndarray
    semantic_no_go_mask: np.ndarray
    aisle_geometric_envelope_mask: np.ndarray
    config: TraversabilityConfig
    source: Mapping[str, Any] = field(default_factory=dict)
    schema: str = TRAVERSABILITY_EVIDENCE_SCHEMA
    status: str = "DRAFT"

    def candidate_occupancy(self) -> np.ndarray:
        shape = self.observed_free_mask.shape
        output = np.full(shape, UNKNOWN, dtype=np.uint8)
        output[self.observed_free_mask | self.inferred_traversable_mask] = FREE
        output[
            self.sensor_obstacle_mask
            | self.semantic_no_go_mask
            | self.hard_blocked_mask
        ] = OCCUPIED
        return output

    def counts(self) -> dict[str, int]:
        return {
            "observed_free": int(np.count_nonzero(self.observed_free_mask)),
            "inferred_traversable": int(
                np.count_nonzero(self.inferred_traversable_mask)
            ),
            "hard_blocked": int(np.count_nonzero(self.hard_blocked_mask)),
            "sensor_obstacle": int(np.count_nonzero(self.sensor_obstacle_mask)),
            "unknown": int(np.count_nonzero(self.unknown_mask)),
            "semantic_no_go": int(np.count_nonzero(self.semantic_no_go_mask)),
            "aisle_geometric_envelope": int(
                np.count_nonzero(self.aisle_geometric_envelope_mask)
            ),
        }


def _grid_centers(navigation: NavigationMapResult) -> tuple[np.ndarray, np.ndarray]:
    rows, cols = np.indices(navigation.occupancy.shape, dtype=np.float64)
    xx = float(navigation.origin_x_m) + (
        cols + 0.5
    ) * float(navigation.resolution_m)
    yy = float(navigation.origin_y_m) + (
        rows + 0.5
    ) * float(navigation.resolution_m)
    return xx, yy


def _rasterize_no_go(
    navigation: NavigationMapResult,
    overrides: Iterable[Mapping[str, Any]] | None,
) -> np.ndarray:
    mask = np.zeros(navigation.occupancy.shape, dtype=bool)
    if not overrides:
        return mask
    xx, yy = _grid_centers(navigation)
    for override in overrides:
        if not isinstance(override, Mapping):
            continue
        if str(override.get("mode", "")) != "no_go":
            continue
        polygon = override.get("polygon_xy")
        if not isinstance(polygon, (list, tuple)) or len(polygon) < 3:
            raise ValueError("NO_GO override polygon_xy requires at least three vertices")
        mask |= _points_inside_polygon(xx, yy, polygon)
    return mask


def _validate_shapes(
    navigation: NavigationMapResult,
    corridor,
) -> tuple[np.ndarray, np.ndarray]:
    shape = navigation.occupancy.shape
    geometric = np.asarray(corridor.aisle_geometric_envelope, dtype=bool)
    row_band = np.asarray(corridor.row_structural_band, dtype=bool)
    if geometric.shape != shape:
        raise ValueError(
            "aisle_geometric_envelope shape does not match Navigation Map"
        )
    if row_band.shape != shape:
        raise ValueError("row_structural_band shape does not match Navigation Map")
    return geometric, row_band


def _normalized_row_direction(structure) -> np.ndarray:
    direction = np.asarray(structure.row_model.direction_xy, dtype=np.float64).reshape(2)
    norm = float(np.linalg.norm(direction))
    if not math.isfinite(norm) or norm <= 1.0e-12:
        raise ValueError("row direction must be finite and non-zero")
    return direction / norm


def _terrain_endpoints_compatible(
    navigation: NavigationMapResult,
    left_index: tuple[int, int],
    right_index: tuple[int, int],
    longitudinal_distance_m: float,
) -> bool:
    left_height = float(navigation.ground_height_m[left_index])
    right_height = float(navigation.ground_height_m[right_index])
    if not math.isfinite(left_height) or not math.isfinite(right_height):
        return False
    rise = abs(right_height - left_height)
    if rise > float(navigation.config.maximum_step_m) + 1.0e-9:
        return False
    distance = max(float(longitudinal_distance_m), float(navigation.resolution_m))
    slope_limit = math.tan(math.radians(float(navigation.config.maximum_slope_deg)))
    return rise / distance <= slope_limit + 1.0e-9


def _recover_longitudinal_unknown_gaps(
    navigation: NavigationMapResult,
    structure,
    *,
    base_free: np.ndarray,
    base_unknown: np.ndarray,
    geometric: np.ndarray,
    row_band: np.ndarray,
    hard_blocked: np.ndarray,
    semantic_no_go: np.ndarray,
    config: TraversabilityConfig,
) -> np.ndarray:
    direction = _normalized_row_direction(structure)
    perpendicular = np.array([-direction[1], direction[0]], dtype=np.float64)
    xx, yy = _grid_centers(navigation)
    u = xx * direction[0] + yy * direction[1]
    v = xx * perpendicular[0] + yy * perpendicular[1]
    resolution = float(navigation.resolution_m)
    v_bin = np.rint((v - float(np.min(v))) / resolution).astype(np.int64)

    admissible_unknown = (
        base_unknown
        & geometric
        & ~row_band
        & ~hard_blocked
        & ~semantic_no_go
    )
    free_support = (
        base_free
        & geometric
        & ~row_band
        & ~hard_blocked
        & ~semantic_no_go
    )

    inferred = np.zeros(base_unknown.shape, dtype=bool)
    maximum_step_between_sorted_cells = 1.75 * resolution

    for bin_id in np.unique(v_bin):
        row_indices, col_indices = np.nonzero(v_bin == bin_id)
        if row_indices.size < 3:
            continue
        values_u = u[row_indices, col_indices]
        order = np.argsort(values_u, kind="mergesort")
        rr = row_indices[order]
        cc = col_indices[order]
        uu = values_u[order]
        candidate = admissible_unknown[rr, cc]

        run_start: int | None = None
        for position in range(candidate.size + 1):
            current_candidate = (
                bool(candidate[position]) if position < candidate.size else False
            )
            contiguous = True
            if position > 0 and position < candidate.size:
                contiguous = (
                    float(uu[position] - uu[position - 1])
                    <= maximum_step_between_sorted_cells + 1.0e-9
                )

            if current_candidate and run_start is None:
                run_start = position
                continue
            if current_candidate and run_start is not None and contiguous:
                continue

            if run_start is not None:
                run_end = position - 1
                previous_position = run_start - 1
                next_position = run_end + 1
                if previous_position >= 0 and next_position < candidate.size:
                    left = (int(rr[previous_position]), int(cc[previous_position]))
                    right = (int(rr[next_position]), int(cc[next_position]))
                    left_gap = float(uu[run_start] - uu[previous_position])
                    right_gap = float(uu[next_position] - uu[run_end])
                    run_length = max(
                        resolution,
                        float(uu[run_end] - uu[run_start]) + resolution,
                    )
                    support_separation = float(
                        uu[next_position] - uu[previous_position]
                    )
                    bracketed = bool(
                        free_support[left]
                        and free_support[right]
                        and left_gap <= maximum_step_between_sorted_cells + 1.0e-9
                        and right_gap <= maximum_step_between_sorted_cells + 1.0e-9
                    )
                    if (
                        bracketed
                        and run_length
                        <= float(config.maximum_inferred_gap_m) + 1.0e-9
                        and _terrain_endpoints_compatible(
                            navigation,
                            left,
                            right,
                            support_separation,
                        )
                    ):
                        inferred[
                            rr[run_start : run_end + 1],
                            cc[run_start : run_end + 1],
                        ] = True

                run_start = position if current_candidate else None

    return inferred


def derive_traversability_evidence(
    navigation: NavigationMapResult,
    structure,
    corridor,
    site_boundary: SiteBoundary,
    config: TraversabilityConfig | None = None,
    *,
    overrides: Iterable[Mapping[str, Any]] | None = None,
    frame_id: str = "map",
    source: Mapping[str, Any] | None = None,
) -> TraversabilityEvidence:
    """Derive rich candidate-map evidence without mutating the current map."""

    cfg = config or TraversabilityConfig()
    cfg.validate()
    if site_boundary is None:
        raise ValueError("READY site_boundary is required for V25-12F candidate generation")
    site_boundary.validate(expected_frame_id=frame_id)

    occupancy = np.asarray(navigation.occupancy, dtype=np.uint8)
    if occupancy.ndim != 2:
        raise ValueError("Navigation Map occupancy must be 2D")
    allowed = np.isin(occupancy, np.array([FREE, OCCUPIED, UNKNOWN], dtype=np.uint8))
    if not bool(np.all(allowed)):
        raise ValueError("Navigation Map occupancy must use AGT trinary values")

    geometric, row_band = _validate_shapes(navigation, corridor)
    base_free = occupancy == FREE
    base_occupied = occupancy == OCCUPIED
    base_unknown = occupancy == UNKNOWN

    inside_boundary = rasterize_site_boundary(
        site_boundary,
        navigation,
        expected_frame_id=frame_id,
    )
    hard_blocked = ~inside_boundary
    semantic_no_go = _rasterize_no_go(navigation, overrides)

    inferred = _recover_longitudinal_unknown_gaps(
        navigation,
        structure,
        base_free=base_free,
        base_unknown=base_unknown,
        geometric=geometric,
        row_band=row_band,
        hard_blocked=hard_blocked,
        semantic_no_go=semantic_no_go,
        config=cfg,
    )

    inferred &= ~hard_blocked & ~semantic_no_go & ~base_occupied
    sensor_obstacle = base_occupied & ~hard_blocked & ~semantic_no_go
    observed_free = base_free & ~hard_blocked & ~semantic_no_go
    unknown = (
        base_unknown
        & ~inferred
        & ~hard_blocked
        & ~semantic_no_go
        & ~sensor_obstacle
    )

    merged_source = dict(source or {})
    merged_source.update(
        {
            "policy": "V25_12F_A_FIRST_BOUNDED_LONGITUDINAL_RECOVERY",
            "maximum_inferred_gap_m": float(cfg.maximum_inferred_gap_m),
            "current_occupied_recoverable": False,
            "site_boundary_required": True,
            "site_boundary_semantics": site_boundary.boundary_semantics,
        }
    )

    return TraversabilityEvidence(
        frame_id=str(frame_id),
        observed_free_mask=observed_free,
        inferred_traversable_mask=inferred,
        hard_blocked_mask=hard_blocked,
        sensor_obstacle_mask=sensor_obstacle,
        unknown_mask=unknown,
        semantic_no_go_mask=semantic_no_go,
        aisle_geometric_envelope_mask=geometric.copy(),
        config=cfg,
        source=merged_source,
    )


def _grid_record(navigation: NavigationMapResult) -> dict[str, Any]:
    return {
        "resolution_m": float(navigation.resolution_m),
        "origin_xy_m": [float(navigation.origin_x_m), float(navigation.origin_y_m)],
        "width": int(navigation.width),
        "height": int(navigation.height),
    }


def _candidate_nav_yaml(navigation: NavigationMapResult) -> dict[str, Any]:
    return {
        "image": "navigation_map_12f.pgm",
        "mode": "trinary",
        "resolution": float(navigation.resolution_m),
        "origin": [float(navigation.origin_x_m), float(navigation.origin_y_m), 0.0],
        "negate": 0,
        "occupied_thresh": 0.65,
        "free_thresh": 0.196,
    }


def write_traversability_candidate(
    evidence: TraversabilityEvidence,
    navigation: NavigationMapResult,
    site_boundary: SiteBoundary,
    output_dir: str | Path,
    *,
    source_navigation_asset: str = "navigation_map.yaml",
    overwrite: bool = False,
) -> Path:
    """Freeze 12F candidate products without modifying canonical map assets."""

    output = Path(output_dir).expanduser().resolve()
    output.mkdir(parents=True, exist_ok=True)
    if evidence.frame_id != site_boundary.frame_id:
        raise ValueError("traversability/site_boundary frame_id mismatch")
    site_boundary.validate(expected_frame_id=evidence.frame_id)
    shape = navigation.occupancy.shape
    for key in _REQUIRED_MASK_KEYS:
        if np.asarray(getattr(evidence, key)).shape != shape:
            raise ValueError(f"{key} shape does not match Navigation Map")

    boundary_path = output / "site_boundary.yaml"
    if not boundary_path.is_file():
        raise FileNotFoundError(
            "site_boundary.yaml must be frozen in the output run before candidate export"
        )

    paths = {
        "evidence_yaml": output / "traversability_evidence.yaml",
        "evidence_npz": output / "traversability_evidence.npz",
        "candidate_yaml": output / "navigation_map_12f.yaml",
        "candidate_pgm": output / "navigation_map_12f.pgm",
        "derivation_yaml": output / "navigation_map_12f_derivation.yaml",
    }
    existing = [path for path in paths.values() if path.exists()]
    if existing and not overwrite:
        raise FileExistsError(
            "12F candidate output already exists: "
            + ", ".join(path.name for path in existing)
        )

    np.savez_compressed(
        paths["evidence_npz"],
        observed_free_mask=evidence.observed_free_mask.astype(np.uint8),
        inferred_traversable_mask=evidence.inferred_traversable_mask.astype(np.uint8),
        hard_blocked_mask=evidence.hard_blocked_mask.astype(np.uint8),
        sensor_obstacle_mask=evidence.sensor_obstacle_mask.astype(np.uint8),
        unknown_mask=evidence.unknown_mask.astype(np.uint8),
        semantic_no_go_mask=evidence.semantic_no_go_mask.astype(np.uint8),
        aisle_geometric_envelope_mask=evidence.aisle_geometric_envelope_mask.astype(
            np.uint8
        ),
    )

    candidate = evidence.candidate_occupancy()
    _write_pgm(paths["candidate_pgm"], np.flipud(candidate).copy())
    paths["candidate_yaml"].write_text(
        yaml.safe_dump(_candidate_nav_yaml(navigation), sort_keys=False),
        encoding="utf-8",
    )

    source_navigation_path = output / source_navigation_asset
    source_navigation_sha = (
        sha256_file(source_navigation_path)
        if source_navigation_path.is_file()
        else None
    )
    requested_padding = float(getattr(navigation.config, "obstacle_padding_m", 0.0))
    effective_padding_cells = int(
        math.ceil(requested_padding / float(navigation.resolution_m))
    )

    evidence_record = {
        "schema": evidence.schema,
        "status": evidence.status,
        "frame_id": evidence.frame_id,
        "grid": _grid_record(navigation),
        "config": {
            "maximum_inferred_gap_m": float(evidence.config.maximum_inferred_gap_m)
        },
        "counts": evidence.counts(),
        "source": dict(evidence.source),
        "outputs": {
            "npz": paths["evidence_npz"].name,
            "candidate_navigation_yaml": paths["candidate_yaml"].name,
        },
    }
    paths["evidence_yaml"].write_text(
        yaml.safe_dump(evidence_record, sort_keys=False, allow_unicode=True),
        encoding="utf-8",
    )

    derivation_record = {
        "schema": TRAVERSABILITY_DERIVATION_SCHEMA,
        "frame_id": evidence.frame_id,
        "status": "DRAFT",
        "source_navigation_asset": source_navigation_asset,
        "source_navigation_sha256": source_navigation_sha,
        "source_site_boundary": boundary_path.name,
        "source_site_boundary_sha256": sha256_file(boundary_path),
        "grid": _grid_record(navigation),
        "config": {
            "maximum_inferred_gap_m": float(evidence.config.maximum_inferred_gap_m),
            "requested_obstacle_padding_m": requested_padding,
            "effective_obstacle_padding_cells": effective_padding_cells,
        },
        "counts": evidence.counts(),
        "outputs": {
            "navigation_map_12f_pgm": paths["candidate_pgm"].name,
            "navigation_map_12f_yaml": paths["candidate_yaml"].name,
            "traversability_evidence_npz": paths["evidence_npz"].name,
            "navigation_map_12f_pgm_sha256": sha256_file(paths["candidate_pgm"]),
            "navigation_map_12f_yaml_sha256": sha256_file(paths["candidate_yaml"]),
            "traversability_evidence_npz_sha256": sha256_file(paths["evidence_npz"]),
        },
    }
    paths["derivation_yaml"].write_text(
        yaml.safe_dump(derivation_record, sort_keys=False, allow_unicode=True),
        encoding="utf-8",
    )
    return output


def load_traversability_evidence(
    path: str | Path,
    *,
    expected_frame_id: str | None = None,
) -> TraversabilityEvidence:
    input_path = Path(path).expanduser().resolve()
    if input_path.is_dir():
        yaml_path = input_path / "traversability_evidence.yaml"
    else:
        yaml_path = input_path
    if not yaml_path.is_file():
        raise FileNotFoundError(f"traversability evidence YAML not found: {yaml_path}")

    payload = yaml.safe_load(yaml_path.read_text(encoding="utf-8")) or {}
    if not isinstance(payload, Mapping):
        raise ValueError("traversability_evidence.yaml must be a mapping")
    if payload.get("schema") != TRAVERSABILITY_EVIDENCE_SCHEMA:
        raise ValueError("traversability evidence schema mismatch")
    frame_id = str(payload.get("frame_id", ""))
    if expected_frame_id is not None and frame_id != expected_frame_id:
        raise ValueError(
            f"traversability frame_id mismatch: expected {expected_frame_id}, got {frame_id}"
        )

    grid = payload.get("grid") or {}
    if not isinstance(grid, Mapping):
        raise ValueError("traversability grid must be a mapping")
    width = int(grid.get("width", 0))
    height = int(grid.get("height", 0))
    if width <= 0 or height <= 0:
        raise ValueError("traversability grid dimensions must be positive")
    shape = (height, width)

    outputs = payload.get("outputs") or {}
    if not isinstance(outputs, Mapping):
        raise ValueError("traversability outputs must be a mapping")
    npz_name = outputs.get("npz")
    candidate_yaml_name = outputs.get("candidate_navigation_yaml")
    if not isinstance(npz_name, str) or not npz_name:
        raise ValueError("traversability outputs.npz is required")
    if not isinstance(candidate_yaml_name, str) or not candidate_yaml_name:
        raise ValueError("traversability candidate_navigation_yaml is required")

    npz_path = yaml_path.parent / npz_name
    if not npz_path.is_file():
        raise FileNotFoundError(f"traversability NPZ not found: {npz_path}")
    with np.load(npz_path) as archive:
        missing = [key for key in _REQUIRED_MASK_KEYS if key not in archive.files]
        if missing:
            raise ValueError(
                "traversability NPZ missing required masks: " + ", ".join(missing)
            )
        masks = {}
        for key in _REQUIRED_MASK_KEYS:
            array = np.asarray(archive[key])
            if array.shape != shape:
                raise ValueError(
                    f"traversability {key} shape {array.shape} does not match {shape}"
                )
            masks[key] = array.astype(bool)

    candidate = load_navigation_grid(yaml_path.parent / candidate_yaml_name)
    if candidate.frame_id != frame_id:
        raise ValueError("candidate Navigation Grid frame_id mismatch")
    if candidate.occupancy.shape != shape:
        raise ValueError("candidate Navigation Grid dimensions mismatch")
    resolution = float(grid.get("resolution_m", 0.0))
    origin = grid.get("origin_xy_m")
    if resolution <= 0.0 or not isinstance(origin, (list, tuple)) or len(origin) != 2:
        raise ValueError("traversability grid geometry is invalid")
    if not math.isclose(candidate.resolution_m, resolution, rel_tol=0.0, abs_tol=1e-12):
        raise ValueError("candidate Navigation Grid resolution mismatch")
    if not (
        math.isclose(candidate.origin_x_m, float(origin[0]), abs_tol=1e-12)
        and math.isclose(candidate.origin_y_m, float(origin[1]), abs_tol=1e-12)
    ):
        raise ValueError("candidate Navigation Grid origin mismatch")

    config_payload = payload.get("config") or {}
    if not isinstance(config_payload, Mapping):
        raise ValueError("traversability config must be a mapping")
    config = TraversabilityConfig(
        maximum_inferred_gap_m=float(
            config_payload.get("maximum_inferred_gap_m", 0.0)
        )
    )
    config.validate()
    source = payload.get("source") or {}
    if not isinstance(source, Mapping):
        raise ValueError("traversability source must be a mapping")

    evidence = TraversabilityEvidence(
        frame_id=frame_id,
        observed_free_mask=masks["observed_free_mask"],
        inferred_traversable_mask=masks["inferred_traversable_mask"],
        hard_blocked_mask=masks["hard_blocked_mask"],
        sensor_obstacle_mask=masks["sensor_obstacle_mask"],
        unknown_mask=masks["unknown_mask"],
        semantic_no_go_mask=masks["semantic_no_go_mask"],
        aisle_geometric_envelope_mask=masks["aisle_geometric_envelope_mask"],
        config=config,
        source=dict(source),
        status=str(payload.get("status", "DRAFT")),
    )
    if not np.array_equal(candidate.occupancy, evidence.candidate_occupancy()):
        raise ValueError("candidate Navigation Grid does not match traversability masks")
    return evidence
