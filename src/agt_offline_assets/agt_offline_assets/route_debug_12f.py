"""Optional V25-12F evidence bundle for the read-only Route Debug renderer.

The stable V25-12E RouteDebugDataset remains authoritative for existing route
production evidence.  This module loads only optional V25-12F candidate assets
so a missing or invalid candidate cannot make the legacy debug dataset fail.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any

from .navigation_grid import NavigationGridEvidence, load_navigation_grid
from .route_debug_dataset import (
    ASSET_INVALID,
    ASSET_LOADED,
    ASSET_MISSING,
    RouteDebugAssetState,
)
from .site_boundary import SiteBoundary, load_site_boundary
from .traversability import TraversabilityEvidence, load_traversability_evidence


@dataclass(frozen=True)
class RouteDebug12FBundle:
    run_dir: Path
    frame_id: str
    candidate_navigation: NavigationGridEvidence | None
    site_boundary: SiteBoundary | None
    traversability: TraversabilityEvidence | None
    asset_states: tuple[RouteDebugAssetState, ...]

    def asset_state(self, key: str) -> RouteDebugAssetState:
        for state in self.asset_states:
            if state.key == key:
                return state
        return RouteDebugAssetState(key, ASSET_MISSING, None, None, None, None)


def _state(
    key: str,
    availability: str,
    path: Path | None,
    *,
    schema: str | None = None,
    frame_id: str | None = None,
    status: str | None = None,
    error: str = "",
) -> RouteDebugAssetState:
    return RouteDebugAssetState(
        key=key,
        availability=availability,
        path=None if path is None else str(path),
        schema=schema,
        frame_id=frame_id,
        status=status,
        error=error,
    )


def load_route_debug_12f(
    run_dir: str | Path,
    *,
    expected_frame_id: str = "map",
) -> RouteDebug12FBundle:
    root = Path(run_dir).expanduser().resolve()
    if not root.is_dir():
        raise NotADirectoryError(f"route debug run directory not found: {root}")

    states: list[RouteDebugAssetState] = []
    candidate_navigation: NavigationGridEvidence | None = None
    site_boundary: SiteBoundary | None = None
    traversability: TraversabilityEvidence | None = None

    boundary_path = root / "site_boundary.yaml"
    if boundary_path.is_file():
        try:
            site_boundary = load_site_boundary(
                boundary_path,
                expected_frame_id=expected_frame_id,
            )
            states.append(
                _state(
                    "site_boundary",
                    ASSET_LOADED,
                    boundary_path,
                    schema=site_boundary.schema,
                    frame_id=site_boundary.frame_id,
                    status=site_boundary.status,
                )
            )
        except Exception as exc:
            states.append(
                _state(
                    "site_boundary",
                    ASSET_INVALID,
                    boundary_path,
                    error=str(exc),
                )
            )
    else:
        states.append(_state("site_boundary", ASSET_MISSING, None))

    candidate_path = root / "navigation_map_12f.yaml"
    if candidate_path.is_file():
        try:
            candidate_navigation = load_navigation_grid(candidate_path)
            if candidate_navigation.frame_id != expected_frame_id:
                raise ValueError(
                    "candidate navigation frame_id mismatch: "
                    f"expected {expected_frame_id}, got {candidate_navigation.frame_id}"
                )
            states.append(
                _state(
                    "navigation_12f",
                    ASSET_LOADED,
                    candidate_path,
                    frame_id=candidate_navigation.frame_id,
                    status="DRAFT",
                )
            )
        except Exception as exc:
            states.append(
                _state(
                    "navigation_12f",
                    ASSET_INVALID,
                    candidate_path,
                    error=str(exc),
                )
            )
    else:
        states.append(_state("navigation_12f", ASSET_MISSING, None))

    evidence_path = root / "traversability_evidence.yaml"
    if evidence_path.is_file():
        try:
            traversability = load_traversability_evidence(
                evidence_path,
                expected_frame_id=expected_frame_id,
            )
            states.append(
                _state(
                    "traversability_evidence",
                    ASSET_LOADED,
                    evidence_path,
                    schema=traversability.schema,
                    frame_id=traversability.frame_id,
                    status=traversability.status,
                )
            )
        except Exception as exc:
            states.append(
                _state(
                    "traversability_evidence",
                    ASSET_INVALID,
                    evidence_path,
                    error=str(exc),
                )
            )
    else:
        states.append(_state("traversability_evidence", ASSET_MISSING, None))

    return RouteDebug12FBundle(
        run_dir=root,
        frame_id=str(expected_frame_id),
        candidate_navigation=candidate_navigation,
        site_boundary=site_boundary,
        traversability=traversability,
        asset_states=tuple(states),
    )


def build_route_debug_12f_features(
    bundle: RouteDebug12FBundle,
) -> list[dict[str, Any]]:
    """Build only vector features; candidate/rich masks remain raster layers."""
    boundary = bundle.site_boundary
    if boundary is None:
        return []
    coordinates = [
        [float(x), float(y)] for x, y in boundary.outer_boundary_xy
    ]
    if coordinates and coordinates[0] != coordinates[-1]:
        coordinates.append(list(coordinates[0]))
    return [
        {
            "type": "Feature",
            "geometry": {
                "type": "Polygon",
                "coordinates": [coordinates],
            },
            "properties": {
                "layer_key": "semantics.site_boundary",
                "feature_id": "semantics:site_boundary",
                "feature_kind": "SITE_BOUNDARY",
                "status": boundary.status,
                "source_asset": "site_boundary.yaml",
                "source_id": "site_boundary",
                "source_field": "outer_boundary_xy",
                "boundary_semantics": boundary.boundary_semantics,
                "is_failure": False,
                "inspector": {
                    "SEMANTICS": {
                        "boundary_semantics": boundary.boundary_semantics,
                        "edge_policy": "TOUCH_OR_CROSS_IS_SITE_BOUNDARY_CONFLICT",
                        "vertex_count": len(boundary.outer_boundary_xy),
                    },
                    "SOURCE": {
                        "asset": "site_boundary.yaml",
                        "field": "outer_boundary_xy",
                    },
                },
            },
        }
    ]
