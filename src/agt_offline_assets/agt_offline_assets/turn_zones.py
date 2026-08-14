"""Deterministic turn-zone candidates for agricultural route production.

Turn zones are offline semantic envelopes that constrain where adjacent aisle
connectors may turn, reverse, or change direction.  They do not assert that the
whole polygon is collision-free; every generated connector still requires the
canonical Navigation Map and full-footprint feasibility gate.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Mapping

import numpy as np
import yaml

from .agricultural_aisle_graph import AgriculturalAisleGraph
from .navigation_map_derivation import FREE, NavigationMapResult


TURN_ZONE_SCHEMA = "agt_turn_zones/v1"


@dataclass(frozen=True)
class TurnZoneConfig:
    outward_extension_m: float = 0.80
    inward_depth_m: float = 1.50
    lateral_padding_m: float = 0.50
    minimum_endpoint_count: int = 2
    round_decimals: int = 6

    def validate(self) -> None:
        if self.outward_extension_m < 0.0:
            raise ValueError("outward_extension_m must be >= 0")
        if self.inward_depth_m <= 0.0:
            raise ValueError("inward_depth_m must be > 0")
        if self.lateral_padding_m < 0.0:
            raise ValueError("lateral_padding_m must be >= 0")
        if self.minimum_endpoint_count < 2:
            raise ValueError("minimum_endpoint_count must be >= 2")
        if self.round_decimals < 0:
            raise ValueError("round_decimals must be >= 0")


@dataclass(frozen=True)
class TurnZone:
    zone_id: str
    side: str
    polygon_xy: tuple[tuple[float, float], ...]
    supported_aisle_ids: tuple[str, ...]
    endpoint_count: int
    free_fraction: float
    allow_turn: bool = True
    allow_reverse: bool = True
    allow_direction_change: bool = True
    source_kind: str = "AUTO_ENDPOINT_ENVELOPE"


@dataclass(frozen=True)
class TurnZoneSet:
    frame_id: str
    row_direction_xy: tuple[float, float]
    zones: tuple[TurnZone, ...]
    source: Mapping[str, Any] = field(default_factory=dict)
    schema: str = TURN_ZONE_SCHEMA
    status: str = "DRAFT"


def _normalize(direction_xy) -> np.ndarray:
    direction = np.asarray(direction_xy, dtype=np.float64).reshape(2)
    norm = float(np.linalg.norm(direction))
    if norm <= 1.0e-12:
        raise ValueError("row direction must be non-zero")
    return direction / norm


def _to_world_polygon(
    direction: np.ndarray,
    perpendicular: np.ndarray,
    u0: float,
    u1: float,
    v0: float,
    v1: float,
) -> tuple[tuple[float, float], ...]:
    corners = ((u0, v0), (u1, v0), (u1, v1), (u0, v1))
    return tuple(
        (
            float(u * direction[0] + v * perpendicular[0]),
            float(u * direction[1] + v * perpendicular[1]),
        )
        for u, v in corners
    )


def _points_inside_polygon(xx: np.ndarray, yy: np.ndarray, polygon_xy) -> np.ndarray:
    """Vectorized even-odd point-in-polygon test for grid-cell centers."""
    polygon = np.asarray(polygon_xy, dtype=np.float64)
    x = xx.ravel()
    y = yy.ravel()
    inside = np.zeros(x.shape, dtype=bool)
    j = polygon.shape[0] - 1
    for i in range(polygon.shape[0]):
        xi, yi = polygon[i]
        xj, yj = polygon[j]
        crosses = ((yi > y) != (yj > y)) & (
            x < (xj - xi) * (y - yi) / (yj - yi + 1.0e-15) + xi
        )
        inside ^= crosses
        j = i
    return inside.reshape(xx.shape)


def _grid_xy(navigation: NavigationMapResult) -> tuple[np.ndarray, np.ndarray]:
    rows, cols = np.indices((navigation.height, navigation.width), dtype=np.float64)
    xx = navigation.origin_x_m + (cols + 0.5) * navigation.resolution_m
    yy = navigation.origin_y_m + (rows + 0.5) * navigation.resolution_m
    return xx, yy


def _zone_free_fraction(
    navigation: NavigationMapResult | None,
    polygon_xy: tuple[tuple[float, float], ...],
) -> float:
    if navigation is None:
        return float("nan")
    xx, yy = _grid_xy(navigation)
    selected = _points_inside_polygon(xx, yy, polygon_xy)
    count = int(np.count_nonzero(selected))
    if count == 0:
        return 0.0
    free = np.asarray(navigation.occupancy, dtype=np.uint8) == FREE
    return float(np.count_nonzero(selected & free) / count)


def derive_turn_zone_candidates(
    graph: AgriculturalAisleGraph,
    navigation: NavigationMapResult | None = None,
    config: TurnZoneConfig | None = None,
    *,
    source: Mapping[str, Any] | None = None,
) -> TurnZoneSet:
    """Create low-u and high-u connector envelopes from aisle endpoints.

    The envelopes define where connector generation is permitted to search.
    They are deliberately not treated as free-space truth.
    """
    cfg = config or TurnZoneConfig()
    cfg.validate()
    if len(graph.aisles) < cfg.minimum_endpoint_count:
        return TurnZoneSet(
            frame_id=graph.frame_id,
            row_direction_xy=graph.row_direction_xy,
            zones=(),
            source=dict(source or {}),
        )

    direction = _normalize(graph.row_direction_xy)
    perpendicular = np.array([-direction[1], direction[0]], dtype=np.float64)

    starts = []
    ends = []
    for aisle in graph.aisles:
        starts.append((aisle.aisle_id, np.asarray(aisle.start_pose[:2], dtype=np.float64)))
        ends.append((aisle.aisle_id, np.asarray(aisle.end_pose[:2], dtype=np.float64)))

    zones: list[TurnZone] = []
    for side, endpoints in (("LOW_U", starts), ("HIGH_U", ends)):
        ids = tuple(item[0] for item in endpoints)
        xy = np.vstack([item[1] for item in endpoints])
        uu = xy @ direction
        vv = xy @ perpendicular
        v0 = float(np.min(vv) - cfg.lateral_padding_m)
        v1 = float(np.max(vv) + cfg.lateral_padding_m)
        if side == "LOW_U":
            reference = float(np.median(uu))
            u0 = float(np.min(uu) - cfg.outward_extension_m)
            u1 = max(reference + cfg.inward_depth_m, float(np.max(uu)))
            zone_id = "turn_low_u"
        else:
            reference = float(np.median(uu))
            u0 = min(reference - cfg.inward_depth_m, float(np.min(uu)))
            u1 = float(np.max(uu) + cfg.outward_extension_m)
            zone_id = "turn_high_u"
        polygon = _to_world_polygon(direction, perpendicular, u0, u1, v0, v1)
        zones.append(
            TurnZone(
                zone_id=zone_id,
                side=side,
                polygon_xy=polygon,
                supported_aisle_ids=ids,
                endpoint_count=len(ids),
                free_fraction=_zone_free_fraction(navigation, polygon),
            )
        )

    merged_source = dict(graph.source)
    merged_source.update(dict(source or {}))
    return TurnZoneSet(
        frame_id=graph.frame_id,
        row_direction_xy=graph.row_direction_xy,
        zones=tuple(zones),
        source=merged_source,
    )


def turn_zones_to_dict(
    zones: TurnZoneSet,
    *,
    round_decimals: int = 6,
) -> dict[str, Any]:
    def r(value: float) -> float | None:
        if not np.isfinite(value):
            return None
        return round(float(value), int(round_decimals))

    return {
        "schema": zones.schema,
        "status": zones.status,
        "frame_id": zones.frame_id,
        "source": dict(zones.source),
        "row_direction_xy": [r(zones.row_direction_xy[0]), r(zones.row_direction_xy[1])],
        "zone_count": len(zones.zones),
        "zones": [
            {
                "zone_id": zone.zone_id,
                "side": zone.side,
                "source_kind": zone.source_kind,
                "polygon_xy": [[r(x), r(y)] for x, y in zone.polygon_xy],
                "supported_aisle_ids": list(zone.supported_aisle_ids),
                "endpoint_count": zone.endpoint_count,
                "free_fraction": r(zone.free_fraction),
                "permissions": {
                    "turn": zone.allow_turn,
                    "reverse": zone.allow_reverse,
                    "direction_change": zone.allow_direction_change,
                },
                "semantics": "SEARCH_ENVELOPE_NOT_FREE_SPACE_TRUTH",
            }
            for zone in zones.zones
        ],
    }


def write_turn_zones(
    zones: TurnZoneSet,
    path: str | Path,
    *,
    round_decimals: int = 6,
) -> Path:
    output = Path(path)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(
        yaml.safe_dump(
            turn_zones_to_dict(zones, round_decimals=round_decimals),
            sort_keys=False,
            allow_unicode=True,
        ),
        encoding="utf-8",
    )
    return output
