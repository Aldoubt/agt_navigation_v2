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


def _require_mapping(value: Any, field_name: str) -> Mapping[str, Any]:
    if not isinstance(value, Mapping):
        raise ValueError(f"{field_name} must be a mapping")
    return value


def _require_sequence(value: Any, field_name: str) -> list[Any] | tuple[Any, ...]:
    if not isinstance(value, (list, tuple)):
        raise ValueError(f"{field_name} must be a sequence")
    return value


def _finite_float(value: Any, field_name: str) -> float:
    numeric = float(value)
    if not np.isfinite(numeric):
        raise ValueError(f"{field_name} must be finite")
    return numeric


def _fixed_floats(value: Any, length: int, field_name: str) -> tuple[float, ...]:
    values = _require_sequence(value, field_name)
    if len(values) != length:
        raise ValueError(f"{field_name} must contain exactly {length} values")
    return tuple(_finite_float(item, field_name) for item in values)


def load_turn_zones(path: str | Path) -> TurnZoneSet:
    """Load and strictly validate one v1 turn-zone asset."""
    input_path = Path(path).expanduser().resolve()
    if not input_path.is_file():
        raise FileNotFoundError(f"turn-zone YAML not found: {input_path}")

    payload = yaml.safe_load(input_path.read_text(encoding="utf-8")) or {}
    data = _require_mapping(payload, "turn-zone YAML")
    required_top = {
        "schema",
        "status",
        "frame_id",
        "source",
        "row_direction_xy",
        "zone_count",
        "zones",
    }
    missing_top = required_top.difference(data)
    if missing_top:
        raise ValueError(f"turn-zone YAML missing required keys: {sorted(missing_top)}")
    if str(data["schema"]) != TURN_ZONE_SCHEMA:
        raise ValueError(f"expected {TURN_ZONE_SCHEMA}, got {data['schema']}")

    frame_id = str(data["frame_id"])
    if not frame_id:
        raise ValueError("turn-zone frame_id must not be empty")
    row_direction = _fixed_floats(data["row_direction_xy"], 2, "row_direction_xy")
    if float(np.linalg.norm(np.asarray(row_direction, dtype=np.float64))) <= 1.0e-12:
        raise ValueError("row_direction_xy must be non-zero")

    source = dict(_require_mapping(data["source"], "source"))
    raw_zones = _require_sequence(data["zones"], "zones")
    zone_count = int(data["zone_count"])
    if zone_count != len(raw_zones):
        raise ValueError("zone_count does not match zones length")

    zones: list[TurnZone] = []
    seen_zone_ids: set[str] = set()
    for raw_zone in raw_zones:
        zone_data = _require_mapping(raw_zone, "turn zone")
        required_zone = {
            "zone_id",
            "side",
            "source_kind",
            "polygon_xy",
            "supported_aisle_ids",
            "endpoint_count",
            "free_fraction",
            "permissions",
            "semantics",
        }
        missing_zone = required_zone.difference(zone_data)
        if missing_zone:
            raise ValueError(
                f"turn zone missing required keys: {sorted(missing_zone)}"
            )

        zone_id = str(zone_data["zone_id"])
        if not zone_id or zone_id in seen_zone_ids:
            raise ValueError(f"duplicate or empty turn-zone id: {zone_id}")
        seen_zone_ids.add(zone_id)

        side = str(zone_data["side"])
        if side not in {"LOW_U", "HIGH_U"}:
            raise ValueError(f"turn zone {zone_id} has invalid side: {side}")

        raw_polygon = _require_sequence(zone_data["polygon_xy"], "polygon_xy")
        if len(raw_polygon) < 3:
            raise ValueError(f"turn zone {zone_id} polygon must have at least 3 points")
        polygon = tuple(
            _fixed_floats(point, 2, f"turn zone {zone_id} polygon_xy")
            for point in raw_polygon
        )

        raw_aisles = _require_sequence(
            zone_data["supported_aisle_ids"], "supported_aisle_ids"
        )
        supported_aisles = tuple(str(value) for value in raw_aisles)
        if any(not value for value in supported_aisles):
            raise ValueError(f"turn zone {zone_id} has empty supported aisle id")
        if len(supported_aisles) != len(set(supported_aisles)):
            raise ValueError(f"turn zone {zone_id} has duplicate supported aisle IDs")

        endpoint_count = int(zone_data["endpoint_count"])
        if endpoint_count < 0 or endpoint_count != len(supported_aisles):
            raise ValueError(
                f"turn zone {zone_id} endpoint_count does not match supported aisles"
            )

        raw_free_fraction = zone_data["free_fraction"]
        if raw_free_fraction is None:
            free_fraction = float("nan")
        else:
            free_fraction = _finite_float(
                raw_free_fraction, f"turn zone {zone_id} free_fraction"
            )
            if not 0.0 <= free_fraction <= 1.0:
                raise ValueError(
                    f"turn zone {zone_id} free_fraction must be within [0, 1]"
                )

        permissions = _require_mapping(zone_data["permissions"], "permissions")
        required_permissions = {"turn", "reverse", "direction_change"}
        if required_permissions.difference(permissions):
            raise ValueError(f"turn zone {zone_id} permissions are incomplete")
        for permission in required_permissions:
            if not isinstance(permissions[permission], bool):
                raise ValueError(
                    f"turn zone {zone_id} permission {permission} must be boolean"
                )

        if str(zone_data["semantics"]) != "SEARCH_ENVELOPE_NOT_FREE_SPACE_TRUTH":
            raise ValueError(f"turn zone {zone_id} has invalid semantics")
        source_kind = str(zone_data["source_kind"])
        if not source_kind:
            raise ValueError(f"turn zone {zone_id} source_kind must not be empty")

        zones.append(
            TurnZone(
                zone_id=zone_id,
                side=side,
                polygon_xy=polygon,
                supported_aisle_ids=supported_aisles,
                endpoint_count=endpoint_count,
                free_fraction=free_fraction,
                allow_turn=permissions["turn"],
                allow_reverse=permissions["reverse"],
                allow_direction_change=permissions["direction_change"],
                source_kind=source_kind,
            )
        )

    return TurnZoneSet(
        frame_id=frame_id,
        row_direction_xy=row_direction,
        zones=tuple(zones),
        source=source,
        schema=str(data["schema"]),
        status=str(data["status"]),
    )
