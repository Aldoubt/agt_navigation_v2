from __future__ import annotations

from dataclasses import dataclass
import hashlib
import json
import math
from pathlib import Path
from typing import Mapping

import numpy as np


SYNTHETIC_FIXTURE_ID = "synthetic_greenhouse_v1"
SYNTHETIC_RESOLUTION_M = 0.10
SYNTHETIC_WIDTH_M = 30.0
SYNTHETIC_HEIGHT_M = 20.0
FREE_PIXEL = np.uint8(254)
OCCUPIED_PIXEL = np.uint8(0)


@dataclass(frozen=True)
class SyntheticGreenhouseFixture:
    image: np.ndarray
    resolution_m: float
    origin: tuple[float, float, float]
    scenarios: Mapping[str, Mapping[str, object]]
    semantic_document: Mapping[str, object]
    region_ids: tuple[str, ...]


def _rect_mask(
    width_cells: int,
    height_cells: int,
    rect: tuple[float, float, float, float],
) -> np.ndarray:
    xmin, xmax, ymin, ymax = rect
    xs = (np.arange(width_cells, dtype=float) + 0.5) * SYNTHETIC_RESOLUTION_M
    ys = (np.arange(height_cells, dtype=float) + 0.5) * SYNTHETIC_RESOLUTION_M
    world_mask = (
        (xs[None, :] >= xmin)
        & (xs[None, :] < xmax)
        & (ys[:, None] >= ymin)
        & (ys[:, None] < ymax)
    )
    return np.flipud(world_mask)


def _polygon_feature(feature_id: str, feature_type: str, rect, **properties):
    xmin, xmax, ymin, ymax = rect
    return {
        "type": "Feature",
        "id": feature_id,
        "properties": {
            "id": feature_id,
            "feature_type": feature_type,
            "enabled": True,
            **properties,
        },
        "geometry": {
            "type": "Polygon",
            "coordinates": [[
                [xmin, ymin],
                [xmax, ymin],
                [xmax, ymax],
                [xmin, ymax],
                [xmin, ymin],
            ]],
        },
    }


def _scenario(
    scenario_id: str,
    description: str,
    start: tuple[float, float, float],
    goal: tuple[float, float, float],
    constraints: tuple[str, ...],
) -> dict[str, object]:
    return {
        "id": scenario_id,
        "level": "p2p",
        "development_fixture": True,
        "description": description,
        "start": {"x": start[0], "y": start[1], "yaw": start[2]},
        "goal": {"x": goal[0], "y": goal[1], "yaw": goal[2]},
        "expected_constraints": list(constraints),
        "synthetic_fixture": SYNTHETIC_FIXTURE_ID,
    }


def build_synthetic_greenhouse() -> SyntheticGreenhouseFixture:
    width_cells = int(round(SYNTHETIC_WIDTH_M / SYNTHETIC_RESOLUTION_M))
    height_cells = int(round(SYNTHETIC_HEIGHT_M / SYNTHETIC_RESOLUTION_M))
    image = np.full((height_cells, width_cells), FREE_PIXEL, dtype=np.uint8)

    occupied_regions: list[tuple[str, tuple[float, float, float, float]]] = [
        ("boundary_left", (0.0, 0.5, 0.0, 20.0)),
        ("boundary_right", (29.5, 30.0, 0.0, 20.0)),
        ("boundary_bottom", (0.0, 30.0, 0.0, 0.5)),
        ("boundary_top", (0.0, 30.0, 19.5, 20.0)),
        ("crop_bed_01", (4.0, 5.5, 4.0, 16.0)),
        ("crop_bed_02", (7.5, 9.0, 4.0, 16.0)),
        ("crop_bed_03", (11.0, 12.5, 4.0, 16.0)),
        ("crop_bed_04", (14.5, 16.0, 4.0, 16.0)),
        ("crop_bed_05", (18.0, 19.5, 4.0, 16.0)),
        # These two rows deliberately extend farther into the upper headland,
        # creating a narrow 1.5 m maneuvering pocket for S04.
        ("crop_bed_06", (21.5, 23.0, 4.0, 18.0)),
        ("crop_bed_07", (25.0, 26.5, 4.0, 18.0)),
        # Occupancy obstacle blocks the center of one row aisle. A legal detour
        # remains possible through the bottom/top headlands rather than through crops.
        ("blocked_row_obstacle", (12.8, 14.2, 9.5, 10.5)),
    ]
    for _, rect in occupied_regions:
        image[_rect_mask(width_cells, height_cells, rect)] = OCCUPIED_PIXEL

    scenarios = {
        "S01_straight_row": _scenario(
            "S01_straight_row",
            "Straight aisle reference with no intended curvature challenge.",
            (6.5, 6.0, math.pi / 2.0),
            (6.5, 13.0, math.pi / 2.0),
            ("free_space",),
        ),
        "S02_90deg_entry": _scenario(
            "S02_90deg_entry",
            "Headland-to-row entry requiring a controlled ninety-degree transition.",
            (2.0, 2.0, 0.0),
            (6.5, 8.0, math.pi / 2.0),
            ("heading_change", "footprint", "minimum_turning_radius"),
        ),
        "S03_headland_uturn": _scenario(
            "S03_headland_uturn",
            "Adjacent-row U-turn using the wide upper headland.",
            (6.5, 15.0, math.pi / 2.0),
            (10.0, 15.0, -math.pi / 2.0),
            ("minimum_turning_radius", "final_heading", "footprint"),
        ),
        "S04_narrow_headland": _scenario(
            "S04_narrow_headland",
            "Narrow upper headland intended to expose reverse or multi-stage Ackermann maneuver requirements.",
            (24.0, 17.0, math.pi / 2.0),
            (27.5, 17.0, -math.pi / 2.0),
            ("minimum_turning_radius", "reverse", "footprint"),
        ),
        "S05_blocked_row": _scenario(
            "S05_blocked_row",
            "Blocked aisle requiring a legal headland detour or an explicit no-path result.",
            (13.5, 6.0, math.pi / 2.0),
            (13.5, 14.0, math.pi / 2.0),
            ("obstacle", "footprint", "detour"),
        ),
    }

    features = [
        _polygon_feature(
            "field_boundary",
            "field_boundary",
            (0.5, 29.5, 0.5, 19.5),
        ),
        _polygon_feature("bottom_headland", "headland", (0.5, 29.5, 0.5, 4.0)),
        _polygon_feature("wide_headland", "headland", (0.5, 21.5, 16.0, 19.5)),
        _polygon_feature("narrow_headland", "headland", (21.5, 29.5, 18.0, 19.5)),
        _polygon_feature(
            "blocked_row_obstacle",
            "keepout_zone",
            (12.8, 14.2, 9.5, 10.5),
            evidence="synthetic_permanent_obstacle",
        ),
    ]
    for region_id, rect in occupied_regions:
        if region_id.startswith("crop_bed_"):
            features.append(_polygon_feature(region_id, "crop_row", rect))

    semantic_document = {
        "type": "FeatureCollection",
        "frame_id": "map",
        "fixture_id": SYNTHETIC_FIXTURE_ID,
        "features": features,
    }

    return SyntheticGreenhouseFixture(
        image=image,
        resolution_m=SYNTHETIC_RESOLUTION_M,
        origin=(0.0, 0.0, 0.0),
        scenarios=scenarios,
        semantic_document=semantic_document,
        region_ids=(
            "straight_aisle",
            "ninety_degree_entry",
            "wide_headland",
            "narrow_headland",
            "blocked_row_obstacle",
        ),
    )


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _write_pgm(path: Path, image: np.ndarray) -> None:
    height, width = image.shape
    path.write_bytes(
        f"P5\n{width} {height}\n255\n".encode("ascii") + image.tobytes(order="C")
    )


def _format_number(value: float) -> str:
    text = f"{float(value):.12f}".rstrip("0").rstrip(".")
    return text if text else "0"


def _write_scenario(path: Path, data: Mapping[str, object]) -> None:
    start = data["start"]
    goal = data["goal"]
    assert isinstance(start, Mapping)
    assert isinstance(goal, Mapping)
    constraints = data["expected_constraints"]
    assert isinstance(constraints, list)
    lines = [
        f"id: {data['id']}",
        "level: p2p",
        "development_fixture: true",
        f"description: {data['description']}",
        "start: {x: %s, y: %s, yaw: %s}" % (
            _format_number(float(start["x"])),
            _format_number(float(start["y"])),
            _format_number(float(start["yaw"])),
        ),
        "goal: {x: %s, y: %s, yaw: %s}" % (
            _format_number(float(goal["x"])),
            _format_number(float(goal["y"])),
            _format_number(float(goal["yaw"])),
        ),
        "expected_constraints: [%s]" % ", ".join(str(v) for v in constraints),
        f"synthetic_fixture: {SYNTHETIC_FIXTURE_ID}",
        "",
    ]
    path.write_text("\n".join(lines), encoding="utf-8")


def write_synthetic_greenhouse(
    map_dir: Path | str,
    scenario_dir: Path | str,
) -> dict[str, object]:
    map_dir = Path(map_dir)
    scenario_dir = Path(scenario_dir)
    map_dir.mkdir(parents=True, exist_ok=True)
    scenario_dir.mkdir(parents=True, exist_ok=True)
    fixture = build_synthetic_greenhouse()

    pgm_path = map_dir / "map.pgm"
    yaml_path = map_dir / "map.yaml"
    semantic_path = map_dir / "semantic.geojson"
    _write_pgm(pgm_path, fixture.image)
    yaml_path.write_text(
        "\n".join([
            "image: map.pgm",
            "mode: trinary",
            "resolution: 0.10",
            "origin: [0.0, 0.0, 0.0]",
            "negate: 0",
            "occupied_thresh: 0.65",
            "free_thresh: 0.25",
            "",
        ]),
        encoding="utf-8",
    )
    semantic_path.write_text(
        json.dumps(fixture.semantic_document, indent=2, sort_keys=True, ensure_ascii=False) + "\n",
        encoding="utf-8",
    )

    scenario_assets: dict[str, dict[str, str]] = {}
    for scenario_id in sorted(fixture.scenarios):
        filename = f"{scenario_id}.yaml"
        path = scenario_dir / filename
        _write_scenario(path, fixture.scenarios[scenario_id])
        scenario_assets[scenario_id] = {
            "path": filename,
            "sha256": _sha256(path),
        }

    manifest: dict[str, object] = {
        "schema_version": "1.0",
        "fixture_id": SYNTHETIC_FIXTURE_ID,
        "formal": False,
        "resolution_m": SYNTHETIC_RESOLUTION_M,
        "size_m": [SYNTHETIC_WIDTH_M, SYNTHETIC_HEIGHT_M],
        "generator": "agt_route_benchmark.synthetic_greenhouse",
        "assets": {
            "map.pgm": _sha256(pgm_path),
            "map.yaml": _sha256(yaml_path),
            "semantic.geojson": _sha256(semantic_path),
        },
        "scenario_assets": scenario_assets,
        "region_ids": list(fixture.region_ids),
    }
    (map_dir / "fixture_manifest.json").write_text(
        json.dumps(manifest, indent=2, sort_keys=True, ensure_ascii=False) + "\n",
        encoding="utf-8",
    )
    return manifest
