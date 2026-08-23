from __future__ import annotations

import csv
import hashlib
import json
from pathlib import Path
from typing import Iterable

from .map_authority import verify_bound_map_authority
from .project import load_project


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _inside(point: tuple[float, float], bounds: tuple[float, float, float, float]) -> bool:
    x, y = point
    min_x, min_y, max_x, max_y = bounds
    return min_x <= x < max_x and min_y <= y < max_y


def _route_points(path: Path) -> tuple[tuple[float, float], ...]:
    with path.open("r", newline="", encoding="utf-8") as stream:
        reader = csv.DictReader(stream)
        fields = set(reader.fieldnames or ())
        if not {"x", "y"}.issubset(fields):
            raise ValueError("route CSV requires x and y columns")
        points = tuple((float(row["x"]), float(row["y"])) for row in reader)
    if not points:
        raise ValueError("route CSV contains no samples")
    return points


def _coordinate_pairs(value) -> Iterable[tuple[float, float]]:
    if isinstance(value, (list, tuple)):
        if len(value) >= 2 and all(isinstance(item, (int, float)) for item in value[:2]):
            yield float(value[0]), float(value[1])
            return
        for item in value:
            yield from _coordinate_pairs(item)


def _semantic_points(path: Path) -> tuple[tuple[float, float], ...]:
    data = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(data, dict) or data.get("type") != "FeatureCollection":
        raise ValueError("semantic map must be a GeoJSON FeatureCollection")
    points: list[tuple[float, float]] = []
    for feature in data.get("features", []):
        if not isinstance(feature, dict):
            continue
        geometry = feature.get("geometry") or {}
        if not isinstance(geometry, dict):
            continue
        points.extend(_coordinate_pairs(geometry.get("coordinates")))
    if not points:
        raise ValueError("semantic map contains no geometry coordinates")
    return tuple(points)


def _summary(points: tuple[tuple[float, float], ...], bounds) -> dict:
    inside_count = sum(1 for point in points if _inside(point, bounds))
    total = len(points)
    return {
        "total_count": total,
        "inside_count": inside_count,
        "outside_count": total - inside_count,
        "inside_ratio": float(inside_count) / float(total),
    }


def _formal_grid(project: dict, frame: dict) -> tuple[dict, str | None, str | None]:
    authority = project.get("map_authority")
    if authority is None:
        return dict(frame.get("grid") or {}), None, None
    verified = verify_bound_map_authority(authority)
    authority_grid = dict(verified["grid"])
    frame_grid = dict(frame.get("grid") or {})
    mirrored = {
        "resolution_m": frame_grid.get("resolution_m"),
        "origin_xy_m": frame_grid.get("origin_xy_m"),
        "width": frame_grid.get("width"),
        "height": frame_grid.get("height"),
    }
    if mirrored != authority_grid:
        raise ValueError("verified frame grid does not match bound map authority grid")
    if frame_grid.get("map_yaml_sha256") != verified["accepted_map_yaml_sha256"]:
        raise ValueError("verified frame map hash does not match bound map authority grid")
    grid = {**authority_grid, "map_yaml_sha256": verified["accepted_map_yaml_sha256"]}
    return grid, str(verified["authority"]), str(verified["status"])


def build_frame_alignment_report(
    project_dir: Path | str,
    route_csv: Path | str | None = None,
    semantic_map: Path | str | None = None,
) -> dict:
    root = Path(project_dir).expanduser().resolve()
    project = load_project(root)
    frame = project.get("frame") or {}
    if frame.get("verification") != "VERIFIED":
        raise ValueError("frame verification report requires a VERIFIED project frame")
    if frame.get("canonical_frame_id") != "map":
        raise ValueError("frame verification report requires canonical frame map")
    grid, authority_name, authority_status = _formal_grid(project, frame)
    try:
        resolution = float(grid["resolution_m"])
        origin_x, origin_y = (float(v) for v in grid["origin_xy_m"])
        width = int(grid["width"])
        height = int(grid["height"])
    except (KeyError, TypeError, ValueError) as exc:
        raise ValueError("verified project frame is missing canonical grid metadata") from exc
    bounds = (
        origin_x,
        origin_y,
        origin_x + width * resolution,
        origin_y + height * resolution,
    )

    route_record = None
    semantic_record = None
    passed = True
    if route_csv is not None:
        route_path = Path(route_csv).expanduser().resolve()
        route_record = _summary(_route_points(route_path), bounds)
        route_record.update({"path": str(route_path), "sha256": _sha256(route_path)})
        passed = passed and route_record["outside_count"] == 0
    if semantic_map is not None:
        semantic_path = Path(semantic_map).expanduser().resolve()
        semantic_record = _summary(_semantic_points(semantic_path), bounds)
        semantic_record.update({"path": str(semantic_path), "sha256": _sha256(semantic_path)})
        passed = passed and semantic_record["outside_count"] == 0

    report = {
        "schema": "agt_frame_alignment_report/v1",
        "status": "PASS" if passed else "FAIL",
        "project_id": project.get("project_id"),
        "site_id": project.get("site_id"),
        "canonical_frame_id": "map",
        "alignment_sha256": frame.get("alignment_sha256"),
        "map_authority": authority_name,
        "map_authority_status": authority_status,
        "grid": {
            "resolution_m": resolution,
            "origin_xy_m": [origin_x, origin_y],
            "width": width,
            "height": height,
            "bounds_m": list(bounds),
            "map_yaml_sha256": grid.get("map_yaml_sha256"),
        },
        "route": route_record,
        "semantic": semantic_record,
        "geometry_modified": False,
    }
    evidence = root / "evidence"
    evidence.mkdir(parents=True, exist_ok=True)
    target = evidence / "frame_alignment_report.json"
    target.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    return report
