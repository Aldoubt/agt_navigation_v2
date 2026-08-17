from __future__ import annotations

import csv
import json
import os
from pathlib import Path
from typing import Any, Sequence

from .contracts import PathPoint

CSV_HEADER = ("index", "x_m", "y_m", "yaw_rad", "direction", "segment_type", "semantic_ref")


def _ensure_parent(path: Path) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)


def write_json_atomic(payload: Any, path: Path | str) -> None:
    path = Path(path)
    _ensure_parent(path)
    tmp = path.with_suffix(path.suffix + ".tmp")
    tmp.write_text(json.dumps(payload, indent=2, sort_keys=True, ensure_ascii=False, allow_nan=False) + "\n", encoding="utf-8")
    os.replace(tmp, path)


def write_path_csv(points: Sequence[PathPoint], path: Path | str) -> None:
    if not points:
        raise ValueError("cannot export empty successful path")
    path = Path(path)
    _ensure_parent(path)
    with path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.writer(f)
        writer.writerow(CSV_HEADER)
        for index, p in enumerate(points):
            writer.writerow((index, f"{p.x_m:.9f}", f"{p.y_m:.9f}", f"{p.yaw_rad:.9f}", p.direction, p.segment_type, p.semantic_ref))


def read_path_csv(path: Path | str) -> list[PathPoint]:
    path = Path(path)
    with path.open(newline="", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        if tuple(reader.fieldnames or ()) != CSV_HEADER:
            raise ValueError(f"unexpected path.csv schema: {reader.fieldnames}")
        return [
            PathPoint(float(row["x_m"]), float(row["y_m"]), float(row["yaw_rad"]), row["direction"], row["segment_type"], row["semantic_ref"])
            for row in reader
        ]


def write_path_geojson(points: Sequence[PathPoint], path: Path | str) -> None:
    if not points:
        raise ValueError("cannot export empty successful path")
    feature = {
        "type": "Feature",
        "properties": {
            "point_semantics": [
                {"index": i, "direction": p.direction, "segment_type": p.segment_type, "semantic_ref": p.semantic_ref}
                for i, p in enumerate(points)
            ]
        },
        "geometry": {"type": "LineString", "coordinates": [[p.x_m, p.y_m] for p in points]},
    }
    write_json_atomic({"type": "FeatureCollection", "features": [feature]}, path)
