from __future__ import annotations

from dataclasses import dataclass
import math
from pathlib import Path

import matplotlib.image as mpimg
import numpy as np
import yaml


@dataclass(frozen=True)
class Nav2Map:
    yaml_path: Path
    image_path: Path
    image: np.ndarray
    resolution_m: float
    origin: tuple[float, float, float]
    extent: tuple[float, float, float, float]
    negate: int
    occupied_thresh: float
    free_thresh: float


def load_nav2_map(path: Path | str) -> Nav2Map:
    yaml_path = Path(path).expanduser().resolve()
    data = yaml.safe_load(yaml_path.read_text(encoding="utf-8"))
    if not isinstance(data, dict):
        raise ValueError("Nav2 map YAML must be a mapping")
    for key in ("image", "resolution", "origin"):
        if key not in data:
            raise ValueError(f"Nav2 map YAML missing {key}")
    resolution = float(data["resolution"])
    if not math.isfinite(resolution) or resolution <= 0.0:
        raise ValueError("map resolution must be positive and finite")
    raw_origin = data["origin"]
    if not isinstance(raw_origin, (list, tuple)) or len(raw_origin) != 3:
        raise ValueError("map origin must be [x, y, yaw]")
    origin = tuple(float(v) for v in raw_origin)
    if not all(math.isfinite(v) for v in origin):
        raise ValueError("map origin must be finite")
    if abs(origin[2]) > 1e-9:
        raise ValueError("rotated map origin is not supported by the axis-aligned paper renderer")
    image_path = (yaml_path.parent / str(data["image"])).expanduser().resolve()
    if not image_path.is_file():
        raise ValueError(f"map image does not exist: {image_path}")
    image = np.asarray(mpimg.imread(image_path))
    if image.ndim not in (2, 3) or image.shape[0] <= 0 or image.shape[1] <= 0:
        raise ValueError("map image must be a non-empty 2D or RGB/RGBA image")
    height, width = image.shape[:2]
    extent = (
        origin[0],
        origin[0] + width * resolution,
        origin[1],
        origin[1] + height * resolution,
    )
    return Nav2Map(
        yaml_path=yaml_path,
        image_path=image_path,
        image=image,
        resolution_m=resolution,
        origin=origin,
        extent=extent,
        negate=int(data.get("negate", 0)),
        occupied_thresh=float(data.get("occupied_thresh", 0.65)),
        free_thresh=float(data.get("free_thresh", 0.196)),
    )
