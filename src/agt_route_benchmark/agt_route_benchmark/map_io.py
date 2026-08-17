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


def nav2_map_occupancy_data(nav_map: Nav2Map) -> tuple[int, ...]:
    """Convert a Nav2 map image into ROS OccupancyGrid row-major data.

    Image files are stored top-row first while ``nav_msgs/OccupancyGrid`` starts
    at the map origin in the lower-left corner. The returned tuple therefore
    flips the image vertically before flattening.
    """
    image = np.asarray(nav_map.image)
    if image.ndim == 3:
        if image.shape[2] < 3:
            raise ValueError("RGB map image must contain at least three channels")
        gray = np.mean(image[..., :3].astype(float), axis=2)
    else:
        gray = image.astype(float)

    if np.issubdtype(image.dtype, np.integer):
        gray = gray / float(np.iinfo(image.dtype).max)
    elif gray.size and float(np.nanmax(gray)) > 1.0 + 1e-9:
        gray = gray / 255.0

    if not np.all(np.isfinite(gray)):
        raise ValueError("map image contains non-finite pixel values")
    if np.any(gray < -1e-9) or np.any(gray > 1.0 + 1e-9):
        raise ValueError("map image pixels must normalize to [0, 1]")

    occupancy_probability = gray if nav_map.negate else 1.0 - gray
    occupancy = np.full(gray.shape, -1, dtype=np.int16)
    occupancy[occupancy_probability > nav_map.occupied_thresh] = 100
    occupancy[occupancy_probability < nav_map.free_thresh] = 0
    return tuple(int(value) for value in np.flipud(occupancy).reshape(-1))
