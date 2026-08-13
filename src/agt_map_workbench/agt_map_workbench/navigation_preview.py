"""Qt overlay rendering for ground-relative Navigation Map evidence."""

from __future__ import annotations

import numpy as np
from PyQt5.QtGui import QImage, QPixmap
from PyQt5.QtWidgets import QGraphicsPixmapItem

from agt_offline_assets import FREE, OCCUPIED, UNKNOWN, NavigationMapResult


_LAYER_NAMES = {"final", "ground", "obstacle", "slope", "step"}


def _rgba_for_result(result: NavigationMapResult, layer: str) -> np.ndarray:
    if layer not in _LAYER_NAMES:
        raise ValueError(f"unsupported navigation preview layer: {layer}")
    height, width = result.occupancy.shape
    rgba = np.zeros((height, width, 4), dtype=np.uint8)

    if layer == "final":
        occupancy = result.occupancy
        rgba[occupancy == FREE] = (60, 210, 100, 80)
        rgba[occupancy == OCCUPIED] = (255, 70, 70, 190)
        rgba[occupancy == UNKNOWN] = (150, 155, 165, 55)
    elif layer == "ground":
        values = result.ground_height_m
        finite = np.isfinite(values)
        if np.any(finite):
            low = float(np.nanpercentile(values, 2.0))
            high = float(np.nanpercentile(values, 98.0))
            scale = max(1e-9, high - low)
            t = np.clip(np.nan_to_num((values - low) / scale), 0.0, 1.0)
            rgba[..., 0] = np.where(finite, (40 + 160 * t).astype(np.uint8), 0)
            rgba[..., 1] = np.where(
                finite,
                (110 + 120 * (1.0 - np.abs(t - 0.5) * 2.0)).astype(np.uint8),
                0,
            )
            rgba[..., 2] = np.where(finite, (230 - 150 * t).astype(np.uint8), 0)
            rgba[..., 3] = np.where(finite, 120, 0).astype(np.uint8)
    elif layer == "obstacle":
        values = result.obstacle_count.astype(np.float64)
        maximum = max(1.0, float(np.max(values)))
        t = np.clip(values / maximum, 0.0, 1.0)
        rgba[..., 0] = np.where(values > 0, 255, 0).astype(np.uint8)
        rgba[..., 1] = np.where(values > 0, (170 * (1.0 - t)).astype(np.uint8), 0)
        rgba[..., 2] = np.where(values > 0, 40, 0).astype(np.uint8)
        rgba[..., 3] = np.where(values > 0, (70 + 170 * t).astype(np.uint8), 0)
    elif layer == "slope":
        values = result.slope_deg
        finite = np.isfinite(values)
        scale = max(1.0, float(result.config.maximum_slope_deg))
        t = np.clip(np.nan_to_num(values) / scale, 0.0, 1.0)
        rgba[..., 0] = np.where(finite, (80 + 175 * t).astype(np.uint8), 0)
        rgba[..., 1] = np.where(finite, (210 - 160 * t).astype(np.uint8), 0)
        rgba[..., 2] = np.where(finite, 50, 0).astype(np.uint8)
        rgba[..., 3] = np.where(finite, 110, 0).astype(np.uint8)
    else:
        values = result.step_m
        finite = np.isfinite(values)
        scale = max(1e-6, float(result.config.maximum_step_m))
        t = np.clip(np.nan_to_num(values) / scale, 0.0, 1.0)
        rgba[..., 0] = np.where(finite, (80 + 175 * t).astype(np.uint8), 0)
        rgba[..., 1] = np.where(finite, (120 - 80 * t).astype(np.uint8), 0)
        rgba[..., 2] = np.where(finite, (210 + 45 * t).astype(np.uint8), 0)
        rgba[..., 3] = np.where(finite, 110, 0).astype(np.uint8)

    # Internal result rows start at minimum world Y; QImage row 0 must be maximum Y.
    return np.flipud(rgba).copy()


def navigation_layer_pixmap(result: NavigationMapResult, layer: str) -> QPixmap:
    rgba = _rgba_for_result(result, layer)
    height, width, _ = rgba.shape
    image = QImage(
        rgba.data,
        width,
        height,
        int(rgba.strides[0]),
        QImage.Format_RGBA8888,
    ).copy()
    return QPixmap.fromImage(image)


class NavigationPreviewItem(QGraphicsPixmapItem):
    """Map-aligned semi-transparent evidence overlay."""

    def __init__(self) -> None:
        super().__init__()
        self.setZValue(5.0)
        self.setVisible(False)

    def set_result(self, result: NavigationMapResult, layer: str) -> None:
        self.setPixmap(navigation_layer_pixmap(result, layer))
        maximum_y = result.origin_y_m + result.height * result.resolution_m
        self.setPos(float(result.origin_x_m), float(-maximum_y))
        self.setScale(float(result.resolution_m))
        self.setVisible(True)

    def clear_result(self) -> None:
        self.setPixmap(QPixmap())
        self.setVisible(False)
