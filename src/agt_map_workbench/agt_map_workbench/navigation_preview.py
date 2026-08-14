"""Qt overlay rendering for ground-relative and agricultural structure evidence."""

from __future__ import annotations

import numpy as np
from PyQt5.QtGui import QImage, QPixmap
from PyQt5.QtWidgets import QGraphicsPixmapItem

from agt_offline_assets import (
    FREE,
    OCCUPIED,
    UNKNOWN,
    NavigationMapResult,
    NavigationStructureResult,
)


_BASE_LAYERS = {"final", "ground", "obstacle", "slope", "step"}
_STRUCTURE_LAYERS = {
    "ground_confidence",
    "robust_slope",
    "plane_residual",
    "row_support",
    "row_regularized",
    "aisle_candidate",
}


def _rgba_for_result(
    result: NavigationMapResult,
    layer: str,
    structure: NavigationStructureResult | None = None,
) -> np.ndarray:
    if layer not in _BASE_LAYERS | _STRUCTURE_LAYERS:
        raise ValueError(f"unsupported navigation preview layer: {layer}")
    if layer in _STRUCTURE_LAYERS and structure is None:
        raise ValueError(f"navigation structure layer requires structure result: {layer}")

    height, width = result.occupancy.shape
    rgba = np.zeros((height, width, 4), dtype=np.uint8)

    if layer == "final":
        occupancy = result.occupancy
        rgba[occupancy == FREE] = (60, 210, 100, 115)
        rgba[occupancy == OCCUPIED] = (255, 70, 70, 215)
        rgba[occupancy == UNKNOWN] = (150, 155, 165, 80)
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
            rgba[..., 3] = np.where(finite, 175, 0).astype(np.uint8)
    elif layer == "obstacle":
        values = result.obstacle_count.astype(np.float64)
        maximum = max(1.0, float(np.max(values)))
        t = np.clip(values / maximum, 0.0, 1.0)
        rgba[..., 0] = np.where(values > 0, 255, 0).astype(np.uint8)
        rgba[..., 1] = np.where(values > 0, (170 * (1.0 - t)).astype(np.uint8), 0)
        rgba[..., 2] = np.where(values > 0, 40, 0).astype(np.uint8)
        rgba[..., 3] = np.where(values > 0, (120 + 120 * t).astype(np.uint8), 0)
    elif layer == "slope":
        values = result.slope_deg
        finite = np.isfinite(values)
        scale = max(1.0, float(result.config.maximum_slope_deg))
        t = np.clip(np.nan_to_num(values) / scale, 0.0, 1.0)
        rgba[..., 0] = np.where(finite, (80 + 175 * t).astype(np.uint8), 0)
        rgba[..., 1] = np.where(finite, (210 - 160 * t).astype(np.uint8), 0)
        rgba[..., 2] = np.where(finite, 50, 0).astype(np.uint8)
        rgba[..., 3] = np.where(finite, 165, 0).astype(np.uint8)
    elif layer == "step":
        values = result.step_m
        finite = np.isfinite(values)
        scale = max(1e-6, float(result.config.maximum_step_m))
        t = np.clip(np.nan_to_num(values) / scale, 0.0, 1.0)
        rgba[..., 0] = np.where(finite, (80 + 175 * t).astype(np.uint8), 0)
        rgba[..., 1] = np.where(finite, (120 - 80 * t).astype(np.uint8), 0)
        rgba[..., 2] = np.where(finite, (210 + 45 * t).astype(np.uint8), 0)
        rgba[..., 3] = np.where(finite, 165, 0).astype(np.uint8)
    elif layer == "ground_confidence":
        values = np.clip(structure.ground_confidence, 0.0, 1.0)
        rgba[..., 0] = (255 * (1.0 - values)).astype(np.uint8)
        rgba[..., 1] = (230 * values).astype(np.uint8)
        rgba[..., 2] = (70 + 120 * values).astype(np.uint8)
        rgba[..., 3] = np.where(values > 0.0, 205, 0).astype(np.uint8)
    elif layer == "robust_slope":
        values = structure.robust_slope_deg
        finite = np.isfinite(values)
        scale = max(1.0, float(result.config.maximum_slope_deg))
        t = np.clip(np.nan_to_num(values) / scale, 0.0, 1.0)
        rgba[..., 0] = np.where(finite, (70 + 185 * t).astype(np.uint8), 0)
        rgba[..., 1] = np.where(finite, (225 - 180 * t).astype(np.uint8), 0)
        rgba[..., 2] = np.where(finite, (90 - 50 * t).astype(np.uint8), 0)
        rgba[..., 3] = np.where(finite, 205, 0).astype(np.uint8)
    elif layer == "plane_residual":
        values = structure.robust_plane_residual_m
        finite = np.isfinite(values)
        high = float(np.nanpercentile(values, 95.0)) if np.any(finite) else 1.0
        t = np.clip(np.nan_to_num(values) / max(high, 1e-6), 0.0, 1.0)
        rgba[..., 0] = np.where(finite, (50 + 205 * t).astype(np.uint8), 0)
        rgba[..., 1] = np.where(finite, (210 - 160 * t).astype(np.uint8), 0)
        rgba[..., 2] = np.where(finite, (230 - 180 * t).astype(np.uint8), 0)
        rgba[..., 3] = np.where(finite, 195, 0).astype(np.uint8)
    elif layer == "row_support":
        values = np.clip(structure.row_support, 0.0, 1.0)
        rgba[..., 0] = np.where(values > 0.0, (90 + 150 * values).astype(np.uint8), 0)
        rgba[..., 1] = np.where(values > 0.0, (70 + 90 * values).astype(np.uint8), 0)
        rgba[..., 2] = np.where(values > 0.0, 255, 0).astype(np.uint8)
        rgba[..., 3] = np.where(values > 0.0, (90 + 150 * values).astype(np.uint8), 0)
    elif layer == "row_regularized":
        mask = structure.row_regularized_obstacle
        rgba[mask] = (255, 80, 180, 220)
    else:
        mask = structure.aisle_candidate
        rgba[mask] = (40, 220, 255, 180)

    # Internal result rows start at minimum world Y; QImage row 0 must be maximum Y.
    return np.flipud(rgba).copy()


def navigation_layer_pixmap(
    result: NavigationMapResult,
    layer: str,
    structure: NavigationStructureResult | None = None,
) -> QPixmap:
    rgba = _rgba_for_result(result, layer, structure)
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
    """Map-aligned evidence overlay."""

    def __init__(self) -> None:
        super().__init__()
        self.setZValue(5.0)
        self.setVisible(False)

    def set_result(
        self,
        result: NavigationMapResult,
        layer: str,
        structure: NavigationStructureResult | None = None,
    ) -> None:
        self.setPixmap(navigation_layer_pixmap(result, layer, structure))
        maximum_y = result.origin_y_m + result.height * result.resolution_m
        self.setPos(float(result.origin_x_m), float(-maximum_y))
        self.setScale(float(result.resolution_m))
        self.setVisible(True)

    def clear_result(self) -> None:
        self.setPixmap(QPixmap())
        self.setVisible(False)
