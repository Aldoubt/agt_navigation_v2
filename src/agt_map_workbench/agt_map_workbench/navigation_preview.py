"""Qt overlay rendering for ground-relative and agricultural evidence."""

from __future__ import annotations

import numpy as np
from PyQt5.QtGui import QImage, QPixmap
from PyQt5.QtWidgets import QGraphicsPixmapItem

from agt_offline_assets import (
    FREE,
    OCCUPIED,
    UNKNOWN,
    CorridorRefinementResult,
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
_CORRIDOR_LAYERS = {
    "row_centerline",
    "row_structural_band",
    "vegetation_envelope",
    "boundary_exclusion",
    "boundary_aisle",
    "boundary_aisle_centerline",
    "aisle_geometric_envelope",
    "refined_aisle",
    "aisle_centerline",
}


def _qimage_from_rgba(rgba: np.ndarray) -> QImage:
    image_data = np.ascontiguousarray(rgba, dtype=np.uint8)
    height, width, _channels = image_data.shape
    return QImage(
        image_data.data,
        width,
        height,
        int(image_data.strides[0]),
        QImage.Format_RGBA8888,
    ).copy()


def _rgba_for_result(
    result: NavigationMapResult,
    layer: str,
    structure: NavigationStructureResult | None = None,
    corridor: CorridorRefinementResult | None = None,
) -> np.ndarray:
    if layer not in _BASE_LAYERS | _STRUCTURE_LAYERS | _CORRIDOR_LAYERS:
        raise ValueError(f"unsupported navigation preview layer: {layer}")
    if layer in _STRUCTURE_LAYERS and structure is None:
        raise ValueError(f"navigation structure layer requires structure result: {layer}")
    if layer in _CORRIDOR_LAYERS and corridor is None:
        raise ValueError(f"corridor layer requires corridor result: {layer}")

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
        rgba[structure.row_regularized_obstacle] = (255, 80, 180, 220)
    elif layer == "aisle_candidate":
        rgba[structure.aisle_candidate] = (40, 220, 255, 180)
    elif layer == "row_centerline":
        rgba[corridor.row_centerline] = (255, 230, 70, 245)
    elif layer == "row_structural_band":
        rgba[corridor.row_structural_band] = (255, 80, 190, 210)
    elif layer == "vegetation_envelope":
        rgba[corridor.vegetation_envelope] = (255, 145, 40, 205)
    elif layer == "boundary_exclusion":
        rgba[corridor.boundary_exclusion] = (255, 65, 65, 210)
    elif layer == "boundary_aisle":
        rgba[corridor.boundary_aisle_candidate] = (40, 170, 255, 235)
    elif layer == "boundary_aisle_centerline":
        rgba[corridor.boundary_aisle_centerline] = (80, 255, 190, 250)
    elif layer == "aisle_geometric_envelope":
        rgba[corridor.aisle_geometric_envelope] = (245, 205, 65, 105)
    elif layer == "refined_aisle":
        rgba[corridor.aisle_candidate] = (35, 225, 255, 220)
    elif layer == "aisle_centerline":
        rgba[corridor.aisle_centerline] = (90, 255, 120, 245)

    return np.flipud(rgba).copy()


def navigation_layer_pixmap(
    result: NavigationMapResult,
    layer: str,
    structure: NavigationStructureResult | None = None,
    corridor: CorridorRefinementResult | None = None,
) -> QPixmap:
    rgba = _rgba_for_result(result, layer, structure, corridor)
    return QPixmap.fromImage(_qimage_from_rgba(rgba))


def navigation_mask_pixmap(
    mask: np.ndarray,
    rgba_value: tuple[int, int, int, int],
) -> QPixmap:
    values = np.asarray(mask, dtype=bool)
    if values.ndim != 2:
        raise ValueError("navigation mask must be 2D")
    rgba = np.zeros((values.shape[0], values.shape[1], 4), dtype=np.uint8)
    rgba[values] = np.asarray(rgba_value, dtype=np.uint8)
    return QPixmap.fromImage(_qimage_from_rgba(np.flipud(rgba).copy()))


class NavigationPreviewItem(QGraphicsPixmapItem):
    """Map-aligned evidence overlay."""

    def __init__(self) -> None:
        super().__init__()
        self.setZValue(5.0)
        self.setVisible(False)

    def _align_to_result(self, result: NavigationMapResult) -> None:
        maximum_y = result.origin_y_m + result.height * result.resolution_m
        self.setPos(float(result.origin_x_m), float(-maximum_y))
        self.setScale(float(result.resolution_m))
        self.setVisible(True)

    def set_result(
        self,
        result: NavigationMapResult,
        layer: str,
        structure: NavigationStructureResult | None = None,
        corridor: CorridorRefinementResult | None = None,
    ) -> None:
        self.setPixmap(navigation_layer_pixmap(result, layer, structure, corridor))
        self._align_to_result(result)

    def set_mask(
        self,
        result: NavigationMapResult,
        mask: np.ndarray,
        rgba_value: tuple[int, int, int, int],
    ) -> None:
        values = np.asarray(mask, dtype=bool)
        if values.shape != result.occupancy.shape:
            raise ValueError("navigation mask shape does not match reference result")
        self.setPixmap(navigation_mask_pixmap(values, rgba_value))
        self._align_to_result(result)

    def clear_result(self) -> None:
        self.setPixmap(QPixmap())
        self.setVisible(False)
