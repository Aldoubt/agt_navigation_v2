"""Shared-scene Route Debug renderer for AGT Map Workbench."""

from __future__ import annotations

from typing import Any, Mapping

import numpy as np
from PyQt5.QtCore import QObject, QPointF, QRectF, Qt, pyqtSignal
from PyQt5.QtGui import QColor, QBrush, QImage, QPainterPath, QPen, QPixmap, QPolygonF
from PyQt5.QtWidgets import (
    QGraphicsEllipseItem,
    QGraphicsItem,
    QGraphicsPathItem,
    QGraphicsPixmapItem,
    QGraphicsPolygonItem,
    QGraphicsRectItem,
    QGraphicsScene,
    QGraphicsSimpleTextItem,
)

from agt_offline_assets.navigation_map_derivation import FREE, OCCUPIED, UNKNOWN
from agt_offline_assets.route_debug_12f import RouteDebug12FBundle
from agt_offline_assets.route_debug_dataset import RouteDebugDataset

ROUTE_DEBUG_PRESETS = {
    "coverage": frozenset({
        "base.navigation", "base.no_go", "structure.aisles",
        "structure.vehicle_safe_lane", "coverage.order", "coverage.requests",
    }),
    "planning": frozenset({
        "structure.aisles", "coverage.requests", "motion.forward_candidates",
        "motion.forward_selected", "motion.reverse", "diagnostics.failed",
    }),
    "collision": frozenset({
        "base.navigation", "base.no_go", "structure.aisles",
        "diagnostics.raw", "diagnostics.geometry", "diagnostics.padding",
        "diagnostics.unknown", "diagnostics.conflicts", "diagnostics.failed",
    }),
    "12f": frozenset({
        "base.navigation", "base.navigation_12f", "base.no_go",
        "semantics.site_boundary", "structure.aisles",
        "traversability.inferred", "traversability.hard_blocked",
        "traversability.sensor_obstacle", "traversability.aisle_geometric_envelope",
    }),
}

ROUTE_DEBUG_LAYER_KEYS = (
    "base.navigation", "base.navigation_12f", "base.no_go",
    "semantics.site_boundary",
    "structure.turn_zones", "structure.aisles", "structure.vehicle_safe_lane",
    "coverage.order", "coverage.requests",
    "motion.forward_candidates", "motion.forward_selected", "motion.reverse",
    "traversability.observed", "traversability.inferred",
    "traversability.hard_blocked", "traversability.sensor_obstacle",
    "traversability.unknown", "traversability.aisle_geometric_envelope",
    "diagnostics.raw", "diagnostics.geometry", "diagnostics.padding",
    "diagnostics.unknown", "diagnostics.conflicts", "diagnostics.failed",
)


def _scene_xy(x: float, y: float) -> QPointF:
    return QPointF(float(x), float(-y))


def _qcolor(value: str, alpha: int = 255) -> QColor:
    color = QColor(value)
    color.setAlpha(alpha)
    return color


_LAYER_STYLE = {
    "base.no_go": ("#ef4444", 2.0, 70),
    "semantics.site_boundary": ("#14b8a6", 3.0, 35),
    "structure.turn_zones": ("#f59e0b", 1.6, 35),
    "structure.aisles": ("#64748b", 2.0, 0),
    "structure.vehicle_safe_lane": ("#22c55e", 3.0, 0),
    "coverage.order": ("#2563eb", 3.2, 0),
    "coverage.requests": ("#eab308", 2.0, 0),
    "motion.forward_candidates": ("#06b6d4", 1.5, 0),
    "motion.forward_selected": ("#10b981", 3.3, 0),
    "motion.reverse": ("#d946ef", 3.6, 0),
    "diagnostics.conflicts": ("#ef4444", 2.0, 90),
    "diagnostics.failed": ("#dc2626", 2.2, 100),
}


class RouteDebugSceneController(QObject):
    featureSelected = pyqtSignal(object)

    def __init__(self, scene: QGraphicsScene, parent: QObject | None = None):
        super().__init__(parent)
        self._scene = scene
        self._dataset: RouteDebugDataset | None = None
        self._bundle_12f: RouteDebug12FBundle | None = None
        self._overlay: Mapping[str, Any] | None = None
        self._active = False
        self._failure_focus = False
        self._groups: dict[str, QGraphicsRectItem] = {}
        self._layer_visible = {key: True for key in ROUTE_DEBUG_LAYER_KEYS}
        self._feature_items: dict[str, QGraphicsItem] = {}
        self._owned_items: list[QGraphicsItem] = []
        self._selection_outline: QGraphicsPolygonItem | None = None
        self._scene.selectionChanged.connect(self._on_scene_selection_changed)

    def _group(self, layer_key: str) -> QGraphicsRectItem:
        group = self._groups.get(layer_key)
        if group is None:
            group = QGraphicsRectItem(QRectF())
            group.setPen(QPen(Qt.NoPen))
            group.setBrush(QBrush(Qt.NoBrush))
            group.setAcceptedMouseButtons(Qt.NoButton)
            group.setZValue(self._z_for_layer(layer_key))
            self._scene.addItem(group)
            self._groups[layer_key] = group
            self._owned_items.append(group)
        return group

    @staticmethod
    def _z_for_layer(layer_key: str) -> float:
        order = {
            "base.navigation": -80.0,
            "base.navigation_12f": -79.0,
            "diagnostics.raw": -70.0,
            "diagnostics.geometry": -69.0,
            "diagnostics.padding": -68.0,
            "diagnostics.unknown": -67.0,
            "traversability.observed": -66.0,
            "traversability.inferred": -65.0,
            "traversability.aisle_geometric_envelope": -64.0,
            "traversability.unknown": -63.0,
            "traversability.sensor_obstacle": -62.0,
            "traversability.hard_blocked": -61.0,
            "base.no_go": -20.0,
            "semantics.site_boundary": -15.0,
            "structure.turn_zones": -10.0,
            "structure.aisles": 0.0,
            "structure.vehicle_safe_lane": 5.0,
            "coverage.order": 10.0,
            "coverage.requests": 12.0,
            "motion.forward_candidates": 20.0,
            "motion.forward_selected": 25.0,
            "motion.reverse": 27.0,
            "diagnostics.conflicts": 40.0,
            "diagnostics.failed": 45.0,
        }
        return order.get(layer_key, 0.0)

    def set_content(self, dataset: RouteDebugDataset, overlay: Mapping[str, Any]) -> None:
        self.clear()
        self._dataset = dataset
        self._overlay = overlay
        self._render_navigation(dataset)
        self._render_source_masks(dataset)
        for feature in overlay.get("features", []):
            if isinstance(feature, Mapping):
                self._render_feature(feature)
        self._refresh_visibility()

    def set_12f_content(self, bundle: RouteDebug12FBundle) -> None:
        self._bundle_12f = bundle
        self._render_12f(bundle)
        self._refresh_visibility()

    def clear(self) -> None:
        self._clear_selection_outline()
        for item in list(self._owned_items):
            if item.scene() is self._scene:
                self._scene.removeItem(item)
        self._groups.clear()
        self._feature_items.clear()
        self._owned_items.clear()
        self._dataset = None
        self._bundle_12f = None
        self._overlay = None

    def set_active(self, active: bool) -> None:
        self._active = bool(active)
        self._refresh_visibility()

    def is_active(self) -> bool:
        return self._active

    def set_layer_visible(self, layer_key: str, visible: bool) -> None:
        if layer_key not in self._layer_visible:
            raise KeyError(f"unknown Route Debug layer: {layer_key}")
        self._layer_visible[layer_key] = bool(visible)
        self._refresh_visibility()

    def layer_visible(self, layer_key: str) -> bool:
        return bool(self._layer_visible.get(layer_key, False))

    def layer_available(self, layer_key: str) -> bool:
        group = self._groups.get(layer_key)
        return bool(group is not None and group.childItems())

    def apply_preset(self, preset_name: str) -> None:
        if preset_name not in ROUTE_DEBUG_PRESETS:
            raise KeyError(f"unknown Route Debug preset: {preset_name}")
        selected = ROUTE_DEBUG_PRESETS[preset_name]
        for key in self._layer_visible:
            self._layer_visible[key] = key in selected
        self._refresh_visibility()

    def set_failure_focus(self, enabled: bool) -> None:
        self._failure_focus = bool(enabled)
        for item in self._feature_items.values():
            payload = item.data(1)
            is_failure = (
                bool(payload.get("is_failure", False))
                if isinstance(payload, Mapping)
                else False
            )
            item.setOpacity(
                1.0 if (not self._failure_focus or is_failure) else 0.12
            )

    def route_bounds(self) -> QRectF:
        rect = QRectF()
        initialized = False
        for group in self._groups.values():
            for child in group.childItems():
                candidate = child.sceneBoundingRect()
                if candidate.isNull():
                    continue
                rect = candidate if not initialized else rect.united(candidate)
                initialized = True
        return rect

    def select_feature(self, feature_id: str) -> bool:
        item = self._feature_items.get(feature_id)
        if item is None:
            return False
        self._scene.clearSelection()
        item.setSelected(True)
        return True

    def _refresh_visibility(self) -> None:
        for key, group in self._groups.items():
            group.setVisible(
                self._active and self._layer_visible.get(key, True)
            )

    def _on_scene_selection_changed(self) -> None:
        selected = [
            item
            for item in self._scene.selectedItems()
            if isinstance(item.data(0), str)
            and item.data(0) in self._feature_items
        ]
        if selected:
            self._emit_selected_item(selected[0])
        else:
            self._clear_selection_outline()

    def _emit_selected_item(self, item: QGraphicsItem) -> None:
        payload = item.data(1)
        if not isinstance(payload, Mapping):
            return
        self._draw_selection_footprint(payload)
        self.featureSelected.emit(dict(payload))

    def _draw_selection_footprint(self, payload: Mapping[str, Any]) -> None:
        self._clear_selection_outline()
        polygon = payload.get("footprint_polygon_xy")
        if not isinstance(polygon, list) or len(polygon) < 3:
            return
        item = QGraphicsPolygonItem(
            QPolygonF([_scene_xy(p[0], p[1]) for p in polygon])
        )
        item.setPen(QPen(_qcolor("#f8fafc"), 0.035, Qt.DashLine))
        item.setBrush(QBrush(Qt.NoBrush))
        item.setZValue(60.0)
        self._scene.addItem(item)
        self._selection_outline = item

    def _clear_selection_outline(self) -> None:
        if self._selection_outline is not None:
            if self._selection_outline.scene() is self._scene:
                self._scene.removeItem(self._selection_outline)
            self._selection_outline = None

    @staticmethod
    def _rgba_pixmap(rgba: np.ndarray) -> QPixmap:
        array = np.ascontiguousarray(rgba, dtype=np.uint8)
        h, w, _ = array.shape
        image = QImage(
            array.data,
            w,
            h,
            4 * w,
            QImage.Format_RGBA8888,
        ).copy()
        return QPixmap.fromImage(image)

    def _add_grid_raster(self, layer_key: str, rgba: np.ndarray, navigation) -> None:
        if navigation is None:
            return
        values = np.asarray(rgba, dtype=np.uint8)
        if values.shape != (navigation.height, navigation.width, 4):
            raise ValueError(
                f"Route Debug raster {layer_key} shape {values.shape} does not match "
                f"grid {(navigation.height, navigation.width)}"
            )
        item = QGraphicsPixmapItem(self._rgba_pixmap(np.flipud(values)))
        maximum_y = (
            navigation.origin_y_m
            + navigation.height * navigation.resolution_m
        )
        item.setPos(float(navigation.origin_x_m), float(-maximum_y))
        item.setScale(float(navigation.resolution_m))
        item.setTransformationMode(Qt.FastTransformation)
        item.setParentItem(self._group(layer_key))

    def _add_raster(
        self,
        layer_key: str,
        rgba: np.ndarray,
        dataset: RouteDebugDataset,
    ) -> None:
        self._add_grid_raster(layer_key, rgba, dataset.navigation)

    @staticmethod
    def _occupancy_rgba(navigation, *, alpha_scale: float = 1.0) -> np.ndarray:
        occupancy = np.asarray(navigation.occupancy, dtype=np.uint8)
        rgba = np.zeros((navigation.height, navigation.width, 4), dtype=np.uint8)
        rgba[occupancy == FREE] = np.array(
            [241, 245, 249, int(190 * alpha_scale)],
            dtype=np.uint8,
        )
        rgba[occupancy == UNKNOWN] = np.array(
            [100, 116, 139, int(170 * alpha_scale)],
            dtype=np.uint8,
        )
        rgba[occupancy == OCCUPIED] = np.array(
            [15, 23, 42, int(230 * alpha_scale)],
            dtype=np.uint8,
        )
        return rgba

    def _render_navigation(self, dataset: RouteDebugDataset) -> None:
        navigation = dataset.navigation
        if navigation is None:
            return
        self._add_grid_raster(
            "base.navigation",
            self._occupancy_rgba(navigation),
            navigation,
        )

    def _render_source_masks(self, dataset: RouteDebugDataset) -> None:
        navigation = dataset.navigation
        masks = dataset.occupancy_source_masks
        if navigation is None or masks is None:
            return
        specs = (
            ("diagnostics.raw", masks.raw_obstacle_direct, (239, 68, 68, 135)),
            ("diagnostics.geometry", masks.geometry_direct, (249, 115, 22, 135)),
            ("diagnostics.padding", masks.padding_only, (168, 85, 247, 135)),
            (
                "diagnostics.unknown",
                navigation.occupancy == UNKNOWN,
                (59, 130, 246, 105),
            ),
        )
        for layer_key, mask, color in specs:
            rgba = np.zeros(
                (navigation.height, navigation.width, 4),
                dtype=np.uint8,
            )
            rgba[np.asarray(mask, dtype=bool)] = np.array(color, dtype=np.uint8)
            self._add_grid_raster(layer_key, rgba, navigation)

    def _render_12f(self, bundle: RouteDebug12FBundle) -> None:
        navigation = bundle.candidate_navigation
        if navigation is not None:
            self._add_grid_raster(
                "base.navigation_12f",
                self._occupancy_rgba(navigation, alpha_scale=0.88),
                navigation,
            )

        evidence = bundle.traversability
        if navigation is None or evidence is None:
            return
        specs = (
            ("traversability.observed", evidence.observed_free_mask, (34, 197, 94, 95)),
            ("traversability.inferred", evidence.inferred_traversable_mask, (6, 182, 212, 205)),
            ("traversability.hard_blocked", evidence.hard_blocked_mask, (239, 68, 68, 190)),
            ("traversability.sensor_obstacle", evidence.sensor_obstacle_mask, (249, 115, 22, 170)),
            ("traversability.unknown", evidence.unknown_mask, (100, 116, 139, 115)),
            (
                "traversability.aisle_geometric_envelope",
                evidence.aisle_geometric_envelope_mask,
                (234, 179, 8, 85),
            ),
        )
        for layer_key, mask, color in specs:
            rgba = np.zeros(
                (navigation.height, navigation.width, 4),
                dtype=np.uint8,
            )
            rgba[np.asarray(mask, dtype=bool)] = np.array(color, dtype=np.uint8)
            self._add_grid_raster(layer_key, rgba, navigation)

    def _render_feature(self, feature: Mapping[str, Any]) -> None:
        props = feature.get("properties")
        geometry = feature.get("geometry")
        if not isinstance(props, Mapping) or not isinstance(geometry, Mapping):
            return
        layer_key = str(props.get("layer_key", ""))
        feature_id = str(props.get("feature_id", ""))
        if layer_key not in ROUTE_DEBUG_LAYER_KEYS or not feature_id:
            return
        geometry_type = str(geometry.get("type", ""))
        item = None
        if geometry_type == "LineString":
            item = self._line_item(
                geometry.get("coordinates") or [],
                layer_key,
            )
        elif geometry_type == "Polygon":
            rings = geometry.get("coordinates") or []
            if rings:
                item = self._polygon_item(rings[0], layer_key)
        elif geometry_type == "Point":
            coords = geometry.get("coordinates") or []
            if len(coords) >= 2:
                item = self._point_item(
                    float(coords[0]),
                    float(coords[1]),
                    layer_key,
                    str(props.get("feature_kind", "")),
                )
        if item is None:
            return
        payload = dict(props.get("inspector") or {})
        payload.update(
            {
                "feature_id": feature_id,
                "feature_kind": props.get("feature_kind"),
                "status": props.get("status"),
                "source_asset": props.get("source_asset"),
                "source_id": props.get("source_id"),
                "source_field": props.get("source_field"),
                "is_failure": bool(props.get("is_failure", False)),
            }
        )
        if props.get("footprint_polygon_xy") is not None:
            payload["footprint_polygon_xy"] = props.get("footprint_polygon_xy")
        item.setData(0, feature_id)
        item.setData(1, payload)
        item.setFlag(QGraphicsItem.ItemIsSelectable, True)
        item.setParentItem(self._group(layer_key))
        self._feature_items[feature_id] = item
        if (
            props.get("feature_kind") == "COVERAGE_TRAVERSAL"
            and props.get("sequence") is not None
        ):
            self._add_sequence_label(
                item,
                str(props["sequence"]),
                layer_key,
            )

    @staticmethod
    def _line_item(coordinates, layer_key: str):
        if not isinstance(coordinates, list) or len(coordinates) < 2:
            return None
        path = QPainterPath()
        path.moveTo(_scene_xy(coordinates[0][0], coordinates[0][1]))
        for xy in coordinates[1:]:
            path.lineTo(_scene_xy(xy[0], xy[1]))
        color, width, _ = _LAYER_STYLE.get(
            layer_key,
            ("#94a3b8", 1.5, 0),
        )
        pen = QPen(_qcolor(color), float(width) * 0.02)
        if layer_key in {"coverage.requests", "motion.forward_candidates"}:
            pen.setStyle(Qt.DashLine)
        item = QGraphicsPathItem(path)
        item.setPen(pen)
        item.setBrush(QBrush(Qt.NoBrush))
        return item

    @staticmethod
    def _polygon_item(coordinates, layer_key: str):
        if not isinstance(coordinates, list) or len(coordinates) < 3:
            return None
        color, width, alpha = _LAYER_STYLE.get(
            layer_key,
            ("#94a3b8", 1.5, 35),
        )
        item = QGraphicsPolygonItem(
            QPolygonF([_scene_xy(xy[0], xy[1]) for xy in coordinates])
        )
        item.setPen(QPen(_qcolor(color, 230), float(width) * 0.02))
        item.setBrush(QBrush(_qcolor(color, alpha)))
        return item

    @staticmethod
    def _point_item(
        x: float,
        y: float,
        layer_key: str,
        feature_kind: str,
    ):
        color, _, alpha = _LAYER_STYLE.get(
            layer_key,
            ("#ef4444", 2.0, 100),
        )
        radius = 0.09 if feature_kind == "CUSP" else 0.12
        center = _scene_xy(x, y)
        item = QGraphicsEllipseItem(
            center.x() - radius,
            center.y() - radius,
            2.0 * radius,
            2.0 * radius,
        )
        item.setPen(QPen(_qcolor(color), 0.035))
        item.setBrush(QBrush(_qcolor(color, max(alpha, 160))))
        return item

    def _add_sequence_label(
        self,
        source_item: QGraphicsItem,
        text: str,
        layer_key: str,
    ) -> None:
        label = QGraphicsSimpleTextItem(text)
        label.setBrush(QBrush(_qcolor("#e2e8f0")))
        label.setFlag(QGraphicsItem.ItemIgnoresTransformations, True)
        label.setPos(source_item.mapToScene(source_item.boundingRect().center()))
        label.setParentItem(self._group(layer_key))
