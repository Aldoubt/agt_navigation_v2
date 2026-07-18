#!/usr/bin/env python3
"""Read-only Qt5 viewer for PCD, Nav2 raster, and semantic GeoJSON alignment."""

from __future__ import annotations

import argparse
from datetime import datetime
import json
import math
from pathlib import Path
import sys
from typing import Iterable, List, Sequence, Tuple

import numpy as np
import yaml
from PyQt5 import QtCore, QtGui, QtWidgets

from pcd_projection import PcdSample, load_pcd_xyz
from rigid_alignment import RigidAlignmentResult, solve_rigid_alignment


REPO_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_MAP = REPO_ROOT / "runtime/maps/greenhouse_ground/greenhouse_ground.yaml"
DEFAULT_PCD = REPO_ROOT / "runtime/maps/greenhouse_ground/pcd/greenhouse_aligned_full2.pcd"
DEFAULT_SEMANTIC = REPO_ROOT / "runtime/maps/greenhouse_ground/semantic/semantic_map.geojson"


class MapProjection:
    def __init__(self, width: int, height: int, resolution: float, origin: Sequence[float]):
        self.width = width
        self.height = height
        self.resolution = float(resolution)
        self.origin_x = float(origin[0])
        self.origin_y = float(origin[1])
        self.yaw = float(origin[2]) if len(origin) > 2 else 0.0
        self.cos_yaw = math.cos(self.yaw)
        self.sin_yaw = math.sin(self.yaw)

    @property
    def corners(self) -> List[Tuple[float, float]]:
        return [self.image_to_world(x, y) for x, y in (
            (0, self.height), (self.width, self.height), (self.width, 0), (0, 0)
        )]

    def world_to_image_arrays(self, x: np.ndarray, y: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        dx = x - self.origin_x
        dy = y - self.origin_y
        local_x = self.cos_yaw * dx + self.sin_yaw * dy
        local_y = -self.sin_yaw * dx + self.cos_yaw * dy
        return local_x / self.resolution, self.height - 1.0 - local_y / self.resolution

    def world_to_image(self, x: float, y: float) -> QtCore.QPointF:
        px, py = self.world_to_image_arrays(np.array([x]), np.array([y]))
        return QtCore.QPointF(float(px[0]), float(py[0]))

    def image_to_world(self, px: float, py: float) -> Tuple[float, float]:
        local_x = px * self.resolution
        local_y = (self.height - py) * self.resolution
        x = self.origin_x + self.cos_yaw * local_x - self.sin_yaw * local_y
        y = self.origin_y + self.sin_yaw * local_x + self.cos_yaw * local_y
        return x, y

    def contains_arrays(self, px: np.ndarray, py: np.ndarray) -> np.ndarray:
        return (px >= 0) & (px < self.width) & (py >= 0) & (py < self.height)


class MapView(QtWidgets.QGraphicsView):
    mouse_world_changed = QtCore.pyqtSignal(float, float)
    calibration_clicked = QtCore.pyqtSignal(float, float)

    def __init__(self, parent=None):
        super().__init__(parent)
        self.projection = None
        self.calibration_mode = False
        self.setRenderHints(QtGui.QPainter.Antialiasing | QtGui.QPainter.SmoothPixmapTransform)
        self.setDragMode(QtWidgets.QGraphicsView.ScrollHandDrag)
        self.setTransformationAnchor(QtWidgets.QGraphicsView.AnchorUnderMouse)
        self.setBackgroundBrush(QtGui.QColor("#17201d"))

    def wheelEvent(self, event):
        factor = 1.18 if event.angleDelta().y() > 0 else 1.0 / 1.18
        self.scale(factor, factor)

    def mouseMoveEvent(self, event):
        super().mouseMoveEvent(event)
        if self.projection is not None:
            point = self.mapToScene(event.pos())
            x, y = self.projection.image_to_world(point.x(), point.y())
            self.mouse_world_changed.emit(x, y)

    def mousePressEvent(self, event):
        if (
            self.calibration_mode
            and event.button() == QtCore.Qt.LeftButton
            and self.projection is not None
        ):
            point = self.mapToScene(event.pos())
            x, y = self.projection.image_to_world(point.x(), point.y())
            self.calibration_clicked.emit(x, y)
            event.accept()
            return
        super().mousePressEvent(event)


class PcdLoader(QtCore.QObject):
    loaded = QtCore.pyqtSignal(object)
    failed = QtCore.pyqtSignal(str)

    def __init__(self, path: Path, max_points: int):
        super().__init__()
        self.path = path
        self.max_points = max_points

    @QtCore.pyqtSlot()
    def run(self):
        try:
            self.loaded.emit(load_pcd_xyz(self.path, self.max_points))
        except Exception as exc:  # Display file and codec errors in the UI.
            self.failed.emit(f"{type(exc).__name__}: {exc}")


class AlignmentWindow(QtWidgets.QMainWindow):
    FEATURE_COLORS = {
        "field": "#19a974",
        "row_centerline": "#ffc857",
        "access_lane": "#00c2ff",
        "exclusion_zone": "#ff4d5a",
        "keepout_zone": "#ff4d5a",
        "entry_pose": "#1967ff",
        "work_direction": "#00a896",
    }

    def __init__(self, map_yaml: Path, pcd_path: Path, semantic_path: Path, max_points: int):
        super().__init__()
        self.map_yaml_path = map_yaml
        self.pcd_path = pcd_path
        self.semantic_path = semantic_path
        self.max_points = max_points
        self.pcd_sample = None
        self.semantic = None
        self.alignment_rotation = np.eye(2, dtype=np.float64)
        self.alignment_translation = np.zeros(2, dtype=np.float64)
        self.alignment_result = None
        self.calibration_source_points = []
        self.calibration_target_points = []
        self.calibration_active = False
        self.scene = QtWidgets.QGraphicsScene(self)
        self.view = MapView()
        self.view.setScene(self.scene)
        self.view.mouse_world_changed.connect(self._show_coordinates)
        self.view.calibration_clicked.connect(self._collect_calibration_point)
        self.setWindowTitle("AGT 温室地图对齐检查器（只读）")
        self.resize(1500, 920)
        self._build_ui()
        self._load_static_layers()
        QtCore.QTimer.singleShot(0, self._start_pcd_load)

    def _build_ui(self):
        central = QtWidgets.QWidget()
        layout = QtWidgets.QHBoxLayout(central)
        layout.setContentsMargins(8, 8, 8, 8)
        layout.addWidget(self.view, 1)

        panel = QtWidgets.QWidget()
        panel.setFixedWidth(390)
        panel_layout = QtWidgets.QVBoxLayout(panel)
        title = QtWidgets.QLabel("地图对齐诊断")
        title.setStyleSheet("font-size: 20px; font-weight: 700; color: #e9c46a;")
        panel_layout.addWidget(title)

        source_box = QtWidgets.QGroupBox("PCD 对齐地图")
        source_layout = QtWidgets.QVBoxLayout(source_box)
        self.pcd_path_edit = QtWidgets.QLineEdit(str(self.pcd_path))
        self.pcd_path_edit.setReadOnly(True)
        self.pcd_path_edit.setToolTip(str(self.pcd_path))
        self.pcd_path_edit.setCursorPosition(0)
        source_layout.addWidget(self.pcd_path_edit)
        source_buttons = QtWidgets.QHBoxLayout()
        self.choose_pcd_button = QtWidgets.QPushButton("浏览 PCD...")
        self.choose_pcd_button.clicked.connect(self._choose_pcd)
        self.reload_pcd_button = QtWidgets.QPushButton("重新加载")
        self.reload_pcd_button.clicked.connect(self._start_pcd_load)
        source_buttons.addWidget(self.choose_pcd_button)
        source_buttons.addWidget(self.reload_pcd_button)
        source_layout.addLayout(source_buttons)
        panel_layout.addWidget(source_box)

        layer_box = QtWidgets.QGroupBox("图层")
        form = QtWidgets.QFormLayout(layer_box)
        self.raster_visible = QtWidgets.QCheckBox("Nav2 PNG 底图")
        self.raster_visible.setChecked(True)
        self.pcd_visible = QtWidgets.QCheckBox("PCD XY 投影")
        self.pcd_visible.setChecked(True)
        self.semantic_visible = QtWidgets.QCheckBox("GeoJSON 语义")
        self.semantic_visible.setChecked(True)
        self.raster_opacity = self._slider(10, 100, 100)
        self.pcd_opacity = self._slider(5, 100, 72)
        self.semantic_opacity = self._slider(10, 100, 95)
        form.addRow(self.raster_visible)
        form.addRow("底图透明度", self.raster_opacity)
        form.addRow(self.pcd_visible)
        form.addRow("点云透明度", self.pcd_opacity)
        form.addRow(self.semantic_visible)
        form.addRow("语义透明度", self.semantic_opacity)
        panel_layout.addWidget(layer_box)

        pcd_box = QtWidgets.QGroupBox("点云显示")
        pcd_form = QtWidgets.QFormLayout(pcd_box)
        self.z_min = QtWidgets.QDoubleSpinBox()
        self.z_max = QtWidgets.QDoubleSpinBox()
        for spin in (self.z_min, self.z_max):
            spin.setRange(-1000.0, 1000.0)
            spin.setDecimals(3)
            spin.setSingleStep(0.1)
            spin.setEnabled(False)
        self.point_size = QtWidgets.QSpinBox()
        self.point_size.setRange(1, 5)
        self.point_size.setValue(2)
        self.show_outside_pcd = QtWidgets.QCheckBox("显示底图外点云")
        self.show_outside_pcd.setChecked(True)
        self.show_outside_pcd.toggled.connect(self._render_pcd)
        self.redraw_button = QtWidgets.QPushButton("按高度范围重绘")
        self.redraw_button.setEnabled(False)
        self.redraw_button.clicked.connect(self._render_pcd)
        pcd_form.addRow("最低 Z (m)", self.z_min)
        pcd_form.addRow("最高 Z (m)", self.z_max)
        pcd_form.addRow("点大小 (px)", self.point_size)
        pcd_form.addRow(self.show_outside_pcd)
        pcd_form.addRow(self.redraw_button)
        panel_layout.addWidget(pcd_box)

        calibration_box = QtWidgets.QGroupBox("四点刚体对齐")
        calibration_layout = QtWidgets.QVBoxLayout(calibration_box)
        self.calibration_status = QtWidgets.QLabel(
            "选择四组同名特征点，只求旋转和平移。"
        )
        self.calibration_status.setWordWrap(True)
        calibration_layout.addWidget(self.calibration_status)
        calibration_buttons = QtWidgets.QGridLayout()
        self.start_calibration_button = QtWidgets.QPushButton("开始四点选择")
        self.start_calibration_button.clicked.connect(self._start_calibration)
        self.undo_calibration_button = QtWidgets.QPushButton("撤销上一点")
        self.undo_calibration_button.clicked.connect(self._undo_calibration_point)
        self.undo_calibration_button.setEnabled(False)
        self.reset_calibration_button = QtWidgets.QPushButton("恢复原始对齐")
        self.reset_calibration_button.clicked.connect(self._reset_calibration)
        self.export_calibration_button = QtWidgets.QPushButton("导出矩阵...")
        self.export_calibration_button.clicked.connect(self._export_calibration)
        self.export_calibration_button.setEnabled(False)
        calibration_buttons.addWidget(self.start_calibration_button, 0, 0)
        calibration_buttons.addWidget(self.undo_calibration_button, 0, 1)
        calibration_buttons.addWidget(self.reset_calibration_button, 1, 0)
        calibration_buttons.addWidget(self.export_calibration_button, 1, 1)
        calibration_layout.addLayout(calibration_buttons)
        panel_layout.addWidget(calibration_box)

        button_row = QtWidgets.QHBoxLayout()
        fit_button = QtWidgets.QPushButton("适配窗口")
        fit_button.clicked.connect(self._fit)
        actual_button = QtWidgets.QPushButton("1:1 像素")
        actual_button.clicked.connect(self._actual_pixels)
        button_row.addWidget(fit_button)
        button_row.addWidget(actual_button)
        panel_layout.addLayout(button_row)

        self.report = QtWidgets.QTextBrowser()
        self.report.setOpenExternalLinks(False)
        self.report.setStyleSheet("font-family: monospace; font-size: 12px;")
        panel_layout.addWidget(self.report, 1)
        layout.addWidget(panel)
        self.setCentralWidget(central)

        self.status = QtWidgets.QLabel("正在读取地图...")
        self.statusBar().addPermanentWidget(self.status)
        self.raster_visible.toggled.connect(lambda value: self.raster_item.setVisible(value))
        self.pcd_visible.toggled.connect(self._toggle_pcd)
        self.semantic_visible.toggled.connect(lambda value: self.semantic_group.setVisible(value))
        self.raster_opacity.valueChanged.connect(
            lambda value: self.raster_item.setOpacity(value / 100.0)
        )
        self.pcd_opacity.valueChanged.connect(self._set_pcd_opacity)
        self.semantic_opacity.valueChanged.connect(
            lambda value: self.semantic_group.setOpacity(value / 100.0)
        )

    @staticmethod
    def _slider(minimum: int, maximum: int, value: int) -> QtWidgets.QSlider:
        slider = QtWidgets.QSlider(QtCore.Qt.Horizontal)
        slider.setRange(minimum, maximum)
        slider.setValue(value)
        return slider

    def _load_static_layers(self):
        with self.map_yaml_path.open("r", encoding="utf-8") as stream:
            map_config = yaml.safe_load(stream)
        image_path = Path(map_config["image"])
        if not image_path.is_absolute():
            image_path = self.map_yaml_path.parent / image_path
        pixmap = QtGui.QPixmap(str(image_path))
        if pixmap.isNull():
            raise ValueError(f"无法读取底图: {image_path}")
        self.image_path = image_path.resolve()
        self.projection = MapProjection(
            pixmap.width(), pixmap.height(), map_config["resolution"], map_config["origin"]
        )
        self.view.projection = self.projection
        self.raster_item = self.scene.addPixmap(pixmap)
        self.raster_item.setZValue(0)
        self.pcd_item = None
        self.semantic_group = QtWidgets.QGraphicsItemGroup()
        self.semantic_group.setZValue(20)
        self.scene.addItem(self.semantic_group)
        self.calibration_group = QtWidgets.QGraphicsItemGroup()
        self.calibration_group.setZValue(30)
        self.scene.addItem(self.calibration_group)
        self.map_scene_rect = QtCore.QRectF(0, 0, pixmap.width(), pixmap.height())
        self.pcd_scene_rect = QtCore.QRectF(self.map_scene_rect)
        self.scene.setSceneRect(self.map_scene_rect)
        self._load_semantic()
        self._fit()
        self._update_report("PCD 正在解压和抽样，请稍候...")

    def _load_semantic(self):
        with self.semantic_path.open("r", encoding="utf-8") as stream:
            self.semantic = json.load(stream)
        for feature in self.semantic.get("features", []):
            geometry = feature.get("geometry") or {}
            properties = feature.get("properties") or {}
            feature_type = properties.get("feature_type", "unknown")
            color = QtGui.QColor(self.FEATURE_COLORS.get(feature_type, "#ffffff"))
            pen = QtGui.QPen(color, 2.2)
            pen.setCosmetic(True)
            brush_color = QtGui.QColor(color)
            brush_color.setAlpha(42)
            brush = QtGui.QBrush(brush_color)
            geometry_type = geometry.get("type")
            coordinates = geometry.get("coordinates", [])
            if geometry_type == "Point" and len(coordinates) >= 2:
                point = self.projection.world_to_image(float(coordinates[0]), float(coordinates[1]))
                item = QtWidgets.QGraphicsEllipseItem(point.x() - 5, point.y() - 5, 10, 10)
                item.setPen(pen)
                item.setBrush(QtGui.QBrush(color))
                self.semantic_group.addToGroup(item)
            elif geometry_type == "LineString":
                self._add_path(coordinates, pen, QtGui.QBrush(QtCore.Qt.NoBrush), False)
            elif geometry_type == "Polygon" and coordinates:
                for index, ring in enumerate(coordinates):
                    self._add_path(ring, pen, brush if index == 0 else QtGui.QBrush(), True)
            elif geometry_type == "MultiLineString":
                for line in coordinates:
                    self._add_path(line, pen, QtGui.QBrush(QtCore.Qt.NoBrush), False)
            elif geometry_type == "MultiPolygon":
                for polygon in coordinates:
                    for index, ring in enumerate(polygon):
                        self._add_path(ring, pen, brush if index == 0 else QtGui.QBrush(), True)

    def _add_path(self, coordinates, pen, brush, close: bool):
        if not coordinates:
            return
        path = QtGui.QPainterPath()
        first = self.projection.world_to_image(float(coordinates[0][0]), float(coordinates[0][1]))
        path.moveTo(first)
        for coordinate in coordinates[1:]:
            path.lineTo(self.projection.world_to_image(float(coordinate[0]), float(coordinate[1])))
        if close:
            path.closeSubpath()
        item = QtWidgets.QGraphicsPathItem(path)
        item.setPen(pen)
        item.setBrush(brush)
        self.semantic_group.addToGroup(item)

    def _choose_pcd(self):
        selected, _ = QtWidgets.QFileDialog.getOpenFileName(
            self,
            "选择用于对齐检查的 PCD 地图",
            str(self.pcd_path.parent),
            "Point Cloud Data (*.pcd);;所有文件 (*)",
        )
        if not selected:
            return
        selected_path = Path(selected).expanduser().resolve()
        if selected_path == self.pcd_path:
            return
        self._reset_calibration(render=False)
        self.pcd_path = selected_path
        self.pcd_path_edit.setText(str(selected_path))
        self.pcd_path_edit.setToolTip(str(selected_path))
        self.pcd_path_edit.setCursorPosition(0)
        self._start_pcd_load()

    def _set_pcd_loading(self, loading: bool):
        self.choose_pcd_button.setEnabled(not loading)
        self.reload_pcd_button.setEnabled(not loading)
        for widget in (self.z_min, self.z_max, self.point_size, self.redraw_button):
            widget.setEnabled(not loading and self.pcd_sample is not None)

    def _start_pcd_load(self):
        if hasattr(self, "loader_thread") and self.loader_thread.isRunning():
            return
        if not self.pcd_path.is_file():
            message = f"文件不存在: {self.pcd_path}"
            self._update_report(f"PCD 加载失败: {message}")
            QtWidgets.QMessageBox.critical(self, "PCD 加载失败", message)
            return
        self.pcd_sample = None
        if self.pcd_item is not None:
            self.pcd_item.setPixmap(QtGui.QPixmap())
        self._set_pcd_loading(True)
        self._update_report(f"正在加载 PCD: {self.pcd_path.name}")
        self.loader_thread = QtCore.QThread(self)
        self.loader = PcdLoader(self.pcd_path, self.max_points)
        self.loader.moveToThread(self.loader_thread)
        self.loader_thread.started.connect(self.loader.run)
        self.loader.loaded.connect(self._pcd_loaded)
        self.loader.failed.connect(self._pcd_failed)
        self.loader.loaded.connect(self.loader_thread.quit)
        self.loader.failed.connect(self.loader_thread.quit)
        self.loader_thread.finished.connect(self.loader.deleteLater)
        self.loader_thread.start()

    @QtCore.pyqtSlot(object)
    def _pcd_loaded(self, sample: PcdSample):
        self.pcd_sample = sample
        z_low, z_high = float(sample.z.min()), float(sample.z.max())
        self.z_min.setValue(z_low)
        self.z_max.setValue(z_high)
        self.z_min.setEnabled(True)
        self.z_max.setEnabled(True)
        self._set_pcd_loading(False)
        self._render_pcd()
        self._update_report(
            f"PCD 已加载: {self.pcd_path.name}；可调 Z 范围检查不同高度层。"
        )

    @QtCore.pyqtSlot(str)
    def _pcd_failed(self, message: str):
        self._set_pcd_loading(False)
        self._update_report(f"PCD 加载失败: {message}")
        QtWidgets.QMessageBox.critical(self, "PCD 加载失败", message)

    def _transform_xy(self, x: np.ndarray, y: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        coordinates = np.vstack((x, y))
        transformed = self.alignment_rotation @ coordinates
        transformed += self.alignment_translation[:, np.newaxis]
        return transformed[0], transformed[1]

    def _start_calibration(self):
        if self.pcd_sample is None:
            QtWidgets.QMessageBox.information(self, "四点对齐", "请先等待 PCD 加载完成。")
            return
        self._reset_calibration()
        self.calibration_active = True
        self.view.calibration_mode = True
        self.undo_calibration_button.setEnabled(False)
        self.export_calibration_button.setEnabled(False)
        self._set_calibration_pick_layer()

    def _set_calibration_pick_layer(self):
        source_next = len(self.calibration_source_points) == len(self.calibration_target_points)
        pair_number = min(len(self.calibration_source_points), len(self.calibration_target_points)) + 1
        if source_next:
            self.raster_item.setVisible(False)
            if self.pcd_item is not None:
                self.pcd_item.setVisible(True)
            self.scene.setSceneRect(self.pcd_scene_rect)
            text = f"第 {pair_number}/4 组：点击 PCD 上的清晰特征点"
        else:
            self.raster_item.setVisible(True)
            if self.pcd_item is not None:
                self.pcd_item.setVisible(False)
            self.scene.setSceneRect(self.map_scene_rect)
            text = f"第 {pair_number}/4 组：点击栅格中同一个物理特征点"
        self.semantic_group.setVisible(False)
        self.calibration_status.setText(text)
        self.status.setText(text)
        self._fit()

    @QtCore.pyqtSlot(float, float)
    def _collect_calibration_point(self, x: float, y: float):
        if not self.calibration_active:
            return
        point = [float(x), float(y)]
        if len(self.calibration_source_points) == len(self.calibration_target_points):
            self.calibration_source_points.append(point)
        else:
            self.calibration_target_points.append(point)
        self.undo_calibration_button.setEnabled(True)
        self._render_calibration_markers()
        if len(self.calibration_target_points) == 4:
            self._solve_calibration()
        else:
            self._set_calibration_pick_layer()

    def _undo_calibration_point(self):
        if not self.calibration_active:
            return
        if len(self.calibration_source_points) > len(self.calibration_target_points):
            self.calibration_source_points.pop()
        elif self.calibration_target_points:
            self.calibration_target_points.pop()
        self.undo_calibration_button.setEnabled(bool(self.calibration_source_points))
        self._render_calibration_markers()
        self._set_calibration_pick_layer()

    def _clear_calibration_markers(self):
        for item in list(self.calibration_group.childItems()):
            self.calibration_group.removeFromGroup(item)
            self.scene.removeItem(item)

    def _add_calibration_marker(self, point, color: str, label: str, cross: bool):
        pixel = self.projection.world_to_image(float(point[0]), float(point[1]))
        pen = QtGui.QPen(QtGui.QColor(color), 2.5)
        pen.setCosmetic(True)
        if cross:
            for x1, y1, x2, y2 in ((-6, -6, 6, 6), (-6, 6, 6, -6)):
                item = QtWidgets.QGraphicsLineItem(
                    pixel.x() + x1, pixel.y() + y1, pixel.x() + x2, pixel.y() + y2
                )
                item.setPen(pen)
                self.calibration_group.addToGroup(item)
        else:
            item = QtWidgets.QGraphicsEllipseItem(pixel.x() - 6, pixel.y() - 6, 12, 12)
            item.setPen(pen)
            self.calibration_group.addToGroup(item)
        text_item = QtWidgets.QGraphicsSimpleTextItem(label)
        text_item.setBrush(QtGui.QBrush(QtGui.QColor(color)))
        text_item.setPos(pixel.x() + 7, pixel.y() - 14)
        self.calibration_group.addToGroup(text_item)

    def _render_calibration_markers(self):
        self._clear_calibration_markers()
        displayed_sources = self.calibration_source_points
        if self.alignment_result is not None:
            displayed_sources = self.alignment_result.transformed_source.tolist()
        for index, source in enumerate(displayed_sources):
            self._add_calibration_marker(source, "#00d9ff", f"P{index + 1}", True)
        for index, target in enumerate(self.calibration_target_points):
            self._add_calibration_marker(target, "#ff3b30", f"M{index + 1}", False)
            if index < len(displayed_sources):
                source_pixel = self.projection.world_to_image(*displayed_sources[index])
                target_pixel = self.projection.world_to_image(*target)
                line = QtWidgets.QGraphicsLineItem(
                    source_pixel.x(), source_pixel.y(), target_pixel.x(), target_pixel.y()
                )
                pen = QtGui.QPen(QtGui.QColor("#ffffff"), 1.2, QtCore.Qt.DashLine)
                pen.setCosmetic(True)
                line.setPen(pen)
                self.calibration_group.addToGroup(line)

    def _solve_calibration(self):
        try:
            result = solve_rigid_alignment(
                self.calibration_source_points, self.calibration_target_points
            )
        except ValueError as exc:
            QtWidgets.QMessageBox.warning(self, "四点不可用", str(exc))
            self.calibration_status.setText(f"四点不可用：{exc}，请撤销后重选。")
            return
        self.alignment_result = result
        self.alignment_rotation = result.rotation.copy()
        self.alignment_translation = result.translation.copy()
        self.calibration_active = False
        self.view.calibration_mode = False
        self.undo_calibration_button.setEnabled(False)
        self.export_calibration_button.setEnabled(True)
        self._restore_layer_visibility()
        self._render_pcd()
        self._render_calibration_markers()
        self._fit()
        scale_error = abs(result.diagnostic_scale - 1.0) * 100.0
        quality = "可继续目视检查" if result.rms_error <= 0.10 and scale_error <= 0.5 else "需要重新选点或检查分辨率"
        summary = (
            f"旋转 {math.degrees(result.angle_rad):+.3f}°，"
            f"平移 ({result.translation[0]:+.3f}, {result.translation[1]:+.3f}) m，"
            f"RMS {result.rms_error:.3f} m，尺度偏差 {scale_error:.2f}%：{quality}"
        )
        self.calibration_status.setText(summary)
        self._update_report(summary)

    def _restore_layer_visibility(self):
        self.raster_item.setVisible(self.raster_visible.isChecked())
        if self.pcd_item is not None:
            self.pcd_item.setVisible(self.pcd_visible.isChecked())
        self.semantic_group.setVisible(self.semantic_visible.isChecked())
        self.scene.setSceneRect(
            self.pcd_scene_rect if self.show_outside_pcd.isChecked() else self.map_scene_rect
        )

    def _reset_calibration(self, render: bool = True):
        self.calibration_active = False
        self.view.calibration_mode = False
        self.calibration_source_points = []
        self.calibration_target_points = []
        self.alignment_result = None
        self.alignment_rotation = np.eye(2, dtype=np.float64)
        self.alignment_translation = np.zeros(2, dtype=np.float64)
        if hasattr(self, "calibration_group"):
            self._clear_calibration_markers()
        if hasattr(self, "undo_calibration_button"):
            self.undo_calibration_button.setEnabled(False)
            self.export_calibration_button.setEnabled(False)
            self.calibration_status.setText("选择四组同名特征点，只求旋转和平移。")
        if hasattr(self, "raster_item"):
            self._restore_layer_visibility()
        if render and self.pcd_sample is not None:
            self._render_pcd()

    @staticmethod
    def _record_path(path: Path) -> str:
        try:
            return str(path.resolve().relative_to(REPO_ROOT))
        except ValueError:
            return str(path.resolve())

    def _export_calibration(self):
        if self.alignment_result is None:
            return
        default_path = self.map_yaml_path.parent / (
            f"{self.pcd_path.stem}_to_{self.map_yaml_path.stem}_four_point.yaml"
        )
        selected, _ = QtWidgets.QFileDialog.getSaveFileName(
            self, "保存四点对齐记录", str(default_path), "YAML (*.yaml)"
        )
        if not selected:
            return
        yaml_path = Path(selected).expanduser()
        if yaml_path.suffix.lower() not in {".yaml", ".yml"}:
            yaml_path = yaml_path.with_suffix(".yaml")
        matrix_path = yaml_path.with_suffix(".txt")
        result = self.alignment_result
        payload = {
            "schema": "agt_four_point_pcd_raster_alignment/v1",
            "created_at": datetime.now().astimezone().isoformat(timespec="seconds"),
            "source_pcd": self._record_path(self.pcd_path),
            "target_map_yaml": self._record_path(self.map_yaml_path),
            "target_image": self._record_path(self.image_path),
            "method": "2d_rigid_svd_no_scale",
            "source_points_xy": self.calibration_source_points,
            "target_points_xy": self.calibration_target_points,
            "rotation_deg": math.degrees(result.angle_rad),
            "translation_xy_m": result.translation.tolist(),
            "rms_error_m": result.rms_error,
            "max_error_m": result.max_error,
            "per_point_error_m": result.residuals.tolist(),
            "diagnostic_scale": result.diagnostic_scale,
            "matrix_4x4": result.matrix_4x4.tolist(),
            "source_files_modified": False,
        }
        yaml_path.parent.mkdir(parents=True, exist_ok=True)
        yaml_path.write_text(
            yaml.safe_dump(payload, allow_unicode=True, sort_keys=False), encoding="utf-8"
        )
        np.savetxt(matrix_path, result.matrix_4x4, fmt="%.12f")
        self.status.setText(f"已保存校准记录 {yaml_path.name} 和矩阵 {matrix_path.name}")

    def _render_pcd(self):
        if self.pcd_sample is None:
            return
        sample = self.pcd_sample
        z_mask = (sample.z >= self.z_min.value()) & (sample.z <= self.z_max.value())
        x, y, z = sample.x[z_mask], sample.y[z_mask], sample.z[z_mask]
        x, y = self._transform_xy(x, y)
        px_float, py_float = self.projection.world_to_image_arrays(x, y)
        inside = self.projection.contains_arrays(px_float, py_float)
        finite = np.isfinite(px_float) & np.isfinite(py_float)
        render_mask = finite if self.show_outside_pcd.isChecked() else inside
        margin = self.point_size.value() + 2
        if self.show_outside_pcd.isChecked() and render_mask.any():
            scene_min_x = min(0, math.floor(float(px_float[render_mask].min())) - margin)
            scene_min_y = min(0, math.floor(float(py_float[render_mask].min())) - margin)
            scene_max_x = max(
                self.projection.width,
                math.ceil(float(px_float[render_mask].max())) + margin + 1,
            )
            scene_max_y = max(
                self.projection.height,
                math.ceil(float(py_float[render_mask].max())) + margin + 1,
            )
        else:
            scene_min_x, scene_min_y = 0, 0
            scene_max_x, scene_max_y = self.projection.width, self.projection.height

        canvas_width = scene_max_x - scene_min_x
        canvas_height = scene_max_y - scene_min_y
        maximum_dimension = 6000
        if canvas_width > maximum_dimension or canvas_height > maximum_dimension:
            scene_min_x, scene_min_y = 0, 0
            scene_max_x, scene_max_y = self.projection.width, self.projection.height
            canvas_width, canvas_height = self.projection.width, self.projection.height
            render_mask = inside
            self.status.setText(
                "点云范围超过 6000 px 安全上限，暂按底图范围显示；请先过滤离群点。"
            )

        px = np.floor(px_float[render_mask] - scene_min_x).astype(np.int32)
        py = np.floor(py_float[render_mask] - scene_min_y).astype(np.int32)
        z_rendered = z[render_mask]
        image = np.zeros((canvas_height, canvas_width, 4), dtype=np.uint8)
        if z_rendered.size:
            low, high = float(z_rendered.min()), float(z_rendered.max())
            normalized = (
                np.zeros_like(z_rendered)
                if high <= low else (z_rendered - low) / (high - low)
            )
            colors = np.empty((z_rendered.size, 4), dtype=np.uint8)
            colors[:, 0] = (30 + normalized * 225).astype(np.uint8)
            colors[:, 1] = (220 - normalized * 130).astype(np.uint8)
            colors[:, 2] = (255 - normalized * 210).astype(np.uint8)
            colors[:, 3] = 255
            radius = self.point_size.value() - 1
            for offset_y in range(-radius, radius + 1):
                for offset_x in range(-radius, radius + 1):
                    target_x, target_y = px + offset_x, py + offset_y
                    valid = (
                        (target_x >= 0) & (target_x < canvas_width)
                        & (target_y >= 0) & (target_y < canvas_height)
                    )
                    image[target_y[valid], target_x[valid]] = colors[valid]
        qimage = QtGui.QImage(
            image.data, canvas_width, canvas_height,
            image.strides[0], QtGui.QImage.Format_RGBA8888
        ).copy()
        pixmap = QtGui.QPixmap.fromImage(qimage)
        if self.pcd_item is None:
            self.pcd_item = self.scene.addPixmap(pixmap)
            self.pcd_item.setZValue(10)
        else:
            self.pcd_item.setPixmap(pixmap)
        self.pcd_item.setPos(scene_min_x, scene_min_y)
        self.pcd_scene_rect = self.map_scene_rect.united(
            QtCore.QRectF(scene_min_x, scene_min_y, canvas_width, canvas_height)
        )
        if not self.calibration_active:
            self.scene.setSceneRect(
                self.pcd_scene_rect if self.show_outside_pcd.isChecked() else self.map_scene_rect
            )
        self.pcd_item.setVisible(self.pcd_visible.isChecked())
        self.pcd_item.setOpacity(self.pcd_opacity.value() / 100.0)
        self.current_pcd_inside = int(inside.sum())
        self.current_pcd_filtered = int(z_mask.sum())

    def _semantic_coordinates(self) -> np.ndarray:
        points = []

        def collect(value):
            if isinstance(value, list) and len(value) >= 2 and all(
                isinstance(item, (int, float)) for item in value[:2]
            ):
                points.append(value[:2])
            elif isinstance(value, list):
                for child in value:
                    collect(child)

        for feature in self.semantic.get("features", []):
            collect((feature.get("geometry") or {}).get("coordinates", []))
        return np.asarray(points, dtype=np.float64).reshape(-1, 2)

    def _update_report(self, note: str):
        corners = self.projection.corners
        map_x = [corner[0] for corner in corners]
        map_y = [corner[1] for corner in corners]
        semantic_points = self._semantic_coordinates()
        semantic_text = "无坐标"
        semantic_inside_text = "-"
        if semantic_points.size:
            sx, sy = semantic_points[:, 0], semantic_points[:, 1]
            spx, spy = self.projection.world_to_image_arrays(sx, sy)
            semantic_inside = self.projection.contains_arrays(spx, spy)
            semantic_text = f"X [{sx.min():.3f}, {sx.max():.3f}]\nY [{sy.min():.3f}, {sy.max():.3f}]"
            semantic_inside_text = f"{semantic_inside.sum()}/{semantic_inside.size} ({semantic_inside.mean()*100:.2f}%)"

        pcd_text = "正在加载"
        conclusion = "等待 PCD 后形成完整结论"
        if self.pcd_sample is not None:
            bounds = self.pcd_sample.bounds_xyz
            bounds_label = "抽样范围" if self.pcd_sample.bounds_are_sampled else "全量范围"
            transformed_x, transformed_y = self._transform_xy(
                self.pcd_sample.x, self.pcd_sample.y
            )
            px, py = self.projection.world_to_image_arrays(transformed_x, transformed_y)
            inside = self.projection.contains_arrays(px, py)
            pcd_text = (
                f"总点数 {self.pcd_sample.point_count:,}\n"
                f"显示抽样 {self.pcd_sample.sampled_count:,}\n"
                f"边界统计 {bounds_label}\n"
                f"X [{bounds[0]:.3f}, {bounds[3]:.3f}]\n"
                f"Y [{bounds[1]:.3f}, {bounds[4]:.3f}]\n"
                f"Z [{bounds[2]:.3f}, {bounds[5]:.3f}]\n"
                f"抽样落图 {inside.sum():,}/{inside.size:,} ({inside.mean()*100:.2f}%)"
            )
            semantic_ok = not semantic_points.size or semantic_inside.all()
            if inside.mean() >= 0.95 and semantic_ok:
                conclusion = "几何边界初检通过；请目视确认墙体、作物行和语义线重合。"
            else:
                conclusion = "存在较多越界坐标；整理前应检查原点、分辨率或坐标系。"

        html = f"""
        <h3>当前文件</h3>
        <b>PCD</b><br>{self.pcd_path}<br><br>
        <b>Nav2 YAML</b><br>{self.map_yaml_path}<br>
        <b>PNG</b><br>{self.image_path}<br><br>
        <b>GeoJSON</b><br>{self.semantic_path}<br>
        <h3>Nav2 米制范围</h3>
        X [{min(map_x):.3f}, {max(map_x):.3f}]<br>
        Y [{min(map_y):.3f}, {max(map_y):.3f}]<br>
        {self.projection.width} x {self.projection.height} px @ {self.projection.resolution:.4f} m/px<br>
        origin yaw {math.degrees(self.projection.yaw):.3f} deg
        <h3>PCD 全量范围</h3><pre>{pcd_text}</pre>
        <h3>语义坐标范围</h3><pre>{semantic_text}</pre>
        坐标落图: {semantic_inside_text}<br>
        要素数: {len(self.semantic.get('features', []))}
        <h3>结论</h3>{conclusion}<br><br><i>{note}</i>
        """
        self.report.setHtml(html)
        self.status.setText(note)

    def _toggle_pcd(self, value: bool):
        if self.pcd_item is not None:
            self.pcd_item.setVisible(value)

    def _set_pcd_opacity(self, value: int):
        if self.pcd_item is not None:
            self.pcd_item.setOpacity(value / 100.0)

    def _fit(self):
        self.view.fitInView(self.scene.sceneRect(), QtCore.Qt.KeepAspectRatio)

    def _actual_pixels(self):
        self.view.resetTransform()

    def _show_coordinates(self, x: float, y: float):
        self.status.setText(f"map: x={x:.3f} m, y={y:.3f} m")


def parse_args(argv: Iterable[str] | None = None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--map", type=Path, default=DEFAULT_MAP, help="Nav2 map YAML")
    parser.add_argument("--pcd", type=Path, default=DEFAULT_PCD, help="Aligned PCD file")
    parser.add_argument("--semantic-map", type=Path, default=DEFAULT_SEMANTIC, help="Semantic GeoJSON")
    parser.add_argument("--max-points", type=int, default=750_000, help="Maximum rendered PCD sample")
    return parser.parse_args(argv)


def main(argv: Iterable[str] | None = None) -> int:
    args = parse_args(argv)
    for label, path in (("map", args.map), ("pcd", args.pcd), ("semantic map", args.semantic_map)):
        if not path.expanduser().is_file():
            print(f"Missing {label}: {path}", file=sys.stderr)
            return 2
    app = QtWidgets.QApplication(sys.argv[:1])
    app.setApplicationName("AGT map alignment viewer")
    window = AlignmentWindow(
        args.map.expanduser().resolve(), args.pcd.expanduser().resolve(),
        args.semantic_map.expanduser().resolve(), args.max_points
    )
    window.show()
    return app.exec_()


if __name__ == "__main__":
    raise SystemExit(main())
