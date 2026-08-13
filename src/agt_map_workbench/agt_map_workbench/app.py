"""AGT Map Workbench offline visual authoring client."""

from __future__ import annotations

from dataclasses import replace
import sys
from pathlib import Path

import numpy as np
from PyQt5.QtCore import QObject, QPointF, QThread, Qt, pyqtSignal
from PyQt5.QtGui import QColor, QBrush, QPen, QPolygonF
from PyQt5.QtWidgets import (
    QApplication,
    QCheckBox,
    QComboBox,
    QDoubleSpinBox,
    QFileDialog,
    QGraphicsEllipseItem,
    QGraphicsItem,
    QGraphicsLineItem,
    QGraphicsPolygonItem,
    QGraphicsScene,
    QGraphicsSimpleTextItem,
    QHBoxLayout,
    QInputDialog,
    QLabel,
    QListWidget,
    QMainWindow,
    QMessageBox,
    QPushButton,
    QSpinBox,
    QSplitter,
    QTabWidget,
    QVBoxLayout,
    QWidget,
)

from agt_offline_assets import (
    GroundRelativeNavigationConfig,
    NavigationMapResult,
    apply_navigation_overrides,
    derive_ground_relative_navigation_map,
    process_pointcloud,
    read_pcd,
    write_navigation_map_derivation,
)

from .frame_calibration import (
    AxisFit,
    MapFrameCalibration,
    fit_horizontal_axis_from_corridor,
    fit_vertical_axis_from_cylinder,
    nearest_xyz_in_window,
    solve_map_frame,
)
from .model import WorkbenchRecipeModel
from .navigation_preview import NavigationPreviewItem
from .view import PointCloudItem, PointCloudView


_OPERATION_NAMES = {
    "delete_polygon": "删除多边形体积",
    "crop_polygon": "保留多边形体积",
    "crop_box": "长方体裁剪",
    "height_range": "高度范围",
    "voxel_downsample": "体素降采样",
    "sor": "统计离群点滤波",
    "radius_outlier": "半径离群点滤波",
    "remove_nonfinite": "移除非有限点",
    "ground_separation": "地面分离",
}

_OVERRIDE_NAMES = {
    "force_free": "强制可通行",
    "force_occupied": "强制占据",
    "unknown": "强制未知",
    "no_go": "禁行区域",
}


class _ProcessingWorker(QObject):
    finished = pyqtSignal(object)
    failed = pyqtSignal(str)

    def __init__(self, input_path: Path, recipe_path: Path, output_dir: Path) -> None:
        super().__init__()
        self.input_path = input_path
        self.recipe_path = recipe_path
        self.output_dir = output_dir

    def run(self) -> None:
        try:
            result = process_pointcloud(self.input_path, self.recipe_path, self.output_dir)
        except Exception as exc:
            self.failed.emit(str(exc))
            return
        self.finished.emit(result)


class _NavigationWorker(QObject):
    finished = pyqtSignal(object)
    failed = pyqtSignal(str)

    def __init__(self, cloud, config: GroundRelativeNavigationConfig) -> None:
        super().__init__()
        self.cloud = cloud
        self.config = config

    def run(self) -> None:
        try:
            result = derive_ground_relative_navigation_map(self.cloud, self.config)
        except Exception as exc:
            self.failed.emit(str(exc))
            return
        self.finished.emit(result)


class MapWorkbenchWindow(QMainWindow):
    def __init__(self) -> None:
        super().__init__()
        self.setWindowTitle("AGT 地图工作台 — V25-12C")
        self.resize(1520, 940)

        self._source_path: Path | None = None
        self._cloud = None
        self._recipe = WorkbenchRecipeModel()
        self._interaction_mode: str | None = None

        self._vertices: list[tuple[float, float]] = []
        self._polygon_item: QGraphicsPolygonItem | None = None
        self._vertex_items: list[QGraphicsItem] = []

        self._frame_origin: np.ndarray | None = None
        self._x_reference_clicks: list[tuple[float, float]] = []
        self._x_fit: AxisFit | None = None
        self._z_fit: AxisFit | None = None
        self._z_reference_xy: tuple[float, float] | None = None
        self._frame_calibration: MapFrameCalibration | None = None
        self._calibration_selection_items: list[QGraphicsItem] = []
        self._frame_preview_items: list[QGraphicsItem] = []

        self._navigation_base_result: NavigationMapResult | None = None
        self._navigation_result: NavigationMapResult | None = None
        self._navigation_overrides: list[dict] = []
        self._nav_vertices: list[tuple[float, float]] = []
        self._nav_polygon_item: QGraphicsPolygonItem | None = None
        self._nav_vertex_items: list[QGraphicsItem] = []

        self._thread: QThread | None = None
        self._worker: _ProcessingWorker | None = None
        self._nav_thread: QThread | None = None
        self._nav_worker: _NavigationWorker | None = None

        self._scene = QGraphicsScene(self)
        self._cloud_item = PointCloudItem()
        self._scene.addItem(self._cloud_item)
        self._navigation_preview_item = NavigationPreviewItem()
        self._scene.addItem(self._navigation_preview_item)
        self._view = PointCloudView(self._scene)
        self._view.mapClicked.connect(self._on_map_clicked)

        controls = self._build_controls()
        splitter = QSplitter()
        splitter.addWidget(self._view)
        splitter.addWidget(controls)
        splitter.setStretchFactor(0, 1)
        splitter.setSizes([1040, 480])
        self.setCentralWidget(splitter)
        self.statusBar().showMessage("请打开一个 PCD 点云文件")
        self._refresh_operations()
        self._refresh_vertex_list()
        self._refresh_z_status()
        self._refresh_calibration_status()
        self._refresh_navigation_override_list()
        self._refresh_navigation_status()

    # ------------------------------------------------------------------ UI
    def _build_controls(self) -> QWidget:
        panel = QWidget()
        panel.setMinimumWidth(445)
        layout = QVBoxLayout(panel)

        open_button = QPushButton("打开 PCD 点云")
        open_button.clicked.connect(self._open_pcd)
        layout.addWidget(open_button)

        self._source_label = QLabel("尚未加载点云")
        self._source_label.setWordWrap(True)
        layout.addWidget(self._source_label)

        layout.addWidget(QLabel("显示设置（仅影响预览，不修改正式 PCD）"))
        display_row = QHBoxLayout()
        self._background_mode = QComboBox()
        self._background_mode.addItem("深色背景", "dark")
        self._background_mode.addItem("浅色背景", "light")
        self._background_mode.currentIndexChanged.connect(self._update_display_options)
        self._color_mode = QComboBox()
        self._color_mode.addItem("按高度着色", "height")
        self._color_mode.addItem("按强度着色", "intensity")
        self._color_mode.addItem("单色", "mono")
        self._color_mode.currentIndexChanged.connect(self._update_display_options)
        display_row.addWidget(self._background_mode)
        display_row.addWidget(self._color_mode)
        layout.addLayout(display_row)

        density_row = QHBoxLayout()
        self._sample_limit = QComboBox()
        for label, value in (
            ("显示 6 万点", 60_000),
            ("显示 15 万点", 150_000),
            ("显示 30 万点", 300_000),
            ("显示 60 万点", 600_000),
        ):
            self._sample_limit.addItem(label, value)
        self._sample_limit.setCurrentIndex(1)
        self._sample_limit.currentIndexChanged.connect(self._reload_display_sample)
        self._point_size = QComboBox()
        for size in (1, 2, 3, 4):
            self._point_size.addItem(f"点大小 {size}px", float(size))
        self._point_size.currentIndexChanged.connect(self._update_display_options)
        density_row.addWidget(self._sample_limit)
        density_row.addWidget(self._point_size)
        layout.addLayout(density_row)

        layout.addWidget(QLabel("显示 Z 范围（只控制点云可视化）"))
        z_row = QHBoxLayout()
        self._z_min = self._new_z_spin("最小 Z：")
        self._z_max = self._new_z_spin("最大 Z：")
        self._z_min.valueChanged.connect(self._update_z_window)
        self._z_max.valueChanged.connect(self._update_z_window)
        z_row.addWidget(self._z_min)
        z_row.addWidget(self._z_max)
        layout.addLayout(z_row)

        self._z_status = QLabel("当前可见采样点：未加载")
        self._z_status.setWordWrap(True)
        layout.addWidget(self._z_status)

        fit_button = QPushButton("点云适配窗口（保留编辑边距）")
        fit_button.clicked.connect(self._fit_cloud)
        layout.addWidget(fit_button)

        tabs = QTabWidget()
        tabs.addTab(self._build_edit_tab(), "点云编辑")
        tabs.addTab(self._build_frame_tab(), "坐标系标定")
        tabs.addTab(self._build_navigation_tab(), "导航地图")
        layout.addWidget(tabs, 1)
        return panel

    @staticmethod
    def _new_z_spin(prefix: str) -> QDoubleSpinBox:
        box = QDoubleSpinBox()
        box.setPrefix(prefix)
        box.setDecimals(3)
        box.setRange(-10000.0, 10000.0)
        box.setSingleStep(0.1)
        return box

    def _build_edit_tab(self) -> QWidget:
        tab = QWidget()
        layout = QVBoxLayout(tab)
        layout.addWidget(QLabel("多边形操作模式"))
        self._mode = QComboBox()
        self._mode.addItem("删除选中三维区域", "delete")
        self._mode.addItem("仅保留选中三维区域（裁剪）", "crop")
        layout.addWidget(self._mode)

        start_button = QPushButton("开始绘制多边形")
        start_button.clicked.connect(self._start_polygon)
        finish_button = QPushButton("完成多边形并加入处理流程")
        finish_button.clicked.connect(self._finish_polygon)
        undo_vertex = QPushButton("撤销上一个顶点")
        undo_vertex.clicked.connect(self._undo_vertex)
        author_row = QHBoxLayout()
        author_row.addWidget(start_button)
        author_row.addWidget(finish_button)
        layout.addLayout(author_row)
        layout.addWidget(undo_vertex)

        self._selection_status = QLabel("当前未绘制多边形")
        self._selection_status.setWordWrap(True)
        layout.addWidget(self._selection_status)
        layout.addWidget(QLabel("当前多边形顶点顺序"))
        self._vertex_list = QListWidget()
        self._vertex_list.setMaximumHeight(115)
        layout.addWidget(self._vertex_list)

        layout.addWidget(QLabel("处理流程 / Recipe 执行顺序"))
        self._operations = QListWidget()
        layout.addWidget(self._operations, 1)

        row = QHBoxLayout()
        undo_operation = QPushButton("撤销上一项操作")
        undo_operation.clicked.connect(self._undo_operation)
        clear_operations = QPushButton("清空全部操作")
        clear_operations.clicked.connect(self._clear_operations)
        row.addWidget(undo_operation)
        row.addWidget(clear_operations)
        layout.addLayout(row)

        export_button = QPushButton("导出处理 Recipe YAML")
        export_button.clicked.connect(self._export_recipe)
        process_button = QPushButton("执行完整分辨率不可变处理")
        process_button.clicked.connect(self._run_processing)
        layout.addWidget(export_button)
        layout.addWidget(process_button)
        return tab

    def _build_calibration_z_row(self, title: str):
        row = QHBoxLayout()
        row.addWidget(QLabel(title))
        minimum = self._new_z_spin("min：")
        maximum = self._new_z_spin("max：")
        row.addWidget(minimum)
        row.addWidget(maximum)
        return row, minimum, maximum

    def _build_frame_tab(self) -> QWidget:
        tab = QWidget()
        layout = QVBoxLayout(tab)
        intro = QLabel(
            "Map Frame Calibration：用稳定结构定义便于导航的右手坐标系。\n"
            "显示 Z 与标定选取 Z 已分离；修改显示 Z 不会改变已经完成的拟合。\n"
            "ENU / UTM 地理配准后续单独绑定，不强制 map 的 X 指向正东。"
        )
        intro.setWordWrap(True)
        layout.addWidget(intro)

        copy_button = QPushButton("把当前显示 Z 复制到全部标定选取范围")
        copy_button.clicked.connect(self._copy_display_z_to_calibration)
        layout.addWidget(copy_button)

        row, self._origin_z_min, self._origin_z_max = self._build_calibration_z_row("原点吸附 Z")
        layout.addLayout(row)
        row, self._x_fit_z_min, self._x_fit_z_max = self._build_calibration_z_row("X 墙拟合 Z")
        layout.addLayout(row)
        row, self._z_fit_z_min, self._z_fit_z_max = self._build_calibration_z_row("Z 柱拟合 Z")
        layout.addLayout(row)

        origin_button = QPushButton("① 选择地图原点")
        origin_button.clicked.connect(self._select_frame_origin)
        x_button = QPushButton("② 选择 X 参考墙两端")
        x_button.clicked.connect(self._select_x_reference)
        z_button = QPushButton("③ 选择 Z 参考立柱中心")
        z_button.clicked.connect(self._select_z_reference)
        layout.addWidget(origin_button)
        layout.addWidget(x_button)
        layout.addWidget(z_button)

        parameter_row = QHBoxLayout()
        self._x_corridor_width = QDoubleSpinBox()
        self._x_corridor_width.setPrefix("X 半宽：")
        self._x_corridor_width.setSuffix(" m")
        self._x_corridor_width.setDecimals(2)
        self._x_corridor_width.setRange(0.05, 2.0)
        self._x_corridor_width.setValue(0.30)
        self._z_pillar_radius = QDoubleSpinBox()
        self._z_pillar_radius.setPrefix("Z 半径：")
        self._z_pillar_radius.setSuffix(" m")
        self._z_pillar_radius.setDecimals(2)
        self._z_pillar_radius.setRange(0.05, 2.0)
        self._z_pillar_radius.setValue(0.25)
        parameter_row.addWidget(self._x_corridor_width)
        parameter_row.addWidget(self._z_pillar_radius)
        layout.addLayout(parameter_row)

        self._origin_status = QLabel("原点：未选择")
        self._x_axis_status = QLabel("X 参考：未拟合")
        self._z_axis_status = QLabel("Z 参考：未拟合")
        self._frame_status = QLabel("坐标系：等待原点、X、Z")
        for label in (
            self._origin_status,
            self._x_axis_status,
            self._z_axis_status,
            self._frame_status,
        ):
            label.setWordWrap(True)
            layout.addWidget(label)

        flip_row = QHBoxLayout()
        flip_x = QPushButton("翻转 X 方向")
        flip_x.clicked.connect(self._flip_frame_x)
        flip_z = QPushButton("翻转 Z 方向")
        flip_z.clicked.connect(self._flip_frame_z)
        flip_row.addWidget(flip_x)
        flip_row.addWidget(flip_z)
        layout.addLayout(flip_row)

        clear_button = QPushButton("清空坐标系标定")
        clear_button.clicked.connect(self._clear_frame_calibration)
        export_button = QPushButton("导出 map_frame.yaml")
        export_button.clicked.connect(self._export_map_frame)
        layout.addWidget(clear_button)
        layout.addWidget(export_button)
        layout.addStretch(1)
        return tab

    def _build_navigation_tab(self) -> QWidget:
        tab = QWidget()
        layout = QVBoxLayout(tab)
        intro = QLabel(
            "Ground-relative Navigation Map：不使用全局绝对 Z 切片。\n"
            "局部地面高度 → 相对高度障碍 → 坡度/台阶 → FREE/OCCUPIED/UNKNOWN。\n"
            "当前派生始终使用已加载 PCD 的坐标系；显示 Z 不参与计算。"
        )
        intro.setWordWrap(True)
        layout.addWidget(intro)

        row = QHBoxLayout()
        self._nav_resolution = QDoubleSpinBox()
        self._nav_resolution.setPrefix("分辨率：")
        self._nav_resolution.setSuffix(" m")
        self._nav_resolution.setDecimals(2)
        self._nav_resolution.setRange(0.03, 0.50)
        self._nav_resolution.setSingleStep(0.01)
        self._nav_resolution.setValue(0.10)
        self._nav_ground_quantile = QDoubleSpinBox()
        self._nav_ground_quantile.setPrefix("地面低分位：")
        self._nav_ground_quantile.setDecimals(2)
        self._nav_ground_quantile.setRange(0.0, 0.50)
        self._nav_ground_quantile.setSingleStep(0.05)
        self._nav_ground_quantile.setValue(0.10)
        row.addWidget(self._nav_resolution)
        row.addWidget(self._nav_ground_quantile)
        layout.addLayout(row)

        row = QHBoxLayout()
        self._nav_fill_distance = QDoubleSpinBox()
        self._nav_fill_distance.setPrefix("地面补洞：")
        self._nav_fill_distance.setSuffix(" m")
        self._nav_fill_distance.setDecimals(2)
        self._nav_fill_distance.setRange(0.0, 2.0)
        self._nav_fill_distance.setValue(0.35)
        self._nav_smoothing = QSpinBox()
        self._nav_smoothing.setPrefix("平滑半径：")
        self._nav_smoothing.setSuffix(" cells")
        self._nav_smoothing.setRange(0, 10)
        self._nav_smoothing.setValue(2)
        row.addWidget(self._nav_fill_distance)
        row.addWidget(self._nav_smoothing)
        layout.addLayout(row)

        row = QHBoxLayout()
        self._nav_obstacle_min = QDoubleSpinBox()
        self._nav_obstacle_min.setPrefix("障碍相对高度 min：")
        self._nav_obstacle_min.setSuffix(" m")
        self._nav_obstacle_min.setDecimals(2)
        self._nav_obstacle_min.setRange(0.0, 2.0)
        self._nav_obstacle_min.setValue(0.12)
        self._nav_obstacle_max = QDoubleSpinBox()
        self._nav_obstacle_max.setPrefix("max：")
        self._nav_obstacle_max.setSuffix(" m")
        self._nav_obstacle_max.setDecimals(2)
        self._nav_obstacle_max.setRange(0.05, 4.0)
        self._nav_obstacle_max.setValue(1.00)
        row.addWidget(self._nav_obstacle_min)
        row.addWidget(self._nav_obstacle_max)
        layout.addLayout(row)

        row = QHBoxLayout()
        self._nav_max_slope = QDoubleSpinBox()
        self._nav_max_slope.setPrefix("最大坡度：")
        self._nav_max_slope.setSuffix("°")
        self._nav_max_slope.setRange(0.0, 45.0)
        self._nav_max_slope.setValue(18.0)
        self._nav_max_step = QDoubleSpinBox()
        self._nav_max_step.setPrefix("最大台阶：")
        self._nav_max_step.setSuffix(" m")
        self._nav_max_step.setDecimals(2)
        self._nav_max_step.setRange(0.02, 1.0)
        self._nav_max_step.setValue(0.12)
        row.addWidget(self._nav_max_slope)
        row.addWidget(self._nav_max_step)
        layout.addLayout(row)

        generate = QPushButton("生成 Ground-relative 导航图预览")
        generate.clicked.connect(self._generate_navigation_preview)
        layout.addWidget(generate)

        preview_row = QHBoxLayout()
        self._nav_layer = QComboBox()
        self._nav_layer.addItem("最终 PGM 三态", "final")
        self._nav_layer.addItem("局部地面高度", "ground")
        self._nav_layer.addItem("障碍点证据", "obstacle")
        self._nav_layer.addItem("坡度", "slope")
        self._nav_layer.addItem("台阶高度", "step")
        self._nav_layer.currentIndexChanged.connect(self._update_navigation_overlay)
        self._nav_overlay_visible = QCheckBox("显示叠加层")
        self._nav_overlay_visible.setChecked(True)
        self._nav_overlay_visible.toggled.connect(self._update_navigation_overlay)
        preview_row.addWidget(self._nav_layer)
        preview_row.addWidget(self._nav_overlay_visible)
        layout.addLayout(preview_row)

        self._nav_status = QLabel("导航图：尚未生成")
        self._nav_status.setWordWrap(True)
        layout.addWidget(self._nav_status)

        layout.addWidget(QLabel("人工 Override（不是手画像素，可随分辨率重新投影）"))
        self._nav_override_mode = QComboBox()
        self._nav_override_mode.addItem("强制可通行 FORCE_FREE", "force_free")
        self._nav_override_mode.addItem("强制占据 FORCE_OCCUPIED", "force_occupied")
        self._nav_override_mode.addItem("强制未知 UNKNOWN", "unknown")
        self._nav_override_mode.addItem("禁行 NO_GO", "no_go")
        layout.addWidget(self._nav_override_mode)

        row = QHBoxLayout()
        start_override = QPushButton("开始绘制 Override")
        start_override.clicked.connect(self._start_navigation_override)
        finish_override = QPushButton("完成并加入 Override")
        finish_override.clicked.connect(self._finish_navigation_override)
        row.addWidget(start_override)
        row.addWidget(finish_override)
        layout.addLayout(row)

        self._nav_override_list = QListWidget()
        self._nav_override_list.setMaximumHeight(110)
        layout.addWidget(self._nav_override_list)
        row = QHBoxLayout()
        undo_override = QPushButton("撤销上一项 Override")
        undo_override.clicked.connect(self._undo_navigation_override)
        clear_override = QPushButton("清空 Override")
        clear_override.clicked.connect(self._clear_navigation_overrides)
        row.addWidget(undo_override)
        row.addWidget(clear_override)
        layout.addLayout(row)

        export = QPushButton("导出 Navigation Map PGM / YAML / 证据")
        export.clicked.connect(self._export_navigation_map)
        layout.addWidget(export)
        layout.addStretch(1)
        return tab

    # -------------------------------------------------------------- PCD/display
    @staticmethod
    def _default_pcd_directory() -> str:
        candidate = Path.cwd() / "runtime" / "maps"
        return str(candidate if candidate.is_dir() else Path.cwd())

    def _open_pcd(self) -> None:
        filename, _ = QFileDialog.getOpenFileName(
            self,
            "打开 PCD 点云",
            self._default_pcd_directory(),
            "PCD 点云 (*.pcd);;所有文件 (*)",
        )
        if not filename:
            return
        try:
            cloud = read_pcd(filename)
            z_values = np.asarray(cloud.points["z"], dtype=np.float64)
            finite_z = z_values[np.isfinite(z_values)]
            if finite_z.size == 0:
                raise ValueError("PCD 中不存在有效的有限 Z 坐标")
        except Exception as exc:
            QMessageBox.critical(self, "打开失败", str(exc))
            return

        self._source_path = Path(filename).resolve()
        self._cloud = cloud
        self._cloud_item.set_cloud(cloud, sample_limit=int(self._sample_limit.currentData()))
        self._update_display_options()
        minimum = float(np.min(finite_z))
        maximum = float(np.max(finite_z))
        self._set_pair_values(self._z_min, self._z_max, minimum, maximum)
        self._cloud_item.set_z_window(minimum, maximum)
        self._set_all_calibration_z_ranges(minimum, maximum)

        self._recipe = WorkbenchRecipeModel(
            recipe_id=f"{self._source_path.stem}_workbench_draft",
            frame_id="map",
        )
        self._refresh_operations()
        self._clear_polygon()
        self._clear_frame_calibration()
        self._clear_navigation_state(clear_overrides=True)
        self._source_label.setText(
            f"当前点云：{self._source_path}\n"
            f"源点数：{int(cloud.points.shape[0]):,} | PCD 数据模式：{cloud.data_mode}"
        )
        self._refresh_z_status()
        self._fit_cloud()
        self.statusBar().showMessage(
            "点云加载完成；显示采样只影响预览，正式处理与导航派生仍读取完整 PCD"
        )

    @staticmethod
    def _set_pair_values(minimum_box, maximum_box, minimum: float, maximum: float) -> None:
        minimum_box.blockSignals(True)
        maximum_box.blockSignals(True)
        minimum_box.setValue(float(minimum))
        maximum_box.setValue(float(maximum))
        minimum_box.blockSignals(False)
        maximum_box.blockSignals(False)

    def _set_all_calibration_z_ranges(self, minimum: float, maximum: float) -> None:
        for minimum_box, maximum_box in (
            (self._origin_z_min, self._origin_z_max),
            (self._x_fit_z_min, self._x_fit_z_max),
            (self._z_fit_z_min, self._z_fit_z_max),
        ):
            self._set_pair_values(minimum_box, maximum_box, minimum, maximum)

    def _copy_display_z_to_calibration(self) -> None:
        self._set_all_calibration_z_ranges(self._z_min.value(), self._z_max.value())
        self.statusBar().showMessage(
            "已把当前显示 Z 复制到原点/X墙/Z柱标定范围；后续可分别修改"
        )

    def _reload_display_sample(self) -> None:
        if self._cloud is None:
            return
        self._cloud_item.set_cloud(
            self._cloud, sample_limit=int(self._sample_limit.currentData())
        )
        if self._z_max.value() >= self._z_min.value():
            self._cloud_item.set_z_window(self._z_min.value(), self._z_max.value())
        self._update_display_options()
        self._refresh_z_status()
        self.statusBar().showMessage(
            f"显示采样已更新为最多 {int(self._sample_limit.currentData()):,} 点；正式 PCD 未改变"
        )

    def _update_display_options(self) -> None:
        self._view.set_background_mode(str(self._background_mode.currentData()))
        self._cloud_item.set_display_options(
            color_mode=str(self._color_mode.currentData()),
            point_size_px=float(self._point_size.currentData()),
        )
        if self._color_mode.currentData() == "intensity" and self._cloud is not None:
            if not self._cloud_item.has_intensity():
                self.statusBar().showMessage("当前 PCD 没有 scalar intensity，将回退为高度着色")

    def _fit_cloud(self) -> None:
        rect = self._cloud_item.padded_bounding_rect(ratio=0.10, minimum_margin=1.0)
        if not rect.isNull():
            self._scene.setSceneRect(rect)
            self._view.fitInView(rect, Qt.KeepAspectRatio)
            self.statusBar().showMessage("已适配点云，并在四周保留 10% / 至少 1 m 的编辑边距")

    def _refresh_z_status(self) -> None:
        total = self._cloud_item.sample_count()
        if total == 0:
            self._z_status.setText("当前可见采样点：未加载")
            return
        visible = self._cloud_item.visible_sample_count()
        ratio = 100.0 * float(visible) / float(total)
        source_count = int(self._cloud.points.shape[0]) if self._cloud is not None else 0
        self._z_status.setText(
            f"当前可见采样点：{visible:,} / {total:,}（{ratio:.1f}%）\n"
            f"源 PCD：{source_count:,} 点 | 显示 Z=[{self._z_min.value():.3f}, {self._z_max.value():.3f}] m"
        )

    def _update_z_window(self) -> None:
        minimum = self._z_min.value()
        maximum = self._z_max.value()
        if maximum < minimum:
            self._z_status.setText("显示 Z 范围无效：最大 Z 必须大于或等于最小 Z")
            return
        self._cloud_item.set_z_window(minimum, maximum)
        self._refresh_z_status()
        self.statusBar().showMessage(
            f"显示 Z 已更新：[{minimum:.3f}, {maximum:.3f}] m；标定 Z 和导航派生参数不随之改变"
        )

    # -------------------------------------------------------------- interaction
    def _set_interaction_mode(self, mode: str | None, message: str) -> None:
        self._interaction_mode = mode
        self._view.set_authoring_enabled(mode is not None)
        self.statusBar().showMessage(message)

    def _on_map_clicked(self, x: float, y: float) -> None:
        if self._interaction_mode == "polygon":
            self._append_vertex(x, y)
        elif self._interaction_mode == "frame_origin":
            self._pick_frame_origin(x, y)
        elif self._interaction_mode == "frame_x":
            self._pick_x_reference(x, y)
        elif self._interaction_mode == "frame_z":
            self._pick_z_reference(x, y)
        elif self._interaction_mode == "nav_override":
            self._append_navigation_override_vertex(x, y)

    # --------------------------------------------------------------- PCD recipe
    def _start_polygon(self) -> None:
        if self._cloud is None:
            QMessageBox.information(self, "尚未加载点云", "请先打开一个 PCD 点云文件")
            return
        self._vertices.clear()
        self._update_polygon_item()
        self._set_interaction_mode(
            "polygon",
            "多边形绘制模式：十字光标左键添加顶点，黄色编号表示点击顺序",
        )

    def _append_vertex(self, x: float, y: float) -> None:
        self._vertices.append((x, y))
        self._update_polygon_item()
        self.statusBar().showMessage(
            f"已添加顶点 {len(self._vertices)}：X={x:.3f} m，Y={y:.3f} m"
        )

    def _undo_vertex(self) -> None:
        if self._vertices:
            self._vertices.pop()
            self._update_polygon_item()

    def _clear_polygon(self) -> None:
        self._vertices.clear()
        if self._polygon_item is not None:
            self._scene.removeItem(self._polygon_item)
            self._polygon_item = None
        for item in self._vertex_items:
            self._scene.removeItem(item)
        self._vertex_items.clear()
        if self._interaction_mode == "polygon":
            self._set_interaction_mode(None, "多边形绘制已结束")
        self._refresh_vertex_list()

    def _update_polygon_item(self) -> None:
        if self._polygon_item is not None:
            self._scene.removeItem(self._polygon_item)
            self._polygon_item = None
        for item in self._vertex_items:
            self._scene.removeItem(item)
        self._vertex_items.clear()
        if self._vertices:
            scene_points = [QPointF(x, -y) for x, y in self._vertices]
            self._polygon_item = QGraphicsPolygonItem(QPolygonF(scene_points))
            pen = QPen(QColor(255, 220, 0))
            pen.setWidth(2)
            pen.setCosmetic(True)
            self._polygon_item.setPen(pen)
            self._polygon_item.setBrush(QBrush(QColor(255, 220, 0, 28)))
            self._polygon_item.setZValue(10.0)
            self._scene.addItem(self._polygon_item)
            for index, point in enumerate(scene_points, start=1):
                self._add_fixed_marker(
                    point, str(index), QColor(255, 220, 0), self._vertex_items
                )
        self._refresh_vertex_list()

    def _refresh_vertex_list(self) -> None:
        if not hasattr(self, "_vertex_list"):
            return
        self._vertex_list.clear()
        if not self._vertices:
            self._vertex_list.addItem("（暂无顶点）")
            self._selection_status.setText("当前未绘制多边形")
            return
        for index, (x, y) in enumerate(self._vertices, start=1):
            self._vertex_list.addItem(f"顶点 {index:02d} | X={x:.3f} m | Y={y:.3f} m")
        self._selection_status.setText(
            f"正在绘制：已选择 {len(self._vertices)} 个顶点 | "
            f"处理 Z=[{self._z_min.value():.3f}, {self._z_max.value():.3f}] m"
        )

    def _finish_polygon(self) -> None:
        if len(self._vertices) < 3:
            QMessageBox.warning(self, "多边形未完成", "至少需要 3 个顶点")
            return
        try:
            self._recipe.add_polygon_volume(
                self._vertices,
                z_min=self._z_min.value(),
                z_max=self._z_max.value(),
                delete=self._mode.currentData() == "delete",
            )
        except ValueError as exc:
            QMessageBox.warning(self, "操作无效", str(exc))
            return
        self._refresh_operations()
        self._clear_polygon()
        self.statusBar().showMessage("操作已加入可重放的 V25-12B Recipe")

    def _refresh_operations(self) -> None:
        if not hasattr(self, "_operations"):
            return
        self._operations.clear()
        if not self._recipe.operations:
            self._operations.addItem("（暂无已加入的处理操作）")
            return
        for index, operation in enumerate(self._recipe.operations):
            params = operation.parameters
            operation_name = _OPERATION_NAMES.get(operation.type, operation.type)
            if operation.type in {"crop_polygon", "delete_polygon"}:
                detail = (
                    f"{len(params.get('polygon_xy', []))} 个顶点 | "
                    f"Z=[{params.get('z_min'):.2f}, {params.get('z_max'):.2f}] m"
                )
            else:
                detail = str(params)
            self._operations.addItem(
                f"执行顺序 {index + 1:02d} | {operation_name}（{operation.type}） | {detail}"
            )

    def _undo_operation(self) -> None:
        self._recipe.undo()
        self._refresh_operations()

    def _clear_operations(self) -> None:
        self._recipe.clear()
        self._refresh_operations()

    # ---------------------------------------------------------- frame calibration
    @staticmethod
    def _cloud_fields(cloud):
        return cloud.points["x"], cloud.points["y"], cloud.points["z"]

    @staticmethod
    def _valid_pair(minimum_box, maximum_box) -> bool:
        return maximum_box.value() >= minimum_box.value()

    def _require_cloud_for_calibration(self) -> bool:
        if self._cloud is None:
            QMessageBox.information(self, "尚未加载点云", "请先打开一个 PCD 点云文件")
            return False
        return True

    def _select_frame_origin(self) -> None:
        if not self._require_cloud_for_calibration():
            return
        if not self._valid_pair(self._origin_z_min, self._origin_z_max):
            QMessageBox.warning(self, "原点 Z 无效", "请设置有效的原点吸附 Z 范围")
            return
        self._set_interaction_mode(
            "frame_origin",
            "原点选择：点击墙角/柱脚；只在“原点吸附 Z”范围内寻找最近 3D 点",
        )

    def _pick_frame_origin(self, x: float, y: float) -> None:
        try:
            point = nearest_xyz_in_window(
                *self._cloud_fields(self._cloud),
                (x, y),
                z_min=self._origin_z_min.value(),
                z_max=self._origin_z_max.value(),
            )
        except ValueError as exc:
            QMessageBox.warning(self, "原点选择失败", str(exc))
            return
        self._frame_origin = point
        self._set_interaction_mode(None, "地图原点已吸附到标定 Z 范围内最近 3D 点")
        self._rebuild_calibration_selection_markers()
        self._recompute_map_frame()

    def _select_x_reference(self) -> None:
        if not self._require_cloud_for_calibration():
            return
        if not self._valid_pair(self._x_fit_z_min, self._x_fit_z_max):
            QMessageBox.warning(self, "X 墙 Z 无效", "请设置有效的 X 墙拟合 Z 范围")
            return
        self._x_reference_clicks.clear()
        self._x_fit = None
        self._frame_calibration = None
        self._set_interaction_mode(
            "frame_x",
            "X 参考墙：沿希望的 +X 方向点击墙壁起点和终点；拟合使用独立 X 墙 Z 范围",
        )
        self._rebuild_calibration_selection_markers()
        self._refresh_calibration_status()

    def _pick_x_reference(self, x: float, y: float) -> None:
        self._x_reference_clicks.append((x, y))
        self._rebuild_calibration_selection_markers()
        if len(self._x_reference_clicks) == 1:
            self.statusBar().showMessage("X 起点已选；请沿希望的 +X 方向点击墙壁终点")
            return
        start, end = self._x_reference_clicks[:2]
        try:
            self._x_fit = fit_horizontal_axis_from_corridor(
                *self._cloud_fields(self._cloud),
                start,
                end,
                z_min=self._x_fit_z_min.value(),
                z_max=self._x_fit_z_max.value(),
                half_width_m=self._x_corridor_width.value(),
            )
        except ValueError as exc:
            self._x_reference_clicks.clear()
            self._x_fit = None
            self._set_interaction_mode(None, "X 参考拟合失败")
            self._rebuild_calibration_selection_markers()
            self._refresh_calibration_status()
            QMessageBox.warning(self, "X 参考拟合失败", str(exc))
            return
        self._set_interaction_mode(None, "X 参考墙拟合完成")
        self._recompute_map_frame()

    def _select_z_reference(self) -> None:
        if not self._require_cloud_for_calibration():
            return
        if not self._valid_pair(self._z_fit_z_min, self._z_fit_z_max):
            QMessageBox.warning(self, "Z 柱 Z 无效", "请设置有效的 Z 柱拟合 Z 范围")
            return
        self._z_fit = None
        self._frame_calibration = None
        self._set_interaction_mode(
            "frame_z",
            "Z 参考立柱：点击立柱 XY 中心；拟合使用独立 Z 柱拟合范围",
        )
        self._refresh_calibration_status()

    def _pick_z_reference(self, x: float, y: float) -> None:
        try:
            self._z_fit = fit_vertical_axis_from_cylinder(
                *self._cloud_fields(self._cloud),
                (x, y),
                z_min=self._z_fit_z_min.value(),
                z_max=self._z_fit_z_max.value(),
                radius_m=self._z_pillar_radius.value(),
            )
        except ValueError as exc:
            self._z_fit = None
            self._set_interaction_mode(None, "Z 参考拟合失败")
            self._refresh_calibration_status()
            QMessageBox.warning(self, "Z 参考拟合失败", str(exc))
            return
        self._z_reference_xy = (x, y)
        self._set_interaction_mode(None, "Z 参考立柱拟合完成")
        self._rebuild_calibration_selection_markers()
        self._recompute_map_frame()

    def _recompute_map_frame(self) -> None:
        if self._frame_origin is not None and self._x_fit is not None and self._z_fit is not None:
            try:
                self._frame_calibration = solve_map_frame(
                    self._frame_origin,
                    self._x_fit.direction,
                    self._z_fit.direction,
                    x_fit=self._x_fit,
                    z_fit=self._z_fit,
                )
            except ValueError as exc:
                self._frame_calibration = None
                QMessageBox.warning(self, "坐标系求解失败", str(exc))
        else:
            self._frame_calibration = None
        self._refresh_calibration_status()
        self._rebuild_calibration_selection_markers()
        self._update_frame_preview()

    def _flip_frame_x(self) -> None:
        if self._frame_calibration is None:
            QMessageBox.information(self, "坐标系未完成", "请先完成原点、X 参考和 Z 参考")
            return
        self._frame_calibration = self._frame_calibration.flipped_x()
        self._refresh_calibration_status()
        self._update_frame_preview()
        self.statusBar().showMessage("已翻转 map +X；Y 自动按右手定律同步更新")

    def _flip_frame_z(self) -> None:
        if self._frame_calibration is None:
            QMessageBox.information(self, "坐标系未完成", "请先完成原点、X 参考和 Z 参考")
            return
        self._frame_calibration = self._frame_calibration.flipped_z()
        self._refresh_calibration_status()
        self._rebuild_calibration_selection_markers()
        self._update_frame_preview()
        self.statusBar().showMessage("已翻转 map +Z；Y 自动按右手定律同步更新")

    def _clear_frame_calibration(self) -> None:
        self._frame_origin = None
        self._x_reference_clicks = []
        self._x_fit = None
        self._z_fit = None
        self._z_reference_xy = None
        self._frame_calibration = None
        if self._interaction_mode in {"frame_origin", "frame_x", "frame_z"}:
            self._set_interaction_mode(None, "坐标系标定已清空")
        for item in self._calibration_selection_items + self._frame_preview_items:
            self._scene.removeItem(item)
        self._calibration_selection_items.clear()
        self._frame_preview_items.clear()
        self._refresh_calibration_status()

    def _refresh_calibration_status(self) -> None:
        if not hasattr(self, "_frame_status"):
            return
        if self._frame_origin is None:
            self._origin_status.setText("原点：未选择")
        else:
            p = self._frame_origin
            self._origin_status.setText(
                f"原点 source=[{p[0]:.3f}, {p[1]:.3f}, {p[2]:.3f}] m | "
                f"原点 Z=[{self._origin_z_min.value():.3f}, {self._origin_z_max.value():.3f}]"
            )
        if self._x_fit is None:
            self._x_axis_status.setText("X 参考：未拟合")
        else:
            d = self._x_fit.direction
            selection = self._x_fit.selection or {}
            z_window = selection.get("z_window_m", [None, None])
            self._x_axis_status.setText(
                f"X 拟合=[{d[0]:.4f}, {d[1]:.4f}, {d[2]:.4f}] | 点数 {self._x_fit.point_count:,} | "
                f"RMS {self._x_fit.rms_residual_m:.3f} m | 线性度 {self._x_fit.linearity_ratio:.1f} | "
                f"实际拟合 Z={z_window}"
            )
        if self._z_fit is None:
            self._z_axis_status.setText("Z 参考：未拟合")
        else:
            d = self._z_fit.direction
            selection = self._z_fit.selection or {}
            z_window = selection.get("z_window_m", [None, None])
            self._z_axis_status.setText(
                f"Z 拟合=[{d[0]:.4f}, {d[1]:.4f}, {d[2]:.4f}] | 点数 {self._z_fit.point_count:,} | "
                f"RMS {self._z_fit.rms_residual_m:.3f} m | 线性度 {self._z_fit.linearity_ratio:.1f} | "
                f"实际拟合 Z={z_window}"
            )
        if self._frame_calibration is None:
            ready = sum(value is not None for value in (self._frame_origin, self._x_fit, self._z_fit))
            self._frame_status.setText(f"坐标系：{ready}/3 条件完成")
        else:
            c = self._frame_calibration
            self._frame_status.setText(
                f"坐标系已求解 | 输入 X/Z 夹角 {c.orthogonality_input_deg:.2f}° | "
                "输出已正交化并满足右手定律 | ENU/UTM 尚未绑定"
            )

    def _add_fixed_marker(
        self,
        point: QPointF,
        text: str,
        color: QColor,
        target: list[QGraphicsItem],
        *,
        label_offset_px: tuple[float, float] = (8.0, -18.0),
    ) -> None:
        """Create a marker whose label offset is in screen pixels, not map metres."""
        marker = QGraphicsEllipseItem(-5.0, -5.0, 10.0, 10.0)
        marker.setPos(point)
        marker.setBrush(QBrush(color))
        marker.setPen(QPen(Qt.black, 1))
        marker.setFlag(QGraphicsItem.ItemIgnoresTransformations, True)
        marker.setZValue(30.0)
        label = QGraphicsSimpleTextItem(text, marker)
        label.setBrush(QBrush(color))
        label.setPos(QPointF(*label_offset_px))
        label.setZValue(1.0)
        self._scene.addItem(marker)
        target.append(marker)

    def _rebuild_calibration_selection_markers(self) -> None:
        for item in self._calibration_selection_items:
            self._scene.removeItem(item)
        self._calibration_selection_items.clear()
        if self._frame_origin is not None:
            origin_text = "O / +Z↑" if self._frame_calibration is not None else "O"
            self._add_fixed_marker(
                QPointF(float(self._frame_origin[0]), float(-self._frame_origin[1])),
                origin_text,
                QColor(255, 100, 220),
                self._calibration_selection_items,
                label_offset_px=(8.0, -20.0),
            )
        for index, (x, y) in enumerate(self._x_reference_clicks[:2], start=1):
            self._add_fixed_marker(
                QPointF(x, -y),
                f"X{index}",
                QColor(80, 220, 255),
                self._calibration_selection_items,
                label_offset_px=(8.0, 4.0) if index == 1 else (8.0, -20.0),
            )
        if self._z_reference_xy is not None:
            x, y = self._z_reference_xy
            self._add_fixed_marker(
                QPointF(x, -y),
                "Z柱",
                QColor(100, 160, 255),
                self._calibration_selection_items,
                label_offset_px=(8.0, 4.0),
            )

    def _update_frame_preview(self) -> None:
        for item in self._frame_preview_items:
            self._scene.removeItem(item)
        self._frame_preview_items.clear()
        if self._frame_calibration is None:
            return
        c = self._frame_calibration
        origin = c.origin_source_m
        base = QPointF(float(origin[0]), float(-origin[1]))
        rect = self._cloud_item.boundingRect()
        axis_length = max(2.0, 0.10 * max(float(rect.width()), float(rect.height())))
        for name, direction, color in (
            ("+X", c.x_axis_in_source, QColor(255, 80, 80)),
            ("+Y", c.y_axis_in_source, QColor(80, 230, 120)),
        ):
            end = QPointF(
                float(origin[0] + axis_length * direction[0]),
                float(-(origin[1] + axis_length * direction[1])),
            )
            line = QGraphicsLineItem(base.x(), base.y(), end.x(), end.y())
            pen = QPen(color)
            pen.setWidth(3)
            pen.setCosmetic(True)
            line.setPen(pen)
            line.setZValue(20.0)
            self._scene.addItem(line)
            self._frame_preview_items.append(line)
            self._add_fixed_marker(end, name, color, self._frame_preview_items)

    def _export_map_frame(self) -> Path | None:
        if self._frame_calibration is None:
            QMessageBox.information(self, "坐标系未完成", "请先完成原点、X 参考和 Z 参考")
            return None
        filename, _ = QFileDialog.getSaveFileName(
            self, "导出地图坐标系标定", "map_frame.yaml", "YAML 文件 (*.yaml *.yml)"
        )
        if not filename:
            return None
        try:
            path = self._frame_calibration.write_yaml(
                filename,
                source_frame_id="source_map",
                target_frame_id="map",
                source_asset=self._source_path.name if self._source_path else None,
            )
        except Exception as exc:
            QMessageBox.critical(self, "坐标系标定导出失败", str(exc))
            return None
        self.statusBar().showMessage(f"map_frame.yaml 已导出：{path}")
        return path

    # -------------------------------------------------------- navigation map
    def _navigation_config(self) -> GroundRelativeNavigationConfig:
        return GroundRelativeNavigationConfig(
            resolution_m=self._nav_resolution.value(),
            ground_quantile=self._nav_ground_quantile.value(),
            maximum_ground_fill_distance_m=self._nav_fill_distance.value(),
            ground_smoothing_radius_cells=self._nav_smoothing.value(),
            obstacle_min_height_m=self._nav_obstacle_min.value(),
            obstacle_max_height_m=self._nav_obstacle_max.value(),
            maximum_slope_deg=self._nav_max_slope.value(),
            maximum_step_m=self._nav_max_step.value(),
        )

    def _generate_navigation_preview(self) -> None:
        if self._cloud is None:
            QMessageBox.information(self, "尚未加载点云", "请先打开一个 PCD 点云文件")
            return
        if self._nav_thread is not None and self._nav_thread.isRunning():
            QMessageBox.information(self, "正在计算", "上一轮导航图派生尚未完成")
            return
        try:
            config = self._navigation_config()
            config.validate()
        except ValueError as exc:
            QMessageBox.warning(self, "导航参数无效", str(exc))
            return
        self._nav_status.setText("导航图：正在使用完整 PCD 计算局部地面与相对高度证据…")
        self._nav_thread = QThread(self)
        self._nav_worker = _NavigationWorker(self._cloud, config)
        self._nav_worker.moveToThread(self._nav_thread)
        self._nav_thread.started.connect(self._nav_worker.run)
        self._nav_worker.finished.connect(self._navigation_finished)
        self._nav_worker.failed.connect(self._navigation_failed)
        self._nav_worker.finished.connect(self._nav_thread.quit)
        self._nav_worker.failed.connect(self._nav_thread.quit)
        self._nav_thread.finished.connect(self._navigation_thread_finished)
        self._nav_thread.start()

    def _navigation_thread_finished(self) -> None:
        self._nav_worker = None
        self._nav_thread = None

    def _navigation_finished(self, result: NavigationMapResult) -> None:
        self._navigation_base_result = result
        self._apply_navigation_overrides_to_result()
        self._refresh_navigation_status()
        self._update_navigation_overlay()
        self.statusBar().showMessage("Ground-relative Navigation Map 预览已生成")

    def _navigation_failed(self, message: str) -> None:
        QMessageBox.critical(self, "导航图派生失败", message)
        self._nav_status.setText("导航图：派生失败")

    def _apply_navigation_overrides_to_result(self) -> None:
        if self._navigation_base_result is None:
            self._navigation_result = None
            return
        occupancy = apply_navigation_overrides(
            self._navigation_base_result, self._navigation_overrides
        )
        self._navigation_result = replace(
            self._navigation_base_result, occupancy=occupancy
        )

    def _refresh_navigation_status(self) -> None:
        if not hasattr(self, "_nav_status"):
            return
        if self._navigation_result is None:
            self._nav_status.setText(
                "导航图：尚未生成 | 计算使用完整已加载 PCD；显示 Z 不参与 | 未观测区域保持 UNKNOWN"
            )
            return
        result = self._navigation_result
        counts = result.counts()
        self._nav_status.setText(
            f"导航图：{result.width}×{result.height} @ {result.resolution_m:.2f} m | "
            f"FREE {counts['free']:,} | OCCUPIED {counts['occupied']:,} | UNKNOWN {counts['unknown']:,}\n"
            f"相对障碍高度=[{result.config.obstacle_min_height_m:.2f}, {result.config.obstacle_max_height_m:.2f}] m | "
            f"坡度≤{result.config.maximum_slope_deg:.1f}° | 台阶≤{result.config.maximum_step_m:.2f} m | "
            "坐标系=当前已加载 PCD"
        )

    def _update_navigation_overlay(self) -> None:
        if not hasattr(self, "_nav_layer"):
            return
        if self._navigation_result is None or not self._nav_overlay_visible.isChecked():
            self._navigation_preview_item.clear_result()
            return
        self._navigation_preview_item.set_result(
            self._navigation_result, str(self._nav_layer.currentData())
        )

    def _start_navigation_override(self) -> None:
        if self._cloud is None:
            QMessageBox.information(self, "尚未加载点云", "请先打开一个 PCD 点云文件")
            return
        self._nav_vertices.clear()
        self._update_navigation_override_polygon()
        self._set_interaction_mode(
            "nav_override",
            "Navigation Override：十字光标左键绘制多边形；它记录世界坐标，不直接改 PGM 像素",
        )

    def _append_navigation_override_vertex(self, x: float, y: float) -> None:
        self._nav_vertices.append((x, y))
        self._update_navigation_override_polygon()
        self.statusBar().showMessage(
            f"Override 顶点 {len(self._nav_vertices)}：X={x:.3f} m，Y={y:.3f} m"
        )

    def _update_navigation_override_polygon(self) -> None:
        if self._nav_polygon_item is not None:
            self._scene.removeItem(self._nav_polygon_item)
            self._nav_polygon_item = None
        for item in self._nav_vertex_items:
            self._scene.removeItem(item)
        self._nav_vertex_items.clear()
        if not self._nav_vertices:
            return
        scene_points = [QPointF(x, -y) for x, y in self._nav_vertices]
        color = QColor(255, 80, 200)
        self._nav_polygon_item = QGraphicsPolygonItem(QPolygonF(scene_points))
        pen = QPen(color)
        pen.setWidth(2)
        pen.setCosmetic(True)
        self._nav_polygon_item.setPen(pen)
        self._nav_polygon_item.setBrush(QBrush(QColor(255, 80, 200, 30)))
        self._nav_polygon_item.setZValue(12.0)
        self._scene.addItem(self._nav_polygon_item)
        for index, point in enumerate(scene_points, start=1):
            self._add_fixed_marker(
                point,
                f"N{index}",
                color,
                self._nav_vertex_items,
                label_offset_px=(8.0, -18.0),
            )

    def _clear_navigation_override_draft(self) -> None:
        self._nav_vertices.clear()
        if self._nav_polygon_item is not None:
            self._scene.removeItem(self._nav_polygon_item)
            self._nav_polygon_item = None
        for item in self._nav_vertex_items:
            self._scene.removeItem(item)
        self._nav_vertex_items.clear()
        if self._interaction_mode == "nav_override":
            self._set_interaction_mode(None, "Navigation Override 绘制已结束")

    def _finish_navigation_override(self) -> None:
        if len(self._nav_vertices) < 3:
            QMessageBox.warning(self, "Override 未完成", "至少需要 3 个顶点")
            return
        mode = str(self._nav_override_mode.currentData())
        self._navigation_overrides.append(
            {
                "mode": mode,
                "polygon_xy": [[float(x), float(y)] for x, y in self._nav_vertices],
                "reason": "workbench_manual_override",
            }
        )
        self._clear_navigation_override_draft()
        self._refresh_navigation_override_list()
        self._apply_navigation_overrides_to_result()
        self._refresh_navigation_status()
        self._update_navigation_overlay()

    def _refresh_navigation_override_list(self) -> None:
        if not hasattr(self, "_nav_override_list"):
            return
        self._nav_override_list.clear()
        if not self._navigation_overrides:
            self._nav_override_list.addItem("（暂无 Override）")
            return
        for index, override in enumerate(self._navigation_overrides, start=1):
            mode = str(override["mode"])
            self._nav_override_list.addItem(
                f"{index:02d} | {_OVERRIDE_NAMES.get(mode, mode)} | "
                f"{len(override.get('polygon_xy', []))} 个顶点"
            )

    def _undo_navigation_override(self) -> None:
        if self._navigation_overrides:
            self._navigation_overrides.pop()
        self._refresh_navigation_override_list()
        self._apply_navigation_overrides_to_result()
        self._refresh_navigation_status()
        self._update_navigation_overlay()

    def _clear_navigation_overrides(self) -> None:
        self._navigation_overrides.clear()
        self._clear_navigation_override_draft()
        self._refresh_navigation_override_list()
        self._apply_navigation_overrides_to_result()
        self._refresh_navigation_status()
        self._update_navigation_overlay()

    def _clear_navigation_state(self, *, clear_overrides: bool) -> None:
        self._navigation_base_result = None
        self._navigation_result = None
        self._navigation_preview_item.clear_result()
        self._clear_navigation_override_draft()
        if clear_overrides:
            self._navigation_overrides.clear()
        self._refresh_navigation_override_list()
        self._refresh_navigation_status()

    def _export_navigation_map(self) -> Path | None:
        if self._navigation_result is None:
            QMessageBox.information(self, "导航图未生成", "请先生成 Ground-relative 导航图预览")
            return None
        parent = QFileDialog.getExistingDirectory(self, "选择 Navigation Map 派生记录父目录")
        if not parent:
            return None
        run_name, accepted = QInputDialog.getText(
            self,
            "Navigation Map 记录名称",
            "请输入新的不可变派生目录名：",
            text="agt_navigation_map_run",
        )
        run_name = run_name.strip()
        if not accepted or not run_name:
            return None
        if Path(run_name).name != run_name or run_name in {".", ".."}:
            QMessageBox.warning(self, "名称无效", "只能使用单层目录名")
            return None
        destination = Path(parent) / run_name
        try:
            output = write_navigation_map_derivation(
                self._navigation_result,
                destination,
                source_asset=self._source_path.name if self._source_path else None,
                frame_id="source_map",
                overrides=self._navigation_overrides,
            )
        except Exception as exc:
            QMessageBox.critical(self, "Navigation Map 导出失败", str(exc))
            return None
        self.statusBar().showMessage(f"Navigation Map 派生记录已导出：{output}")
        return output

    # --------------------------------------------------------------- export/run
    def _export_recipe(self) -> Path | None:
        if not self._recipe.operations:
            QMessageBox.information(self, "处理流程为空", "请至少添加一项处理操作")
            return None
        filename, _ = QFileDialog.getSaveFileName(
            self,
            "导出 V25-12B Recipe",
            f"{self._recipe.recipe_id}.yaml",
            "YAML 文件 (*.yaml *.yml)",
        )
        if not filename:
            return None
        try:
            path = self._recipe.write_yaml(filename)
        except Exception as exc:
            QMessageBox.critical(self, "Recipe 导出失败", str(exc))
            return None
        self.statusBar().showMessage(f"Recipe 已导出：{path}")
        return path

    def _run_processing(self) -> None:
        if self._source_path is None:
            QMessageBox.information(self, "尚未加载点云", "请先打开一个 PCD 点云文件")
            return
        if not self._recipe.operations:
            QMessageBox.information(self, "处理流程为空", "请至少添加一项处理操作")
            return
        parent = QFileDialog.getExistingDirectory(self, "选择新的不可变处理记录所在父目录")
        if not parent:
            return
        run_name, accepted = QInputDialog.getText(
            self, "处理记录名称", "请输入新的处理记录目录名：", text="agt_workbench_run"
        )
        run_name = run_name.strip()
        if not accepted or not run_name:
            return
        if Path(run_name).name != run_name or run_name in {".", ".."}:
            QMessageBox.warning(self, "名称无效", "处理记录名称只能是单层目录名")
            return
        destination = Path(parent) / run_name
        if destination.exists():
            QMessageBox.warning(self, "不可变输出冲突", "该目录已经存在，请使用新的名称")
            return
        recipe_path = destination.parent / f".{destination.name}.recipe.yaml"
        try:
            self._recipe.write_yaml(recipe_path)
        except Exception as exc:
            QMessageBox.critical(self, "Recipe 生成失败", str(exc))
            return

        self.statusBar().showMessage("正在对完整分辨率 PCD 执行处理…")
        self._thread = QThread(self)
        self._worker = _ProcessingWorker(self._source_path, recipe_path, destination)
        self._worker.moveToThread(self._thread)
        self._thread.started.connect(self._worker.run)
        self._worker.finished.connect(lambda result: self._processing_finished(result, recipe_path))
        self._worker.failed.connect(lambda message: self._processing_failed(message, recipe_path))
        self._worker.finished.connect(self._thread.quit)
        self._worker.failed.connect(self._thread.quit)
        self._thread.start()

    def _processing_finished(self, result, recipe_path: Path) -> None:
        recipe_path.unlink(missing_ok=True)
        self.statusBar().showMessage(
            f"处理 PASS：{result.input_points:,} → {result.output_points:,} 点"
        )
        answer = QMessageBox.question(
            self,
            "处理完成",
            f"不可变处理记录已写入：\n{result.run_dir}\n\n是否加载处理后的 PCD 进行可视检查？",
        )
        if answer == QMessageBox.Yes:
            try:
                cloud = read_pcd(result.output_path)
                self._cloud = cloud
                self._source_path = Path(result.output_path).resolve()
                self._cloud_item.set_cloud(
                    cloud, sample_limit=int(self._sample_limit.currentData())
                )
                self._update_display_options()
                self._clear_navigation_state(clear_overrides=True)
                self._fit_cloud()
            except Exception as exc:
                QMessageBox.warning(self, "处理结果加载失败", str(exc))

    def _processing_failed(self, message: str, recipe_path: Path) -> None:
        recipe_path.unlink(missing_ok=True)
        QMessageBox.critical(self, "处理失败", message)
        self.statusBar().showMessage("处理失败")


def main(argv=None) -> int:
    app = QApplication(list(sys.argv if argv is None else argv))
    window = MapWorkbenchWindow()
    window.show()
    return app.exec_()
