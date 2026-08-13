"""AGT Map Workbench MVP main window.

This is an offline authoring client. It never publishes ROS topics or TF and
never edits READY assets in place. Visual operations are exported as a V25-12B
recipe and formal processing is delegated to agt_offline_assets.
"""

from __future__ import annotations

import sys
from pathlib import Path

import numpy as np
from PyQt5.QtCore import QObject, QPointF, QThread, Qt, pyqtSignal
from PyQt5.QtGui import QColor, QBrush, QPen, QPolygonF
from PyQt5.QtWidgets import (
    QApplication,
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
    QSplitter,
    QTabWidget,
    QVBoxLayout,
    QWidget,
)

from agt_offline_assets import process_pointcloud, read_pcd

from .frame_calibration import (
    AxisFit,
    MapFrameCalibration,
    fit_horizontal_axis_from_corridor,
    fit_vertical_axis_from_cylinder,
    nearest_xyz_in_window,
    solve_map_frame,
)
from .model import WorkbenchRecipeModel
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
        except Exception as exc:  # UI boundary reports backend error text
            self.failed.emit(str(exc))
            return
        self.finished.emit(result)


class MapWorkbenchWindow(QMainWindow):
    def __init__(self) -> None:
        super().__init__()
        self.setWindowTitle("AGT 地图工作台 — V25-12C")
        self.resize(1460, 920)

        self._source_path: Path | None = None
        self._cloud = None
        self._recipe = WorkbenchRecipeModel()
        self._vertices: list[tuple[float, float]] = []
        self._polygon_item: QGraphicsPolygonItem | None = None
        self._vertex_items: list[QGraphicsItem] = []
        self._interaction_mode: str | None = None

        self._frame_origin: np.ndarray | None = None
        self._x_reference_clicks: list[tuple[float, float]] = []
        self._x_fit: AxisFit | None = None
        self._z_fit: AxisFit | None = None
        self._frame_calibration: MapFrameCalibration | None = None
        self._calibration_selection_items: list[QGraphicsItem] = []
        self._frame_preview_items: list[QGraphicsItem] = []

        self._thread: QThread | None = None
        self._worker: _ProcessingWorker | None = None

        self._scene = QGraphicsScene(self)
        self._cloud_item = PointCloudItem()
        self._scene.addItem(self._cloud_item)
        self._view = PointCloudView(self._scene)
        self._view.mapClicked.connect(self._on_map_clicked)

        controls = self._build_controls()
        splitter = QSplitter()
        splitter.addWidget(self._view)
        splitter.addWidget(controls)
        splitter.setStretchFactor(0, 1)
        splitter.setSizes([1010, 450])
        self.setCentralWidget(splitter)
        self.statusBar().showMessage("请打开一个 PCD 点云文件")
        self._refresh_operations()
        self._refresh_vertex_list()
        self._refresh_z_status()
        self._refresh_calibration_status()

    def _build_controls(self) -> QWidget:
        panel = QWidget()
        panel.setMinimumWidth(420)
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

        layout.addWidget(QLabel("显示 / 选择 Z 高度范围（m）"))
        z_row = QHBoxLayout()
        self._z_min = QDoubleSpinBox()
        self._z_max = QDoubleSpinBox()
        self._z_min.setPrefix("最小 Z：")
        self._z_max.setPrefix("最大 Z：")
        for box in (self._z_min, self._z_max):
            box.setDecimals(3)
            box.setRange(-10000.0, 10000.0)
            box.setSingleStep(0.1)
            box.valueChanged.connect(self._update_z_window)
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
        layout.addWidget(tabs, 1)
        return panel

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
        self._vertex_list.setMaximumHeight(125)
        layout.addWidget(self._vertex_list)

        layout.addWidget(QLabel("处理流程 / Recipe 执行顺序"))
        self._operations = QListWidget()
        layout.addWidget(self._operations, 1)

        undo_operation = QPushButton("撤销上一项操作")
        undo_operation.clicked.connect(self._undo_operation)
        clear_operations = QPushButton("清空全部操作")
        clear_operations.clicked.connect(self._clear_operations)
        operation_row = QHBoxLayout()
        operation_row.addWidget(undo_operation)
        operation_row.addWidget(clear_operations)
        layout.addLayout(operation_row)

        export_button = QPushButton("导出处理 Recipe YAML")
        export_button.clicked.connect(self._export_recipe)
        process_button = QPushButton("执行完整分辨率不可变处理")
        process_button.clicked.connect(self._run_processing)
        layout.addWidget(export_button)
        layout.addWidget(process_button)
        return tab

    def _build_frame_tab(self) -> QWidget:
        tab = QWidget()
        layout = QVBoxLayout(tab)
        intro = QLabel(
            "Map Frame Calibration：用场景稳定结构定义便于导航的右手坐标系。\n"
            "1 原点吸附到当前 Z 窗口内最近 3D 点\n"
            "2 X 轴由墙壁参考段走廊点云拟合\n"
            "3 Z 轴由立柱中心圆柱点云拟合\n"
            "ENU / UTM 地理配准后续单独绑定，不强制 map 的 X 指向正东"
        )
        intro.setWordWrap(True)
        layout.addWidget(intro)

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
        self._x_corridor_width.setSingleStep(0.05)
        self._x_corridor_width.setValue(0.30)
        self._z_pillar_radius = QDoubleSpinBox()
        self._z_pillar_radius.setPrefix("Z 半径：")
        self._z_pillar_radius.setSuffix(" m")
        self._z_pillar_radius.setDecimals(2)
        self._z_pillar_radius.setRange(0.05, 2.0)
        self._z_pillar_radius.setSingleStep(0.05)
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
        self._z_min.blockSignals(True)
        self._z_max.blockSignals(True)
        self._z_min.setValue(minimum)
        self._z_max.setValue(maximum)
        self._z_min.blockSignals(False)
        self._z_max.blockSignals(False)
        self._cloud_item.set_z_window(minimum, maximum)
        self._recipe = WorkbenchRecipeModel(
            recipe_id=f"{self._source_path.stem}_workbench_draft",
            frame_id="map",
        )
        self._refresh_operations()
        self._clear_polygon()
        self._clear_frame_calibration()
        self._source_label.setText(
            f"当前点云：{self._source_path}\n"
            f"源点数：{int(cloud.points.shape[0]):,} | PCD 数据模式：{cloud.data_mode}"
        )
        self._refresh_z_status()
        self._fit_cloud()
        self.statusBar().showMessage(
            "点云加载完成；显示采样只影响预览，Recipe 与正式处理仍使用完整点云"
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
                self.statusBar().showMessage("当前 PCD 没有可用 scalar intensity，显示将回退为高度着色")

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
            f"源 PCD：{source_count:,} 点 | 当前 Z=[{self._z_min.value():.3f}, {self._z_max.value():.3f}] m"
        )

    def _update_z_window(self) -> None:
        minimum = self._z_min.value()
        maximum = self._z_max.value()
        if maximum < minimum:
            self._z_status.setText("Z 范围无效：最大 Z 必须大于或等于最小 Z")
            self.statusBar().showMessage("Z 范围无效，点云显示保持上一次有效范围")
            return
        self._cloud_item.set_z_window(minimum, maximum)
        self._refresh_z_status()
        self.statusBar().showMessage(
            f"Z 显示/选择范围已更新：[{minimum:.3f}, {maximum:.3f}] m，"
            f"当前可见采样点 {self._cloud_item.visible_sample_count():,}"
        )

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

    def _start_polygon(self) -> None:
        if self._cloud is None:
            QMessageBox.information(self, "尚未加载点云", "请先打开一个 PCD 点云文件")
            return
        self._vertices.clear()
        self._update_polygon_item()
        self._set_interaction_mode(
            "polygon",
            "多边形绘制模式：十字光标左键依次添加顶点，黄色编号表示点击顺序",
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
            polygon = QPolygonF(scene_points)
            self._polygon_item = QGraphicsPolygonItem(polygon)
            pen = QPen(QColor(255, 220, 0))
            pen.setWidth(2)
            pen.setCosmetic(True)
            self._polygon_item.setPen(pen)
            self._polygon_item.setBrush(QBrush(QColor(255, 220, 0, 28)))
            self._polygon_item.setZValue(10.0)
            self._scene.addItem(self._polygon_item)
            for index, point in enumerate(scene_points, start=1):
                self._add_fixed_marker(point, str(index), QColor(255, 220, 0), self._vertex_items)
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
            f"Z=[{self._z_min.value():.3f}, {self._z_max.value():.3f}] m"
        )

    def _finish_polygon(self) -> None:
        if len(self._vertices) < 3:
            QMessageBox.warning(self, "多边形未完成", "至少需要 3 个顶点")
            return
        if self._z_max.value() < self._z_min.value():
            QMessageBox.warning(self, "Z 范围无效", "最大 Z 必须大于或等于最小 Z")
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

    @staticmethod
    def _cloud_fields(cloud):
        return cloud.points["x"], cloud.points["y"], cloud.points["z"]

    def _require_cloud_for_calibration(self) -> bool:
        if self._cloud is None:
            QMessageBox.information(self, "尚未加载点云", "请先打开一个 PCD 点云文件")
            return False
        if self._z_max.value() < self._z_min.value():
            QMessageBox.warning(self, "Z 范围无效", "请先设置有效的 Z 选择范围")
            return False
        return True

    def _select_frame_origin(self) -> None:
        if not self._require_cloud_for_calibration():
            return
        self._set_interaction_mode(
            "frame_origin",
            "原点选择：点击物理意义明确的墙角/柱脚；将吸附到当前 Z 范围内最近 3D 点",
        )

    def _pick_frame_origin(self, x: float, y: float) -> None:
        fields = self._cloud_fields(self._cloud)
        try:
            point = nearest_xyz_in_window(
                *fields,
                (x, y),
                z_min=self._z_min.value(),
                z_max=self._z_max.value(),
            )
        except ValueError as exc:
            QMessageBox.warning(self, "原点选择失败", str(exc))
            return
        self._frame_origin = point
        self._set_interaction_mode(None, "地图原点已吸附到最近 3D 点")
        self._rebuild_calibration_selection_markers()
        self._recompute_map_frame()

    def _select_x_reference(self) -> None:
        if not self._require_cloud_for_calibration():
            return
        self._x_reference_clicks.clear()
        self._x_fit = None
        self._frame_calibration = None
        self._set_interaction_mode(
            "frame_x",
            "X 参考墙：沿希望的 +X 方向依次点击墙壁起点和终点",
        )
        self._rebuild_calibration_selection_markers()
        self._refresh_calibration_status()

    def _pick_x_reference(self, x: float, y: float) -> None:
        self._x_reference_clicks.append((x, y))
        self._rebuild_calibration_selection_markers()
        if len(self._x_reference_clicks) == 1:
            self.statusBar().showMessage("已选择 X 参考起点；请沿希望的 +X 方向点击墙壁终点")
            return
        start, end = self._x_reference_clicks[:2]
        fields = self._cloud_fields(self._cloud)
        try:
            self._x_fit = fit_horizontal_axis_from_corridor(
                *fields,
                start,
                end,
                z_min=self._z_min.value(),
                z_max=self._z_max.value(),
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
        self._z_fit = None
        self._frame_calibration = None
        self._set_interaction_mode(
            "frame_z",
            "Z 参考立柱：点击可靠立柱的 XY 中心；将提取当前 Z 范围内圆柱点云拟合主方向",
        )
        self._refresh_calibration_status()

    def _pick_z_reference(self, x: float, y: float) -> None:
        fields = self._cloud_fields(self._cloud)
        try:
            self._z_fit = fit_vertical_axis_from_cylinder(
                *fields,
                (x, y),
                z_min=self._z_min.value(),
                z_max=self._z_max.value(),
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
        self._update_frame_preview()
        self.statusBar().showMessage("已翻转 map +Z；Y 自动按右手定律同步更新")

    def _clear_frame_calibration(self) -> None:
        self._frame_origin = None
        self._x_reference_clicks = []
        self._x_fit = None
        self._z_fit = None
        self._frame_calibration = None
        self._z_reference_xy = None
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
                f"原点 source = [{p[0]:.3f}, {p[1]:.3f}, {p[2]:.3f}] m"
            )
        if self._x_fit is None:
            self._x_axis_status.setText("X 参考：未拟合")
        else:
            d = self._x_fit.direction
            self._x_axis_status.setText(
                f"X 拟合 = [{d[0]:.4f}, {d[1]:.4f}, {d[2]:.4f}] | "
                f"点数 {self._x_fit.point_count:,} | RMS {self._x_fit.rms_residual_m:.3f} m | "
                f"线性度 {self._x_fit.linearity_ratio:.1f}"
            )
        if self._z_fit is None:
            self._z_axis_status.setText("Z 参考：未拟合")
        else:
            d = self._z_fit.direction
            self._z_axis_status.setText(
                f"Z 拟合 = [{d[0]:.4f}, {d[1]:.4f}, {d[2]:.4f}] | "
                f"点数 {self._z_fit.point_count:,} | RMS {self._z_fit.rms_residual_m:.3f} m | "
                f"线性度 {self._z_fit.linearity_ratio:.1f}"
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
    ) -> None:
        marker = QGraphicsEllipseItem(-5.0, -5.0, 10.0, 10.0)
        marker.setPos(point)
        marker.setBrush(QBrush(color))
        marker.setPen(QPen(Qt.black, 1))
        marker.setFlag(QGraphicsItem.ItemIgnoresTransformations, True)
        marker.setZValue(30.0)
        label = QGraphicsSimpleTextItem(text)
        label.setBrush(QBrush(color))
        label.setPos(point + QPointF(7.0, -16.0))
        label.setFlag(QGraphicsItem.ItemIgnoresTransformations, True)
        label.setZValue(31.0)
        self._scene.addItem(marker)
        self._scene.addItem(label)
        target.extend((marker, label))

    def _rebuild_calibration_selection_markers(self) -> None:
        for item in self._calibration_selection_items:
            self._scene.removeItem(item)
        self._calibration_selection_items.clear()
        if self._frame_origin is not None:
            self._add_fixed_marker(
                QPointF(float(self._frame_origin[0]), float(-self._frame_origin[1])),
                "O",
                QColor(255, 80, 220),
                self._calibration_selection_items,
            )
        for index, (x, y) in enumerate(self._x_reference_clicks[:2], start=1):
            self._add_fixed_marker(
                QPointF(x, -y),
                f"X{index}",
                QColor(80, 220, 255),
                self._calibration_selection_items,
            )
        if getattr(self, "_z_reference_xy", None) is not None:
            x, y = self._z_reference_xy
            self._add_fixed_marker(
                QPointF(x, -y),
                "Z柱",
                QColor(100, 160, 255),
                self._calibration_selection_items,
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
        axes = (
            ("+X", c.x_axis_in_source, QColor(255, 80, 80)),
            ("+Y", c.y_axis_in_source, QColor(80, 230, 120)),
        )
        for name, direction, color in axes:
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
        self._add_fixed_marker(base, "O / +Z↑", QColor(90, 150, 255), self._frame_preview_items)

    def _export_map_frame(self) -> Path | None:
        if self._frame_calibration is None:
            QMessageBox.information(self, "坐标系未完成", "请先完成原点、X 参考和 Z 参考")
            return None
        filename, _ = QFileDialog.getSaveFileName(
            self,
            "导出地图坐标系标定",
            "map_frame.yaml",
            "YAML 文件 (*.yaml *.yml)",
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
        parent = QFileDialog.getExistingDirectory(
            self, "选择新的不可变处理记录所在父目录"
        )
        if not parent:
            return
        run_name, accepted = QInputDialog.getText(
            self,
            "处理记录名称",
            "请输入新的处理记录目录名：",
            text="agt_workbench_run",
        )
        run_name = run_name.strip()
        if not accepted or not run_name:
            return
        if Path(run_name).name != run_name or run_name in {".", ".."}:
            QMessageBox.warning(self, "名称无效", "处理记录名称只能是单层目录名")
            return
        destination = Path(parent) / run_name
        if destination.exists():
            QMessageBox.warning(self, "不可变输出冲突", "该处理记录目录已经存在，请使用新的名称")
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
                self._cloud_item.set_cloud(
                    cloud, sample_limit=int(self._sample_limit.currentData())
                )
                self._update_display_options()
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
