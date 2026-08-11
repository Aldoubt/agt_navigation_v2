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
from PyQt5.QtGui import QPen, QPolygonF
from PyQt5.QtWidgets import (
    QApplication,
    QComboBox,
    QDoubleSpinBox,
    QFileDialog,
    QGraphicsPolygonItem,
    QGraphicsScene,
    QHBoxLayout,
    QInputDialog,
    QLabel,
    QListWidget,
    QMainWindow,
    QMessageBox,
    QPushButton,
    QSplitter,
    QVBoxLayout,
    QWidget,
)

from agt_offline_assets import process_pointcloud, read_pcd

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
        self.setWindowTitle("AGT 地图工作台 — V25-12C MVP")
        self.resize(1400, 900)

        self._source_path: Path | None = None
        self._cloud = None
        self._recipe = WorkbenchRecipeModel()
        self._vertices: list[tuple[float, float]] = []
        self._polygon_item: QGraphicsPolygonItem | None = None
        self._thread: QThread | None = None
        self._worker: _ProcessingWorker | None = None

        self._scene = QGraphicsScene(self)
        self._cloud_item = PointCloudItem()
        self._scene.addItem(self._cloud_item)
        self._view = PointCloudView(self._scene)
        self._view.mapClicked.connect(self._append_vertex)

        controls = self._build_controls()
        splitter = QSplitter()
        splitter.addWidget(self._view)
        splitter.addWidget(controls)
        splitter.setStretchFactor(0, 1)
        splitter.setSizes([1050, 350])
        self.setCentralWidget(splitter)
        self.statusBar().showMessage("请打开一个 PCD 点云文件")

    def _build_controls(self) -> QWidget:
        panel = QWidget()
        layout = QVBoxLayout(panel)

        open_button = QPushButton("打开 PCD 点云")
        open_button.clicked.connect(self._open_pcd)
        layout.addWidget(open_button)

        self._source_label = QLabel("尚未加载点云")
        self._source_label.setWordWrap(True)
        layout.addWidget(self._source_label)

        layout.addWidget(QLabel("显示 / 编辑 Z 高度范围（m）"))
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

        layout.addWidget(QLabel("处理流程 / Recipe 操作历史"))
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
        layout.addWidget(export_button)

        process_button = QPushButton("执行完整分辨率不可变处理")
        process_button.clicked.connect(self._run_processing)
        layout.addWidget(process_button)

        fit_button = QPushButton("点云适配窗口")
        fit_button.clicked.connect(self._fit_cloud)
        layout.addWidget(fit_button)
        return panel

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
        self._cloud_item.set_cloud(cloud)
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
        self._source_label.setText(
            f"当前点云：{self._source_path}\n"
            f"点数：{int(cloud.points.shape[0]):,} | PCD 数据模式：{cloud.data_mode}"
        )
        self._fit_cloud()
        self.statusBar().showMessage("点云加载完成；界面显示使用确定性采样，正式处理仍使用完整点云")

    def _fit_cloud(self) -> None:
        rect = self._cloud_item.boundingRect()
        if not rect.isNull():
            self._view.fitInView(rect, Qt.KeepAspectRatio)

    def _update_z_window(self) -> None:
        minimum = self._z_min.value()
        maximum = self._z_max.value()
        if maximum >= minimum:
            self._cloud_item.set_z_window(minimum, maximum)

    def _start_polygon(self) -> None:
        if self._cloud is None:
            QMessageBox.information(self, "尚未加载点云", "请先打开一个 PCD 点云文件")
            return
        self._vertices.clear()
        self._update_polygon_item()
        self._view.set_authoring_enabled(True)
        self.statusBar().showMessage("多边形绘制模式：鼠标左键依次添加顶点，完成后点击“完成多边形并加入处理流程”")

    def _append_vertex(self, x: float, y: float) -> None:
        self._vertices.append((x, y))
        self._update_polygon_item()

    def _undo_vertex(self) -> None:
        if self._vertices:
            self._vertices.pop()
            self._update_polygon_item()

    def _clear_polygon(self) -> None:
        self._vertices.clear()
        if self._polygon_item is not None:
            self._scene.removeItem(self._polygon_item)
            self._polygon_item = None
        self._view.set_authoring_enabled(False)

    def _update_polygon_item(self) -> None:
        if self._polygon_item is not None:
            self._scene.removeItem(self._polygon_item)
            self._polygon_item = None
        if not self._vertices:
            return
        scene_points = [QPointF(x, -y) for x, y in self._vertices]
        polygon = QPolygonF(scene_points)
        self._polygon_item = QGraphicsPolygonItem(polygon)
        self._polygon_item.setPen(QPen(Qt.white, 0))
        self._polygon_item.setZValue(10.0)
        self._scene.addItem(self._polygon_item)

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
        self._operations.clear()
        for index, operation in enumerate(self._recipe.operations):
            params = operation.parameters
            operation_name = _OPERATION_NAMES.get(operation.type, operation.type)
            if operation.type in {"crop_polygon", "delete_polygon"}:
                detail = (
                    f"{len(params.get('polygon_xy', []))} 个顶点，"
                    f"Z=[{params.get('z_min'):.2f}, {params.get('z_max'):.2f}] m"
                )
            else:
                detail = str(params)
            self._operations.addItem(
                f"{index + 1}. {operation_name}（{operation.type}）— {detail}"
            )

    def _undo_operation(self) -> None:
        self._recipe.undo()
        self._refresh_operations()

    def _clear_operations(self) -> None:
        self._recipe.clear()
        self._refresh_operations()

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
                self._cloud_item.set_cloud(cloud)
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
