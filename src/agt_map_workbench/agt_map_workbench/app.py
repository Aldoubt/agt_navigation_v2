"""AGT Map Workbench MVP main window.

This is an offline authoring client.  It never publishes ROS topics or TF and
never edits READY assets in place.  Visual operations are exported as a V25-12B
recipe and formal processing is delegated to agt_offline_assets.
"""

from __future__ import annotations

import sys
from pathlib import Path

from PyQt5.QtCore import QObject, QThread, pyqtSignal
from PyQt5.QtGui import QPen, QPolygonF
from PyQt5.QtCore import QPointF, Qt
from PyQt5.QtWidgets import (
    QApplication,
    QComboBox,
    QDoubleSpinBox,
    QFileDialog,
    QGraphicsPolygonItem,
    QGraphicsScene,
    QHBoxLayout,
    QLabel,
    QListWidget,
    QMainWindow,
    QMessageBox,
    QPushButton,
    QSplitter,
    QVBoxLayout,
    QWidget,
)

from agt_offline_assets import process_pointcloud, read_pcd, summarize_pointcloud

from .model import WorkbenchRecipeModel
from .view import PointCloudItem, PointCloudView


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
        self.setWindowTitle("AGT Map Workbench — V25-12C MVP")
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
        self.statusBar().showMessage("Open a PCD to begin")

    def _build_controls(self) -> QWidget:
        panel = QWidget()
        layout = QVBoxLayout(panel)

        open_button = QPushButton("Open PCD")
        open_button.clicked.connect(self._open_pcd)
        layout.addWidget(open_button)

        self._source_label = QLabel("No source")
        self._source_label.setWordWrap(True)
        layout.addWidget(self._source_label)

        layout.addWidget(QLabel("Visible / authored Z range"))
        z_row = QHBoxLayout()
        self._z_min = QDoubleSpinBox()
        self._z_max = QDoubleSpinBox()
        for box in (self._z_min, self._z_max):
            box.setDecimals(3)
            box.setRange(-10000.0, 10000.0)
            box.setSingleStep(0.1)
            box.valueChanged.connect(self._update_z_window)
        z_row.addWidget(self._z_min)
        z_row.addWidget(self._z_max)
        layout.addLayout(z_row)

        self._mode = QComboBox()
        self._mode.addItem("Delete selected volume", "delete")
        self._mode.addItem("Keep selected volume (crop)", "crop")
        layout.addWidget(self._mode)

        start_button = QPushButton("Start polygon")
        start_button.clicked.connect(self._start_polygon)
        finish_button = QPushButton("Finish polygon → recipe")
        finish_button.clicked.connect(self._finish_polygon)
        undo_vertex = QPushButton("Undo vertex")
        undo_vertex.clicked.connect(self._undo_vertex)
        author_row = QHBoxLayout()
        author_row.addWidget(start_button)
        author_row.addWidget(finish_button)
        layout.addLayout(author_row)
        layout.addWidget(undo_vertex)

        layout.addWidget(QLabel("Recipe operations"))
        self._operations = QListWidget()
        layout.addWidget(self._operations, 1)

        undo_operation = QPushButton("Undo operation")
        undo_operation.clicked.connect(self._undo_operation)
        clear_operations = QPushButton("Clear operations")
        clear_operations.clicked.connect(self._clear_operations)
        operation_row = QHBoxLayout()
        operation_row.addWidget(undo_operation)
        operation_row.addWidget(clear_operations)
        layout.addLayout(operation_row)

        export_button = QPushButton("Export recipe YAML")
        export_button.clicked.connect(self._export_recipe)
        layout.addWidget(export_button)

        process_button = QPushButton("Run immutable processing")
        process_button.clicked.connect(self._run_processing)
        layout.addWidget(process_button)

        fit_button = QPushButton("Fit cloud")
        fit_button.clicked.connect(self._fit_cloud)
        layout.addWidget(fit_button)
        return panel

    def _open_pcd(self) -> None:
        filename, _ = QFileDialog.getOpenFileName(self, "Open PCD", "", "PCD files (*.pcd)")
        if not filename:
            return
        try:
            cloud = read_pcd(filename)
            summary = summarize_pointcloud(cloud)
        except Exception as exc:
            QMessageBox.critical(self, "Open failed", str(exc))
            return
        self._source_path = Path(filename).resolve()
        self._cloud = cloud
        self._cloud_item.set_points(cloud.xyz())
        bounds = summary.get("bounds_xyz")
        if bounds:
            minimum = float(bounds["min"][2])
            maximum = float(bounds["max"][2])
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
            f"{self._source_path}\n{summary['point_count']:,} points | {summary['data_mode']}"
        )
        self._fit_cloud()
        self.statusBar().showMessage("PCD loaded; display is deterministically sampled")

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
            QMessageBox.information(self, "No cloud", "Open a PCD first")
            return
        self._vertices.clear()
        self._update_polygon_item()
        self._view.set_authoring_enabled(True)
        self.statusBar().showMessage("Polygon authoring: left-click vertices; Finish when done")

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
            QMessageBox.warning(self, "Incomplete polygon", "At least three vertices are required")
            return
        if self._z_max.value() < self._z_min.value():
            QMessageBox.warning(self, "Invalid Z", "Z max must be >= Z min")
            return
        try:
            self._recipe.add_polygon_volume(
                self._vertices,
                z_min=self._z_min.value(),
                z_max=self._z_max.value(),
                delete=self._mode.currentData() == "delete",
            )
        except ValueError as exc:
            QMessageBox.warning(self, "Invalid operation", str(exc))
            return
        self._refresh_operations()
        self._clear_polygon()
        self.statusBar().showMessage("Operation added to replayable recipe")

    def _refresh_operations(self) -> None:
        self._operations.clear()
        for index, operation in enumerate(self._recipe.operations):
            params = operation.parameters
            if operation.type in {"crop_polygon", "delete_polygon"}:
                detail = f"{len(params.get('polygon_xy', []))} vertices, z=[{params.get('z_min'):.2f}, {params.get('z_max'):.2f}]"
            else:
                detail = str(params)
            self._operations.addItem(f"{index + 1}. {operation.type} — {detail}")

    def _undo_operation(self) -> None:
        self._recipe.undo()
        self._refresh_operations()

    def _clear_operations(self) -> None:
        self._recipe.clear()
        self._refresh_operations()

    def _export_recipe(self) -> Path | None:
        if not self._recipe.operations:
            QMessageBox.information(self, "Empty recipe", "Add at least one operation first")
            return None
        filename, _ = QFileDialog.getSaveFileName(
            self,
            "Export V25-12B recipe",
            f"{self._recipe.recipe_id}.yaml",
            "YAML (*.yaml *.yml)",
        )
        if not filename:
            return None
        try:
            path = self._recipe.write_yaml(filename)
        except Exception as exc:
            QMessageBox.critical(self, "Export failed", str(exc))
            return None
        self.statusBar().showMessage(f"Recipe exported: {path}")
        return path

    def _run_processing(self) -> None:
        if self._source_path is None:
            QMessageBox.information(self, "No source", "Open a PCD first")
            return
        if not self._recipe.operations:
            QMessageBox.information(self, "Empty recipe", "Add at least one operation first")
            return
        output_dir = QFileDialog.getExistingDirectory(self, "Choose parent directory for new processing run")
        if not output_dir:
            return
        run_name, ok = QFileDialog.getSaveFileName(
            self,
            "Choose new processing run directory name",
            str(Path(output_dir) / "agt_workbench_run"),
            "Directory name (*)",
        )
        if not ok or not run_name:
            return
        destination = Path(run_name)
        if destination.exists():
            QMessageBox.warning(self, "Immutable output", "Selected processing run already exists")
            return
        recipe_path = destination.parent / f".{destination.name}.recipe.yaml"
        try:
            self._recipe.write_yaml(recipe_path)
        except Exception as exc:
            QMessageBox.critical(self, "Recipe error", str(exc))
            return

        self.statusBar().showMessage("Processing full-resolution PCD…")
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
            f"Processing PASS: {result.input_points:,} → {result.output_points:,} points"
        )
        answer = QMessageBox.question(
            self,
            "Processing complete",
            f"Run written to:\n{result.run_dir}\n\nLoad processed PCD for visual review?",
        )
        if answer == QMessageBox.Yes:
            try:
                cloud = read_pcd(result.output_path)
                self._cloud = cloud
                self._cloud_item.set_points(cloud.xyz())
                self._fit_cloud()
            except Exception as exc:
                QMessageBox.warning(self, "Review load failed", str(exc))

    def _processing_failed(self, message: str, recipe_path: Path) -> None:
        recipe_path.unlink(missing_ok=True)
        QMessageBox.critical(self, "Processing failed", message)
        self.statusBar().showMessage("Processing failed")


def main(argv=None) -> int:
    app = QApplication(list(sys.argv if argv is None else argv))
    window = MapWorkbenchWindow()
    window.show()
    return app.exec_()
