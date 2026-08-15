"""Route Debug controls and read-only Inspector."""

from __future__ import annotations

from pathlib import Path
from typing import Any, Mapping

from PyQt5.QtCore import Qt
from PyQt5.QtWidgets import (
    QFileDialog,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QMessageBox,
    QPushButton,
    QTextBrowser,
    QTreeWidget,
    QTreeWidgetItem,
    QVBoxLayout,
    QWidget,
)

from agt_offline_assets.route_debug_dataset import (
    ASSET_INVALID,
    ASSET_LOADED,
    RouteDebugDataset,
    load_route_debug_dataset,
)
from agt_offline_assets.route_debug_overlay import (
    build_route_debug_overlay,
    write_route_debug_overlay,
)

from .route_debug_view import ROUTE_DEBUG_LAYER_KEYS, RouteDebugSceneController

_LAYER_LABELS = {
    "base.navigation": "Navigation Map",
    "base.no_go": "NO_GO",
    "structure.turn_zones": "Turn Zones",
    "structure.aisles": "Structural Aisles",
    "structure.vehicle_safe_lane": "Vehicle-safe Lane",
    "coverage.order": "Coverage Order",
    "coverage.requests": "Connector Requests",
    "motion.forward_candidates": "Forward Candidates",
    "motion.forward_selected": "Forward Selected / F Segments",
    "motion.reverse": "Reverse Segments / Cusps",
    "diagnostics.raw": "RAW_OBSTACLE_DIRECT",
    "diagnostics.geometry": "GEOMETRY_DIRECT",
    "diagnostics.padding": "PADDING_ONLY",
    "diagnostics.unknown": "UNKNOWN",
    "diagnostics.conflicts": "Collision Stations",
    "diagnostics.failed": "Failed / Unsolved Connectors",
}


class RouteDebugPanel(QWidget):
    def __init__(self, scene, view, parent=None):
        super().__init__(parent)
        self._scene = scene
        self._view = view
        self.controller = RouteDebugSceneController(scene, self)
        self.controller.featureSelected.connect(self._show_inspector_payload)
        self._current_run_dir: Path | None = None
        self._dataset: RouteDebugDataset | None = None
        self._layer_items: dict[str, QTreeWidgetItem] = {}
        self._updating_tree = False
        self._build_ui()
        self.controller.set_active(False)

    def _build_ui(self) -> None:
        layout = QVBoxLayout(self)
        layout.setContentsMargins(8, 8, 8, 8)
        layout.setSpacing(8)

        top = QHBoxLayout()
        choose = QPushButton("选择运行目录")
        reload_button = QPushButton("重载")
        fit_button = QPushButton("适配全图")
        choose.clicked.connect(self._choose_directory)
        reload_button.clicked.connect(self.reload_current_directory)
        fit_button.clicked.connect(self._fit_route)
        top.addWidget(choose)
        top.addWidget(reload_button)
        top.addWidget(fit_button)
        layout.addLayout(top)

        self._path_label = QLabel("未加载路径规划运行目录")
        self._path_label.setWordWrap(True)
        self._path_label.setTextInteractionFlags(Qt.TextSelectableByMouse)
        layout.addWidget(self._path_label)

        presets = QHBoxLayout()
        coverage = QPushButton("Coverage 总览")
        planning = QPushButton("规划结果")
        collision = QPushButton("碰撞诊断")
        coverage.clicked.connect(lambda: self.apply_preset("coverage"))
        planning.clicked.connect(lambda: self.apply_preset("planning"))
        collision.clicked.connect(lambda: self.apply_preset("collision"))
        presets.addWidget(coverage)
        presets.addWidget(planning)
        presets.addWidget(collision)
        layout.addLayout(presets)

        actions = QHBoxLayout()
        self._failure_button = QPushButton("仅失败")
        self._failure_button.setCheckable(True)
        self._failure_button.toggled.connect(self.set_failure_focus)
        export_button = QPushButton("导出 PNG")
        export_button.clicked.connect(self._choose_export_path)
        actions.addWidget(self._failure_button)
        actions.addWidget(export_button)
        layout.addLayout(actions)

        layer_box = QGroupBox("Layers")
        layer_layout = QVBoxLayout(layer_box)
        self._layers = QTreeWidget()
        self._layers.setHeaderLabels(["图层", "状态"])
        self._layers.itemChanged.connect(self._on_layer_changed)
        groups: dict[str, QTreeWidgetItem] = {}
        for layer_key in ROUTE_DEBUG_LAYER_KEYS:
            group_name = layer_key.split(".", 1)[0].upper()
            parent_item = groups.get(group_name)
            if parent_item is None:
                parent_item = QTreeWidgetItem([group_name, ""])
                parent_item.setFlags(parent_item.flags() & ~Qt.ItemIsUserCheckable)
                self._layers.addTopLevelItem(parent_item)
                groups[group_name] = parent_item
            child = QTreeWidgetItem([_LAYER_LABELS.get(layer_key, layer_key), "未加载"])
            child.setData(0, Qt.UserRole, layer_key)
            child.setFlags(child.flags() | Qt.ItemIsUserCheckable)
            child.setCheckState(0, Qt.Checked)
            parent_item.addChild(child)
            self._layer_items[layer_key] = child
        self._layers.expandAll()
        self._layers.header().setStretchLastSection(False)
        self._layers.header().resizeSection(0, 190)
        layer_layout.addWidget(self._layers)
        layout.addWidget(layer_box, 2)

        inspector_box = QGroupBox("Inspector")
        inspector_layout = QVBoxLayout(inspector_box)
        self._inspector = QTextBrowser()
        self._inspector.setReadOnly(True)
        self._inspector.setPlaceholderText("点击行道、路径、Connector 或碰撞点查看冻结证据")
        inspector_layout.addWidget(self._inspector)
        layout.addWidget(inspector_box, 3)

        self._status = QLabel("Route Debug 为只读预览，不修改路径生产真值")
        self._status.setWordWrap(True)
        layout.addWidget(self._status)

    def load_run_directory(self, path: str | Path) -> RouteDebugDataset:
        root = Path(path).expanduser().resolve()
        dataset = load_route_debug_dataset(root)
        overlay = build_route_debug_overlay(dataset)
        write_route_debug_overlay(
            overlay,
            dataset.run_dir / "route_debug_overlay.geojson",
            overwrite=True,
        )
        self.controller.set_content(dataset, overlay)
        self._dataset = dataset
        self._current_run_dir = dataset.run_dir
        self._path_label.setText(str(dataset.run_dir))
        self._refresh_layer_tree()
        self.apply_preset("coverage")
        self._fit_route()

        invalid = [state for state in dataset.asset_states if state.availability == ASSET_INVALID]
        loaded = [state for state in dataset.asset_states if state.availability == ASSET_LOADED]
        if invalid:
            self._status.setText(
                f"已加载 {len(loaded)} 个证据层，{len(invalid)} 个证据层因 schema/frame/内容无效而关闭"
            )
        else:
            self._status.setText(
                f"已加载 {len(loaded)} 个证据层；Route Debug 只写 route_debug_overlay.geojson"
            )
        return dataset

    def reload_current_directory(self) -> RouteDebugDataset | None:
        if self._current_run_dir is None:
            return None
        try:
            return self.load_run_directory(self._current_run_dir)
        except Exception as exc:
            QMessageBox.critical(self, "Route Debug 重载失败", str(exc))
            return None

    def set_active(self, active: bool) -> None:
        self.controller.set_active(active)
        if active and self._dataset is not None:
            self._fit_route()

    def apply_preset(self, preset_name: str) -> None:
        self.controller.apply_preset(preset_name)
        self._sync_tree_checks()

    def set_failure_focus(self, enabled: bool) -> None:
        self.controller.set_failure_focus(enabled)

    def export_current_view(self, path: str | Path) -> bool:
        return bool(self._view.grab().save(str(Path(path)), "PNG"))

    def current_run_directory(self) -> Path | None:
        return self._current_run_dir

    def layer_enabled(self, layer_key: str) -> bool:
        return self.controller.layer_available(layer_key) and self.controller.layer_visible(layer_key)

    def inspector_text(self) -> str:
        return self._inspector.toPlainText()

    def _choose_directory(self) -> None:
        selected = QFileDialog.getExistingDirectory(
            self,
            "选择 Route Debug 运行目录",
            str(self._current_run_dir or Path.cwd()),
        )
        if not selected:
            return
        try:
            self.load_run_directory(selected)
        except Exception as exc:
            QMessageBox.critical(self, "Route Debug 加载失败", str(exc))

    def _choose_export_path(self) -> None:
        default = (
            self._current_run_dir / "route_debug_view.png"
            if self._current_run_dir is not None
            else Path.cwd() / "route_debug_view.png"
        )
        selected, _ = QFileDialog.getSaveFileName(
            self, "导出 Route Debug PNG", str(default), "PNG (*.png)"
        )
        if selected and not self.export_current_view(selected):
            QMessageBox.warning(self, "导出失败", f"无法写入 {selected}")

    def _fit_route(self) -> None:
        bounds = self.controller.route_bounds()
        if not bounds.isNull() and bounds.width() > 0.0 and bounds.height() > 0.0:
            margin_x = max(0.5, bounds.width() * 0.04)
            margin_y = max(0.5, bounds.height() * 0.04)
            self._view.fitInView(
                bounds.adjusted(-margin_x, -margin_y, margin_x, margin_y),
                Qt.KeepAspectRatio,
            )

    def _refresh_layer_tree(self) -> None:
        self._updating_tree = True
        try:
            for layer_key, item in self._layer_items.items():
                available = self.controller.layer_available(layer_key)
                item.setDisabled(not available)
                item.setText(1, "可用" if available else "无证据")
                item.setCheckState(
                    0,
                    Qt.Checked if available and self.controller.layer_visible(layer_key) else Qt.Unchecked,
                )
        finally:
            self._updating_tree = False

    def _sync_tree_checks(self) -> None:
        self._updating_tree = True
        try:
            for layer_key, item in self._layer_items.items():
                item.setCheckState(
                    0,
                    Qt.Checked
                    if self.controller.layer_available(layer_key)
                    and self.controller.layer_visible(layer_key)
                    else Qt.Unchecked,
                )
        finally:
            self._updating_tree = False

    def _on_layer_changed(self, item: QTreeWidgetItem, column: int) -> None:
        if self._updating_tree or column != 0:
            return
        layer_key = item.data(0, Qt.UserRole)
        if not isinstance(layer_key, str) or item.isDisabled():
            return
        self.controller.set_layer_visible(layer_key, item.checkState(0) == Qt.Checked)

    def _show_inspector_payload(self, payload: Mapping[str, Any]) -> None:
        lines = [
            f"FEATURE  {payload.get('feature_id', '')}",
            f"KIND     {payload.get('feature_kind', '')}",
            f"STATUS   {payload.get('status', '')}",
            "",
        ]
        preferred = (
            "STRUCTURE", "COVERAGE", "VEHICLE", "NAVIGATION / VEHICLE FEASIBILITY",
            "CONFLICT", "RELATED", "FORWARD", "R5.6", "R6A", "R6B",
            "FORWARD CANDIDATE", "SOURCE",
        )
        seen = set()
        for section in preferred:
            value = payload.get(section)
            if value is None:
                continue
            seen.add(section)
            lines.extend(self._format_section(section, value))
        for section, value in payload.items():
            if section in seen or section in {
                "feature_id", "feature_kind", "status", "source_asset", "source_id",
                "source_field", "is_failure", "footprint_polygon_xy",
            }:
                continue
            if isinstance(value, Mapping):
                lines.extend(self._format_section(str(section), value))
        lines.extend([
            "SOURCE",
            f"  asset: {payload.get('source_asset', '')}",
            f"  id: {payload.get('source_id', '')}",
            f"  field: {payload.get('source_field', '')}",
        ])
        self._inspector.setPlainText("\n".join(lines))

    @staticmethod
    def _format_section(section: str, value: Any) -> list[str]:
        lines = [section]
        if isinstance(value, Mapping):
            for key, item in value.items():
                lines.append(f"  {key}: {item}")
        else:
            lines.append(f"  {value}")
        lines.append("")
        return lines
