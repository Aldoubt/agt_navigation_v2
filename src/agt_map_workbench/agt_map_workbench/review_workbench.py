"""Compose the stable 2D agricultural Workbench with review-only extensions."""

from __future__ import annotations

from dataclasses import replace
import sys
from pathlib import Path

from PyQt5.QtCore import QPointF
from PyQt5.QtGui import QBrush, QColor, QPen, QPolygonF
from PyQt5.QtWidgets import (
    QApplication,
    QDoubleSpinBox,
    QFileDialog,
    QGraphicsPolygonItem,
    QHBoxLayout,
    QLabel,
    QMessageBox,
    QPushButton,
    QScrollArea,
    QSplitter,
    QTabWidget,
    QVBoxLayout,
    QWidget,
)

from agt_offline_assets import (
    AisleGraphConfig,
    SiteBoundary,
    TraversabilityConfig,
    TraversabilityEvidence,
    VehicleCorridorConfig,
    VehicleCorridorResult,
    derive_agricultural_aisle_graph,
    derive_traversability_evidence,
    derive_vehicle_corridor,
    load_site_boundary,
    load_vehicle_feasible_segment_plan,
    validate_site_boundary,
    write_agricultural_aisle_graph,
    write_site_boundary,
    write_traversability_candidate,
)

from .agricultural_workbench import AgriculturalMapWorkbenchWindow
from .review_3d import ThreeDReviewWidget
from .route_debug_panel import RouteDebugPanel
from .vehicle_feasible_segment_preview import VehicleFeasibleSegmentPreview


_TRAVERSABILITY_LAYER_SPECS = {
    "12f_observed": ("observed_free_mask", (70, 220, 120, 160)),
    "12f_inferred": ("inferred_traversable_mask", (40, 220, 255, 220)),
    "12f_hard_blocked": ("hard_blocked_mask", (255, 55, 55, 205)),
    "12f_sensor_obstacle": ("sensor_obstacle_mask", (255, 145, 40, 205)),
    "12f_unknown": ("unknown_mask", (155, 160, 175, 120)),
    "12f_geometric": ("aisle_geometric_envelope_mask", (245, 205, 65, 125)),
}


class ReviewMapWorkbenchWindow(AgriculturalMapWorkbenchWindow):
    """Keep 2D authoring authoritative while adding independent review planes."""

    def __init__(self) -> None:
        self._vehicle_corridor_result: VehicleCorridorResult | None = None
        self._review_3d: ThreeDReviewWidget | None = None
        self._review_tabs: QTabWidget | None = None
        self._route_debug_panel: RouteDebugPanel | None = None
        self._route_debug_tab_index = -1
        self._route_debug_active = False
        self._pre_route_cloud_visible = True
        self._pre_route_navigation_visible = False

        self._site_boundary: SiteBoundary | None = None
        self._site_boundary_vertices: list[tuple[float, float]] = []
        self._site_boundary_item: QGraphicsPolygonItem | None = None
        self._site_boundary_vertex_items = []
        self._site_boundary_status: QLabel | None = None
        self._site_boundary_export_button: QPushButton | None = None
        self._site_boundary_last_error = ""

        self._traversability_evidence: TraversabilityEvidence | None = None
        self._navigation_12f_result = None
        self._traversability_gap: QDoubleSpinBox | None = None
        self._traversability_status: QLabel | None = None
        self._traversability_export_button: QPushButton | None = None

        self._vehicle_feasible_segment_plan = None
        self._vehicle_feasible_segment_last_error = ""
        self._vehicle_feasible_segment_preview: VehicleFeasibleSegmentPreview | None = None

        super().__init__()
        self._vehicle_feasible_segment_preview = VehicleFeasibleSegmentPreview(self._scene)
        self._control_tabs = self._locate_control_tabs()
        self.setWindowTitle(
            "AGT 地图工作台 — V25-12F 农业结构 + 3D 审查 + 路径调试"
        )
        self._install_site_boundary_authoring()
        self._install_12f_candidate_controls()
        self._install_3d_review()
        self._install_offline_asset_actions()
        self._install_route_debug()

    def _locate_control_tabs(self) -> QTabWidget:
        """Resolve the existing right-side authoring tabs without rebuilding the base UI."""
        expected = ["点云编辑", "坐标系标定", "导航地图"]
        for tabs in self.findChildren(QTabWidget):
            labels = [tabs.tabText(index) for index in range(tabs.count())]
            if labels[:3] == expected:
                return tabs
        raise RuntimeError("unable to locate the existing Workbench control QTabWidget")

    def _navigation_authoring_layout(self):
        page = self._control_tabs.widget(2)
        if page is None:
            raise RuntimeError("unable to locate Navigation Map authoring page")
        scroll = page.findChild(QScrollArea)
        content = scroll.widget() if scroll is not None else page
        layout = content.layout() if content is not None else None
        if layout is None:
            raise RuntimeError("Navigation Map authoring page has no layout")
        return layout

    # ------------------------------------------------------- Site Boundary authoring
    def _install_site_boundary_authoring(self) -> None:
        panel = QWidget()
        layout = QVBoxLayout(panel)
        layout.setContentsMargins(0, 6, 0, 0)

        title = QLabel("Site Boundary（车辆允许区域内边界）")
        title.setWordWrap(True)
        layout.addWidget(title)

        note = QLabel(
            "沿车辆允许进入区域的内边界点选多边形；边界已经包含墙厚与贴墙安全距离，"
            "车辆 footprint 接触或越过该边界都视为冲突"
        )
        note.setWordWrap(True)
        layout.addWidget(note)

        row = QHBoxLayout()
        start = QPushButton("开始绘制")
        finish = QPushButton("完成")
        undo = QPushButton("撤销顶点")
        clear = QPushButton("清除")
        start.clicked.connect(self._start_site_boundary_authoring)
        finish.clicked.connect(self._finish_site_boundary_authoring)
        undo.clicked.connect(self._undo_site_boundary_vertex)
        clear.clicked.connect(self._clear_site_boundary)
        row.addWidget(start)
        row.addWidget(finish)
        row.addWidget(undo)
        row.addWidget(clear)
        layout.addLayout(row)

        export = QPushButton("导出 site_boundary.yaml")
        export.clicked.connect(self._export_site_boundary)
        layout.addWidget(export)
        self._site_boundary_export_button = export

        status = QLabel("Site Boundary：未定义")
        status.setWordWrap(True)
        layout.addWidget(status)
        self._site_boundary_status = status

        target_layout = self._navigation_authoring_layout()
        target_layout.insertWidget(max(0, target_layout.count() - 1), panel)
        self._refresh_site_boundary_status()

    def _on_map_clicked(self, x: float, y: float) -> None:
        if self._interaction_mode == "site_boundary":
            self._append_site_boundary_vertex(x, y)
            return
        super()._on_map_clicked(x, y)

    def _start_site_boundary_authoring(self) -> None:
        if self._cloud is None and self._navigation_result is None:
            QMessageBox.information(
                self,
                "尚未加载地图",
                "请先打开 PCD 或生成 Navigation Map，再绘制 Site Boundary",
            )
            return
        self._site_boundary_vertices = []
        self._site_boundary_last_error = ""
        self._refresh_site_boundary_graphics()
        self._refresh_site_boundary_status()
        self._set_interaction_mode(
            "site_boundary",
            "Site Boundary：左键按顺序添加车辆允许区域内边界顶点",
        )

    def _append_site_boundary_vertex(self, x: float, y: float) -> None:
        self._site_boundary_vertices.append((float(x), float(y)))
        self._site_boundary_last_error = ""
        self._refresh_site_boundary_graphics()
        self._refresh_site_boundary_status()
        self.statusBar().showMessage(
            f"Site Boundary 顶点 {len(self._site_boundary_vertices)}："
            f"X={x:.3f} m，Y={y:.3f} m"
        )

    def _undo_site_boundary_vertex(self) -> None:
        if self._site_boundary_vertices:
            self._site_boundary_vertices.pop()
        self._site_boundary_last_error = ""
        self._refresh_site_boundary_graphics()
        self._refresh_site_boundary_status()

    def _finish_site_boundary_authoring(
        self,
        _checked=False,
        *,
        show_errors: bool = True,
    ) -> bool:
        if len(self._site_boundary_vertices) < 3:
            self._site_boundary_last_error = "至少需要 3 个顶点"
            self._refresh_site_boundary_status()
            if show_errors:
                QMessageBox.warning(
                    self,
                    "Site Boundary 未完成",
                    self._site_boundary_last_error,
                )
            return False

        candidate = SiteBoundary(
            frame_id="map",
            outer_boundary_xy=tuple(
                (float(x), float(y)) for x, y in self._site_boundary_vertices
            ),
            source={"authoring_mode": "WORKBENCH_MANUAL_POLYGON"},
        )
        try:
            validate_site_boundary(candidate, expected_frame_id="map")
        except Exception as exc:
            self._site_boundary_last_error = str(exc)
            self._refresh_site_boundary_status()
            if show_errors:
                QMessageBox.warning(self, "Site Boundary 无效", str(exc))
            return False

        self._site_boundary = candidate
        self._site_boundary_vertices = list(candidate.outer_boundary_xy)
        self._site_boundary_last_error = ""
        self._clear_12f_candidate()
        if self._interaction_mode == "site_boundary":
            self._set_interaction_mode(None, "Site Boundary 已冻结为 READY 草案")
        self._refresh_site_boundary_graphics()
        self._refresh_site_boundary_status()
        return True

    def _clear_site_boundary(self, _checked=False) -> None:
        self._site_boundary = None
        self._site_boundary_vertices = []
        self._site_boundary_last_error = ""
        self._clear_12f_candidate()
        if self._interaction_mode == "site_boundary":
            self._set_interaction_mode(None, "Site Boundary 已清除")
        self._refresh_site_boundary_graphics()
        self._refresh_site_boundary_status()

    def _refresh_site_boundary_graphics(self) -> None:
        if self._site_boundary_item is not None:
            self._scene.removeItem(self._site_boundary_item)
            self._site_boundary_item = None
        for item in self._site_boundary_vertex_items:
            self._scene.removeItem(item)
        self._site_boundary_vertex_items.clear()

        points = list(self._site_boundary_vertices)
        if not points and self._site_boundary is not None:
            points = list(self._site_boundary.outer_boundary_xy)
        if not points:
            return

        scene_points = [QPointF(float(x), float(-y)) for x, y in points]
        color = QColor(40, 225, 165)
        item = QGraphicsPolygonItem(QPolygonF(scene_points))
        pen = QPen(color)
        pen.setWidth(3)
        pen.setCosmetic(True)
        item.setPen(pen)
        item.setBrush(QBrush(QColor(40, 225, 165, 24)))
        item.setZValue(18.0)
        self._scene.addItem(item)
        self._site_boundary_item = item

        if self._interaction_mode == "site_boundary":
            for index, point in enumerate(scene_points, start=1):
                self._add_fixed_marker(
                    point,
                    f"B{index}",
                    color,
                    self._site_boundary_vertex_items,
                    label_offset_px=(8.0, -18.0),
                )

    def _set_site_boundary_authoring_visible(self, visible: bool) -> None:
        if self._site_boundary_item is not None:
            self._site_boundary_item.setVisible(bool(visible))
        for item in self._site_boundary_vertex_items:
            item.setVisible(bool(visible))

    def _refresh_site_boundary_status(self) -> None:
        if self._site_boundary_status is None:
            return
        if self._site_boundary_last_error:
            ready_suffix = (
                "；上一 READY 边界仍保留"
                if self._site_boundary is not None
                else ""
            )
            text = (
                f"Site Boundary：INVALID 草稿 | "
                f"{self._site_boundary_last_error}{ready_suffix}"
            )
        elif self._interaction_mode == "site_boundary" or (
            self._site_boundary_vertices and self._site_boundary is None
        ):
            text = f"Site Boundary：草稿 {len(self._site_boundary_vertices)} 点"
        elif self._site_boundary is not None:
            text = (
                f"Site Boundary：READY | {len(self._site_boundary.outer_boundary_xy)} 点 | "
                "车辆 footprint 接触边界即冲突"
            )
        else:
            text = "Site Boundary：未定义"
        self._site_boundary_status.setText(text)
        if self._site_boundary_export_button is not None:
            self._site_boundary_export_button.setEnabled(self._site_boundary is not None)

    def _load_site_boundary_sibling(self) -> None:
        self._site_boundary = None
        self._site_boundary_vertices = []
        self._site_boundary_last_error = ""
        self._clear_12f_candidate()
        if self._source_path is None:
            self._refresh_site_boundary_graphics()
            self._refresh_site_boundary_status()
            return

        candidate = self._source_path.parent / "site_boundary.yaml"
        if not candidate.is_file():
            self._refresh_site_boundary_graphics()
            self._refresh_site_boundary_status()
            return
        try:
            boundary = load_site_boundary(candidate, expected_frame_id="map")
        except Exception as exc:
            self._site_boundary_last_error = str(exc)
            self._refresh_site_boundary_graphics()
            self._refresh_site_boundary_status()
            QMessageBox.warning(
                self,
                "Site Boundary 加载失败",
                f"{candidate}\n\n{exc}",
            )
            return
        self._site_boundary = boundary
        self._site_boundary_vertices = list(boundary.outer_boundary_xy)
        self._refresh_site_boundary_graphics()
        self._refresh_site_boundary_status()
        self.statusBar().showMessage(f"Site Boundary 已加载：{candidate}")

    def _load_vehicle_feasible_segment_sibling(self) -> None:
        """Clear stale A1 state and quietly tolerate a missing sibling asset."""
        if self._vehicle_feasible_segment_preview is not None:
            self._vehicle_feasible_segment_preview.clear()
        self._vehicle_feasible_segment_plan = None
        self._vehicle_feasible_segment_last_error = ""

        if self._source_path is None:
            return
        candidate = self._source_path.parent / "vehicle_feasible_segments.yaml"
        if not candidate.is_file():
            return

        try:
            plan = load_vehicle_feasible_segment_plan(candidate)
        except Exception as exc:
            self._vehicle_feasible_segment_last_error = str(exc)
            return
        self._vehicle_feasible_segment_plan = plan
        if self._vehicle_feasible_segment_preview is not None:
            self._vehicle_feasible_segment_preview.set_plan(plan)
            self._vehicle_feasible_segment_preview.set_visible(False)

    def _export_site_boundary(self, _checked=False) -> Path | None:
        if self._site_boundary is None:
            QMessageBox.information(
                self,
                "Site Boundary 未就绪",
                "请先完成并验证 Site Boundary",
            )
            return None

        if self._source_path is not None:
            output = self._source_path.parent / "site_boundary.yaml"
        else:
            filename, _selected_filter = QFileDialog.getSaveFileName(
                self,
                "导出 Site Boundary",
                "site_boundary.yaml",
                "YAML (*.yaml *.yml)",
            )
            if not filename:
                return None
            output = Path(filename)

        try:
            path = write_site_boundary(self._site_boundary, output, overwrite=True)
        except Exception as exc:
            QMessageBox.critical(self, "Site Boundary 导出失败", str(exc))
            return None
        self.statusBar().showMessage(f"site_boundary.yaml 已导出：{path}")
        return path

    # ------------------------------------------------------- V25-12F candidate
    def _install_12f_candidate_controls(self) -> None:
        existing_keys = {
            self._nav_layer.itemData(index)
            for index in range(self._nav_layer.count())
        }
        if "aisle_geometric_envelope" not in existing_keys:
            self._nav_layer.addItem(
                "结构行道几何包络（未经过 Ground FREE 过滤）",
                "aisle_geometric_envelope",
            )
        for label, key in (
            ("12G-A1 Vehicle-Feasible Segments", "vehicle_feasible_segments"),
            ("12F Candidate 三态", "12f_candidate"),
            ("12F OBSERVED_FREE", "12f_observed"),
            ("12F INFERRED_TRAVERSABLE", "12f_inferred"),
            ("12F HARD_BLOCKED", "12f_hard_blocked"),
            ("12F SENSOR_OBSTACLE", "12f_sensor_obstacle"),
            ("12F UNKNOWN", "12f_unknown"),
            ("12F Aisle Geometric Envelope", "12f_geometric"),
        ):
            if key not in existing_keys:
                self._nav_layer.addItem(label, key)

        panel = QWidget()
        layout = QVBoxLayout(panel)
        layout.setContentsMargins(0, 6, 0, 0)
        title = QLabel("V25-12F Candidate（不会覆盖当前 Navigation Map）")
        title.setWordWrap(True)
        layout.addWidget(title)

        gap = QDoubleSpinBox()
        gap.setPrefix("最大遮挡补全缺口：")
        gap.setSuffix(" m")
        gap.setDecimals(2)
        gap.setRange(0.10, 2.00)
        gap.setSingleStep(0.10)
        gap.setValue(0.60)
        gap.valueChanged.connect(self._on_12f_gap_changed)
        layout.addWidget(gap)
        self._traversability_gap = gap

        row = QHBoxLayout()
        generate = QPushButton("生成 12F Candidate")
        export = QPushButton("导出到当前 run 目录")
        generate.clicked.connect(self._generate_12f_candidate)
        export.clicked.connect(self._export_12f_candidate)
        row.addWidget(generate)
        row.addWidget(export)
        layout.addLayout(row)
        self._traversability_export_button = export

        status = QLabel("12F Candidate：未生成")
        status.setWordWrap(True)
        layout.addWidget(status)
        self._traversability_status = status

        target_layout = self._navigation_authoring_layout()
        target_layout.insertWidget(max(0, target_layout.count() - 1), panel)
        self._refresh_12f_status()

    def _on_12f_gap_changed(self, _value: float) -> None:
        self._clear_12f_candidate()

    def _clear_12f_candidate(self) -> None:
        self._traversability_evidence = None
        self._navigation_12f_result = None
        self._refresh_12f_status()
        if hasattr(self, "_nav_layer"):
            layer = str(self._nav_layer.currentData())
            if layer == "12f_candidate" or layer in _TRAVERSABILITY_LAYER_SPECS:
                self._navigation_preview_item.clear_result()

    def _refresh_12f_status(self) -> None:
        if self._traversability_status is None:
            return
        if self._traversability_evidence is None or self._navigation_12f_result is None:
            self._traversability_status.setText(
                "12F Candidate：未生成 | 需要 Navigation / Row / Corridor / READY Site Boundary"
            )
            if self._traversability_export_button is not None:
                self._traversability_export_button.setEnabled(False)
            return
        counts = self._traversability_evidence.counts()
        self._traversability_status.setText(
            "12F Candidate：DRAFT | "
            f"OBSERVED_FREE {counts['observed_free']:,} | "
            f"INFERRED {counts['inferred_traversable']:,} | "
            f"HARD {counts['hard_blocked']:,} | "
            f"SENSOR {counts['sensor_obstacle']:,} | "
            f"UNKNOWN {counts['unknown']:,}"
        )
        if self._traversability_export_button is not None:
            self._traversability_export_button.setEnabled(True)

    def _generate_12f_candidate(
        self,
        _checked=False,
        *,
        show_errors: bool = True,
    ) -> bool:
        navigation = self._navigation_result
        structure = self._navigation_structure_result
        corridor = self._corridor_refinement_result
        boundary = self._site_boundary
        missing = []
        if navigation is None:
            missing.append("Navigation Map")
        if structure is None:
            missing.append("Row Structure")
        if corridor is None:
            missing.append("Corridor")
        if boundary is None:
            missing.append("READY Site Boundary")
        if missing:
            self._traversability_evidence = None
            self._navigation_12f_result = None
            self._refresh_12f_status()
            if show_errors:
                QMessageBox.information(
                    self,
                    "12F Candidate 条件不足",
                    "缺少：" + " / ".join(missing),
                )
            return False

        maximum_gap = (
            float(self._traversability_gap.value())
            if self._traversability_gap is not None
            else 0.60
        )
        try:
            evidence = derive_traversability_evidence(
                navigation,
                structure,
                corridor,
                boundary,
                TraversabilityConfig(maximum_inferred_gap_m=maximum_gap),
                overrides=self._navigation_overrides,
                frame_id="map",
                source={
                    "workbench": "ReviewMapWorkbenchWindow",
                    "source_pcd": (
                        self._source_path.name if self._source_path is not None else ""
                    ),
                },
            )
            candidate = replace(
                navigation,
                occupancy=evidence.candidate_occupancy(),
            )
        except Exception as exc:
            self._traversability_evidence = None
            self._navigation_12f_result = None
            self._refresh_12f_status()
            if show_errors:
                QMessageBox.critical(self, "12F Candidate 生成失败", str(exc))
            return False

        self._traversability_evidence = evidence
        self._navigation_12f_result = candidate
        self._refresh_12f_status()
        self._update_navigation_overlay()
        self.statusBar().showMessage(
            "V25-12F Candidate 已生成；当前 frozen Navigation Map 未被修改"
        )
        return True

    def _export_12f_candidate(self, _checked=False) -> Path | None:
        if (
            self._traversability_evidence is None
            or self._navigation_12f_result is None
            or self._site_boundary is None
            or self._navigation_result is None
        ):
            QMessageBox.information(
                self,
                "12F Candidate 未就绪",
                "请先生成 12F Candidate",
            )
            return None

        if self._source_path is not None:
            output_dir = self._source_path.parent
        else:
            selected = QFileDialog.getExistingDirectory(
                self,
                "选择 12F Candidate 输出目录",
            )
            if not selected:
                return None
            output_dir = Path(selected)

        boundary_path = output_dir / "site_boundary.yaml"
        if not boundary_path.is_file():
            QMessageBox.information(
                self,
                "Site Boundary 尚未冻结",
                f"请先导出 {boundary_path.name}，再导出 12F Candidate",
            )
            return None

        candidate_names = (
            "traversability_evidence.yaml",
            "traversability_evidence.npz",
            "navigation_map_12f.yaml",
            "navigation_map_12f.pgm",
            "navigation_map_12f_derivation.yaml",
        )
        existing = [name for name in candidate_names if (output_dir / name).exists()]
        overwrite = False
        if existing:
            answer = QMessageBox.question(
                self,
                "覆盖已有 12F Candidate？",
                "仅覆盖以下 12F 候选文件，不会改 navigation_map.yaml/pgm 或 derivation.yaml：\n\n"
                + "\n".join(existing),
                QMessageBox.Yes | QMessageBox.No,
                QMessageBox.No,
            )
            if answer != QMessageBox.Yes:
                return None
            overwrite = True

        try:
            output = write_traversability_candidate(
                self._traversability_evidence,
                self._navigation_result,
                self._site_boundary,
                output_dir,
                source_navigation_asset="navigation_map.yaml",
                overwrite=overwrite,
            )
        except Exception as exc:
            QMessageBox.critical(self, "12F Candidate 导出失败", str(exc))
            return None
        self.statusBar().showMessage(f"12F Candidate 已导出：{output}")
        return output

    def _update_navigation_overlay(self) -> None:
        if not hasattr(self, "_nav_layer"):
            return
        layer = str(self._nav_layer.currentData())
        if layer == "vehicle_feasible_segments":
            self._navigation_preview_item.clear_result()
            if self._vehicle_feasible_segment_preview is not None:
                self._vehicle_feasible_segment_preview.set_visible(
                    self._vehicle_feasible_segment_plan is not None
                    and self._nav_overlay_visible.isChecked()
                )
            return
        if self._vehicle_feasible_segment_preview is not None:
            self._vehicle_feasible_segment_preview.set_visible(False)
        if layer == "aisle_geometric_envelope":
            if (
                self._navigation_result is None
                or self._corridor_refinement_result is None
                or not self._nav_overlay_visible.isChecked()
            ):
                self._navigation_preview_item.clear_result()
                return
            self._navigation_preview_item.set_result(
                self._navigation_result,
                "aisle_geometric_envelope",
                self._navigation_structure_result,
                self._corridor_refinement_result,
            )
            return
        if layer == "12f_candidate":
            if (
                self._navigation_12f_result is None
                or not self._nav_overlay_visible.isChecked()
            ):
                self._navigation_preview_item.clear_result()
                return
            self._navigation_preview_item.set_result(
                self._navigation_12f_result,
                "final",
            )
            return
        if layer in _TRAVERSABILITY_LAYER_SPECS:
            if (
                self._navigation_result is None
                or self._traversability_evidence is None
                or not self._nav_overlay_visible.isChecked()
            ):
                self._navigation_preview_item.clear_result()
                return
            attribute, rgba_value = _TRAVERSABILITY_LAYER_SPECS[layer]
            self._navigation_preview_item.set_mask(
                self._navigation_result,
                getattr(self._traversability_evidence, attribute),
                rgba_value,
            )
            return
        super()._update_navigation_overlay()

    # ------------------------------------------------------- Route Debug
    def _install_route_debug(self) -> None:
        panel = RouteDebugPanel(self._scene, self._view, self)
        self._route_debug_panel = panel
        self._route_debug_tab_index = self._control_tabs.addTab(panel, "路径调试")
        self._control_tabs.currentChanged.connect(self._on_control_tab_changed)
        self._on_control_tab_changed(self._control_tabs.currentIndex())

    def _on_control_tab_changed(self, index: int) -> None:
        if self._route_debug_panel is None or self._route_debug_tab_index < 0:
            return
        active = int(index) == int(self._route_debug_tab_index)
        if active == self._route_debug_active:
            self._route_debug_panel.set_active(active)
            self._set_site_boundary_authoring_visible(not active)
            return
        if active:
            self._pre_route_cloud_visible = bool(self._cloud_item.isVisible())
            self._pre_route_navigation_visible = bool(
                self._navigation_preview_item.isVisible()
            )
            self._cloud_item.setVisible(False)
            self._navigation_preview_item.setVisible(False)
            if self._vehicle_feasible_segment_preview is not None:
                self._vehicle_feasible_segment_preview.set_visible(False)
            self._set_site_boundary_authoring_visible(False)
            if self._review_tabs is not None:
                self._review_tabs.setCurrentIndex(0)
        else:
            self._cloud_item.setVisible(self._pre_route_cloud_visible)
            self._navigation_preview_item.setVisible(
                self._pre_route_navigation_visible
            )
            self._set_site_boundary_authoring_visible(True)
        self._route_debug_active = active
        self._route_debug_panel.set_active(active)

    # ------------------------------------------------------- 3D review
    def _install_3d_review(self) -> None:
        splitter = self.centralWidget()
        if not isinstance(splitter, QSplitter):
            raise RuntimeError("3D Review expects the Workbench central QSplitter")

        old_sizes = splitter.sizes()
        old_view = splitter.widget(0)
        if old_view is None:
            raise RuntimeError("unable to locate the existing 2D Workbench view")
        old_view.setParent(None)

        tabs = QTabWidget()
        tabs.setMinimumWidth(640)
        tabs.addTab(old_view, "2D 编辑 / 分析")

        review = ThreeDReviewWidget()
        review.vehicleProfileChanged.connect(self._vehicle_profile_changed)
        review.sampleLimitChanged.connect(self._review_sample_changed)
        tabs.addTab(review, "3D 审查")

        splitter.insertWidget(0, tabs)
        splitter.setCollapsible(0, False)
        splitter.setCollapsible(1, False)
        splitter.setStretchFactor(0, 1)
        splitter.setStretchFactor(1, 0)
        splitter.setHandleWidth(6)

        if len(old_sizes) >= 2 and old_sizes[0] >= 320 and old_sizes[1] >= 240:
            splitter.setSizes([int(old_sizes[0]), int(old_sizes[1])])
        else:
            splitter.setSizes([1040, 480])

        tabs.setCurrentIndex(0)
        old_view.show()
        tabs.show()
        self._review_tabs = tabs
        self._review_3d = review

        if self._cloud is not None:
            review.set_cloud(self._cloud)
        self._sync_3d_analysis()

    def _install_offline_asset_actions(self) -> None:
        menu = self.menuBar().addMenu("离线资产")
        export_aisle_graph = menu.addAction("导出 Aisle Graph YAML")
        export_aisle_graph.setToolTip(
            "把当前 Ground / Hybrid Row / Corridor evidence 固化成 DRAFT aisle_graph.yaml"
        )
        export_aisle_graph.triggered.connect(self._export_aisle_graph)

    def _export_aisle_graph(self) -> None:
        navigation = self._navigation_result
        structure = self._navigation_structure_result
        corridor = self._corridor_refinement_result
        if navigation is None or structure is None or corridor is None:
            QMessageBox.information(
                self,
                "农业结构尚未完成",
                "请先生成 Ground-relative 导航图并完成 Ground / Row / Corridor 证据计算",
            )
            return
        graph = derive_agricultural_aisle_graph(
            navigation,
            structure,
            corridor,
            AisleGraphConfig(
                centerline_sample_spacing_m=max(0.10, float(navigation.resolution_m))
            ),
            source={
                "pcd_name": (
                    self._source_path.name if self._source_path is not None else ""
                ),
                "navigation_resolution_m": float(navigation.resolution_m),
                "corridor_pair_count": len(corridor.aisle_pair_diagnostics),
            },
        )
        if not graph.aisles:
            QMessageBox.warning(
                self,
                "没有可导出的行道",
                "当前 Corridor evidence 没有形成至少两点的 ACCEPTED aisle centerline",
            )
            return

        default_path = "aisle_graph.yaml"
        if self._source_path is not None:
            default_path = str(self._source_path.parent / default_path)
        filename, _selected_filter = QFileDialog.getSaveFileName(
            self,
            "导出农业 Aisle Graph",
            default_path,
            "YAML (*.yaml *.yml)",
        )
        if not filename:
            return
        output = write_agricultural_aisle_graph(graph, filename)
        interior = sum(aisle.kind == "interior" for aisle in graph.aisles)
        boundary = sum(aisle.kind == "boundary" for aisle in graph.aisles)
        self.statusBar().showMessage(
            f"Aisle Graph 已导出：{output.name} | {len(graph.aisles)} 条 | "
            f"interior={interior} boundary={boundary} | DRAFT"
        )
        QMessageBox.information(
            self,
            "Aisle Graph 已导出",
            f"{output}\n\n"
            f"DRAFT aisles: {len(graph.aisles)}\n"
            f"Interior: {interior}\nBoundary: {boundary}\n\n"
            "下一步将进入 Turn Zone / Vehicle Profile / Coverage Ordering，"
            "此文件当前还不是 READY Route Asset",
        )

    def _open_pcd(self) -> None:
        previous = self._source_path
        super()._open_pcd()
        if self._source_path != previous:
            self._load_site_boundary_sibling()
            self._load_vehicle_feasible_segment_sibling()
        if self._review_3d is None or self._cloud is None:
            return
        if self._source_path != previous:
            self._vehicle_corridor_result = None
            self._review_3d.set_cloud(self._cloud)
            self._review_3d.set_analysis(None)

    def _review_sample_changed(self, _sample_limit: int) -> None:
        if self._review_3d is not None and self._cloud is not None:
            self._review_3d.reload_cloud_sample(self._cloud)

    def _vehicle_profile_changed(self, _width: float, _margin: float) -> None:
        self._recompute_vehicle_corridor()
        self._sync_3d_analysis()

    def _vehicle_config(self) -> VehicleCorridorConfig:
        if self._review_3d is None:
            return VehicleCorridorConfig()
        return VehicleCorridorConfig(
            vehicle_width_m=float(self._review_3d.vehicle_width.value()),
            lateral_safety_margin_m=float(self._review_3d.vehicle_margin.value()),
        )

    def _recompute_vehicle_corridor(self) -> None:
        if (
            self._navigation_result is None
            or self._corridor_refinement_result is None
        ):
            self._vehicle_corridor_result = None
            return
        self._vehicle_corridor_result = derive_vehicle_corridor(
            self._navigation_result,
            self._corridor_refinement_result,
            self._vehicle_config(),
        )

    def _sync_3d_analysis(self) -> None:
        if self._review_3d is None:
            return
        self._review_3d.set_analysis(
            self._navigation_result,
            self._corridor_refinement_result,
            self._vehicle_corridor_result,
        )
        vehicle = self._vehicle_corridor_result
        navigation = self._navigation_result
        if vehicle is not None and navigation is not None:
            self._review_3d.canvas.set_layer_xyz(
                "vehicle_corridor",
                self._review_3d._grid_xyz(
                    navigation,
                    vehicle.required_envelope_mask,
                    z_offset_m=0.10,
                ),
            )
            self._review_3d.vehicle_status.setText(
                f"要求宽度：{vehicle.required_width_m:.2f} m | "
                f"安全 {vehicle.corridor_cells:,} cells | "
                f"冲突 {vehicle.conflict_cells:,} cells"
            )

    def _recompute_navigation_structure(self) -> None:
        super()._recompute_navigation_structure()
        self._clear_12f_candidate()
        self._recompute_vehicle_corridor()
        self._sync_3d_analysis()

    def _clear_navigation_state(self, *, clear_overrides: bool) -> None:
        self._vehicle_corridor_result = None
        self._clear_12f_candidate()
        super()._clear_navigation_state(clear_overrides=clear_overrides)
        if self._review_3d is not None:
            self._sync_3d_analysis()


def main(argv=None) -> int:
    app = QApplication(list(sys.argv if argv is None else argv))
    window = ReviewMapWorkbenchWindow()
    window.show()
    return app.exec_()
