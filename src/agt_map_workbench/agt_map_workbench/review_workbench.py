"""Compose the stable 2D agricultural Workbench with review-only extensions."""

from __future__ import annotations

import sys

from PyQt5.QtWidgets import QApplication, QFileDialog, QMessageBox, QSplitter, QTabWidget

from agt_offline_assets import (
    AisleGraphConfig,
    VehicleCorridorConfig,
    VehicleCorridorResult,
    derive_agricultural_aisle_graph,
    derive_vehicle_corridor,
    write_agricultural_aisle_graph,
)

from .agricultural_workbench import AgriculturalMapWorkbenchWindow
from .review_3d import ThreeDReviewWidget
from .route_debug_panel import RouteDebugPanel


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
        super().__init__()
        self._control_tabs = self._locate_control_tabs()
        self.setWindowTitle("AGT 地图工作台 — V25-12E 农业结构 + 3D 审查 + 路径调试")
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
            return
        if active:
            self._pre_route_cloud_visible = bool(self._cloud_item.isVisible())
            self._pre_route_navigation_visible = bool(self._navigation_preview_item.isVisible())
            self._cloud_item.setVisible(False)
            self._navigation_preview_item.setVisible(False)
            if self._review_tabs is not None:
                self._review_tabs.setCurrentIndex(0)
        else:
            self._cloud_item.setVisible(self._pre_route_cloud_visible)
            self._navigation_preview_item.setVisible(self._pre_route_navigation_visible)
        self._route_debug_active = active
        self._route_debug_panel.set_active(active)

    def _install_3d_review(self) -> None:
        splitter = self.centralWidget()
        if not isinstance(splitter, QSplitter):
            raise RuntimeError("3D Review expects the Workbench central QSplitter")

        # Avoid replacing the splitter child in place. On some Qt5 builds an
        # in-place replacement inherits a collapsed/zero splitter size, leaving
        # the controls visible while the whole 2D/3D review plane appears missing.
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

        # Re-establish a useful initial geometry after reparenting the old view.
        # Keep a sane historical ratio even if Qt reports a zero-width first
        # pane from a previous/collapsed layout state.
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
                "pcd_name": self._source_path.name if self._source_path is not None else "",
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
        if self._navigation_result is None or self._corridor_refinement_result is None:
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
            # Review the complete requested vehicle envelope, not only the part
            # clipped by the refined aisle. This makes geometric intrusion
            # visible instead of hiding the unsafe part of the requested width.
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
        self._recompute_vehicle_corridor()
        self._sync_3d_analysis()

    def _clear_navigation_state(self, *, clear_overrides: bool) -> None:
        self._vehicle_corridor_result = None
        super()._clear_navigation_state(clear_overrides=clear_overrides)
        if self._review_3d is not None:
            self._sync_3d_analysis()


def main(argv=None) -> int:
    app = QApplication(list(sys.argv if argv is None else argv))
    window = ReviewMapWorkbenchWindow()
    window.show()
    return app.exec_()
