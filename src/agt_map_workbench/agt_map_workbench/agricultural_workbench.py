"""Agricultural structure extension for the V25-12C Map Workbench."""

from __future__ import annotations

import sys

import numpy as np
from PyQt5.QtWidgets import (
    QApplication,
    QCheckBox,
    QComboBox,
    QDoubleSpinBox,
    QHBoxLayout,
    QLabel,
    QMessageBox,
    QPushButton,
    QScrollArea,
    QVBoxLayout,
    QWidget,
)

from agt_offline_assets import (
    CorridorRefinementConfig,
    CorridorRefinementResult,
    NavigationStructureConfig,
    NavigationStructureResult,
    derive_corridor_refinement,
    derive_navigation_structure,
)

from .app import MapWorkbenchWindow


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
    "refined_aisle",
    "aisle_centerline",
}
_STATUS_TEXT = {
    "ACCEPTED": "接受",
    "ACCEPTED_NO_CENTERLINE": "接受，但当前安全证据未形成中心线",
    "REJECTED_TOO_NARROW": "拒绝：几何宽度不足",
    "REJECTED_NO_LONGITUDINAL_OVERLAP": "拒绝：两垄纵向重叠不足",
    "REJECTED_MISSING_ROW_SUPPORT": "拒绝：至少一条垄缺少有效纵向支持",
    "REJECTED_NO_SAFE_CELLS": "拒绝：Ground / 坡度 / 障碍净空后无安全栅格",
}


class AgriculturalMapWorkbenchWindow(MapWorkbenchWindow):
    """Adds terrain, crop-row, and explicit aisle evidence to the base editor."""

    def __init__(self) -> None:
        self._navigation_structure_result: NavigationStructureResult | None = None
        self._corridor_refinement_result: CorridorRefinementResult | None = None
        super().__init__()
        self.setWindowTitle("AGT 地图工作台 — V25-12C 农业结构")

    def _build_navigation_tab(self) -> QWidget:
        tab = super()._build_navigation_tab()
        layout = tab.layout()

        self._nav_layer.addItem("Ground Confidence", "ground_confidence")
        self._nav_layer.addItem("0.5m Robust Plane 坡度", "robust_slope")
        self._nav_layer.addItem("局部平面残差", "plane_residual")
        self._nav_layer.addItem("混合垄支持（植被 + 地形）", "row_support")
        self._nav_layer.addItem("规则化种植行（旧结构证据）", "row_regularized")
        self._nav_layer.addItem("行道候选（旧剩余区域逻辑）", "aisle_candidate")
        self._nav_layer.addItem("垄中心线", "row_centerline")
        self._nav_layer.addItem("垄结构带（不随叶片包络变化）", "row_structural_band")
        self._nav_layer.addItem("植被 / 原始障碍包络", "vegetation_envelope")
        self._nav_layer.addItem("墙体 / 地图边界排除带", "boundary_exclusion")
        self._nav_layer.addItem("精炼行道候选（仅两垄之间）", "refined_aisle")
        self._nav_layer.addItem("行道中心线", "aisle_centerline")

        panel = QWidget()
        panel_layout = QVBoxLayout(panel)
        panel_layout.setContentsMargins(0, 4, 0, 0)
        title = QLabel(
            "农业结构：Ground Confidence → Robust Slope → Hybrid Row Skeleton → "
            "Structural Band / Vegetation Envelope → Refined Aisle"
        )
        title.setWordWrap(True)
        panel_layout.addWidget(title)
        evidence_note = QLabel(
            "垄检测默认融合：植被/障碍证据 55% + 地形隆起证据 45%；"
            "裸垄只要 Ground Surface 仍保留隆起，也可进入 Row Support"
        )
        evidence_note.setWordWrap(True)
        panel_layout.addWidget(evidence_note)

        row = QHBoxLayout()
        self._structure_slope_window = QDoubleSpinBox()
        self._structure_slope_window.setPrefix("坡度窗口：")
        self._structure_slope_window.setSuffix(" m")
        self._structure_slope_window.setDecimals(2)
        self._structure_slope_window.setRange(0.30, 1.50)
        self._structure_slope_window.setSingleStep(0.10)
        self._structure_slope_window.setValue(0.50)
        self._row_direction_mode = QComboBox()
        self._row_direction_mode.addItem("优先使用标定 +X（无标定则自动）", "calibration_or_auto")
        self._row_direction_mode.addItem("自动估计种植行方向", "auto")
        row.addWidget(self._structure_slope_window)
        row.addWidget(self._row_direction_mode)
        panel_layout.addLayout(row)

        row = QHBoxLayout()
        self._row_half_width = QDoubleSpinBox()
        self._row_half_width.setPrefix("检测带半宽：")
        self._row_half_width.setSuffix(" m")
        self._row_half_width.setDecimals(2)
        self._row_half_width.setRange(0.08, 0.80)
        self._row_half_width.setSingleStep(0.02)
        self._row_half_width.setValue(0.22)
        self._row_structural_half_width = QDoubleSpinBox()
        self._row_structural_half_width.setPrefix("结构半宽：")
        self._row_structural_half_width.setSuffix(" m")
        self._row_structural_half_width.setDecimals(2)
        self._row_structural_half_width.setRange(0.08, 0.60)
        self._row_structural_half_width.setSingleStep(0.02)
        self._row_structural_half_width.setValue(0.20)
        row.addWidget(self._row_half_width)
        row.addWidget(self._row_structural_half_width)
        panel_layout.addLayout(row)

        row = QHBoxLayout()
        self._row_max_gap = QDoubleSpinBox()
        self._row_max_gap.setPrefix("短缺口：")
        self._row_max_gap.setSuffix(" m")
        self._row_max_gap.setDecimals(2)
        self._row_max_gap.setRange(0.0, 2.0)
        self._row_max_gap.setSingleStep(0.10)
        self._row_max_gap.setValue(0.60)
        self._row_min_spacing = QDoubleSpinBox()
        self._row_min_spacing.setPrefix("最小垄距：")
        self._row_min_spacing.setSuffix(" m")
        self._row_min_spacing.setDecimals(2)
        self._row_min_spacing.setRange(0.30, 3.0)
        self._row_min_spacing.setSingleStep(0.05)
        self._row_min_spacing.setValue(0.55)
        row.addWidget(self._row_max_gap)
        row.addWidget(self._row_min_spacing)
        panel_layout.addLayout(row)

        row = QHBoxLayout()
        self._row_min_length = QDoubleSpinBox()
        self._row_min_length.setPrefix("最短连续垄：")
        self._row_min_length.setSuffix(" m")
        self._row_min_length.setDecimals(2)
        self._row_min_length.setRange(0.50, 10.0)
        self._row_min_length.setSingleStep(0.25)
        self._row_min_length.setValue(1.50)
        self._boundary_exclusion = QDoubleSpinBox()
        self._boundary_exclusion.setPrefix("边界排除：")
        self._boundary_exclusion.setSuffix(" m")
        self._boundary_exclusion.setDecimals(2)
        self._boundary_exclusion.setRange(0.0, 2.0)
        self._boundary_exclusion.setSingleStep(0.05)
        self._boundary_exclusion.setValue(0.45)
        row.addWidget(self._row_min_length)
        row.addWidget(self._boundary_exclusion)
        panel_layout.addLayout(row)

        row = QHBoxLayout()
        self._aisle_side_clearance = QDoubleSpinBox()
        self._aisle_side_clearance.setPrefix("垄侧净空：")
        self._aisle_side_clearance.setSuffix(" m")
        self._aisle_side_clearance.setDecimals(2)
        self._aisle_side_clearance.setRange(0.0, 1.0)
        self._aisle_side_clearance.setSingleStep(0.02)
        self._aisle_side_clearance.setValue(0.12)
        self._raw_obstacle_clearance = QDoubleSpinBox()
        self._raw_obstacle_clearance.setPrefix("障碍净空：")
        self._raw_obstacle_clearance.setSuffix(" m")
        self._raw_obstacle_clearance.setDecimals(2)
        self._raw_obstacle_clearance.setRange(0.0, 1.0)
        self._raw_obstacle_clearance.setSingleStep(0.02)
        self._raw_obstacle_clearance.setValue(0.12)
        row.addWidget(self._aisle_side_clearance)
        row.addWidget(self._raw_obstacle_clearance)
        panel_layout.addLayout(row)

        analyze = QPushButton("重新计算 Ground / Row / Corridor 证据")
        analyze.clicked.connect(self._recompute_navigation_structure)
        panel_layout.addWidget(analyze)

        self._only_analysis_layer = QCheckBox("仅看分析层（隐藏点云底图）")
        self._only_analysis_layer.toggled.connect(self._update_cloud_analysis_visibility)
        panel_layout.addWidget(self._only_analysis_layer)

        self._structure_status = QLabel("农业结构：等待 Ground-relative 导航图")
        self._structure_status.setWordWrap(True)
        panel_layout.addWidget(self._structure_status)

        diagnostic_row = QHBoxLayout()
        diagnostic_row.addWidget(QLabel("行道对诊断："))
        self._aisle_diagnostic_combo = QComboBox()
        self._aisle_diagnostic_combo.currentIndexChanged.connect(
            self._refresh_aisle_pair_detail
        )
        diagnostic_row.addWidget(self._aisle_diagnostic_combo, 1)
        panel_layout.addLayout(diagnostic_row)

        self._aisle_diagnostic_detail = QLabel("暂无行道对诊断")
        self._aisle_diagnostic_detail.setWordWrap(True)
        panel_layout.addWidget(self._aisle_diagnostic_detail)

        layout.insertWidget(max(0, layout.count() - 1), panel)

        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setWidget(tab)
        wrapper = QWidget()
        wrapper_layout = QVBoxLayout(wrapper)
        wrapper_layout.setContentsMargins(0, 0, 0, 0)
        wrapper_layout.addWidget(scroll)
        return wrapper

    def _structure_config(self) -> NavigationStructureConfig:
        return NavigationStructureConfig(
            robust_slope_window_m=float(self._structure_slope_window.value()),
            row_direction_mode="auto",
            row_half_width_m=float(self._row_half_width.value()),
            row_max_gap_m=float(self._row_max_gap.value()),
            row_minimum_segment_length_m=float(self._row_min_length.value()),
            row_minimum_spacing_m=float(self._row_min_spacing.value()),
        )

    def _corridor_config(self) -> CorridorRefinementConfig:
        return CorridorRefinementConfig(
            boundary_exclusion_m=float(self._boundary_exclusion.value()),
            row_structural_half_width_m=float(self._row_structural_half_width.value()),
            aisle_side_clearance_m=float(self._aisle_side_clearance.value()),
            raw_obstacle_clearance_m=float(self._raw_obstacle_clearance.value()),
            minimum_row_longitudinal_span_m=float(self._row_min_length.value()),
        )

    def _row_direction_for_structure(self):
        if self._row_direction_mode.currentData() != "calibration_or_auto":
            return None
        calibration = self._frame_calibration
        if calibration is None:
            return None
        direction = np.asarray(calibration.x_axis_in_source[:2], dtype=np.float64)
        if np.linalg.norm(direction) <= 1e-9:
            return None
        return direction

    def _recompute_navigation_structure(self) -> None:
        if self._navigation_result is None:
            QMessageBox.information(
                self,
                "导航图未生成",
                "请先生成 Ground-relative 导航图，再计算农业结构层",
            )
            return
        try:
            row_direction = self._row_direction_for_structure()
            structure = derive_navigation_structure(
                self._navigation_result,
                self._structure_config(),
                row_direction_xy=row_direction,
            )
            corridor = derive_corridor_refinement(
                self._navigation_result,
                structure,
                self._corridor_config(),
            )
            self._navigation_structure_result = structure
            self._corridor_refinement_result = corridor
        except Exception as exc:
            self._navigation_structure_result = None
            self._corridor_refinement_result = None
            self._structure_status.setText("农业结构：计算失败")
            self._refresh_aisle_pair_diagnostics()
            QMessageBox.critical(self, "农业结构分析失败", str(exc))
            return
        self._refresh_structure_status()
        self._refresh_aisle_pair_diagnostics()
        self._refresh_navigation_status()
        self._update_navigation_overlay()
        self.statusBar().showMessage(
            "农业结构完成：Ground / Hybrid Row / Structural Band / Refined Aisle"
        )

    def _navigation_finished(self, result) -> None:
        self._navigation_structure_result = None
        self._corridor_refinement_result = None
        super()._navigation_finished(result)
        self._recompute_navigation_structure()

    def _clear_navigation_state(self, *, clear_overrides: bool) -> None:
        self._navigation_structure_result = None
        self._corridor_refinement_result = None
        super()._clear_navigation_state(clear_overrides=clear_overrides)
        if hasattr(self, "_structure_status"):
            self._structure_status.setText("农业结构：等待 Ground-relative 导航图")
        if hasattr(self, "_aisle_diagnostic_combo"):
            self._refresh_aisle_pair_diagnostics()

    def _refresh_structure_status(self) -> None:
        if not hasattr(self, "_structure_status"):
            return
        structure = self._navigation_structure_result
        corridor = self._corridor_refinement_result
        if structure is None or corridor is None:
            self._structure_status.setText("农业结构：等待 Ground-relative 导航图")
            return
        model = structure.row_model
        direction_source = (
            "标定 +X"
            if self._row_direction_mode.currentData() == "calibration_or_auto"
            and self._frame_calibration is not None
            else "自动估计"
        )
        confident = int(np.count_nonzero(structure.ground_confidence >= 0.30))
        old_aisle = int(np.count_nonzero(structure.aisle_candidate))
        refined_aisle = int(np.count_nonzero(corridor.aisle_candidate))
        accepted_pairs = sum(
            diagnostic.status.startswith("ACCEPTED")
            for diagnostic in corridor.aisle_pair_diagnostics
        )
        spacing = (
            "n/a"
            if corridor.nominal_row_spacing_m is None
            else f"{corridor.nominal_row_spacing_m:.2f} m"
        )
        self._structure_status.setText(
            f"农业结构：方向={direction_source} / {model.angle_deg:.1f}° | "
            f"原始 Row {len(model.centers_v_m)} → 有效 Row {len(corridor.accepted_row_centers_v_m)} | "
            f"名义垄距 {spacing}\n"
            f"Hybrid Row=障碍55%+地形45% | Ground Confidence≥0.30：{confident:,} | "
            f"旧行道 {old_aisle:,} → 精炼行道 {refined_aisle:,} | "
            f"行道对 {accepted_pairs}/{len(corridor.aisle_pair_diagnostics)} 接受"
        )

    def _refresh_aisle_pair_diagnostics(self) -> None:
        if not hasattr(self, "_aisle_diagnostic_combo"):
            return
        previous = self._aisle_diagnostic_combo.currentIndex()
        self._aisle_diagnostic_combo.blockSignals(True)
        self._aisle_diagnostic_combo.clear()
        corridor = self._corridor_refinement_result
        if corridor is not None:
            for diagnostic in corridor.aisle_pair_diagnostics:
                self._aisle_diagnostic_combo.addItem(
                    f"A{diagnostic.pair_index:02d} | "
                    f"Row{diagnostic.pair_index:02d}↔Row{diagnostic.pair_index + 1:02d} | "
                    f"{diagnostic.status}",
                    diagnostic.pair_index - 1,
                )
        if self._aisle_diagnostic_combo.count() > 0:
            self._aisle_diagnostic_combo.setCurrentIndex(
                min(max(previous, 0), self._aisle_diagnostic_combo.count() - 1)
            )
        self._aisle_diagnostic_combo.blockSignals(False)
        self._refresh_aisle_pair_detail()

    def _refresh_aisle_pair_detail(self) -> None:
        if not hasattr(self, "_aisle_diagnostic_detail"):
            return
        corridor = self._corridor_refinement_result
        index = self._aisle_diagnostic_combo.currentIndex()
        if corridor is None or index < 0 or index >= len(corridor.aisle_pair_diagnostics):
            self._aisle_diagnostic_detail.setText("暂无行道对诊断")
            self._aisle_diagnostic_detail.setStyleSheet("")
            return
        diagnostic = corridor.aisle_pair_diagnostics[index]
        overlap = (
            "n/a"
            if diagnostic.longitudinal_overlap_m is None
            else f"{diagnostic.longitudinal_overlap_m:.2f} m"
        )
        status_text = _STATUS_TEXT.get(diagnostic.status, diagnostic.status)
        self._aisle_diagnostic_detail.setText(
            f"中心距 {diagnostic.center_distance_m:.2f} m | "
            f"结构占用 {diagnostic.structural_reserved_m:.2f} m | "
            f"两侧净空 {diagnostic.side_clearance_reserved_m:.2f} m\n"
            f"几何剩余 {diagnostic.geometric_available_width_m:.2f} m / "
            f"最小要求 {diagnostic.minimum_required_width_m:.2f} m | "
            f"纵向重叠 {overlap}\n"
            f"栅格：几何 {diagnostic.geometric_cell_count:,} | "
            f"安全 {diagnostic.safe_cell_count:,} | "
            f"中心线 {diagnostic.centerline_cell_count:,}\n"
            f"状态：{status_text}（{diagnostic.status}）"
        )
        if diagnostic.status == "ACCEPTED":
            self._aisle_diagnostic_detail.setStyleSheet("color: #2e8b57;")
        elif diagnostic.status.startswith("ACCEPTED"):
            self._aisle_diagnostic_detail.setStyleSheet("color: #b8860b;")
        else:
            self._aisle_diagnostic_detail.setStyleSheet("color: #b22222;")

    def _refresh_navigation_status(self) -> None:
        MapWorkbenchWindow._refresh_navigation_status(self)
        if not hasattr(self, "_nav_status") or self._navigation_structure_result is None:
            return
        model = self._navigation_structure_result.row_model
        suffix = f"\n农业结构：row angle={model.angle_deg:.1f}° | rows={len(model.centers_v_m)}"
        if self._corridor_refinement_result is not None:
            corridor = self._corridor_refinement_result
            suffix += (
                f" | accepted={len(corridor.accepted_row_centers_v_m)}"
                f" | refined aisle={int(np.count_nonzero(corridor.aisle_candidate)):,}"
            )
        self._nav_status.setText(self._nav_status.text() + suffix)

    def _update_navigation_overlay(self) -> None:
        if not hasattr(self, "_nav_layer"):
            return
        if self._navigation_result is None or not self._nav_overlay_visible.isChecked():
            self._navigation_preview_item.clear_result()
            return
        layer = str(self._nav_layer.currentData())
        if layer in _STRUCTURE_LAYERS:
            if self._navigation_structure_result is None:
                self._navigation_preview_item.clear_result()
                return
            self._navigation_preview_item.set_result(
                self._navigation_result,
                layer,
                self._navigation_structure_result,
            )
            return
        if layer in _CORRIDOR_LAYERS:
            if self._navigation_structure_result is None or self._corridor_refinement_result is None:
                self._navigation_preview_item.clear_result()
                return
            self._navigation_preview_item.set_result(
                self._navigation_result,
                layer,
                self._navigation_structure_result,
                self._corridor_refinement_result,
            )
            return
        self._navigation_preview_item.set_result(self._navigation_result, layer)

    def _update_cloud_analysis_visibility(self, checked: bool) -> None:
        self._cloud_item.setOpacity(0.0 if checked else 1.0)
        self._scene.update()


def main(argv=None) -> int:
    app = QApplication(list(sys.argv if argv is None else argv))
    window = AgriculturalMapWorkbenchWindow()
    window.show()
    return app.exec_()
