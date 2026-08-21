"""Paper I formal-map authoring extension for the V25-12F review Workbench."""

from __future__ import annotations

import shutil
import sys
from pathlib import Path

import numpy as np
from PyQt5.QtWidgets import (
    QApplication,
    QDialog,
    QFileDialog,
    QInputDialog,
    QLabel,
    QMessageBox,
    QPushButton,
    QVBoxLayout,
    QWidget,
)

from agt_offline_assets import (
    AisleGraphConfig,
    derive_agricultural_aisle_graph,
    write_agricultural_aisle_graph,
    write_site_boundary,
    write_traversability_candidate,
)
from agt_offline_assets.navigation_map_derivation import (
    write_navigation_map_freeze_bundle,
)

from .navigation_override_metadata import (
    EVIDENCE_CATEGORIES,
    build_navigation_override_record,
    next_override_id,
    validate_navigation_override_records,
)
from .resource_bundle import (
    ResourceBundleGroup,
    export_map_resource_bundle,
)
from .resource_bundle_dialog import ResourceBundleDialog
from .review_workbench import ReviewMapWorkbenchWindow


class Paper1MapWorkbenchWindow(ReviewMapWorkbenchWindow):
    """Restrict formal map edits to auditable FREE/OCCUPIED overrides."""

    def __init__(self) -> None:
        self._bundle_original_pcd: Path | None = None
        self._bundle_processed_pcd: Path | None = None
        super().__init__()
        self.setWindowTitle(
            "AGT 地图工作台 — Paper I greenhouse formal-map curation"
        )
        self._install_resource_bundle_action()

    # ------------------------------------------------------- PCD identity for bundle export
    def _open_pcd(self) -> None:
        previous = self._source_path
        super()._open_pcd()
        if self._source_path == previous or self._source_path is None:
            return
        source = self._source_path.resolve()
        if source.name.lower() == "processed.pcd":
            self._bundle_processed_pcd = source
            self._bundle_original_pcd = None
        else:
            self._bundle_original_pcd = source
            self._bundle_processed_pcd = None

    def _processing_finished(self, result, recipe_path: Path) -> None:
        self._bundle_processed_pcd = Path(result.output_path).resolve()
        if self._bundle_original_pcd is None and self._source_path is not None:
            current = self._source_path.resolve()
            if current != self._bundle_processed_pcd:
                self._bundle_original_pcd = current
        super()._processing_finished(result, recipe_path)

    # ------------------------------------------------------- formal override authoring
    def _finish_navigation_override(self) -> None:
        if len(self._nav_vertices) < 3:
            QMessageBox.warning(self, "Override 未完成", "至少需要 3 个顶点")
            return

        mode = str(self._nav_override_mode.currentData())
        if mode not in {"force_free", "force_occupied"}:
            QMessageBox.warning(
                self,
                "Paper I formal Override 不允许该模式",
                "正式地图冻结只允许 FORCE_FREE / FORCE_OCCUPIED；"
                "UNKNOWN / NO_GO 仍可用于通用 Workbench，但不能进入本论文正式地图修订。",
            )
            return

        reason, accepted = QInputDialog.getText(
            self,
            "Override 依据",
            "请输入该修订的现场/点云依据（不能为空）：",
        )
        reason = reason.strip()
        if not accepted or not reason:
            return

        categories = sorted(EVIDENCE_CATEGORIES)
        evidence, accepted = QInputDialog.getItem(
            self,
            "Override 证据类别",
            "选择该修订的独立证据来源：",
            categories,
            0,
            False,
        )
        if not accepted:
            return

        try:
            record = build_navigation_override_record(
                override_id=next_override_id(self._navigation_overrides),
                mode=mode,
                polygon_xy=self._nav_vertices,
                reason=reason,
                evidence_category=str(evidence),
            )
        except ValueError as exc:
            QMessageBox.warning(self, "Override 元数据无效", str(exc))
            return

        self._navigation_overrides.append(record)
        self._clear_navigation_override_draft()
        self._refresh_navigation_override_list()
        self._apply_navigation_overrides_to_result()
        self._refresh_navigation_status()
        self._update_navigation_overlay()

    def _validated_formal_overrides(self) -> list[dict]:
        return list(validate_navigation_override_records(self._navigation_overrides))

    def _export_navigation_map(self) -> Path | None:
        if self._navigation_base_result is None:
            QMessageBox.information(
                self,
                "导航图未生成",
                "请先生成 Ground-relative 导航图预览",
            )
            return None

        try:
            overrides = self._validated_formal_overrides()
        except ValueError as exc:
            QMessageBox.warning(
                self,
                "Paper I Override 审计失败",
                f"当前 Override 不能形成正式地图修订：\n{exc}",
            )
            return None

        parent = QFileDialog.getExistingDirectory(
            self,
            "选择 Paper I Navigation Map freeze revision 父目录",
        )
        if not parent:
            return None
        run_name, accepted = QInputDialog.getText(
            self,
            "Paper I map revision 名称",
            "请输入新的不可变 freeze revision 目录名：",
            text="greenhouse_01_map_revision_001",
        )
        run_name = run_name.strip()
        if not accepted or not run_name:
            return None
        if Path(run_name).name != run_name or run_name in {".", ".."}:
            QMessageBox.warning(self, "名称无效", "只能使用单层目录名")
            return None

        destination = Path(parent) / run_name
        try:
            output = write_navigation_map_freeze_bundle(
                self._navigation_base_result,
                destination,
                source_asset=(
                    self._source_path.name if self._source_path is not None else None
                ),
                frame_id="map",
                overrides=overrides,
            )
        except Exception as exc:
            QMessageBox.critical(self, "Paper I map revision 导出失败", str(exc))
            return None

        self.statusBar().showMessage(
            f"Paper I generated/accepted map revision 已导出：{output}"
        )
        QMessageBox.information(
            self,
            "Paper I map revision 已导出",
            f"{output}\n\n"
            "generated/ 是人工修订前的确定性地图；accepted/ 仅由已记录 Override 重放得到。\n"
            "下一步必须运行 planner-independent map QA 后才能填写 acceptance。",
        )
        return output

    # ------------------------------------------------------- resource bundle UI
    def _install_resource_bundle_action(self) -> None:
        panel = QWidget()
        layout = QVBoxLayout(panel)
        layout.setContentsMargins(0, 8, 0, 0)
        note = QLabel(
            "地图资源包：按需另存当前 Workbench 已生成资源；原始 PCD 默认不复制。"
        )
        note.setWordWrap(True)
        button = QPushButton("另存为地图资源包…")
        button.clicked.connect(self._save_resource_bundle)
        layout.addWidget(note)
        layout.addWidget(button)
        target_layout = self._navigation_authoring_layout()
        target_layout.insertWidget(max(0, target_layout.count() - 1), panel)

        for action in self.menuBar().actions():
            menu = action.menu()
            if menu is not None and action.text().replace("&", "") == "离线资产":
                menu.addSeparator()
                save_as = menu.addAction("另存为地图资源包…")
                save_as.triggered.connect(self._save_resource_bundle)
                break

    @staticmethod
    def _copy_writer(source: Path, relative_path: str):
        def write(root: Path) -> None:
            target = root / relative_path
            target.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(source, target)

        return write

    @staticmethod
    def _save_array(root: Path, relative_path: str, value) -> None:
        target = root / relative_path
        target.parent.mkdir(parents=True, exist_ok=True)
        np.save(target, np.asarray(value))

    def _navigation_revision_writer(self):
        base = self._navigation_base_result
        overrides = self._validated_formal_overrides()

        def write(root: Path) -> None:
            staging = root / ".navigation_revision_stage"
            write_navigation_map_freeze_bundle(
                base,
                staging,
                source_asset=(self._source_path.name if self._source_path else None),
                frame_id="map",
                overrides=overrides,
            )
            target = root / "navigation"
            target.mkdir(parents=True, exist_ok=True)
            shutil.move(str(staging / "generated"), str(target / "generated"))
            shutil.move(str(staging / "accepted"), str(target / "accepted"))
            shutil.move(str(staging / "derivation.yaml"), str(target / "derivation.yaml"))
            shutil.rmtree(staging)

        return write

    def _terrain_writer(self):
        navigation = self._navigation_base_result
        structure = self._navigation_structure_result

        def write(root: Path) -> None:
            self._save_array(root, "layers/terrain/ground_height.npy", navigation.ground_height_m)
            self._save_array(root, "layers/terrain/ground_valid.npy", navigation.ground_valid)
            self._save_array(root, "layers/terrain/slope_deg.npy", navigation.slope_deg)
            self._save_array(root, "layers/terrain/step_m.npy", navigation.step_m)
            self._save_array(root, "layers/obstacle/obstacle_count.npy", navigation.obstacle_count)
            self._save_array(
                root,
                "layers/obstacle/obstacle_mask.npy",
                navigation.obstacle_count >= navigation.config.minimum_obstacle_points,
            )
            if structure is not None:
                self._save_array(
                    root,
                    "layers/terrain/ground_confidence.npy",
                    structure.ground_confidence,
                )
                self._save_array(
                    root,
                    "layers/terrain/robust_slope_deg.npy",
                    structure.robust_slope_deg,
                )
                self._save_array(
                    root,
                    "layers/terrain/robust_plane_residual_m.npy",
                    structure.robust_plane_residual_m,
                )

        return write

    def _structure_writer(self):
        navigation = self._navigation_result
        structure = self._navigation_structure_result
        corridor = self._corridor_refinement_result

        def write(root: Path) -> None:
            arrays = {
                "layers/structure/row_support.npy": structure.row_support,
                "layers/structure/row_regularized_obstacle.npy": structure.row_regularized_obstacle,
                "layers/structure/aisle_candidate_initial.npy": structure.aisle_candidate,
                "layers/structure/row_centerline.npy": corridor.row_centerline,
                "layers/structure/row_structural_band.npy": corridor.row_structural_band,
                "layers/structure/vegetation_envelope.npy": corridor.vegetation_envelope,
                "layers/structure/boundary_exclusion.npy": corridor.boundary_exclusion,
                "layers/structure/aisle_geometric_envelope.npy": corridor.aisle_geometric_envelope,
                "layers/structure/aisle_candidate.npy": corridor.aisle_candidate,
                "layers/structure/aisle_centerline.npy": corridor.aisle_centerline,
                "layers/structure/boundary_aisle_candidate.npy": corridor.boundary_aisle_candidate,
                "layers/structure/boundary_aisle_centerline.npy": corridor.boundary_aisle_centerline,
            }
            for relative_path, value in arrays.items():
                self._save_array(root, relative_path, value)

            graph = derive_agricultural_aisle_graph(
                navigation,
                structure,
                corridor,
                AisleGraphConfig(
                    centerline_sample_spacing_m=max(0.10, float(navigation.resolution_m))
                ),
                source={
                    "pcd_name": self._source_path.name if self._source_path else "",
                    "navigation_resolution_m": float(navigation.resolution_m),
                    "corridor_pair_count": len(corridor.aisle_pair_diagnostics),
                },
            )
            if graph.aisles:
                graph_path = root / "layers" / "structure" / "aisle_graph.yaml"
                graph_path.parent.mkdir(parents=True, exist_ok=True)
                write_agricultural_aisle_graph(graph, graph_path)

        return write

    def _traversability_writer(self):
        evidence = self._traversability_evidence
        navigation = self._navigation_result
        boundary = self._site_boundary

        def write(root: Path) -> None:
            target = root / "layers" / "traversability"
            target.mkdir(parents=True, exist_ok=True)
            write_traversability_candidate(
                evidence,
                navigation,
                boundary,
                target,
                source_navigation_asset="navigation/accepted/navigation_map.yaml",
                overwrite=False,
            )

        return write

    def _recipe_writer(self):
        def write(root: Path) -> None:
            target = root / "authoring" / "processing_recipe.yaml"
            target.parent.mkdir(parents=True, exist_ok=True)
            self._recipe.write_yaml(target)

        return write

    def _map_frame_writer(self):
        def write(root: Path) -> None:
            target = root / "authoring" / "map_frame.yaml"
            target.parent.mkdir(parents=True, exist_ok=True)
            self._frame_calibration.write_yaml(
                target,
                source_frame_id="source_map",
                target_frame_id="map",
                source_asset=self._source_path.name if self._source_path else None,
            )

        return write

    def _site_boundary_writer(self):
        def write(root: Path) -> None:
            target = root / "authoring" / "site_boundary.yaml"
            target.parent.mkdir(parents=True, exist_ok=True)
            write_site_boundary(self._site_boundary, target, overwrite=False)

        return write

    def _vehicle_segments_path(self) -> Path | None:
        if self._source_path is None:
            return None
        candidate = self._source_path.parent / "vehicle_feasible_segments.yaml"
        return candidate.resolve() if candidate.is_file() else None

    def _bundle_groups(self) -> list[ResourceBundleGroup]:
        processed = self._bundle_processed_pcd
        if processed is not None and not processed.is_file():
            processed = None

        navigation_reason = ""
        navigation_available = self._navigation_base_result is not None
        if navigation_available:
            try:
                self._validated_formal_overrides()
            except ValueError as exc:
                navigation_available = False
                navigation_reason = str(exc)
        elif self._navigation_base_result is None:
            navigation_reason = "请先生成 Ground-relative Navigation Map"

        structure_available = (
            self._navigation_structure_result is not None
            and self._corridor_refinement_result is not None
            and self._navigation_result is not None
        )
        traversability_available = (
            self._traversability_evidence is not None
            and self._navigation_12f_result is not None
            and self._site_boundary is not None
            and self._navigation_result is not None
        )
        vehicle_path = self._vehicle_segments_path()

        return [
            ResourceBundleGroup(
                "processed_pointcloud",
                "处理后的点云 processed.pcd",
                processed is not None,
                True,
                None if processed is None else self._copy_writer(processed, "pointcloud/processed.pcd"),
                "请先执行完整分辨率处理，或直接打开 processed.pcd",
            ),
            ResourceBundleGroup(
                "navigation_revision",
                "正式 Navigation Map（Generated + Accepted + Derivation）",
                navigation_available,
                True,
                self._navigation_revision_writer() if navigation_available else None,
                navigation_reason,
            ),
            ResourceBundleGroup(
                "terrain_layers",
                "地形 / Ground / Slope / Step / Obstacle 派生层",
                self._navigation_base_result is not None,
                True,
                self._terrain_writer() if self._navigation_base_result is not None else None,
                "请先生成 Ground-relative Navigation Map",
            ),
            ResourceBundleGroup(
                "structure_layers",
                "农业结构层（Row / Aisle / Aisle Graph）",
                structure_available,
                True,
                self._structure_writer() if structure_available else None,
                "请先完成 Ground / Row / Corridor 证据计算",
            ),
            ResourceBundleGroup(
                "traversability_12f",
                "V25-12F Traversability / Candidate 资源",
                traversability_available,
                False,
                self._traversability_writer() if traversability_available else None,
                "请先生成 12F Candidate，并具备 READY Site Boundary",
            ),
            ResourceBundleGroup(
                "vehicle_feasible_segments",
                "Vehicle-Feasible Segments",
                vehicle_path is not None,
                False,
                None if vehicle_path is None else self._copy_writer(vehicle_path, "planning/vehicle_feasible_segments.yaml"),
                "当前 PCD 同目录没有 vehicle_feasible_segments.yaml",
            ),
            ResourceBundleGroup(
                "recipe",
                "处理 Recipe processing_recipe.yaml",
                bool(self._recipe.operations),
                True,
                self._recipe_writer() if self._recipe.operations else None,
                "当前没有点云处理 Recipe",
            ),
            ResourceBundleGroup(
                "map_frame",
                "Map Frame 标定 map_frame.yaml",
                self._frame_calibration is not None,
                True,
                self._map_frame_writer() if self._frame_calibration is not None else None,
                "当前尚未完成 Map Frame 标定",
            ),
            ResourceBundleGroup(
                "site_boundary",
                "Site Boundary",
                self._site_boundary is not None,
                True,
                self._site_boundary_writer() if self._site_boundary is not None else None,
                "当前尚未定义 READY Site Boundary",
            ),
        ]

    def _default_bundle_parent(self) -> Path:
        if self._source_path is not None:
            return self._source_path.parent
        candidate = Path.cwd() / "runtime" / "maps"
        return candidate if candidate.is_dir() else Path.cwd()

    def _default_bundle_name(self) -> str:
        stem = self._source_path.stem if self._source_path is not None else "greenhouse_01"
        if stem == "processed":
            stem = self._source_path.parent.name or "greenhouse_01"
        return f"{stem}_map_assets_v01"

    def _save_resource_bundle(self, _checked=False) -> Path | None:
        groups = self._bundle_groups()
        original = self._bundle_original_pcd
        if original is not None and not original.is_file():
            original = None
        dialog = ResourceBundleDialog(
            groups,
            original_pcd_available=original is not None,
            default_parent=self._default_bundle_parent(),
            default_name=self._default_bundle_name(),
            parent=self,
        )
        if dialog.exec_() != QDialog.Accepted:
            return None

        try:
            output = export_map_resource_bundle(
                dialog.parent_dir(),
                dialog.bundle_name(),
                groups=groups,
                selection=dialog.selection(),
                current_pcd=self._source_path,
                original_pcd=original,
            )
        except Exception as exc:
            QMessageBox.critical(self, "地图资源包另存失败", str(exc))
            return None

        self.statusBar().showMessage(f"地图资源包已另存：{output}")
        QMessageBox.information(
            self,
            "地图资源包已另存",
            f"{output}\n\n"
            "map_resource_manifest.yaml 已记录所有实际写入文件及 SHA256。\n"
            "原始 PCD 只有在你显式勾选时才会复制。",
        )
        return output


def main(argv=None) -> int:
    app = QApplication(list(sys.argv if argv is None else argv))
    window = Paper1MapWorkbenchWindow()
    window.show()
    return app.exec_()
