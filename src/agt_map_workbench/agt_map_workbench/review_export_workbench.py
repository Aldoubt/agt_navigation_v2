"""Review-focused V25 Workbench additions.

This bounded layer keeps the formal map authority in ``UnifiedMapWorkbenchWindow``
while adding two review-only capabilities:

* a geometric aisle centerline that is independent from Ground/safe evidence;
* a one-click screenshot + metadata package for repeatable map review.
"""

from __future__ import annotations

from dataclasses import asdict
from datetime import datetime
from pathlib import Path
import sys

import numpy as np
from PyQt5.QtWidgets import QApplication, QFileDialog, QMessageBox, QPushButton

from agt_offline_assets.aisle_centerlines import derive_geometric_aisle_centerlines
from agt_offline_assets.contracts import sha256_file

from .review_export import REVIEW_LAYER_EXPORTS, git_head, write_review_summary
from .unified_workbench import UnifiedMapWorkbenchWindow


_GEOMETRIC_CENTERLINE_KEY = "aisle_geometric_centerline"


class ReviewExportMapWorkbenchWindow(UnifiedMapWorkbenchWindow):
    """Canonical Workbench plus deterministic visual-review export."""

    def __init__(self) -> None:
        super().__init__()
        self._install_geometric_centerline_layer()
        self._install_review_export_action()

    # ------------------------------------------------------- geometric aisle semantics
    def _install_geometric_centerline_layer(self) -> None:
        if self._find_layer_index(_GEOMETRIC_CENTERLINE_KEY) >= 0:
            return
        envelope_index = self._find_layer_index("aisle_geometric_envelope")
        insert_at = envelope_index + 1 if envelope_index >= 0 else self._nav_layer.count()
        self._nav_layer.insertItem(
            insert_at,
            "结构行道几何中心线（不受 Ground/障碍断裂）",
            _GEOMETRIC_CENTERLINE_KEY,
        )
        safe_index = self._find_layer_index("aisle_centerline")
        if safe_index >= 0:
            self._nav_layer.setItemText(safe_index, "导航安全中心线（允许断开）")

    def _install_review_export_action(self) -> None:
        parent = self._nav_status.parentWidget()
        layout = parent.layout() if parent is not None else None
        if layout is None:
            return
        self._review_export_button = QPushButton("一键导出地图审查包")
        self._review_export_button.clicked.connect(self._export_map_review_package)
        layout.insertWidget(max(0, layout.count() - 1), self._review_export_button)

    def _find_layer_index(self, key: str) -> int:
        for index in range(self._nav_layer.count()):
            if str(self._nav_layer.itemData(index)) == str(key):
                return index
        return -1

    def _geometric_centerlines(self):
        if (
            self._navigation_base_result is None
            or self._navigation_structure_result is None
            or self._corridor_refinement_result is None
        ):
            return None
        return derive_geometric_aisle_centerlines(
            self._navigation_base_result,
            self._navigation_structure_result,
            self._corridor_refinement_result,
        )

    def _update_navigation_overlay(self) -> None:
        if not hasattr(self, "_nav_layer"):
            return
        layer = str(self._nav_layer.currentData())
        if layer != _GEOMETRIC_CENTERLINE_KEY:
            super()._update_navigation_overlay()
            return
        if not self._nav_overlay_visible.isChecked():
            self._navigation_preview_item.clear_result()
            return
        geometric = self._geometric_centerlines()
        if geometric is None or self._navigation_base_result is None:
            self._navigation_preview_item.clear_result()
            return
        self._navigation_preview_item.set_mask(
            self._navigation_base_result,
            geometric.mask,
            (40, 235, 255, 245),
        )

    def _refresh_navigation_status(self) -> None:
        super()._refresh_navigation_status()
        if not hasattr(self, "_nav_status"):
            return
        geometric = self._geometric_centerlines()
        if geometric is None:
            return

        qa = self._formal_navigation_qa or {}
        interior_reports = [
            dict(item)
            for item in (qa.get("aisles") or [])
            if str(item.get("pair_kind")) == "ROW_ROW"
        ]
        traversable = len(interior_reports)
        connected = sum(bool(item.get("grid_connectivity")) for item in interior_reports)
        expected = int(geometric.expected_interior_aisles)
        line_count = sum(
            pair.pair_kind == "ROW_ROW" and pair.centerline_cell_count > 0
            for pair in geometric.pairs
        )
        suffix = (
            f"\n行道语义：Geometric={geometric.interior_geometric_aisle_count}/{expected} | "
            f"Geometric Centerline={line_count}/{expected} | "
            f"Traversable={traversable}/{expected} | Connected={connected}/{expected}"
        )
        self._nav_status.setText(self._nav_status.text() + suffix)

    # ------------------------------------------------------- review package
    @staticmethod
    def _dataclass_dict(value) -> dict[str, object] | None:
        if value is None:
            return None
        return asdict(value)

    def _review_summary_payload(self) -> dict[str, object]:
        # A review package is explicitly allowed while formal_ready is false.
        # _build_formal_navigation_state still materializes Generated/Accepted/QA
        # before returning REVIEW_REQUIRED for usability.
        self._build_formal_navigation_state()
        if (
            self._navigation_base_result is None
            or self._navigation_structure_result is None
            or self._corridor_refinement_result is None
            or self._formal_materialization is None
            or self._formal_accepted_result is None
            or self._formal_navigation_qa is None
        ):
            raise ValueError(
                self._formal_last_error
                or "地图审查包需要 Ground、农业结构、Site Boundary 与 Formal map state"
            )

        navigation = self._navigation_base_result
        structure = self._navigation_structure_result
        corridor = self._corridor_refinement_result
        formal = self._formal_materialization
        accepted = self._formal_accepted_result
        qa = self._formal_navigation_qa
        geometric = derive_geometric_aisle_centerlines(navigation, structure, corridor)
        interior_reports = [
            dict(item)
            for item in (qa.get("aisles") or [])
            if str(item.get("pair_kind")) == "ROW_ROW"
        ]
        connected_interior = sum(
            bool(item.get("grid_connectivity")) for item in interior_reports
        )
        geometric_centerlines = sum(
            pair.pair_kind == "ROW_ROW" and pair.centerline_cell_count > 0
            for pair in geometric.pairs
        )

        source_path = self._source_path.resolve() if self._source_path is not None else None
        source_sha = None
        if source_path is not None and source_path.is_file():
            try:
                source_sha = sha256_file(source_path)
            except OSError:
                source_sha = None

        try:
            recipe = self._recipe.to_dict()
        except ValueError:
            recipe = {
                "recipe_id": getattr(self._recipe, "recipe_id", "workbench_draft"),
                "frame_id": getattr(self._recipe, "frame_id", "map"),
                "operations": [],
            }
        map_frame = (
            self._frame_calibration.to_dict(
                source_frame_id="source_map",
                target_frame_id="map",
                source_asset=source_path.name if source_path is not None else None,
            )
            if self._frame_calibration is not None
            else None
        )
        site_boundary = (
            {
                "frame_id": self._site_boundary.frame_id,
                "outer_boundary_xy": [list(point) for point in self._site_boundary.outer_boundary_xy],
                "source": dict(self._site_boundary.source or {}),
            }
            if self._site_boundary is not None
            else None
        )

        return {
            "schema": "agt_map_workbench_review/v1",
            "source": {
                "pcd": str(source_path) if source_path is not None else None,
                "pcd_sha256": source_sha,
                "points": int(self._cloud.points.shape[0]) if self._cloud is not None else 0,
                "display_sample_points": int(self._cloud_item.sample_count()),
            },
            "git": {
                "commit": git_head(Path.cwd()),
                "branch_hint": "refactor/v25-unified-map-authoring",
            },
            "processing_recipe": recipe,
            "map_frame": map_frame,
            "navigation_config": self._dataclass_dict(navigation.config),
            "structure_config": self._dataclass_dict(structure.config),
            "corridor_config": self._dataclass_dict(corridor.config),
            "site_boundary": site_boundary,
            "agricultural_structure": {
                "row_angle_deg": float(structure.row_model.angle_deg),
                "raw_rows": len(structure.row_model.centers_v_m),
                "accepted_rows": len(corridor.accepted_row_centers_v_m),
                "expected_interior_aisles": int(geometric.expected_interior_aisles),
                "geometric_aisles": int(geometric.interior_geometric_aisle_count),
                "geometric_centerlines": int(geometric_centerlines),
                "traversable_aisles": int(len(interior_reports)),
                "connected_aisles": int(connected_interior),
            },
            "ground_navigation": navigation.counts(),
            "formal_navigation": {
                "generated_counts": formal.navigation.counts(),
                "accepted_counts": accepted.navigation.counts(),
                "materialization_counts": formal.counts(),
                "structural_safety_status": str(
                    qa.get("structural_safety_status", qa.get("status", "NOT_EVALUATED"))
                ),
                "navigation_usability_status": str(
                    qa.get("navigation_usability_status", "NOT_EVALUATED")
                ),
                "largest_free_component_fraction": float(
                    qa.get("largest_free_component_fraction", 0.0)
                ),
                "map_unknown_fraction": float(qa.get("map_unknown_fraction", 0.0)),
                "outside_site_boundary_free_count": int(
                    qa.get("outside_site_boundary_free_count", 0)
                ),
                "row_structural_band_free_leak_count": int(
                    qa.get("row_structural_band_free_leak_count", 0)
                ),
                "override_diagnostics": [
                    dict(item) for item in (qa.get("override_diagnostics") or [])
                ],
                "formal_ready": bool(self._formal_ready),
                "review_status": str(self._formal_review_status),
                "last_error": str(self._formal_last_error),
            },
            "overrides": [dict(item) for item in self._navigation_overrides],
        }

    @staticmethod
    def _save_widget_grab(widget, path: Path) -> None:
        pixmap = widget.grab()
        if pixmap.isNull() or not pixmap.save(str(path), "PNG"):
            raise RuntimeError(f"无法写出审查截图：{path}")

    def _capture_review_layers(self, destination: Path) -> None:
        old_index = self._nav_layer.currentIndex()
        old_overlay = bool(self._nav_overlay_visible.isChecked())
        old_opacity = float(self._cloud_item.opacity())
        try:
            self._cloud_item.setOpacity(1.0)
            QApplication.processEvents()
            self._save_widget_grab(self, destination / "00_workbench_window.png")

            for filename, key in REVIEW_LAYER_EXPORTS:
                if key is None:
                    self._nav_overlay_visible.setChecked(False)
                else:
                    index = self._find_layer_index(key)
                    if index < 0:
                        raise RuntimeError(f"审查截图层不存在：{key}")
                    self._nav_layer.setCurrentIndex(index)
                    self._nav_overlay_visible.setChecked(True)
                    self._update_navigation_overlay()
                QApplication.processEvents()
                self._save_widget_grab(self._view, destination / filename)
        finally:
            self._cloud_item.setOpacity(old_opacity)
            self._nav_layer.setCurrentIndex(old_index)
            self._nav_overlay_visible.setChecked(old_overlay)
            self._update_navigation_overlay()
            QApplication.processEvents()

    def _export_map_review_package_to(self, destination: str | Path) -> Path:
        output = Path(destination).expanduser().resolve()
        if output.exists():
            raise FileExistsError(f"地图审查包目录已存在：{output}")
        output.mkdir(parents=True, exist_ok=False)
        try:
            summary = self._review_summary_payload()
            self._capture_review_layers(output)
            write_review_summary(output, summary)
        except Exception:
            # Review export is not a formal authority product, but still avoids
            # leaving a misleading half-written package behind.
            import shutil

            shutil.rmtree(output, ignore_errors=True)
            raise
        return output

    def _export_map_review_package(self) -> Path | None:
        parent = QFileDialog.getExistingDirectory(
            self,
            "选择地图审查包保存目录",
            str((Path.cwd() / "runtime" / "maps").resolve()),
        )
        if not parent:
            return None
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        destination = Path(parent) / f"map_review_{stamp}"
        try:
            output = self._export_map_review_package_to(destination)
        except Exception as exc:
            QMessageBox.critical(self, "地图审查包导出失败", str(exc))
            return None
        self.statusBar().showMessage(f"地图审查包已导出：{output}")
        QMessageBox.information(
            self,
            "地图审查包已导出",
            f"{output}\n\n已保存窗口截图、11 个审查视图以及 review_summary.json/txt。",
        )
        return output


def main(argv=None) -> int:
    app = QApplication(list(sys.argv if argv is None else argv))
    window = ReviewExportMapWorkbenchWindow()
    window.show()
    return app.exec_()
