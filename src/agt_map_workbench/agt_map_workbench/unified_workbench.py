"""Canonical Paper I Workbench with one structure-aware formal map authority.

This class layers the approved V25 structure-aware Generated/Accepted map
contract onto the existing Paper I review Workbench without duplicating the
underlying point-cloud, agricultural-structure, Site Boundary, or resource
bundle implementations.  The `agt_map_workbench_paper1` launcher points here so
formal map production has one user-facing GUI entry point.
"""

from __future__ import annotations

from dataclasses import replace
from pathlib import Path
import shutil
import sys

from PyQt5.QtWidgets import QApplication, QFileDialog, QInputDialog, QMessageBox

from agt_offline_assets.formal_navigation_map import (
    materialize_structure_aware_navigation_map,
)
from agt_offline_assets.formal_navigation_override import (
    replay_formal_navigation_overrides,
)
from agt_offline_assets.formal_navigation_qa import (
    evaluate_formal_navigation_qa,
    write_formal_navigation_qa,
)
from agt_offline_assets.formal_navigation_revision import (
    export_structure_aware_navigation_revision,
)

from .paper1_workbench import Paper1MapWorkbenchWindow


_FORMAL_LAYER_KEYS = {
    "formal_generated",
    "formal_inferred",
    "formal_row_block",
    "formal_boundary_block",
    "formal_unknown",
}
_HUMAN_BOUNDARY_SOURCES = {
    "WORKBENCH_MANUAL_POLYGON",
    "HUMAN_CONFIRMED_WORKBENCH",
}


def _is_human_confirmed_boundary(boundary) -> bool:
    source = getattr(boundary, "source", {}) or {}
    return str(
        source.get("boundary_source", source.get("authoring_mode", ""))
    ) in _HUMAN_BOUNDARY_SOURCES


class UnifiedMapWorkbenchWindow(Paper1MapWorkbenchWindow):
    """Single formal V25 authoring entry point used by Paper I."""

    def __init__(self) -> None:
        self._formal_materialization = None
        self._formal_accepted_result = None
        self._formal_navigation_qa = None
        self._formal_last_error = ""
        self._formal_ready = False
        self._formal_review_status = "HUMAN_REVIEW_REQUIRED"
        self._formal_boundary_source = None
        super().__init__()
        self._install_formal_navigation_layers()
        self.setWindowTitle(
            "AGT 地图工作台 — V25 统一地图生产 / Structure-Aware PGM"
        )

    # ------------------------------------------------------- formal state
    def _install_formal_navigation_layers(self) -> None:
        existing = {
            str(self._nav_layer.itemData(index))
            for index in range(self._nav_layer.count())
        }
        for label, key in (
            ("正式 Generated PGM（结构融合）", "formal_generated"),
            ("Structure-Inferred FREE", "formal_inferred"),
            ("Row Structural Block", "formal_row_block"),
            ("Site Boundary Block", "formal_boundary_block"),
            ("Unresolved UNKNOWN", "formal_unknown"),
        ):
            if key not in existing:
                self._nav_layer.addItem(label, key)

    def _invalidate_formal_navigation_state(self, reason: str = "") -> None:
        self._formal_materialization = None
        self._formal_accepted_result = None
        self._formal_navigation_qa = None
        self._formal_last_error = str(reason)
        self._formal_ready = False
        self._formal_review_status = "HUMAN_REVIEW_REQUIRED"
        self._formal_boundary_source = None

    def _build_formal_navigation_state(self) -> bool:
        """Build Generated, Accepted and QA from the current Workbench evidence."""

        if hasattr(self, "_invalidate_formal_navigation_state"):
            self._invalidate_formal_navigation_state()
        else:
            self._formal_materialization = None
            self._formal_accepted_result = None
            self._formal_navigation_qa = None
            self._formal_last_error = ""
            self._formal_ready = False
            self._formal_review_status = "HUMAN_REVIEW_REQUIRED"
            self._formal_boundary_source = None
        if self._navigation_base_result is None:
            self._formal_last_error = "缺少 Ground-relative Navigation Evidence"
            return False
        if self._navigation_structure_result is None:
            self._formal_last_error = "缺少农业 Row / terrain structure evidence"
            return False
        if self._corridor_refinement_result is None:
            self._formal_last_error = "缺少 Corridor / aisle geometry evidence"
            return False
        if self._site_boundary is None:
            self._formal_last_error = "缺少 READY Site Boundary"
            return False

        try:
            overrides = self._validated_formal_overrides()
            materialized = materialize_structure_aware_navigation_map(
                self._navigation_base_result,
                self._corridor_refinement_result,
                self._site_boundary,
                frame_id="map",
            )
            accepted = replay_formal_navigation_overrides(
                materialized.navigation,
                self._corridor_refinement_result,
                self._site_boundary,
                overrides,
                frame_id="map",
            )
            qa = evaluate_formal_navigation_qa(
                ground_evidence=self._navigation_base_result,
                materialized=materialized,
                accepted=accepted,
                overrides=overrides,
                structure=self._navigation_structure_result,
                corridor=self._corridor_refinement_result,
                site_boundary=self._site_boundary,
            )
        except Exception as exc:
            self._formal_last_error = str(exc)
            return False

        self._formal_materialization = materialized
        self._formal_accepted_result = accepted
        self._formal_navigation_qa = qa
        self._formal_boundary_source = str(
            (getattr(self._site_boundary, "source", {}) or {}).get(
                "boundary_source",
                (getattr(self._site_boundary, "source", {}) or {}).get(
                    "authoring_mode", "UNKNOWN"
                ),
            )
        )
        if qa.get("status") != "PASS":
            failures = qa.get("hard_failures") or []
            self._formal_last_error = (
                "Formal Navigation QA FAIL: " + ", ".join(str(v) for v in failures)
            )
            return False
        if not _is_human_confirmed_boundary(self._site_boundary):
            self._formal_ready = False
            self._formal_review_status = "HUMAN_REVIEW_REQUIRED"
            self._formal_last_error = (
                "QA PASS，但 Site Boundary 未由 Workbench 人工确认；formal_ready=false"
            )
            return False
        self._formal_ready = True
        self._formal_review_status = "READY"
        self._formal_last_error = ""
        return True

    def _clear_navigation_state(self, *, clear_overrides: bool) -> None:
        self._invalidate_formal_navigation_state("Navigation Evidence 已失效")
        super()._clear_navigation_state(clear_overrides=clear_overrides)

    def _recompute_navigation_structure(self) -> None:
        self._invalidate_formal_navigation_state("农业结构正在重算")
        super()._recompute_navigation_structure()
        if (
            self._navigation_structure_result is not None
            and self._corridor_refinement_result is not None
            and self._site_boundary is not None
        ):
            self._build_formal_navigation_state()
            self._refresh_navigation_status()
            self._update_navigation_overlay()

    def _finish_site_boundary_authoring(
        self,
        _checked=False,
        *,
        show_errors: bool = True,
    ) -> bool:
        result = super()._finish_site_boundary_authoring(
            _checked,
            show_errors=show_errors,
        )
        self._invalidate_formal_navigation_state("Site Boundary 已变化")
        if result:
            self._build_formal_navigation_state()
            self._refresh_navigation_status()
            self._update_navigation_overlay()
        return result

    def _clear_site_boundary(self, _checked=False) -> None:
        super()._clear_site_boundary(_checked)
        self._invalidate_formal_navigation_state("缺少 READY Site Boundary")
        self._refresh_navigation_status()
        self._update_navigation_overlay()

    def _load_site_boundary_sibling(self) -> None:
        self._invalidate_formal_navigation_state("Site Boundary 正在重新加载")
        super()._load_site_boundary_sibling()
        if self._site_boundary is not None:
            self._build_formal_navigation_state()

    def _finish_navigation_override(self) -> None:
        before = len(self._navigation_overrides)
        super()._finish_navigation_override()
        if len(self._navigation_overrides) != before:
            self._invalidate_formal_navigation_state("Formal Override 已变化")
            self._build_formal_navigation_state()
            self._refresh_navigation_status()
            self._update_navigation_overlay()

    def _undo_navigation_override(self) -> None:
        super()._undo_navigation_override()
        self._invalidate_formal_navigation_state("Formal Override 已撤销")
        self._build_formal_navigation_state()
        self._refresh_navigation_status()
        self._update_navigation_overlay()

    def _clear_navigation_overrides(self) -> None:
        super()._clear_navigation_overrides()
        self._invalidate_formal_navigation_state("Formal Override 已清空")
        self._build_formal_navigation_state()
        self._refresh_navigation_status()
        self._update_navigation_overlay()

    # ------------------------------------------------------- preview / status
    def _update_navigation_overlay(self) -> None:
        if not hasattr(self, "_nav_layer"):
            return
        layer = str(self._nav_layer.currentData())
        if layer not in _FORMAL_LAYER_KEYS:
            super()._update_navigation_overlay()
            return
        if not self._nav_overlay_visible.isChecked():
            self._navigation_preview_item.clear_result()
            return
        if self._formal_materialization is None:
            self._build_formal_navigation_state()
        formal = self._formal_materialization
        if formal is None:
            self._navigation_preview_item.clear_result()
            return

        if layer == "formal_generated":
            self._navigation_preview_item.set_result(formal.navigation, "final")
        elif layer == "formal_inferred":
            self._navigation_preview_item.set_mask(
                formal.navigation,
                formal.structure_inferred_free_mask,
                (40, 220, 255, 220),
            )
        elif layer == "formal_row_block":
            self._navigation_preview_item.set_mask(
                formal.navigation,
                formal.row_structural_blocked_mask,
                (255, 80, 190, 215),
            )
        elif layer == "formal_boundary_block":
            self._navigation_preview_item.set_mask(
                formal.navigation,
                formal.site_boundary_blocked_mask,
                (255, 55, 55, 195),
            )
        else:
            self._navigation_preview_item.set_mask(
                formal.navigation,
                formal.unresolved_unknown_mask,
                (155, 160, 175, 180),
            )

    def _refresh_navigation_status(self) -> None:
        super()._refresh_navigation_status()
        if not hasattr(self, "_nav_status"):
            return
        qa = self._formal_navigation_qa
        if qa is None:
            if self._formal_last_error:
                self._nav_status.setText(
                    self._nav_status.text() + f"\n正式 Generated：未就绪 | {self._formal_last_error}"
                )
            return
        inferred = self._formal_materialization.counts()["structure_inferred_free"]
        connected = sum(
            bool(item.get("grid_connectivity")) for item in qa.get("aisles", [])
        )
        total = int(qa.get("accepted_aisle_count", 0))
        self._nav_status.setText(
            self._nav_status.text()
            + f"\n正式 Generated：{qa['status']} | Structure-Inferred FREE={inferred:,} | "
            f"aisle connectivity={connected}/{total} | "
            f"UNKNOWN={100.0 * float(qa.get('map_unknown_fraction', 0.0)):.1f}%"
        )

    # ------------------------------------------------------- formal export
    def _export_formal_navigation_revision_to(self, destination: Path | str) -> Path:
        destination = Path(destination).expanduser().resolve()
        build = getattr(self, "_build_formal_navigation_state", None)
        if build is None:
            build = UnifiedMapWorkbenchWindow._build_formal_navigation_state.__get__(self)
        if not build():
            raise ValueError(self._formal_last_error or "Formal Navigation state is not ready")
        assert self._formal_materialization is not None
        assert self._formal_accepted_result is not None
        assert self._formal_navigation_qa is not None
        overrides = self._validated_formal_overrides()
        output = export_structure_aware_navigation_revision(
            destination,
            ground_evidence=self._navigation_base_result,
            materialized=self._formal_materialization,
            accepted=self._formal_accepted_result,
            overrides=overrides,
            source_asset=(
                self._source_path.name if self._source_path is not None else None
            ),
            frame_id="map",
            boundary_source="HUMAN_CONFIRMED_WORKBENCH",
        )
        try:
            write_formal_navigation_qa(
                self._formal_navigation_qa,
                output / "validation" / "navigation_validation.json",
            )
        except Exception:
            shutil.rmtree(output, ignore_errors=True)
            raise
        return output

    def _export_navigation_map(self) -> Path | None:
        if not self._build_formal_navigation_state():
            QMessageBox.warning(
                self,
                "正式 Navigation Map 尚未就绪",
                self._formal_last_error
                or "需要 Ground Evidence、农业结构和 READY Site Boundary",
            )
            return None

        parent = QFileDialog.getExistingDirectory(
            self,
            "选择 Structure-Aware Navigation Map revision 父目录",
        )
        if not parent:
            return None
        run_name, accepted = QInputDialog.getText(
            self,
            "统一 map revision 名称",
            "请输入新的不可变 revision 目录名：",
            text="greenhouse_01_map_revision_unified_001",
        )
        run_name = run_name.strip()
        if not accepted or not run_name:
            return None
        if Path(run_name).name != run_name or run_name in {".", ".."}:
            QMessageBox.warning(self, "名称无效", "只能使用单层目录名")
            return None

        try:
            output = self._export_formal_navigation_revision_to(Path(parent) / run_name)
        except Exception as exc:
            QMessageBox.critical(self, "Structure-Aware map revision 导出失败", str(exc))
            return None

        self.statusBar().showMessage(f"统一 Navigation Map revision 已导出：{output}")
        QMessageBox.information(
            self,
            "Structure-Aware map revision 已导出",
            f"{output}\n\n"
            "evidence/ 为 Ground-only 证据；generated/ 为农业结构融合自动地图；"
            "accepted/ 仅由已审计 Formal Override 重放得到。\n"
            "validation/navigation_validation.json 已记录 planner-independent QA。",
        )
        return output

    # ------------------------------------------------------- resource bundle
    def _navigation_revision_writer(self):
        def write(root: Path) -> None:
            staging = root / ".navigation_revision_stage"
            self._export_formal_navigation_revision_to(staging)
            target = root / "navigation"
            target.mkdir(parents=True, exist_ok=True)
            for name in ("evidence", "generated", "accepted", "validation"):
                source = staging / name
                if source.exists():
                    shutil.move(str(source), str(target / name))
            shutil.move(
                str(staging / "derivation.yaml"),
                str(target / "derivation.yaml"),
            )
            shutil.rmtree(staging)

        return write

    def _bundle_groups(self):
        formal_ready = self._build_formal_navigation_state()
        formal_reason = self._formal_last_error
        groups = super()._bundle_groups()
        output = []
        for group in groups:
            if group.key != "navigation_revision":
                output.append(group)
                continue
            output.append(
                replace(
                    group,
                    available=bool(formal_ready),
                    writer=(self._navigation_revision_writer() if formal_ready else None),
                    unavailable_reason=("" if formal_ready else formal_reason),
                )
            )
        return output


def main(argv=None) -> int:
    app = QApplication(list(sys.argv if argv is None else argv))
    window = UnifiedMapWorkbenchWindow()
    window.show()
    return app.exec_()
