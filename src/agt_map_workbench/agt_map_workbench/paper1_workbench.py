"""Paper I formal-map authoring extension for the V25-12F review Workbench."""

from __future__ import annotations

import sys
from pathlib import Path

from PyQt5.QtWidgets import QApplication, QFileDialog, QInputDialog, QMessageBox

from agt_offline_assets.navigation_map_derivation import (
    write_navigation_map_freeze_bundle,
)

from .navigation_override_metadata import (
    EVIDENCE_CATEGORIES,
    build_navigation_override_record,
    next_override_id,
    validate_navigation_override_records,
)
from .review_workbench import ReviewMapWorkbenchWindow


class Paper1MapWorkbenchWindow(ReviewMapWorkbenchWindow):
    """Restrict formal map edits to auditable FREE/OCCUPIED overrides."""

    def __init__(self) -> None:
        super().__init__()
        self.setWindowTitle(
            "AGT 地图工作台 — Paper I greenhouse formal-map curation"
        )

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

    def _export_navigation_map(self) -> Path | None:
        if self._navigation_base_result is None:
            QMessageBox.information(
                self,
                "导航图未生成",
                "请先生成 Ground-relative 导航图预览",
            )
            return None

        try:
            overrides = validate_navigation_override_records(
                self._navigation_overrides
            )
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


def main(argv=None) -> int:
    app = QApplication(list(sys.argv if argv is None else argv))
    window = Paper1MapWorkbenchWindow()
    window.show()
    return app.exec_()
