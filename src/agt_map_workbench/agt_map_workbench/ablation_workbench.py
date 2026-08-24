"""Workbench wrapper for controlled Ground-navigation A0/A1/A2/A3 ablations."""

from __future__ import annotations

import sys

from PyQt5.QtWidgets import QApplication, QComboBox, QHBoxLayout, QLabel

from agt_offline_assets.navigation_ablation import (
    ABLATION_PROFILE_KEYS,
    apply_navigation_ablation_profile,
    navigation_ablation_spec,
)

from .review_export_workbench import ReviewExportMapWorkbenchWindow


class AblationMapWorkbenchWindow(ReviewExportMapWorkbenchWindow):
    """Review Workbench that changes one Ground-map mechanism at a time.

    The selector is deliberately experimental.  The heavy Ground derivation is
    still run once with the normal Workbench parameters; the selected profile is
    applied to that evidence before agricultural structure, formal
    materialization and QA are recomputed.
    """

    def __init__(self) -> None:
        self._ablation_profile_applied = "A0"
        self._ablation_profile_pending = "A0"
        super().__init__()
        self._install_ablation_selector()
        self.setWindowTitle(
            "AGT 地图工作台 — V25 统一地图生产 / Structure-Aware PGM / Ablation"
        )

    def _install_ablation_selector(self) -> None:
        parent = self._nav_status.parentWidget()
        layout = parent.layout() if parent is not None else None
        if layout is None:
            return
        row = QHBoxLayout()
        row.addWidget(QLabel("Ground 消融："))
        self._ablation_profile_combo = QComboBox()
        for key in ABLATION_PROFILE_KEYS:
            spec = navigation_ablation_spec(key)
            self._ablation_profile_combo.addItem(spec.label, key)
        self._ablation_profile_combo.currentIndexChanged.connect(
            self._on_ablation_profile_changed
        )
        row.addWidget(self._ablation_profile_combo, 1)
        # Keep the experimental control close to the review/export controls so
        # it is not mistaken for a normal production tuning parameter.
        layout.insertLayout(max(0, layout.count() - 2), row)

    def _selected_ablation_profile(self) -> str:
        if not hasattr(self, "_ablation_profile_combo"):
            return "A0"
        return str(self._ablation_profile_combo.currentData() or "A0")

    def _on_ablation_profile_changed(self, _index: int) -> None:
        selected = self._selected_ablation_profile()
        if selected == self._ablation_profile_applied:
            self.statusBar().showMessage(
                f"Ground 消融 {selected} 已应用于当前导航结果"
            )
            return
        self.statusBar().showMessage(
            f"Ground 消融已选择 {selected}；请重新点击“生成 Ground-relative 导航图预览”后生效"
        )

    def _generate_navigation_preview(self) -> None:
        self._ablation_profile_pending = self._selected_ablation_profile()
        super()._generate_navigation_preview()

    def _navigation_finished(self, result) -> None:
        profile = str(self._ablation_profile_pending or "A0")
        ablated = apply_navigation_ablation_profile(result, profile)
        self._ablation_profile_applied = profile
        super()._navigation_finished(ablated)

    def _refresh_navigation_status(self) -> None:
        super()._refresh_navigation_status()
        if not hasattr(self, "_nav_status"):
            return
        if getattr(self, "_navigation_base_result", None) is None:
            return
        spec = navigation_ablation_spec(self._ablation_profile_applied)
        self._nav_status.setText(
            self._nav_status.text()
            + f"\nGround Ablation={spec.key} | {spec.label}"
        )

    def _review_summary_payload(self) -> dict[str, object]:
        payload = super()._review_summary_payload()
        spec = navigation_ablation_spec(self._ablation_profile_applied)
        payload["ablation"] = {
            **spec.to_dict(),
            "status": "EXPERIMENTAL_REVIEW_EVIDENCE",
            "note": (
                "A0-A3 are controlled review profiles.  Select a final policy "
                "from the real-PCD comparison before promoting it into the "
                "canonical production contract."
            ),
        }
        return payload


def main(argv=None) -> int:
    app = QApplication(list(sys.argv if argv is None else argv))
    window = AblationMapWorkbenchWindow()
    window.show()
    return app.exec_()
