"""Compose the stable 2D agricultural Workbench with a review-only 3D tab."""

from __future__ import annotations

import sys

from PyQt5.QtWidgets import QApplication, QSplitter, QTabWidget

from agt_offline_assets import (
    VehicleCorridorConfig,
    VehicleCorridorResult,
    derive_vehicle_corridor,
)

from .agricultural_workbench import AgriculturalMapWorkbenchWindow
from .review_3d import ThreeDReviewWidget


class ReviewMapWorkbenchWindow(AgriculturalMapWorkbenchWindow):
    """Keep 2D authoring authoritative and add an independent 3D review plane."""

    def __init__(self) -> None:
        self._vehicle_corridor_result: VehicleCorridorResult | None = None
        self._review_3d: ThreeDReviewWidget | None = None
        super().__init__()
        self.setWindowTitle("AGT 地图工作台 — V25-12C 农业结构 + 3D 审查")
        self._install_3d_review()

    def _install_3d_review(self) -> None:
        splitter = self.centralWidget()
        if not isinstance(splitter, QSplitter):
            raise RuntimeError("3D Review expects the Workbench central QSplitter")

        # Do not use QSplitter.replaceWidget() here.  On some Qt5 builds the
        # replacement inherits a collapsed/zero splitter size, leaving the
        # controls visible while the whole 2D/3D review plane appears missing.
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
        self._review_3d = review

        if self._cloud is not None:
            review.set_cloud(self._cloud)
        self._sync_3d_analysis()

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
            # clipped by the refined aisle.  This makes geometric intrusion
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
