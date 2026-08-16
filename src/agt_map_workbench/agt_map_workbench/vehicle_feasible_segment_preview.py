"""Review-only Workbench preview for V25-12G-A1 vehicle-feasible segments."""

from __future__ import annotations

from PyQt5.QtGui import QColor, QPainterPath, QPen
from PyQt5.QtWidgets import QGraphicsItem, QGraphicsPathItem, QGraphicsScene

from agt_offline_assets.vehicle_feasible_segment import VehicleFeasibleSegmentPlan


_REVIEW_Z = 12.0


class VehicleFeasibleSegmentPreview:
    """Own non-editable map graphics derived only from explicit A1 geometry."""

    def __init__(self, scene: QGraphicsScene) -> None:
        self._scene = scene
        self._active_items: list[QGraphicsPathItem] = []
        self._visible = True
        self._rejected_fragment_count = 0
        self._rejected_fragment_total_length_m = 0.0

    @property
    def active_item_count(self) -> int:
        return len(self._active_items)

    @property
    def rejected_fragment_count(self) -> int:
        return self._rejected_fragment_count

    @property
    def rejected_fragment_total_length_m(self) -> float:
        return self._rejected_fragment_total_length_m

    def set_plan(self, plan: VehicleFeasibleSegmentPlan | None) -> None:
        self.clear()
        if plan is None:
            return

        self._rejected_fragment_count = sum(
            len(aisle.rejected_fragments) for aisle in plan.aisles
        )
        self._rejected_fragment_total_length_m = float(
            sum(
                fragment.length_m
                for aisle in plan.aisles
                for fragment in aisle.rejected_fragments
            )
        )

        for aisle in plan.aisles:
            for segment in aisle.active_segments:
                if not segment.centerline_xyz:
                    continue
                first = segment.centerline_xyz[0]
                path = QPainterPath()
                path.moveTo(float(first[0]), float(-first[1]))
                for point in segment.centerline_xyz[1:]:
                    path.lineTo(float(point[0]), float(-point[1]))

                item = QGraphicsPathItem(path)
                pen = QPen(QColor(40, 220, 255))
                pen.setWidth(3)
                pen.setCosmetic(True)
                item.setPen(pen)
                item.setZValue(_REVIEW_Z)
                item.setFlag(QGraphicsItem.ItemIsMovable, False)
                item.setFlag(QGraphicsItem.ItemIsSelectable, False)
                item.setVisible(self._visible)
                self._scene.addItem(item)
                self._active_items.append(item)

    def set_visible(self, visible: bool) -> None:
        self._visible = bool(visible)
        for item in self._active_items:
            item.setVisible(self._visible)

    def clear(self) -> None:
        for item in self._active_items:
            if item.scene() is self._scene:
                self._scene.removeItem(item)
        self._active_items.clear()
        self._rejected_fragment_count = 0
        self._rejected_fragment_total_length_m = 0.0
