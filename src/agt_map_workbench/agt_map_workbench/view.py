"""Qt5 2.5D point-cloud canvas for AGT Map Workbench MVP."""

from __future__ import annotations

import numpy as np
from PyQt5.QtCore import QPointF, QRectF, Qt, pyqtSignal
from PyQt5.QtGui import QColor, QPainter, QPen, QPolygonF
from PyQt5.QtWidgets import QGraphicsItem, QGraphicsView


class PointCloudItem(QGraphicsItem):
    """One graphics item draws the whole sampled cloud to avoid millions of items."""

    def __init__(self) -> None:
        super().__init__()
        self._xy = np.empty((0, 2), dtype=np.float64)
        self._z = np.empty(0, dtype=np.float64)
        self._z_min = -np.inf
        self._z_max = np.inf
        self._rect = QRectF()

    def set_cloud(self, cloud, *, sample_limit: int = 60_000) -> None:
        """Sample structured PCD fields before expanding to an Nx3 float64 matrix."""
        point_count = int(cloud.points.shape[0])
        for field in ("x", "y", "z"):
            if field not in cloud.schema.fields:
                raise ValueError(f"point cloud is missing {field}")
            field_index = cloud.schema.fields.index(field)
            if int(cloud.schema.counts[field_index]) != 1:
                raise ValueError(f"point cloud field {field} must be scalar")
        if point_count > sample_limit:
            indices = np.linspace(0, point_count - 1, sample_limit, dtype=np.int64)
            points = cloud.points[indices]
        else:
            points = cloud.points
        xyz = np.column_stack(
            (
                np.asarray(points["x"], dtype=np.float64),
                np.asarray(points["y"], dtype=np.float64),
                np.asarray(points["z"], dtype=np.float64),
            )
        )
        self.set_points(xyz)

    def set_points(self, xyz: np.ndarray) -> None:
        xyz = np.asarray(xyz, dtype=np.float64)
        finite = np.all(np.isfinite(xyz), axis=1)
        xyz = xyz[finite]
        self.prepareGeometryChange()
        self._xy = np.column_stack((xyz[:, 0], -xyz[:, 1])) if xyz.size else np.empty((0, 2))
        self._z = xyz[:, 2].copy() if xyz.size else np.empty(0)
        if self._xy.shape[0]:
            minimum = np.min(self._xy, axis=0)
            maximum = np.max(self._xy, axis=0)
            self._rect = QRectF(
                float(minimum[0]),
                float(minimum[1]),
                max(1e-6, float(maximum[0] - minimum[0])),
                max(1e-6, float(maximum[1] - minimum[1])),
            )
            self._z_min = float(np.min(self._z))
            self._z_max = float(np.max(self._z))
        else:
            self._rect = QRectF()
        self.update()

    def set_z_window(self, minimum: float, maximum: float) -> None:
        self._z_min = float(minimum)
        self._z_max = float(maximum)
        self.update()

    def sample_count(self) -> int:
        """Number of finite points currently held by the display sample."""
        return int(self._z.size)

    def visible_sample_count(self) -> int:
        """Number of sampled points inside the active Z display window."""
        if self._z.size == 0:
            return 0
        visible = (self._z >= self._z_min) & (self._z <= self._z_max)
        return int(np.count_nonzero(visible))

    def padded_bounding_rect(
        self, *, ratio: float = 0.08, minimum_margin: float = 0.75
    ) -> QRectF:
        """Return a view/scene rectangle with authoring margin around cloud edges."""
        if self._rect.isNull():
            return QRectF(self._rect)
        margin_x = max(float(minimum_margin), float(self._rect.width()) * float(ratio))
        margin_y = max(float(minimum_margin), float(self._rect.height()) * float(ratio))
        return self._rect.adjusted(-margin_x, -margin_y, margin_x, margin_y)

    def boundingRect(self) -> QRectF:  # noqa: N802 - Qt API
        return self._rect

    def paint(self, painter: QPainter, option, widget=None) -> None:  # noqa: N802 - Qt API
        del option, widget
        if self._xy.shape[0] == 0:
            return
        visible = (self._z >= self._z_min) & (self._z <= self._z_max)
        if not np.any(visible):
            return
        xy = self._xy[visible]
        z = self._z[visible]
        z_low = float(np.min(z))
        z_high = float(np.max(z))
        scale = max(1e-9, z_high - z_low)
        bucket = np.clip(((z - z_low) / scale * 15.0).astype(np.int32), 0, 15)
        painter.setRenderHint(QPainter.Antialiasing, False)
        for index in range(16):
            selected = xy[bucket == index]
            if selected.shape[0] == 0:
                continue
            t = index / 15.0
            color = QColor.fromHsvF((2.0 / 3.0) * (1.0 - t), 0.9, 0.9)
            painter.setPen(QPen(color, 0))
            polygon = QPolygonF([QPointF(float(x), float(y)) for x, y in selected])
            painter.drawPoints(polygon)


class PointCloudView(QGraphicsView):
    mapClicked = pyqtSignal(float, float)

    def __init__(self, scene, parent=None) -> None:
        super().__init__(scene, parent)
        self.authoring_enabled = False
        self.setRenderHint(QPainter.Antialiasing, False)
        self.setTransformationAnchor(QGraphicsView.AnchorUnderMouse)
        self.setResizeAnchor(QGraphicsView.AnchorViewCenter)
        self.setDragMode(QGraphicsView.ScrollHandDrag)

    def set_authoring_enabled(self, enabled: bool) -> None:
        self.authoring_enabled = bool(enabled)
        self.setDragMode(QGraphicsView.NoDrag if enabled else QGraphicsView.ScrollHandDrag)
        self.setCursor(Qt.CrossCursor if enabled else Qt.ArrowCursor)

    def mousePressEvent(self, event) -> None:  # noqa: N802 - Qt API
        if self.authoring_enabled and event.button() == Qt.LeftButton:
            point = self.mapToScene(event.pos())
            self.mapClicked.emit(float(point.x()), float(-point.y()))
            event.accept()
            return
        super().mousePressEvent(event)

    def wheelEvent(self, event) -> None:  # noqa: N802 - Qt API
        factor = 1.15 if event.angleDelta().y() > 0 else 1.0 / 1.15
        self.scale(factor, factor)
        event.accept()
