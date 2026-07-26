#include "display/task_waypoint_item.h"

#include <QGraphicsSceneMouseEvent>
#include <QPainter>
#include <QStyleOptionGraphicsItem>

#include <cmath>

namespace {
constexpr qreal kMarkerRadius = 11.0;
constexpr qreal kHeadingLength = 32.0;
}

TaskWaypointItem::TaskWaypointItem(int index, QGraphicsItem *parent,
                                   Changed changed, Selected selected)
    : QGraphicsObject(parent),
      index_(index),
      changed_(std::move(changed)),
      selected_callback_(std::move(selected)) {
  setFlag(QGraphicsItem::ItemIgnoresTransformations, true);
  setAcceptedMouseButtons(Qt::LeftButton);
  setAcceptHoverEvents(true);
  setZValue(80.0);
}

QRectF TaskWaypointItem::boundingRect() const {
  return QRectF(-18.0, -18.0, 210.0, 54.0);
}

QPointF TaskWaypointItem::headingTip() const {
  return QPointF(std::cos(heading_scene_) * kHeadingLength,
                 std::sin(heading_scene_) * kHeadingLength);
}

void TaskWaypointItem::paint(QPainter *painter,
                             const QStyleOptionGraphicsItem *, QWidget *) {
  painter->setRenderHint(QPainter::Antialiasing, true);
  const QColor color = enabled_ ? QColor(0, 121, 107) : QColor(117, 117, 117);
  painter->setOpacity(enabled_ ? 1.0 : 0.55);
  painter->setPen(QPen(selected_ ? QColor(255, 193, 7) : Qt::white,
                       selected_ ? 4.0 : 2.0));
  painter->setBrush(color);
  painter->drawEllipse(QPointF(0.0, 0.0), kMarkerRadius, kMarkerRadius);
  painter->setPen(Qt::white);
  painter->drawText(QRectF(-10.0, -9.0, 20.0, 18.0), Qt::AlignCenter,
                    QString::number(index_ + 1));

  const QPointF tip = headingTip();
  painter->setPen(QPen(color, selected_ ? 4.0 : 3.0));
  painter->drawLine(QPointF(0.0, 0.0), tip);
  const double arrow_angle = std::atan2(tip.y(), tip.x());
  const QPointF left = tip - QPointF(std::cos(arrow_angle - 0.55) * 9.0,
                                     std::sin(arrow_angle - 0.55) * 9.0);
  const QPointF right = tip - QPointF(std::cos(arrow_angle + 0.55) * 9.0,
                                      std::sin(arrow_angle + 0.55) * 9.0);
  painter->drawLine(tip, left);
  painter->drawLine(tip, right);
  painter->setBrush(color);
  painter->drawEllipse(tip, editing_ ? 5.0 : 3.0, editing_ ? 5.0 : 3.0);

  painter->setPen(QPen(QColor(32, 33, 36), 3.0));
  painter->drawText(QPointF(42.0, 5.0), name_);
  painter->setPen(Qt::white);
  painter->drawText(QPointF(42.0, 4.0), name_);
}

void TaskWaypointItem::setWaypoint(const QString &name, double heading_scene,
                                   bool enabled, bool selected) {
  name_ = name;
  heading_scene_ = heading_scene;
  enabled_ = enabled;
  selected_ = selected;
  update();
}

void TaskWaypointItem::setEditing(bool enabled) {
  editing_ = enabled;
  setCursor(enabled ? Qt::OpenHandCursor : Qt::ArrowCursor);
  update();
}

void TaskWaypointItem::mousePressEvent(QGraphicsSceneMouseEvent *event) {
  if (selected_callback_) selected_callback_(index_);
  if (!editing_ || event->button() != Qt::LeftButton) {
    event->accept();
    return;
  }
  drag_mode_ = QLineF(event->pos(), headingTip()).length() <= 10.0
                   ? DragMode::Heading
                   : DragMode::Position;
  setCursor(drag_mode_ == DragMode::Heading ? Qt::CrossCursor
                                             : Qt::ClosedHandCursor);
  event->accept();
}

void TaskWaypointItem::mouseMoveEvent(QGraphicsSceneMouseEvent *event) {
  if (!editing_ || drag_mode_ == DragMode::None) return;
  if (drag_mode_ == DragMode::Position && parentItem()) {
    setPos(parentItem()->mapFromScene(event->scenePos()));
  } else if (drag_mode_ == DragMode::Heading) {
    const QPointF local = mapFromScene(event->scenePos());
    if (std::hypot(local.x(), local.y()) > 1.0) {
      heading_scene_ = std::atan2(local.y(), local.x());
      update();
    }
  }
  event->accept();
}

void TaskWaypointItem::mouseReleaseEvent(QGraphicsSceneMouseEvent *event) {
  if (editing_ && drag_mode_ != DragMode::None && changed_) {
    changed_(index_, scenePos(), mapToScene(headingTip()));
  }
  drag_mode_ = DragMode::None;
  setCursor(editing_ ? Qt::OpenHandCursor : Qt::ArrowCursor);
  event->accept();
}
