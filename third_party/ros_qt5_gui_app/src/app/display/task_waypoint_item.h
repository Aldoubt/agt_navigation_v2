#pragma once

#include <QGraphicsObject>
#include <QString>

#include <functional>

class TaskWaypointItem final : public QGraphicsObject {
 public:
  using Changed = std::function<void(int, const QPointF &, const QPointF &)>;
  using Selected = std::function<void(int)>;

  TaskWaypointItem(int index, QGraphicsItem *parent, Changed changed,
                   Selected selected);

  QRectF boundingRect() const override;
  void paint(QPainter *painter, const QStyleOptionGraphicsItem *option,
             QWidget *widget = nullptr) override;
  void setWaypoint(const QString &name, double heading_scene, bool enabled,
                   bool selected);
  void setEditing(bool enabled);

 protected:
  void mousePressEvent(QGraphicsSceneMouseEvent *event) override;
  void mouseMoveEvent(QGraphicsSceneMouseEvent *event) override;
  void mouseReleaseEvent(QGraphicsSceneMouseEvent *event) override;

 private:
  enum class DragMode { None, Position, Heading };
  QPointF headingTip() const;

  int index_{0};
  QString name_;
  double heading_scene_{0.0};
  bool enabled_{true};
  bool selected_{false};
  bool editing_{false};
  DragMode drag_mode_{DragMode::None};
  Changed changed_;
  Selected selected_callback_;
};
