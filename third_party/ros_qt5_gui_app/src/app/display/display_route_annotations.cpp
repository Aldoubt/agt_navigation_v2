#include "display/display_route_annotations.h"

#include <algorithm>
#include <cmath>
#include <limits>

#include <QFont>
#include <QPainter>
#include <QPainterPath>
#include <QTimer>

#include "core/framework/framework.h"
#include "display/manager/display_factory.h"
#include "msg/msg_info.h"
#include "ui_language.h"

namespace Display {
namespace {

QColor EventColor(const std::string &kind) {
  if (kind == "START") return QColor(0, 137, 123);
  if (kind == "END") return QColor(211, 47, 47);
  if (kind == "TURN_LEFT") return QColor(46, 125, 50);
  if (kind == "TURN_RIGHT") return QColor(239, 108, 0);
  if (kind.find("U_TURN") == 0) return QColor(123, 31, 162);
  return QColor(194, 24, 91);
}

QString EventLabel(const std::string &kind) {
  if (kind == "START") return UiLanguage::Text("起点", "Start");
  if (kind == "END") return UiLanguage::Text("终点", "End");
  if (kind == "TURN_LEFT") return UiLanguage::Text("左转", "Left");
  if (kind == "TURN_RIGHT") return UiLanguage::Text("右转", "Right");
  if (kind == "U_TURN_LEFT") return UiLanguage::Text("左向掉头", "U-turn left");
  if (kind == "U_TURN_RIGHT") return UiLanguage::Text("右向掉头", "U-turn right");
  if (kind == "IN_PLACE_LEFT") return UiLanguage::Text("原地左转", "Pivot left");
  if (kind == "IN_PLACE_RIGHT") return UiLanguage::Text("原地右转", "Pivot right");
  return QString::fromStdString(kind);
}

double SizeCompensation(const QPainter *painter) {
  const QTransform transform = painter->worldTransform();
  const double view_scale = std::hypot(transform.m11(), transform.m12());
  return std::clamp(1.0 / std::max(view_scale, 0.01), 0.65, 1.6);
}

}  // namespace

DisplayRouteAnnotations::DisplayRouteAnnotations(
    const std::string &display_type, int z_value, const std::string &parent_name)
    : VirtualDisplay(display_type, z_value, parent_name) {
  setZValue(z_value);
  SUBSCRIBE(MSG_ID_OCCUPANCY_MAP, [this](const OccupancyMap &data) {
    map_data_ = data;
    rebuildSceneAnnotations();
  });
  SUBSCRIBE(MSG_ID_TEACH_ROUTE_ANNOTATIONS,
            [this](const RouteAnnotations &data) {
              annotations_ = data;
              rebuildSceneAnnotations();
            });
}

void DisplayRouteAnnotations::rebuildSceneAnnotations() {
  prepareGeometryChange();
  scene_annotations_.clear();
  if (map_data_.Cols() == 0 || map_data_.Rows() == 0) {
    SetBoundingRect(QRectF());
    update();
    return;
  }
  double min_x = std::numeric_limits<double>::max();
  double min_y = std::numeric_limits<double>::max();
  double max_x = std::numeric_limits<double>::lowest();
  double max_y = std::numeric_limits<double>::lowest();
  const double map_yaw = map_data_.map_config.origin.size() >= 3
                             ? map_data_.map_config.origin[2]
                             : 0.0;
  for (const auto &annotation : annotations_) {
    double scene_x = 0.0;
    double scene_y = 0.0;
    map_data_.xy2ScenePose(annotation.x, annotation.y, scene_x, scene_y);
    scene_annotations_.push_back(
        {{scene_x, scene_y},
         -(annotation.theta - map_yaw) * 180.0 / M_PI,
         annotation.kind});
    min_x = std::min(min_x, scene_x);
    min_y = std::min(min_y, scene_y);
    max_x = std::max(max_x, scene_x);
    max_y = std::max(max_y, scene_y);
  }
  if (scene_annotations_.empty()) {
    SetBoundingRect(QRectF());
  } else {
    SetBoundingRect(QRectF(min_x - 48.0, min_y - 48.0,
                           max_x - min_x + 96.0, max_y - min_y + 96.0));
    if (!initial_fit_done_ &&
        GET_CONFIG_VALUE("AutoFitTeachRoute", "false") == "true") {
      initial_fit_done_ = true;
      QTimer::singleShot(1500, [this]() {
        const QRectF scene_rect = mapRectToScene(boundingRect()).adjusted(
            -24.0, -24.0, 24.0, 24.0);
        FactoryDisplay::Instance()->FitInView(scene_rect);
      });
    }
  }
  update();
}

void DisplayRouteAnnotations::drawDirection(
    QPainter *painter, const SceneAnnotation &annotation) {
  const double size_compensation = SizeCompensation(painter);
  painter->save();
  painter->translate(annotation.position);
  painter->scale(size_compensation, size_compensation);
  painter->rotate(annotation.angle_degrees);
  painter->setPen(QPen(QColor(18, 82, 163, 220), 1.6));
  painter->setBrush(QColor(30, 112, 219, 210));
  QPolygonF arrow;
  arrow << QPointF(6.0, 0.0) << QPointF(-2.5, -3.0)
        << QPointF(-1.0, 0.0) << QPointF(-2.5, 3.0);
  painter->drawPolygon(arrow);
  painter->restore();
}

void DisplayRouteAnnotations::drawEvent(
    QPainter *painter, const SceneAnnotation &annotation) {
  const QColor color = EventColor(annotation.kind);
  const double size_compensation = SizeCompensation(painter);
  painter->save();
  painter->translate(annotation.position);
  painter->scale(size_compensation, size_compensation);
  painter->setPen(QPen(Qt::white, 1.2));
  painter->setBrush(color);
  painter->drawEllipse(QPointF(0.0, 0.0), 5.0, 5.0);
  painter->rotate(annotation.angle_degrees);
  painter->setPen(QPen(Qt::white, 1.4, Qt::SolidLine, Qt::RoundCap));
  if (annotation.kind == "START") {
    QPolygonF triangle;
    triangle << QPointF(3.0, 0.0) << QPointF(-2.0, -2.5)
             << QPointF(-2.0, 2.5);
    painter->setBrush(Qt::white);
    painter->drawPolygon(triangle);
  } else if (annotation.kind == "END") {
    painter->setBrush(Qt::white);
    painter->drawRect(QRectF(-2.2, -2.2, 4.4, 4.4));
  } else {
    painter->drawLine(QPointF(-2.5, 0.0), QPointF(2.8, 0.0));
    painter->drawLine(QPointF(2.8, 0.0), QPointF(0.5, -2.0));
    painter->drawLine(QPointF(2.8, 0.0), QPointF(0.5, 2.0));
  }
  painter->restore();

  painter->save();
  painter->translate(annotation.position);
  painter->scale(size_compensation, size_compensation);
  QFont font = painter->font();
  font.setPixelSize(11);
  font.setBold(true);
  painter->setFont(font);
  const QString label = EventLabel(annotation.kind);
  const QRect text_bounds = painter->fontMetrics().boundingRect(label);
  QRectF background(7.0, -text_bounds.height() / 2.0 - 2.0,
                    text_bounds.width() + 8.0, text_bounds.height() + 4.0);
  painter->setPen(Qt::NoPen);
  painter->setBrush(QColor(255, 255, 255, 225));
  painter->drawRoundedRect(background, 3.0, 3.0);
  painter->setPen(color.darker(120));
  painter->drawText(background, Qt::AlignCenter, label);
  painter->restore();
}

void DisplayRouteAnnotations::paint(
    QPainter *painter, const QStyleOptionGraphicsItem *, QWidget *) {
  painter->setRenderHint(QPainter::Antialiasing, true);
  for (const auto &annotation : scene_annotations_) {
    if (annotation.kind == "DIRECTION") {
      drawDirection(painter, annotation);
    } else {
      drawEvent(painter, annotation);
    }
  }
}

}  // namespace Display
