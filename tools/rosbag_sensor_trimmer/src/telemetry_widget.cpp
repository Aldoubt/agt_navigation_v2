#include "rosbag_sensor_trimmer/telemetry_widget.hpp"

#include <QPainter>

#include <algorithm>
#include <cmath>

namespace rosbag_sensor_trimmer
{

TelemetryWidget::TelemetryWidget(QWidget * parent)
: QWidget(parent)
{
  setMinimumHeight(360);
  setAutoFillBackground(true);
}

QSize TelemetryWidget::minimumSizeHint() const
{
  return QSize(700, 420);
}

void TelemetryWidget::clear()
{
  trajectory_.clear();
  transform_count_ = 0;
  tf_frames_.clear();
  update();
}

void TelemetryWidget::add_odometry(double x, double y)
{
  trajectory_.push_back(QPointF(x, y));
  constexpr int max_points = 20000;
  if (trajectory_.size() > max_points) {
    trajectory_.remove(0, trajectory_.size() - max_points);
  }
  update();
}

void TelemetryWidget::set_tf_summary(int transform_count, const QString & frames)
{
  transform_count_ = transform_count;
  tf_frames_ = frames;
  update();
}

void TelemetryWidget::paintEvent(QPaintEvent * event)
{
  Q_UNUSED(event);
  QPainter painter(this);
  painter.fillRect(rect(), palette().base());
  const auto plot = rect().adjusted(56, 42, -22, -42);
  painter.setPen(QColor("#b8c2cc"));
  painter.drawRect(plot);

  painter.setPen(palette().text().color());
  painter.drawText(12, 22, QString("里程计轨迹：%1 个采样点    TF 最近消息：%2 个变换")
    .arg(trajectory_.size()).arg(transform_count_));
  if (!tf_frames_.isEmpty()) {
    painter.setPen(QColor("#66727f"));
    painter.drawText(12, height() - 14, QFontMetrics(font()).elidedText(
      "TF frame：" + tf_frames_, Qt::ElideMiddle, width() - 24));
  }

  if (trajectory_.isEmpty()) {
    painter.setPen(QColor("#66727f"));
    painter.drawText(plot, Qt::AlignCenter, "播放包含里程计话题后显示轨迹");
    return;
  }

  QPointF minimum = trajectory_.front();
  QPointF maximum = trajectory_.front();
  for (const auto & point : trajectory_) {
    minimum.setX(std::min(minimum.x(), point.x()));
    minimum.setY(std::min(minimum.y(), point.y()));
    maximum.setX(std::max(maximum.x(), point.x()));
    maximum.setY(std::max(maximum.y(), point.y()));
  }
  const double span_x = std::max(1.0e-6, maximum.x() - minimum.x());
  const double span_y = std::max(1.0e-6, maximum.y() - minimum.y());
  const double span = std::max(span_x, span_y);
  const double scale = 0.88 * std::min(plot.width(), plot.height()) / span;
  const QPointF center = (minimum + maximum) * 0.5;
  auto map_point = [&](const QPointF & point) {
      return QPointF(plot.center().x() + (point.x() - center.x()) * scale,
        plot.center().y() - (point.y() - center.y()) * scale);
    };

  painter.setPen(QColor("#d3dbe3"));
  for (int index = -4; index <= 4; ++index) {
    const int x = plot.center().x() + index * plot.width() / 8;
    const int y = plot.center().y() + index * plot.height() / 8;
    painter.drawLine(x, plot.top(), x, plot.bottom());
    painter.drawLine(plot.left(), y, plot.right(), y);
  }
  painter.setPen(QPen(QColor("#00a6a6"), 2.0));
  QPolygonF polyline;
  for (const auto & point : trajectory_) {
    polyline.push_back(map_point(point));
  }
  painter.drawPolyline(polyline);
  painter.setBrush(QColor("#e07a5f"));
  painter.setPen(Qt::NoPen);
  painter.drawEllipse(map_point(trajectory_.back()), 5, 5);
}

}  // namespace rosbag_sensor_trimmer
