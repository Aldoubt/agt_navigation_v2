#include "rosbag_sensor_trimmer/imu_plot_widget.hpp"

#include <QFontMetrics>
#include <QMouseEvent>
#include <QPainter>
#include <QPainterPath>
#include <QToolTip>

#include <algorithm>
#include <cmath>
#include <limits>

namespace rosbag_sensor_trimmer
{

namespace
{

struct Range
{
  double minimum{0.0};
  double maximum{1.0};
};

Range value_range(const std::vector<ImuSample> & samples, bool acceleration)
{
  Range range{std::numeric_limits<double>::max(), std::numeric_limits<double>::lowest()};
  for (const auto & sample : samples) {
    const auto value = acceleration ? sample.acceleration_magnitude :
      sample.angular_velocity_magnitude;
    range.minimum = std::min(range.minimum, value);
    range.maximum = std::max(range.maximum, value);
  }
  if (!std::isfinite(range.minimum) || !std::isfinite(range.maximum)) {
    return {0.0, 1.0};
  }
  const auto span = std::max(1.0e-6, range.maximum - range.minimum);
  range.minimum -= span * 0.08;
  range.maximum += span * 0.08;
  return range;
}

}  // namespace

ImuPlotWidget::ImuPlotWidget(QWidget * parent)
: QWidget(parent)
{
  setMouseTracking(true);
  setMinimumHeight(480);
  setAutoFillBackground(true);
}

void ImuPlotWidget::set_data(
  const std::vector<ImuSample> & samples,
  const ImuMotionEstimate & estimate,
  std::int64_t bag_start_timestamp_ns)
{
  samples_ = samples;
  estimate_ = estimate;
  bag_start_timestamp_ns_ = bag_start_timestamp_ns;
  update();
}

QSize ImuPlotWidget::minimumSizeHint() const
{
  return QSize(760, 480);
}

QRect ImuPlotWidget::plot_rect() const
{
  return rect().adjusted(90, 42, -28, -52);
}

double ImuPlotWidget::duration_seconds() const
{
  if (samples_.size() < 2) {
    return 1.0;
  }
  return std::max(1.0e-9, static_cast<double>(
    samples_.back().timestamp_ns - bag_start_timestamp_ns_) / 1.0e9);
}

void ImuPlotWidget::paintEvent(QPaintEvent * event)
{
  Q_UNUSED(event);
  QPainter painter(this);
  painter.fillRect(rect(), palette().base());
  painter.setRenderHint(QPainter::Antialiasing, false);

  if (samples_.empty()) {
    painter.setPen(palette().text().color());
    painter.drawText(rect(), Qt::AlignCenter, "未找到 sensor_msgs/msg/Imu 话题");
    return;
  }

  const auto plot = plot_rect();
  const int gap = 28;
  const int panel_height = (plot.height() - gap) / 2;
  const QRect acceleration_panel(plot.left(), plot.top(), plot.width(), panel_height);
  const QRect gyro_panel(plot.left(), plot.top() + panel_height + gap, plot.width(), panel_height);
  const auto acceleration_range = value_range(samples_, true);
  const auto gyro_range = value_range(samples_, false);
  const auto duration = duration_seconds();

  auto draw_panel = [&](const QRect & panel, const Range & range, bool acceleration,
                        const QString & title, const QColor & color) {
      painter.fillRect(panel, palette().alternateBase());
      painter.setPen(QColor("#b8c2cc"));
      for (int tick = 0; tick <= 4; ++tick) {
        const int y = panel.bottom() - (panel.height() * tick) / 4;
        painter.drawLine(panel.left(), y, panel.right(), y);
        const auto value = range.minimum + (range.maximum - range.minimum) * tick / 4.0;
        painter.setPen(palette().text().color());
        painter.drawText(QRect(4, y - 10, 80, 20), Qt::AlignRight,
          QString::number(value, 'f', 2));
        painter.setPen(QColor("#b8c2cc"));
      }
      painter.setPen(palette().text().color());
      painter.drawText(4, panel.top() + 16, title);

      if (estimate_.valid) {
        const double threshold = acceleration ?
          estimate_.acceleration_baseline + estimate_.acceleration_threshold :
          estimate_.angular_velocity_threshold;
        const int threshold_y = panel.bottom() - static_cast<int>(
          (threshold - range.minimum) / (range.maximum - range.minimum) * panel.height());
        if (threshold_y >= panel.top() && threshold_y <= panel.bottom()) {
          QPen threshold_pen(QColor("#d97706"));
          threshold_pen.setStyle(Qt::DashLine);
          painter.setPen(threshold_pen);
          painter.drawLine(panel.left(), threshold_y, panel.right(), threshold_y);
        }
      }

      QPainterPath path;
      const std::size_t step = std::max<std::size_t>(1, samples_.size() / 100000 + 1);
      bool first = true;
      std::size_t index = 0;
      for (const auto & sample : samples_) {
        if (index++ % step != 0) {
          continue;
        }
        const double relative_seconds = static_cast<double>(
          sample.timestamp_ns - bag_start_timestamp_ns_) / 1.0e9;
        const auto value = acceleration ? sample.acceleration_magnitude :
          sample.angular_velocity_magnitude;
        const int x = panel.left() + static_cast<int>(
          relative_seconds / duration * panel.width());
        const int y = panel.bottom() - static_cast<int>(
          (value - range.minimum) / (range.maximum - range.minimum) * panel.height());
        if (first) {
          path.moveTo(x, y);
          first = false;
        } else {
          path.lineTo(x, y);
        }
      }
      painter.setPen(QPen(color, 1.2));
      painter.drawPath(path);
    };

  draw_panel(acceleration_panel, acceleration_range, true, "加速度模长 (m/s²)", QColor("#087f8c"));
  draw_panel(gyro_panel, gyro_range, false, "角速度模长 (rad/s)", QColor("#c2410c"));

  if (estimate_.valid) {
    const int marker_x = plot.left() + static_cast<int>(
      estimate_.relative_start_seconds / duration * plot.width());
    QPen marker_pen(QColor("#b91c1c"), 2);
    marker_pen.setStyle(Qt::DashLine);
    painter.setPen(marker_pen);
    painter.drawLine(marker_x, plot.top(), marker_x, plot.bottom());
    painter.setPen(QColor("#b91c1c"));
    painter.drawText(QRect(marker_x - 75, 8, 150, 24), Qt::AlignHCenter,
      QString("估计启动：%1 s").arg(estimate_.relative_start_seconds, 0, 'f', 3));
  } else {
    painter.setPen(QColor("#a16207"));
    painter.drawText(8, 24, "未检测到明确启动点，请检查初始静止段或手动输入起点");
  }

  painter.setPen(QColor("#9aa5b1"));
  for (int tick = 0; tick <= 4; ++tick) {
    const int x = plot.left() + (plot.width() * tick) / 4;
    painter.drawLine(x, plot.top(), x, plot.bottom());
    painter.drawText(QRect(x - 50, plot.bottom() + 10, 100, 22), Qt::AlignHCenter,
      QString::number(duration * tick / 4.0, 'f', 2) + " s");
  }
  painter.drawText(plot.left(), height() - 14, "记录接收时间，相对 bag 起点");
}

void ImuPlotWidget::mouseMoveEvent(QMouseEvent * event)
{
  const auto plot = plot_rect();
  if (samples_.empty() || !plot.contains(event->pos())) {
    QToolTip::hideText();
    return;
  }
  const auto duration = duration_seconds();
  const auto relative_seconds = std::clamp(
    static_cast<double>(event->pos().x() - plot.left()) / plot.width() * duration,
    0.0, duration);
  const auto timestamp = bag_start_timestamp_ns_ +
    static_cast<std::int64_t>(relative_seconds * 1.0e9);
  const auto closest = std::lower_bound(samples_.begin(), samples_.end(), timestamp,
    [](const ImuSample & sample, std::int64_t value) {return sample.timestamp_ns < value;});
  if (closest != samples_.end()) {
    QToolTip::showText(event->globalPos(),
      QString("相对时间：%1 s\n加速度：%2 m/s²\n角速度：%3 rad/s")
      .arg(relative_seconds, 0, 'f', 3)
      .arg(closest->acceleration_magnitude, 0, 'f', 3)
      .arg(closest->angular_velocity_magnitude, 0, 'f', 3), this);
  }
}

void ImuPlotWidget::mousePressEvent(QMouseEvent * event)
{
  if (event->button() != Qt::LeftButton || samples_.empty()) {
    return;
  }
  const auto plot = plot_rect();
  if (!plot.contains(event->pos())) {
    return;
  }
  const auto relative_seconds = std::clamp(
    static_cast<double>(event->pos().x() - plot.left()) / plot.width() * duration_seconds(),
    0.0, duration_seconds());
  emit time_selected(relative_seconds);
}

void ImuPlotWidget::leaveEvent(QEvent * event)
{
  Q_UNUSED(event);
  QToolTip::hideText();
}

}  // namespace rosbag_sensor_trimmer
