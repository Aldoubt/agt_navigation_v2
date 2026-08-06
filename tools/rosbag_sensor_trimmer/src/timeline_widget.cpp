#include "rosbag_sensor_trimmer/timeline_widget.hpp"

#include <QFontMetrics>
#include <QMouseEvent>
#include <QPainter>
#include <QToolTip>

#include <algorithm>
#include <cmath>

#include "rosbag_sensor_trimmer/topic_filter.hpp"

namespace rosbag_sensor_trimmer
{

namespace
{

QColor topic_color(int index)
{
  static const QList<QColor> colors{
    QColor("#00a6a6"), QColor("#e07a5f"), QColor("#3d5a80"),
    QColor("#f2c14e"), QColor("#6a994e"), QColor("#9b5de5"),
    QColor("#f15bb5"), QColor("#577590")};
  return colors.at(index % colors.size());
}

QString format_seconds(double seconds)
{
  if (seconds >= 60.0) {
    return QString::number(seconds / 60.0, 'f', 2) + " min";
  }
  return QString::number(seconds, 'f', 3) + " s";
}

}  // namespace

TimelineWidget::TimelineWidget(QWidget * parent)
: QWidget(parent)
{
  setMouseTracking(true);
  setMinimumHeight(360);
  setAutoFillBackground(true);
}

void TimelineWidget::set_data(
  const BagStatistics & statistics, const std::vector<IndexEntry> & entries)
{
  statistics_ = statistics;
  entries_ = entries;
  update();
}

void TimelineWidget::set_gaps(const std::vector<TopicGap> & gaps)
{
  gaps_ = gaps;
  update();
}

void TimelineWidget::set_visible_topics(const QSet<QString> & topics)
{
  visible_topics_ = topics;
  hovered_topic_.clear();
  update();
}

QSize TimelineWidget::minimumSizeHint() const
{
  return QSize(700, 360);
}

QRect TimelineWidget::plot_rect() const
{
  return rect().adjusted(190, 30, -30, -50);
}

void TimelineWidget::paintEvent(QPaintEvent * event)
{
  Q_UNUSED(event);
  QPainter painter(this);
  painter.fillRect(rect(), palette().base());
  painter.setRenderHint(QPainter::Antialiasing, false);

  const auto plot = plot_rect();
  if (statistics_.topics.empty()) {
    painter.setPen(palette().text().color());
    painter.drawText(rect(), Qt::AlignCenter, "读取一个 bag 后显示消息时间轴");
    return;
  }

  QVector<QString> row_topics;
  for (const auto & topic : statistics_.topics) {
    if (visible_topics_.contains(QString::fromStdString(topic.metadata.name))) {
      row_topics.push_back(QString::fromStdString(topic.metadata.name));
    }
  }
  if (row_topics.isEmpty()) {
    painter.setPen(palette().text().color());
    painter.drawText(plot, Qt::AlignCenter, "请在话题表中至少选择一个话题");
    return;
  }

  const double duration_seconds = std::max(
    1.0e-9, static_cast<double>(statistics_.duration_ns) / 1.0e9);
  const int row_height = std::max(24, plot.height() / row_topics.size());

  QHash<QString, int> row_indices;
  for (int index = 0; index < row_topics.size(); ++index) {
    row_indices.insert(row_topics.at(index), index);
    const int top = plot.top() + index * row_height;
    if (index % 2 == 0) {
      painter.fillRect(QRect(plot.left(), top, plot.width(), row_height),
        palette().alternateBase());
    }
    painter.setPen(QColor("#b8c2cc"));
    painter.drawLine(plot.left(), top + row_height - 1, plot.right(), top + row_height - 1);
    painter.setPen(palette().text().color());
    const auto label = QFontMetrics(font()).elidedText(
      row_topics.at(index), Qt::ElideMiddle, 175);
    painter.drawText(QRect(8, top, 174, row_height), Qt::AlignVCenter | Qt::AlignRight, label);
  }

  painter.setPen(QColor("#9aa5b1"));
  for (int tick = 0; tick <= 4; ++tick) {
    const int x = plot.left() + (plot.width() * tick) / 4;
    painter.drawLine(x, plot.top(), x, plot.bottom());
    const auto label = format_seconds(duration_seconds * tick / 4.0);
    painter.drawText(QRect(x - 50, plot.bottom() + 8, 100, 22), Qt::AlignHCenter, label);
  }

  for (const auto & gap : gaps_) {
    const auto row = row_indices.constFind(QString::fromStdString(gap.topic_name));
    if (row == row_indices.cend()) {
      continue;
    }
    const auto start_seconds = static_cast<double>(
      gap.start_timestamp_ns - statistics_.start_time_ns) / 1.0e9;
    const auto end_seconds = static_cast<double>(
      gap.end_timestamp_ns - statistics_.start_time_ns) / 1.0e9;
    const int x1 = plot.left() + static_cast<int>(
      std::round(std::clamp(start_seconds, 0.0, duration_seconds) / duration_seconds * plot.width()));
    const int x2 = plot.left() + static_cast<int>(
      std::round(std::clamp(end_seconds, 0.0, duration_seconds) / duration_seconds * plot.width()));
    const int top = plot.top() + row.value() * row_height;
    painter.fillRect(QRect(x1, top, std::max(2, x2 - x1), row_height),
      QColor(210, 55, 55, 80));
    painter.setPen(QColor(180, 35, 35, 190));
    painter.drawLine(x1, top, x1, top + row_height - 1);
    painter.drawLine(x2, top, x2, top + row_height - 1);
  }

  const std::size_t max_points = 120000;
  const std::size_t step = std::max<std::size_t>(1, entries_.size() / max_points + 1);
  std::size_t entry_index = 0;
  for (const auto & entry : entries_) {
    if (entry_index++ % step != 0) {
      continue;
    }
    const QString topic = QString::fromStdString(entry.topic_name);
    const auto row = row_indices.constFind(topic);
    if (row == row_indices.cend()) {
      continue;
    }
    const double relative_seconds = static_cast<double>(
      entry.timestamp_ns - statistics_.start_time_ns) / 1.0e9;
    if (relative_seconds < 0.0 || relative_seconds > duration_seconds) {
      continue;
    }
    const int x = plot.left() + static_cast<int>(
      std::round(relative_seconds / duration_seconds * plot.width()));
    const int y = plot.top() + row.value() * row_height + row_height / 2;
    painter.setPen(topic_color(row.value()));
    painter.drawLine(x, y - 5, x, y + 5);
  }

  painter.setPen(palette().text().color());
  painter.drawText(8, 10, QString("消息时间轴：%1 条消息，显示 %2 个话题")
    .arg(statistics_.message_count).arg(row_topics.size()));
  painter.setPen(QColor("#9aa5b1"));
  painter.drawText(plot.left(), height() - 12,
    QString("记录接收时间，相对 bag 起点；红色区域：疑似断流（%1 段）").arg(gaps_.size()));
}

QString TimelineWidget::topic_at_position(const QPoint & position) const
{
  const auto plot = plot_rect();
  if (!plot.contains(position)) {
    return {};
  }
  QVector<QString> row_topics;
  for (const auto & topic : statistics_.topics) {
    if (visible_topics_.contains(QString::fromStdString(topic.metadata.name))) {
      row_topics.push_back(QString::fromStdString(topic.metadata.name));
    }
  }
  if (row_topics.isEmpty()) {
    return {};
  }
  const int row_height = std::max(24, plot.height() / row_topics.size());
  const int row = (position.y() - plot.top()) / row_height;
  return row >= 0 && row < row_topics.size() ? row_topics.at(row) : QString();
}

const TopicGap * TimelineWidget::gap_at_position(const QPoint & position) const
{
  const auto plot = plot_rect();
  if (!plot.contains(position) || statistics_.duration_ns <= 0) {
    return nullptr;
  }
  QVector<QString> row_topics;
  for (const auto & topic : statistics_.topics) {
    if (visible_topics_.contains(QString::fromStdString(topic.metadata.name))) {
      row_topics.push_back(QString::fromStdString(topic.metadata.name));
    }
  }
  if (row_topics.isEmpty()) {
    return nullptr;
  }
  const int row_height = std::max(24, plot.height() / row_topics.size());
  const int row = (position.y() - plot.top()) / row_height;
  if (row < 0 || row >= row_topics.size()) {
    return nullptr;
  }
  const auto topic = row_topics.at(row).toStdString();
  const double duration_seconds = static_cast<double>(statistics_.duration_ns) / 1.0e9;
  const double relative_seconds = static_cast<double>(position.x() - plot.left()) /
    static_cast<double>(plot.width()) * duration_seconds;
  for (const auto & gap : gaps_) {
    if (gap.topic_name != topic) {
      continue;
    }
    const double start = static_cast<double>(gap.start_timestamp_ns - statistics_.start_time_ns) / 1.0e9;
    const double end = static_cast<double>(gap.end_timestamp_ns - statistics_.start_time_ns) / 1.0e9;
    if (relative_seconds >= start && relative_seconds <= end) {
      return &gap;
    }
  }
  return nullptr;
}

void TimelineWidget::mouseMoveEvent(QMouseEvent * event)
{
  const auto plot = plot_rect();
  const auto topic = topic_at_position(event->pos());
  if (topic.isEmpty() || statistics_.duration_ns <= 0 || !plot.contains(event->pos())) {
    QToolTip::hideText();
    hovered_topic_.clear();
    return;
  }

  if (const auto * gap = gap_at_position(event->pos())) {
    QToolTip::showText(event->globalPos(), QString("疑似断流\n%1\n间隔：%2 s\n检测阈值：%3 s")
      .arg(topic)
      .arg(static_cast<double>(gap->duration_ns) / 1.0e9, 0, 'f', 3)
      .arg(static_cast<double>(gap->threshold_ns) / 1.0e9, 0, 'f', 3), this);
    return;
  }

  const double duration_seconds = static_cast<double>(statistics_.duration_ns) / 1.0e9;
  const double relative_seconds = static_cast<double>(event->pos().x() - plot.left()) /
    static_cast<double>(plot.width()) * duration_seconds;
  if (topic != hovered_topic_ || std::abs(relative_seconds - hovered_time_seconds_) > 0.05) {
    hovered_topic_ = topic;
    hovered_time_seconds_ = relative_seconds;
    QToolTip::showText(event->globalPos(),
      QString("%1\n相对时间：%2 s").arg(topic).arg(relative_seconds, 0, 'f', 3), this);
  }
}

void TimelineWidget::leaveEvent(QEvent * event)
{
  Q_UNUSED(event);
  hovered_topic_.clear();
  QToolTip::hideText();
}

}  // namespace rosbag_sensor_trimmer
