#ifndef ROSBAG_SENSOR_TRIMMER__TIMELINE_WIDGET_HPP_
#define ROSBAG_SENSOR_TRIMMER__TIMELINE_WIDGET_HPP_

#include <QSet>
#include <QWidget>

#include <vector>

#include "rosbag_sensor_trimmer/bag_index.hpp"
#include "rosbag_sensor_trimmer/gap_analysis.hpp"

namespace rosbag_sensor_trimmer
{

class TimelineWidget : public QWidget
{
  Q_OBJECT

public:
  explicit TimelineWidget(QWidget * parent = nullptr);

  void set_data(const BagStatistics & statistics, const std::vector<IndexEntry> & entries);
  void set_gaps(const std::vector<TopicGap> & gaps);
  void set_visible_topics(const QSet<QString> & topics);

  QSize minimumSizeHint() const override;

protected:
  void paintEvent(QPaintEvent * event) override;
  void mouseMoveEvent(QMouseEvent * event) override;
  void leaveEvent(QEvent * event) override;

private:
  QString topic_at_position(const QPoint & position) const;
  const TopicGap * gap_at_position(const QPoint & position) const;
  QRect plot_rect() const;

  BagStatistics statistics_;
  std::vector<IndexEntry> entries_;
  std::vector<TopicGap> gaps_;
  QSet<QString> visible_topics_;
  QString hovered_topic_;
  double hovered_time_seconds_{0.0};
};

}  // namespace rosbag_sensor_trimmer

#endif  // ROSBAG_SENSOR_TRIMMER__TIMELINE_WIDGET_HPP_
