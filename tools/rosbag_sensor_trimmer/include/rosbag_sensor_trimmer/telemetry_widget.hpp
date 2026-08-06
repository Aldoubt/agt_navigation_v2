#ifndef ROSBAG_SENSOR_TRIMMER__TELEMETRY_WIDGET_HPP_
#define ROSBAG_SENSOR_TRIMMER__TELEMETRY_WIDGET_HPP_

#include <QPointF>
#include <QVector>
#include <QWidget>

#include <cstdint>

namespace rosbag_sensor_trimmer
{

class TelemetryWidget : public QWidget
{
  Q_OBJECT

public:
  explicit TelemetryWidget(QWidget * parent = nullptr);

  void clear();
  void add_odometry(double x, double y);
  void set_tf_summary(int transform_count, const QString & frames);

  QSize minimumSizeHint() const override;

protected:
  void paintEvent(QPaintEvent * event) override;

private:
  QVector<QPointF> trajectory_;
  int transform_count_{0};
  QString tf_frames_;
};

}  // namespace rosbag_sensor_trimmer

#endif  // ROSBAG_SENSOR_TRIMMER__TELEMETRY_WIDGET_HPP_
