#ifndef ROSBAG_SENSOR_TRIMMER__IMU_PLOT_WIDGET_HPP_
#define ROSBAG_SENSOR_TRIMMER__IMU_PLOT_WIDGET_HPP_

#include <QWidget>

#include <vector>

#include "rosbag_sensor_trimmer/imu_data.hpp"

namespace rosbag_sensor_trimmer
{

class ImuPlotWidget : public QWidget
{
  Q_OBJECT

public:
  explicit ImuPlotWidget(QWidget * parent = nullptr);

  void set_data(
    const std::vector<ImuSample> & samples,
    const ImuMotionEstimate & estimate,
    std::int64_t bag_start_timestamp_ns);

  QSize minimumSizeHint() const override;

protected:
  void paintEvent(QPaintEvent * event) override;
  void mouseMoveEvent(QMouseEvent * event) override;
  void mousePressEvent(QMouseEvent * event) override;
  void leaveEvent(QEvent * event) override;

signals:
  void time_selected(double relative_seconds);

private:
  QRect plot_rect() const;
  double duration_seconds() const;

  std::vector<ImuSample> samples_;
  ImuMotionEstimate estimate_;
  std::int64_t bag_start_timestamp_ns_{0};
};

}  // namespace rosbag_sensor_trimmer

#endif  // ROSBAG_SENSOR_TRIMMER__IMU_PLOT_WIDGET_HPP_
