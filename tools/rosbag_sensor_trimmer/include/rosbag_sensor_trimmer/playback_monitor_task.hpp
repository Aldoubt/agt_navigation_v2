#ifndef ROSBAG_SENSOR_TRIMMER__PLAYBACK_MONITOR_TASK_HPP_
#define ROSBAG_SENSOR_TRIMMER__PLAYBACK_MONITOR_TASK_HPP_

#include <QThread>

#include <atomic>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "rosbag_sensor_trimmer/pointcloud_preview_widget.hpp"

namespace rclcpp
{
namespace executors
{
class SingleThreadedExecutor;
}
}

namespace rosbag_sensor_trimmer
{

class PlaybackMonitorTask : public QThread
{
  Q_OBJECT

public:
  PlaybackMonitorTask(
    const std::vector<std::string> & odometry_topics,
    const std::vector<std::string> & tf_topics,
    const std::string & pointcloud_topic,
    QObject * parent = nullptr);
  ~PlaybackMonitorTask() override;

  void request_stop();

signals:
  void odometry_changed(double x, double y, qlonglong timestamp_ns);
  void tf_summary_changed(int transform_count, const QString & frames);
  void cloud_ready(const PointCloudFrame & frame);
  void monitor_error(const QString & message);

protected:
  void run() override;

private:
  std::vector<std::string> odometry_topics_;
  std::vector<std::string> tf_topics_;
  std::string pointcloud_topic_;
  std::atomic_bool stop_requested_{false};
  std::mutex executor_mutex_;
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
};

}  // namespace rosbag_sensor_trimmer

#endif  // ROSBAG_SENSOR_TRIMMER__PLAYBACK_MONITOR_TASK_HPP_
