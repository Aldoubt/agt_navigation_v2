#include "rosbag_sensor_trimmer/playback_monitor_task.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <exception>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/executors/single_threaded_executor.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/node_options.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/point_field.hpp"
#include "tf2_msgs/msg/tf_message.hpp"

#include <QStringList>

namespace rosbag_sensor_trimmer
{

namespace
{

std::int64_t timestamp_ns(const builtin_interfaces::msg::Time & stamp)
{
  return static_cast<std::int64_t>(stamp.sec) * 1000000000LL +
         static_cast<std::int64_t>(stamp.nanosec);
}

std::uint32_t reverse_u32(std::uint32_t value)
{
  return ((value & 0x000000FFU) << 24U) |
         ((value & 0x0000FF00U) << 8U) |
         ((value & 0x00FF0000U) >> 8U) |
         ((value & 0xFF000000U) >> 24U);
}

std::uint64_t reverse_u64(std::uint64_t value)
{
  return ((value & 0x00000000000000FFULL) << 56U) |
         ((value & 0x000000000000FF00ULL) << 40U) |
         ((value & 0x0000000000FF0000ULL) << 24U) |
         ((value & 0x00000000FF000000ULL) << 8U) |
         ((value & 0x000000FF00000000ULL) >> 8U) |
         ((value & 0x0000FF0000000000ULL) >> 24U) |
         ((value & 0x00FF000000000000ULL) >> 40U) |
         ((value & 0xFF00000000000000ULL) >> 56U);
}

bool read_point_value(
  const std::vector<std::uint8_t> & data,
  std::size_t offset,
  std::uint8_t datatype,
  bool big_endian,
  float & value)
{
  if (datatype == sensor_msgs::msg::PointField::FLOAT32) {
    if (offset + sizeof(std::uint32_t) > data.size()) {
      return false;
    }
    std::uint32_t bits = 0;
    std::memcpy(&bits, data.data() + offset, sizeof(bits));
    if (big_endian) {
      bits = reverse_u32(bits);
    }
    std::memcpy(&value, &bits, sizeof(value));
    return std::isfinite(value);
  }
  if (datatype == sensor_msgs::msg::PointField::FLOAT64) {
    if (offset + sizeof(std::uint64_t) > data.size()) {
      return false;
    }
    std::uint64_t bits = 0;
    std::memcpy(&bits, data.data() + offset, sizeof(bits));
    if (big_endian) {
      bits = reverse_u64(bits);
    }
    double double_value = 0.0;
    std::memcpy(&double_value, &bits, sizeof(double_value));
    value = static_cast<float>(double_value);
    return std::isfinite(double_value);
  }
  return false;
}

PointCloudFrame decode_point_cloud(const sensor_msgs::msg::PointCloud2 & cloud)
{
  const sensor_msgs::msg::PointField * x_field = nullptr;
  const sensor_msgs::msg::PointField * y_field = nullptr;
  const sensor_msgs::msg::PointField * z_field = nullptr;
  for (const auto & field : cloud.fields) {
    if (field.name == "x") {
      x_field = &field;
    } else if (field.name == "y") {
      y_field = &field;
    } else if (field.name == "z") {
      z_field = &field;
    }
  }
  if (!x_field || !y_field || !z_field || cloud.point_step == 0) {
    return {};
  }

  const std::size_t row_width = cloud.width == 0 ?
    cloud.data.size() / cloud.point_step : cloud.width;
  const std::size_t rows = cloud.height == 0 ? 1 : cloud.height;
  const std::size_t point_count = std::min(
    rows * row_width, cloud.data.size() / cloud.point_step);
  if (point_count == 0) {
    return {};
  }
  constexpr std::size_t max_points = 60000;
  const std::size_t sample_step = std::max<std::size_t>(1, point_count / max_points + 1);
  PointCloudFrame points;
  points.reserve(std::min(max_points, point_count));
  for (std::size_t index = 0; index < point_count; index += sample_step) {
    const auto row = index / row_width;
    const auto column = index % row_width;
    const auto base = row * static_cast<std::size_t>(cloud.row_step) +
      column * static_cast<std::size_t>(cloud.point_step);
    PointCloudPoint point;
    if (!read_point_value(cloud.data, base + x_field->offset, x_field->datatype,
      cloud.is_bigendian, point.x) ||
      !read_point_value(cloud.data, base + y_field->offset, y_field->datatype,
      cloud.is_bigendian, point.y) ||
      !read_point_value(cloud.data, base + z_field->offset, z_field->datatype,
      cloud.is_bigendian, point.z))
    {
      continue;
    }
    if (std::abs(point.x) > 100000.0F || std::abs(point.y) > 100000.0F ||
      std::abs(point.z) > 100000.0F)
    {
      continue;
    }
    points.push_back(point);
  }
  return points;
}

}  // namespace

PlaybackMonitorTask::PlaybackMonitorTask(
  const std::vector<std::string> & odometry_topics,
  const std::vector<std::string> & tf_topics,
  const std::string & pointcloud_topic,
  QObject * parent)
: QThread(parent), odometry_topics_(odometry_topics), tf_topics_(tf_topics),
  pointcloud_topic_(pointcloud_topic)
{
}

PlaybackMonitorTask::~PlaybackMonitorTask()
{
  if (isRunning()) {
    request_stop();
    wait();
  }
}

void PlaybackMonitorTask::request_stop()
{
  stop_requested_ = true;
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor;
  {
    std::lock_guard<std::mutex> lock(executor_mutex_);
    executor = executor_;
  }
  if (executor) {
    executor->cancel();
  }
}

void PlaybackMonitorTask::run()
{
  if (odometry_topics_.empty() && tf_topics_.empty() && pointcloud_topic_.empty()) {
    return;
  }
  if (!rclcpp::ok()) {
    emit monitor_error("ROS 播放上下文未运行，无法启动播放监视器");
    return;
  }

  try {
    rclcpp::NodeOptions node_options;
    auto node = std::make_shared<rclcpp::Node>("rosbag_sensor_trimmer_monitor", node_options);
    const auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();
    std::vector<rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr> odometry_subscriptions;
    std::vector<rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr> tf_subscriptions;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_subscription;

    for (const auto & topic : odometry_topics_) {
      odometry_subscriptions.push_back(node->create_subscription<nav_msgs::msg::Odometry>(
        topic, qos, [this](const nav_msgs::msg::Odometry::SharedPtr message) {
          if (!message) {
            return;
          }
          emit odometry_changed(message->pose.pose.position.x, message->pose.pose.position.y,
            static_cast<qlonglong>(timestamp_ns(message->header.stamp)));
        }));
    }
    for (const auto & topic : tf_topics_) {
      auto tf_qos = qos;
      if (topic == "/tf_static") {
        tf_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
      }
      tf_subscriptions.push_back(node->create_subscription<tf2_msgs::msg::TFMessage>(
        topic, tf_qos, [this](const tf2_msgs::msg::TFMessage::SharedPtr message) {
          if (!message) {
            return;
          }
          QStringList frames;
          for (const auto & transform : message->transforms) {
            if (!transform.header.frame_id.empty()) {
              frames.push_back(QString::fromStdString(transform.header.frame_id) +
                " -> " + QString::fromStdString(transform.child_frame_id));
            }
            if (frames.size() >= 8) {
              break;
            }
          }
          emit tf_summary_changed(
            static_cast<int>(message->transforms.size()), frames.join(", "));
        }));
    }
    if (!pointcloud_topic_.empty()) {
      pointcloud_subscription = node->create_subscription<sensor_msgs::msg::PointCloud2>(
        pointcloud_topic_, qos, [this](const sensor_msgs::msg::PointCloud2::SharedPtr message) {
          if (message) {
            emit cloud_ready(decode_point_cloud(*message));
          }
        });
    }

    rclcpp::ExecutorOptions executor_options;
    executor_options.context = node->get_node_base_interface()->get_context();
    auto executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>(executor_options);
    executor->add_node(node);
    {
      std::lock_guard<std::mutex> lock(executor_mutex_);
      executor_ = executor;
    }
    if (stop_requested_) {
      executor->cancel();
    }
    executor->spin();
    {
      std::lock_guard<std::mutex> lock(executor_mutex_);
      executor_.reset();
    }
  } catch (const std::exception & exception) {
    emit monitor_error(QString::fromUtf8(exception.what()));
  }
}

}  // namespace rosbag_sensor_trimmer
