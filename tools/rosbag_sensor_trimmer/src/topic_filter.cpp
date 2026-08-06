#include "rosbag_sensor_trimmer/topic_filter.hpp"

#include <algorithm>

namespace rosbag_sensor_trimmer
{

TopicKind classify_topic(const rosbag2_storage::TopicMetadata & topic)
{
  if (topic.type == "sensor_msgs/msg/PointCloud2" ||
    topic.type == "livox_ros_driver2/msg/CustomMsg") {
    return TopicKind::Lidar;
  }
  if (topic.type == "sensor_msgs/msg/Imu") {
    return TopicKind::Imu;
  }
  if (topic.type == "sensor_msgs/msg/LaserScan") {
    return TopicKind::LaserScan;
  }
  if (topic.type == "tf2_msgs/msg/TFMessage") {
    return TopicKind::Tf;
  }
  if (topic.type == "nav_msgs/msg/Odometry") {
    return TopicKind::Odometry;
  }
  return TopicKind::Unknown;
}

std::string topic_kind_to_string(TopicKind kind)
{
  switch (kind) {
    case TopicKind::Lidar: return "LiDAR";
    case TopicKind::Imu: return "IMU";
    case TopicKind::LaserScan: return "LaserScan";
    case TopicKind::Tf: return "TF";
    case TopicKind::Odometry: return "Odometry";
    default: return "Unknown";
  }
}

void TopicFilter::set_include_topics(const std::vector<std::string> & topics)
{
  include_topics_ = topics;
  include_topic_set_ = std::unordered_set<std::string>(topics.begin(), topics.end());
}

void TopicFilter::set_exclude_topics(const std::vector<std::string> & topics)
{
  exclude_topics_ = topics;
  exclude_topic_set_ = std::unordered_set<std::string>(topics.begin(), topics.end());
}

void TopicFilter::set_include_types(const std::vector<std::string> & types)
{
  include_types_ = types;
  include_type_set_ = std::unordered_set<std::string>(types.begin(), types.end());
}

bool TopicFilter::matches(const rosbag2_storage::TopicMetadata & topic) const
{
  if (!include_topics_.empty() && include_topic_set_.count(topic.name) == 0) {
    return false;
  }
  if (exclude_topic_set_.count(topic.name) != 0) {
    return false;
  }
  if (!include_types_.empty() && include_type_set_.count(topic.type) == 0) {
    return false;
  }
  return true;
}

bool TopicFilter::matches(const std::string & topic_name) const
{
  if (!include_topics_.empty() && include_topic_set_.count(topic_name) == 0) {
    return false;
  }
  return exclude_topic_set_.count(topic_name) == 0;
}

bool TopicFilter::has_include_topics() const noexcept
{
  return !include_topics_.empty();
}

const std::vector<std::string> & TopicFilter::include_topics() const noexcept
{
  return include_topics_;
}

const std::vector<std::string> & TopicFilter::exclude_topics() const noexcept
{
  return exclude_topics_;
}

const std::vector<std::string> & TopicFilter::include_types() const noexcept
{
  return include_types_;
}

std::vector<rosbag2_storage::TopicMetadata> TopicFilter::select(
  const std::vector<rosbag2_storage::TopicMetadata> & topics) const
{
  std::vector<rosbag2_storage::TopicMetadata> selected;
  std::copy_if(topics.begin(), topics.end(), std::back_inserter(selected),
    [this](const auto & topic) {return matches(topic);});
  return selected;
}

std::vector<rosbag2_storage::TopicMetadata> recommended_topics(
  const std::vector<rosbag2_storage::TopicMetadata> & topics)
{
  std::vector<rosbag2_storage::TopicMetadata> selected;
  for (const auto & topic : topics) {
    const auto kind = classify_topic(topic);
    const bool is_static_tf = kind == TopicKind::Tf && topic.name == "/tf_static";
    if (kind == TopicKind::Lidar || kind == TopicKind::Imu || is_static_tf) {
      selected.push_back(topic);
    }
  }
  return selected;
}

}  // namespace rosbag_sensor_trimmer
