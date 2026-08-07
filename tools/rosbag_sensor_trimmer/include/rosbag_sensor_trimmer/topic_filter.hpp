#ifndef ROSBAG_SENSOR_TRIMMER__TOPIC_FILTER_HPP_
#define ROSBAG_SENSOR_TRIMMER__TOPIC_FILTER_HPP_

#include <string>
#include <unordered_set>
#include <vector>

#include "rosbag2_storage/topic_metadata.hpp"

namespace rosbag_sensor_trimmer
{

enum class TopicKind
{
  Unknown,
  Lidar,
  Imu,
  LaserScan,
  Tf,
  Odometry
};

TopicKind classify_topic(const rosbag2_storage::TopicMetadata & topic);
std::string topic_kind_to_string(TopicKind kind);

class TopicFilter
{
public:
  void set_include_topics(const std::vector<std::string> & topics);
  void set_exclude_topics(const std::vector<std::string> & topics);
  void set_include_types(const std::vector<std::string> & types);

  bool matches(const rosbag2_storage::TopicMetadata & topic) const;
  bool matches(const std::string & topic_name) const;
  bool has_include_topics() const noexcept;

  const std::vector<std::string> & include_topics() const noexcept;
  const std::vector<std::string> & exclude_topics() const noexcept;
  const std::vector<std::string> & include_types() const noexcept;

  std::vector<rosbag2_storage::TopicMetadata> select(
    const std::vector<rosbag2_storage::TopicMetadata> & topics) const;

private:
  std::vector<std::string> include_topics_;
  std::vector<std::string> exclude_topics_;
  std::vector<std::string> include_types_;
  std::unordered_set<std::string> include_topic_set_;
  std::unordered_set<std::string> exclude_topic_set_;
  std::unordered_set<std::string> include_type_set_;
};

std::vector<rosbag2_storage::TopicMetadata> recommended_topics(
  const std::vector<rosbag2_storage::TopicMetadata> & topics);

}  // namespace rosbag_sensor_trimmer

#endif  // ROSBAG_SENSOR_TRIMMER__TOPIC_FILTER_HPP_
