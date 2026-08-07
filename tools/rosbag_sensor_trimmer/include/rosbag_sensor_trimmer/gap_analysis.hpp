#ifndef ROSBAG_SENSOR_TRIMMER__GAP_ANALYSIS_HPP_
#define ROSBAG_SENSOR_TRIMMER__GAP_ANALYSIS_HPP_

#include <cstdint>
#include <string>
#include <vector>

#include "rosbag_sensor_trimmer/bag_index.hpp"

namespace rosbag_sensor_trimmer
{

struct TopicGap
{
  std::string topic_name;
  std::int64_t start_timestamp_ns{0};
  std::int64_t end_timestamp_ns{0};
  std::int64_t duration_ns{0};
  std::int64_t threshold_ns{0};
};

std::vector<TopicGap> detect_topic_gaps(
  const BagStatistics & statistics,
  const std::vector<IndexEntry> & entries,
  double period_multiplier = 3.0,
  std::int64_t minimum_threshold_ns = 100000000);

}  // namespace rosbag_sensor_trimmer

#endif  // ROSBAG_SENSOR_TRIMMER__GAP_ANALYSIS_HPP_
