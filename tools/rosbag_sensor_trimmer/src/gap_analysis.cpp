#include "rosbag_sensor_trimmer/gap_analysis.hpp"

#include <algorithm>
#include <cmath>
#include <unordered_map>

namespace rosbag_sensor_trimmer
{

std::vector<TopicGap> detect_topic_gaps(
  const BagStatistics & statistics,
  const std::vector<IndexEntry> & entries,
  double period_multiplier,
  std::int64_t minimum_threshold_ns)
{
  std::unordered_map<std::string, std::int64_t> thresholds;
  for (const auto & topic : statistics.topics) {
    std::int64_t threshold = minimum_threshold_ns;
    if (topic.average_frequency_hz > 0.0 && std::isfinite(topic.average_frequency_hz)) {
      const auto expected_period_ns = static_cast<std::int64_t>(
        1.0e9 / topic.average_frequency_hz * period_multiplier);
      threshold = std::max(threshold, expected_period_ns);
    }
    thresholds.emplace(topic.metadata.name, threshold);
  }

  std::unordered_map<std::string, std::int64_t> last_timestamps;
  std::vector<TopicGap> gaps;
  for (const auto & entry : entries) {
    const auto threshold_it = thresholds.find(entry.topic_name);
    if (threshold_it == thresholds.end()) {
      continue;
    }
    const auto last_it = last_timestamps.find(entry.topic_name);
    if (last_it != last_timestamps.end()) {
      const auto delta = entry.timestamp_ns - last_it->second;
      if (delta > threshold_it->second) {
        gaps.push_back(TopicGap{
          entry.topic_name,
          last_it->second,
          entry.timestamp_ns,
          delta,
          threshold_it->second});
      }
    }
    last_timestamps[entry.topic_name] = entry.timestamp_ns;
  }

  std::sort(gaps.begin(), gaps.end(), [](const TopicGap & left, const TopicGap & right) {
    if (left.start_timestamp_ns != right.start_timestamp_ns) {
      return left.start_timestamp_ns < right.start_timestamp_ns;
    }
    return left.topic_name < right.topic_name;
  });
  return gaps;
}

}  // namespace rosbag_sensor_trimmer
