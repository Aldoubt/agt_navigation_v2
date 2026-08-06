#ifndef ROSBAG_SENSOR_TRIMMER__BAG_METADATA_HPP_
#define ROSBAG_SENSOR_TRIMMER__BAG_METADATA_HPP_

#include <cstdint>
#include <chrono>
#include <filesystem>
#include <string>
#include <vector>

#include "rosbag2_storage/bag_metadata.hpp"

namespace rosbag_sensor_trimmer
{

struct TopicStatistics
{
  rosbag2_storage::TopicMetadata metadata;
  std::uint64_t message_count{0};
  std::uint64_t serialized_bytes{0};
  std::int64_t first_timestamp_ns{0};
  std::int64_t last_timestamp_ns{0};
  std::int64_t maximum_gap_ns{0};
  double average_frequency_hz{0.0};
  bool has_messages{false};
};

struct BagStatistics
{
  std::filesystem::path uri;
  std::string storage_id;
  std::string compression_mode;
  std::string compression_format;
  std::uint64_t file_size_bytes{0};
  std::uint64_t message_count{0};
  std::int64_t start_time_ns{0};
  std::int64_t end_time_ns{0};
  std::int64_t duration_ns{0};
  bool empty{true};
  std::vector<TopicStatistics> topics;
};

std::int64_t time_point_to_nanoseconds(
  const std::chrono::time_point<std::chrono::high_resolution_clock> & time_point);

std::uint64_t directory_size_bytes(const std::filesystem::path & uri);

std::string format_duration_seconds(std::int64_t duration_ns);

}  // namespace rosbag_sensor_trimmer

#endif  // ROSBAG_SENSOR_TRIMMER__BAG_METADATA_HPP_
