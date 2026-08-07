#ifndef ROSBAG_SENSOR_TRIMMER__INTEGRITY_VALIDATOR_HPP_
#define ROSBAG_SENSOR_TRIMMER__INTEGRITY_VALIDATOR_HPP_

#include <cstdint>
#include <filesystem>
#include <string>
#include <unordered_map>
#include <vector>

#include "rosbag_sensor_trimmer/bag_metadata.hpp"
#include "rosbag_sensor_trimmer/trim_job.hpp"

namespace rosbag_sensor_trimmer
{

struct IntegrityOptions
{
  bool require_lidar_imu_overlap{true};
  double maximum_imu_gap_sec{0.1};
  double maximum_lidar_gap_sec{0.5};
};

struct IntegrityReport
{
  bool ok{false};
  bool metadata_present{false};
  bool topic_set_correct{false};
  bool time_range_correct{false};
  bool timestamps_monotonic{false};
  bool storage_consistent{false};
  bool lidar_imu_overlap{false};
  bool imu_covers_lidar_start{false};
  bool has_zero_message_topics{false};
  BagStatistics output_statistics;
  std::vector<std::string> errors;
  std::vector<std::string> warnings;
};

class IntegrityValidator
{
public:
  static IntegrityReport validate(
    const TrimJob & job,
    const std::vector<rosbag2_storage::TopicMetadata> & expected_topics,
    const std::unordered_map<std::string, std::uint64_t> & expected_counts,
    const IntegrityOptions & options = IntegrityOptions());

  static IntegrityReport validate_basic(
    const std::filesystem::path & uri,
    const std::string & storage_id = "",
    const IntegrityOptions & options = IntegrityOptions());

  static void write_json(const std::filesystem::path & path, const IntegrityReport & report);
  static void write_markdown(const std::filesystem::path & path, const IntegrityReport & report);
};

}  // namespace rosbag_sensor_trimmer

#endif  // ROSBAG_SENSOR_TRIMMER__INTEGRITY_VALIDATOR_HPP
