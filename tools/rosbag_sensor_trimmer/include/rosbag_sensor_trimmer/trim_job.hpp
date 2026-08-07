#ifndef ROSBAG_SENSOR_TRIMMER__TRIM_JOB_HPP_
#define ROSBAG_SENSOR_TRIMMER__TRIM_JOB_HPP_

#include <filesystem>
#include <string>
#include <vector>

#include "rosbag_sensor_trimmer/time_range.hpp"

namespace rosbag_sensor_trimmer
{

struct TrimJob
{
  std::filesystem::path input_uri;
  std::filesystem::path output_uri;

  std::string input_storage_id;
  std::string output_storage_id;

  std::int64_t start_time_ns{0};
  std::int64_t end_time_ns{0};

  std::vector<std::string> selected_topics;
  std::vector<std::string> excluded_topics;

  bool enable_compression{false};
  std::string compression_mode{"file"};
  std::string compression_format{"zstd"};
  bool overwrite_output{false};
};

void validate_trim_job(const TrimJob & job);

std::filesystem::path default_output_uri(
  const std::filesystem::path & input_uri, double start_seconds, double end_seconds);

}  // namespace rosbag_sensor_trimmer

#endif  // ROSBAG_SENSOR_TRIMMER__TRIM_JOB_HPP_
