#ifndef ROSBAG_SENSOR_TRIMMER__ROS1_LIVOX_CONVERTER_HPP_
#define ROSBAG_SENSOR_TRIMMER__ROS1_LIVOX_CONVERTER_HPP_

#include <atomic>
#include <cstdint>
#include <filesystem>
#include <functional>
#include <optional>
#include <string>
#include <vector>

namespace rosbag_sensor_trimmer
{

extern const char * const kDefaultRos1LivoxLidarTopic;
extern const char * const kDefaultRos1LivoxImuTopic;
extern const char * const kDefaultRos2LivoxLidarTopic;
extern const char * const kDefaultRos2LivoxImuTopic;

struct Ros1LivoxTopicStats
{
  std::string topic;
  std::string datatype;
  std::uint64_t count{0};
  std::optional<std::int64_t> first_timestamp_ns;
  std::optional<std::int64_t> last_timestamp_ns;
};

struct Ros1LivoxConversionStats
{
  std::vector<std::filesystem::path> input_files;
  std::filesystem::path output_uri;
  std::optional<std::int64_t> start_timestamp_ns;
  std::optional<std::int64_t> end_timestamp_ns;
  std::uint64_t read_messages{0};
  std::uint64_t skipped_messages{0};
  std::uint64_t written_lidar{0};
  std::uint64_t written_imu{0};
  std::vector<Ros1LivoxTopicStats> topics;
  std::vector<std::string> warnings;
};

struct Ros1LivoxConversionProgress
{
  std::uint64_t read_messages{0};
  std::uint64_t skipped_messages{0};
  std::uint64_t written_lidar{0};
  std::uint64_t written_imu{0};
};

struct Ros1LivoxConversionOptions
{
  std::vector<std::filesystem::path> inputs;
  std::filesystem::path output_uri;
  std::string output_storage_id{"sqlite3"};
  bool overwrite_output{false};
  std::uint64_t max_lidar_messages{0};
  std::optional<std::int64_t> start_time_ns;
  std::optional<std::int64_t> end_time_ns;
  std::string input_lidar_topic{kDefaultRos1LivoxLidarTopic};
  std::string input_imu_topic{kDefaultRos1LivoxImuTopic};
  std::string output_lidar_topic{kDefaultRos2LivoxLidarTopic};
  std::string output_imu_topic{kDefaultRos2LivoxImuTopic};
};

using Ros1LivoxProgressCallback =
  std::function<void(const Ros1LivoxConversionProgress &)>;

std::vector<std::filesystem::path> resolve_ros1_livox_input_files(
  const std::vector<std::filesystem::path> & inputs);

Ros1LivoxConversionStats scan_ros1_livox_bag(
  const Ros1LivoxConversionOptions & options,
  const std::atomic_bool * cancel_requested = nullptr);

Ros1LivoxConversionStats convert_ros1_livox_bag_to_ros2(
  const Ros1LivoxConversionOptions & options,
  const Ros1LivoxProgressCallback & progress_callback = {},
  const std::atomic_bool * cancel_requested = nullptr);

std::string format_ros1_livox_conversion_summary(
  const Ros1LivoxConversionStats & stats,
  bool include_output);

}  // namespace rosbag_sensor_trimmer

#endif  // ROSBAG_SENSOR_TRIMMER__ROS1_LIVOX_CONVERTER_HPP_
