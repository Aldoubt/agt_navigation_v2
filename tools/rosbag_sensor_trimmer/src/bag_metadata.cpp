#include "rosbag_sensor_trimmer/bag_metadata.hpp"

#include <chrono>
#include <iomanip>
#include <sstream>

namespace rosbag_sensor_trimmer
{

std::int64_t time_point_to_nanoseconds(
  const std::chrono::time_point<std::chrono::high_resolution_clock> & time_point)
{
  return std::chrono::duration_cast<std::chrono::nanoseconds>(time_point.time_since_epoch()).count();
}

std::uint64_t directory_size_bytes(const std::filesystem::path & uri)
{
  std::error_code error;
  if (std::filesystem::is_regular_file(uri, error)) {
    return std::filesystem::file_size(uri, error);
  }

  std::uint64_t size = 0;
  if (!std::filesystem::is_directory(uri, error)) {
    return size;
  }

  for (const auto & entry : std::filesystem::recursive_directory_iterator(uri, error)) {
    if (error) {
      break;
    }
    const auto path = entry.path();
    const auto filename = path.filename().string();
    const auto extension = path.extension().string();
    const bool is_bag_file = filename == "metadata.yaml" || extension == ".db3" ||
      extension == ".mcap" || extension == ".zstd";
    const bool is_decompressed_temporary = extension == ".db3" &&
      std::filesystem::exists(path.string() + ".zstd");
    if (is_bag_file && !is_decompressed_temporary && entry.is_regular_file(error)) {
      size += entry.file_size(error);
    }
  }
  return size;
}

std::string format_duration_seconds(std::int64_t duration_ns)
{
  std::ostringstream stream;
  stream << std::fixed << std::setprecision(3)
         << static_cast<double>(duration_ns) / 1.0e9 << " s";
  return stream.str();
}

}  // namespace rosbag_sensor_trimmer
