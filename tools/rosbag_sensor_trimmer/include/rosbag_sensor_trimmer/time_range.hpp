#ifndef ROSBAG_SENSOR_TRIMMER__TIME_RANGE_HPP_
#define ROSBAG_SENSOR_TRIMMER__TIME_RANGE_HPP_

#include <cstdint>
#include <string>

namespace rosbag_sensor_trimmer
{

struct TimeRange
{
  std::int64_t start_time_ns{0};
  std::int64_t end_time_ns{0};

  bool contains(std::int64_t timestamp_ns) const noexcept;
  std::int64_t duration_ns() const noexcept;
  void validate() const;
};

TimeRange make_relative_time_range(
  std::int64_t bag_start_time_ns, double start_seconds, double end_seconds);

TimeRange make_absolute_time_range(std::int64_t start_time_ns, std::int64_t end_time_ns);

std::string time_range_to_string(const TimeRange & range);

}  // namespace rosbag_sensor_trimmer

#endif  // ROSBAG_SENSOR_TRIMMER__TIME_RANGE_HPP_
