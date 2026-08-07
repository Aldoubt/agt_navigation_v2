#include "rosbag_sensor_trimmer/time_range.hpp"

#include <cmath>
#include <iomanip>
#include <limits>
#include <sstream>
#include <stdexcept>

namespace rosbag_sensor_trimmer
{

bool TimeRange::contains(std::int64_t timestamp_ns) const noexcept
{
  return start_time_ns <= timestamp_ns && timestamp_ns < end_time_ns;
}

std::int64_t TimeRange::duration_ns() const noexcept
{
  return end_time_ns - start_time_ns;
}

void TimeRange::validate() const
{
  if (end_time_ns <= start_time_ns) {
    throw std::invalid_argument("时间范围无效：必须满足 start_time_ns < end_time_ns");
  }
  if (static_cast<long double>(end_time_ns) - static_cast<long double>(start_time_ns) >
    static_cast<long double>(std::numeric_limits<std::int64_t>::max())) {
    throw std::invalid_argument("时间范围跨度超出 int64 纳秒表示范围");
  }
}

TimeRange make_relative_time_range(
  std::int64_t bag_start_time_ns, double start_seconds, double end_seconds)
{
  if (!std::isfinite(start_seconds) || !std::isfinite(end_seconds)) {
    throw std::invalid_argument("相对时间必须是有限数字");
  }
  if (start_seconds < 0.0 || end_seconds <= start_seconds) {
    throw std::invalid_argument("相对时间必须满足 0 <= start < end");
  }

  const long double start_offset = static_cast<long double>(start_seconds) * 1.0e9L;
  const long double end_offset = static_cast<long double>(end_seconds) * 1.0e9L;
  const auto minimum = static_cast<long double>(std::numeric_limits<std::int64_t>::min());
  const auto maximum = static_cast<long double>(std::numeric_limits<std::int64_t>::max());
  if (start_offset > maximum || end_offset > maximum ||
    static_cast<long double>(bag_start_time_ns) + end_offset > maximum ||
    static_cast<long double>(bag_start_time_ns) + start_offset < minimum) {
    throw std::invalid_argument("相对时间转换后超出 int64 纳秒时间戳范围");
  }

  TimeRange range{
    bag_start_time_ns + static_cast<std::int64_t>(std::llround(start_offset)),
    bag_start_time_ns + static_cast<std::int64_t>(std::llround(end_offset))};
  range.validate();
  return range;
}

TimeRange make_absolute_time_range(std::int64_t start_time_ns, std::int64_t end_time_ns)
{
  TimeRange range{start_time_ns, end_time_ns};
  range.validate();
  return range;
}

std::string time_range_to_string(const TimeRange & range)
{
  std::ostringstream stream;
  stream << "[" << range.start_time_ns << ", " << range.end_time_ns << ")";
  return stream.str();
}

}  // namespace rosbag_sensor_trimmer
