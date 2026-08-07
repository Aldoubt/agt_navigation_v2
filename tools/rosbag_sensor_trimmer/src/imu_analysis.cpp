#include "rosbag_sensor_trimmer/imu_data.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>
#include <stdexcept>
#include <vector>

namespace rosbag_sensor_trimmer
{

namespace
{

double median(std::vector<double> values)
{
  if (values.empty()) {
    return 0.0;
  }
  const auto middle = values.begin() + static_cast<std::ptrdiff_t>(values.size() / 2);
  std::nth_element(values.begin(), middle, values.end());
  if (values.size() % 2 == 1) {
    return *middle;
  }
  const auto lower = *std::max_element(values.begin(), middle);
  return (lower + *middle) / 2.0;
}

double median_absolute_deviation(const std::vector<double> & values, double center)
{
  std::vector<double> deviations;
  deviations.reserve(values.size());
  for (const auto value : values) {
    deviations.push_back(std::abs(value - center));
  }
  return median(std::move(deviations));
}

}  // namespace

ImuMotionEstimate estimate_imu_motion(
  const std::vector<ImuSample> & samples,
  std::int64_t bag_start_timestamp_ns)
{
  ImuMotionEstimate estimate;
  if (samples.size() < 20) {
    return estimate;
  }

  const auto first_timestamp = samples.front().timestamp_ns;
  const auto baseline_end = first_timestamp + std::min<std::int64_t>(
    2'000'000'000LL,
    std::max<std::int64_t>(500'000'000LL, samples.back().timestamp_ns - first_timestamp) / 4);
  std::vector<double> acceleration_values;
  std::vector<double> angular_velocity_values;
  for (const auto & sample : samples) {
    if (sample.timestamp_ns > baseline_end) {
      break;
    }
    acceleration_values.push_back(sample.acceleration_magnitude);
    angular_velocity_values.push_back(sample.angular_velocity_magnitude);
  }
  if (acceleration_values.size() < 10) {
    return estimate;
  }

  estimate.acceleration_baseline = median(acceleration_values);
  estimate.angular_velocity_baseline = median(angular_velocity_values);
  const auto acceleration_mad = median_absolute_deviation(
    acceleration_values, estimate.acceleration_baseline);
  const auto angular_velocity_mad = median_absolute_deviation(
    angular_velocity_values, estimate.angular_velocity_baseline);
  estimate.acceleration_threshold = std::max(
    0.35, 6.0 * acceleration_mad + 0.05);
  estimate.angular_velocity_threshold = std::max(
    0.08, estimate.angular_velocity_baseline + 6.0 * angular_velocity_mad + 0.03);

  std::size_t consecutive_active = 0;
  constexpr std::size_t required_consecutive_samples = 10;
  for (const auto & sample : samples) {
    if (sample.timestamp_ns <= baseline_end) {
      continue;
    }
    const bool active =
      std::abs(sample.acceleration_magnitude - estimate.acceleration_baseline) >
      estimate.acceleration_threshold ||
      sample.angular_velocity_magnitude > estimate.angular_velocity_threshold;
    if (active) {
      ++consecutive_active;
      if (consecutive_active >= required_consecutive_samples) {
        estimate.valid = true;
        estimate.start_timestamp_ns = sample.timestamp_ns;
        estimate.relative_start_seconds = static_cast<double>(
          estimate.start_timestamp_ns - bag_start_timestamp_ns) / 1.0e9;
        return estimate;
      }
    } else {
      consecutive_active = 0;
    }
  }
  return estimate;
}

}  // namespace rosbag_sensor_trimmer
