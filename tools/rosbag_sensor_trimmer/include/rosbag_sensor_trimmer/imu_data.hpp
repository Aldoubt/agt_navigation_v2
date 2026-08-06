#ifndef ROSBAG_SENSOR_TRIMMER__IMU_DATA_HPP_
#define ROSBAG_SENSOR_TRIMMER__IMU_DATA_HPP_

#include <cstdint>
#include <vector>

namespace rosbag_sensor_trimmer
{

struct ImuSample
{
  std::int64_t timestamp_ns{0};
  double acceleration_magnitude{0.0};
  double angular_velocity_magnitude{0.0};
};

struct ImuMotionEstimate
{
  bool valid{false};
  std::int64_t start_timestamp_ns{0};
  double relative_start_seconds{0.0};
  double acceleration_baseline{0.0};
  double angular_velocity_baseline{0.0};
  double acceleration_threshold{0.0};
  double angular_velocity_threshold{0.0};
};

ImuMotionEstimate estimate_imu_motion(
  const std::vector<ImuSample> & samples,
  std::int64_t bag_start_timestamp_ns);

}  // namespace rosbag_sensor_trimmer

#endif  // ROSBAG_SENSOR_TRIMMER__IMU_DATA_HPP_
