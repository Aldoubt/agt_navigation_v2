#include <gtest/gtest.h>

#include <cmath>
#include <vector>

#include "rosbag_sensor_trimmer/imu_data.hpp"

TEST(ImuAnalysis, DetectsMotionAfterInitialStationaryWindow)
{
  std::vector<rosbag_sensor_trimmer::ImuSample> samples;
  for (int index = 0; index < 800; ++index) {
    const auto timestamp = static_cast<std::int64_t>(index) * 5'000'000LL;
    const bool moving = index >= 500;
    samples.push_back({timestamp, moving ? 10.5 : 9.81, moving ? 0.35 : 0.01});
  }

  const auto estimate = rosbag_sensor_trimmer::estimate_imu_motion(samples, 0);

  ASSERT_TRUE(estimate.valid);
  EXPECT_NEAR(estimate.relative_start_seconds, 2.5, 0.1);
  EXPECT_GT(estimate.acceleration_threshold, 0.0);
  EXPECT_GT(estimate.angular_velocity_threshold, 0.0);
}

TEST(ImuAnalysis, DoesNotInventStartWithoutMotion)
{
  std::vector<rosbag_sensor_trimmer::ImuSample> samples;
  for (int index = 0; index < 800; ++index) {
    samples.push_back({static_cast<std::int64_t>(index) * 5'000'000LL, 9.81, 0.01});
  }

  const auto estimate = rosbag_sensor_trimmer::estimate_imu_motion(samples, 0);

  EXPECT_FALSE(estimate.valid);
}
