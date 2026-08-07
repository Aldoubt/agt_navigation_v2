#include <gtest/gtest.h>

#include <stdexcept>

#include "rosbag_sensor_trimmer/playback_support.hpp"

TEST(PlaybackSupport, ExtractsMessagePackage)
{
  EXPECT_EQ(
    rosbag_sensor_trimmer::message_package_name("livox_ros_driver2/msg/CustomMsg"),
    "livox_ros_driver2");
  EXPECT_THROW(
    rosbag_sensor_trimmer::message_package_name("CustomMsg"), std::invalid_argument);
}

TEST(PlaybackSupport, DetectsInstalledAndMissingTypeSupport)
{
  const auto installed = rosbag_sensor_trimmer::check_playback_type_support(
    "sensor_msgs/msg/Imu");
  EXPECT_TRUE(installed.available) << installed.error;

  const auto missing = rosbag_sensor_trimmer::check_playback_type_support(
    "rosbag_sensor_trimmer_missing_msgs/msg/DefinitelyMissing");
  EXPECT_FALSE(missing.available);
  EXPECT_EQ(missing.package_name, "rosbag_sensor_trimmer_missing_msgs");
  EXPECT_FALSE(missing.error.empty());
}
