#include <gtest/gtest.h>

#include "rosbag_sensor_trimmer/time_range.hpp"

using rosbag_sensor_trimmer::TimeRange;

TEST(TimeRange, RelativeConversionUsesNanoseconds)
{
  const auto range = rosbag_sensor_trimmer::make_relative_time_range(1000000000, 1.5, 3.0);
  EXPECT_EQ(range.start_time_ns, 2500000000);
  EXPECT_EQ(range.end_time_ns, 4000000000);
  EXPECT_TRUE(range.contains(2500000000));
  EXPECT_FALSE(range.contains(4000000000));
}

TEST(TimeRange, AbsoluteRangeUsesHalfOpenBoundary)
{
  const auto range = rosbag_sensor_trimmer::make_absolute_time_range(10, 20);
  EXPECT_TRUE(range.contains(10));
  EXPECT_TRUE(range.contains(19));
  EXPECT_FALSE(range.contains(20));
}

TEST(TimeRange, InvalidRangesThrow)
{
  EXPECT_THROW(rosbag_sensor_trimmer::make_absolute_time_range(20, 20), std::invalid_argument);
  EXPECT_THROW(rosbag_sensor_trimmer::make_relative_time_range(0, -0.1, 1.0), std::invalid_argument);
  EXPECT_THROW(rosbag_sensor_trimmer::make_relative_time_range(0, 2.0, 1.0), std::invalid_argument);
}
