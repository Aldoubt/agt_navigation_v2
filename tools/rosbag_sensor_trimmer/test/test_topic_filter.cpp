#include <gtest/gtest.h>

#include "rosbag_sensor_trimmer/topic_filter.hpp"

namespace
{

rosbag2_storage::TopicMetadata topic(const std::string & name, const std::string & type)
{
  rosbag2_storage::TopicMetadata metadata;
  metadata.name = name;
  metadata.type = type;
  metadata.serialization_format = "cdr";
  return metadata;
}

}  // namespace

TEST(TopicFilter, ClassifiesByMessageType)
{
  EXPECT_EQ(
    rosbag_sensor_trimmer::classify_topic(topic("/points", "sensor_msgs/msg/PointCloud2")),
    rosbag_sensor_trimmer::TopicKind::Lidar);
  EXPECT_EQ(
    rosbag_sensor_trimmer::classify_topic(topic("/imu", "sensor_msgs/msg/Imu")),
    rosbag_sensor_trimmer::TopicKind::Imu);
  EXPECT_EQ(
    rosbag_sensor_trimmer::classify_topic(topic("/custom", "livox_ros_driver2/msg/CustomMsg")),
    rosbag_sensor_trimmer::TopicKind::Lidar);
}

TEST(TopicFilter, IncludeAndExcludeAreIndependent)
{
  const auto lidar = topic("/points", "sensor_msgs/msg/PointCloud2");
  const auto imu = topic("/imu", "sensor_msgs/msg/Imu");
  const auto tf = topic("/tf_static", "tf2_msgs/msg/TFMessage");
  rosbag_sensor_trimmer::TopicFilter filter;
  filter.set_include_topics({"/points", "/imu"});
  filter.set_exclude_topics({"/imu"});
  EXPECT_TRUE(filter.matches(lidar));
  EXPECT_FALSE(filter.matches(imu));
  EXPECT_FALSE(filter.matches(tf));
}

TEST(TopicFilter, RecommendedTopicsUseTypesAndStaticTfName)
{
  const auto selected = rosbag_sensor_trimmer::recommended_topics({
      topic("/scan", "sensor_msgs/msg/PointCloud2"),
      topic("/imu_raw", "sensor_msgs/msg/Imu"),
      topic("/tf_static", "tf2_msgs/msg/TFMessage"),
      topic("/other", "std_msgs/msg/String")});
  ASSERT_EQ(selected.size(), 3U);
  EXPECT_EQ(selected[0].name, "/scan");
  EXPECT_EQ(selected[1].name, "/imu_raw");
  EXPECT_EQ(selected[2].name, "/tf_static");
}
