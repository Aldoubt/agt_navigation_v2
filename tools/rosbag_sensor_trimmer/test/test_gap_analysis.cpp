#include <gtest/gtest.h>

#include "rosbag_sensor_trimmer/gap_analysis.hpp"

TEST(GapAnalysis, DetectsOnlyIntervalsAboveTopicSpecificThreshold)
{
  rosbag_sensor_trimmer::BagStatistics statistics;
  rosbag2_storage::TopicMetadata metadata;
  metadata.name = "/points";
  metadata.type = "sensor_msgs/msg/PointCloud2";
  rosbag_sensor_trimmer::TopicStatistics topic;
  topic.metadata = metadata;
  topic.average_frequency_hz = 10.0;
  statistics.topics.push_back(topic);

  const std::vector<rosbag_sensor_trimmer::IndexEntry> entries{
    {0, "/points", 0, 1},
    {100000000, "/points", 1, 1},
    {900000000, "/points", 2, 1},
    {1000000000, "/points", 3, 1}};
  const auto gaps = rosbag_sensor_trimmer::detect_topic_gaps(statistics, entries);

  ASSERT_EQ(gaps.size(), 1U);
  EXPECT_EQ(gaps.front().start_timestamp_ns, 100000000);
  EXPECT_EQ(gaps.front().end_timestamp_ns, 900000000);
  EXPECT_EQ(gaps.front().duration_ns, 800000000);
  EXPECT_EQ(gaps.front().threshold_ns, 300000000);
}
