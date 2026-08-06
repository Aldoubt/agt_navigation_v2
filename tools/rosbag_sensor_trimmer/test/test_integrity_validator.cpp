#include <fstream>
#include <gtest/gtest.h>

#include "rosbag_sensor_trimmer/integrity_validator.hpp"

TEST(IntegrityValidator, WritesMachineAndHumanReports)
{
  const auto root = std::filesystem::temp_directory_path() / "rosbag_sensor_trimmer_report_test";
  std::filesystem::remove_all(root);
  std::filesystem::create_directories(root);
  rosbag_sensor_trimmer::IntegrityReport report;
  report.ok = true;
  report.metadata_present = true;
  report.topic_set_correct = true;
  report.time_range_correct = true;
  report.timestamps_monotonic = true;
  report.storage_consistent = true;
  report.output_statistics.storage_id = "sqlite3";
  report.output_statistics.message_count = 2;
  report.warnings.push_back("test warning");

  const auto json = root / "trim_report.json";
  const auto markdown = root / "trim_report.md";
  rosbag_sensor_trimmer::IntegrityValidator::write_json(json, report);
  rosbag_sensor_trimmer::IntegrityValidator::write_markdown(markdown, report);
  ASSERT_TRUE(std::filesystem::exists(json));
  ASSERT_TRUE(std::filesystem::exists(markdown));
  std::ifstream json_input(json);
  const std::string json_contents(
    (std::istreambuf_iterator<char>(json_input)), std::istreambuf_iterator<char>());
  EXPECT_NE(json_contents.find("\"ok\": true"), std::string::npos);
  std::filesystem::remove_all(root);
}
