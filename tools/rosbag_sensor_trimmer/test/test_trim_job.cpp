#include <gtest/gtest.h>

#include <fstream>

#include "rosbag_sensor_trimmer/time_range.hpp"
#include "rosbag_sensor_trimmer/trim_job.hpp"

TEST(TrimJob, DefaultOutputNameContainsRange)
{
  const auto output = rosbag_sensor_trimmer::default_output_uri(
    "/tmp/greenhouse_01", 35.0, 185.0);
  EXPECT_EQ(output.filename().string(), "greenhouse_01_trimmed_35.0_185.0");
}

TEST(TrimJob, BoundaryRuleIsExplicit)
{
  rosbag_sensor_trimmer::TrimJob job;
  job.start_time_ns = 100;
  job.end_time_ns = 200;
  const rosbag_sensor_trimmer::TimeRange range{job.start_time_ns, job.end_time_ns};
  EXPECT_TRUE(range.contains(100));
  EXPECT_FALSE(range.contains(200));
}

TEST(TrimJob, ExistingOutputRequiresExplicitOverwrite)
{
  const auto root = std::filesystem::temp_directory_path() / "rosbag_sensor_trimmer_job_test";
  std::filesystem::remove_all(root);
  std::filesystem::create_directories(root / "input");
  std::ofstream(root / "input" / "metadata.yaml") << "placeholder";
  std::filesystem::create_directories(root / "output");

  rosbag_sensor_trimmer::TrimJob job;
  job.input_uri = root / "input";
  job.output_uri = root / "output";
  job.output_storage_id = "sqlite3";
  job.start_time_ns = 10;
  job.end_time_ns = 20;
  EXPECT_THROW(rosbag_sensor_trimmer::validate_trim_job(job), std::invalid_argument);

  job.overwrite_output = true;
  EXPECT_NO_THROW(rosbag_sensor_trimmer::validate_trim_job(job));
  std::filesystem::remove_all(root);
}
