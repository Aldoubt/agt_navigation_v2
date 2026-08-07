#include <fstream>
#include <gtest/gtest.h>

#include "rosbag_sensor_trimmer/bag_metadata.hpp"

TEST(BagMetadata, FileSizeCountsBagFilesAndSkipsCompressedTemporaryDatabase)
{
  const auto root = std::filesystem::temp_directory_path() / "rosbag_sensor_trimmer_size_test";
  std::filesystem::remove_all(root);
  std::filesystem::create_directories(root);
  {
    std::ofstream(root / "metadata.yaml") << "metadata";
    std::ofstream(root / "bag_0.db3.zstd") << "compressed";
    std::ofstream(root / "bag_0.db3") << "temporary";
    std::ofstream(root / "notes.txt") << "ignored";
  }
  EXPECT_EQ(
    rosbag_sensor_trimmer::directory_size_bytes(root),
    static_cast<std::uint64_t>(8 + 10));
  std::filesystem::remove_all(root);
}
