#ifndef ROSBAG_SENSOR_TRIMMER__BAG_READER_HPP_
#define ROSBAG_SENSOR_TRIMMER__BAG_READER_HPP_

#include <filesystem>
#include <memory>
#include <string>
#include <vector>

#include "rosbag2_cpp/reader.hpp"
#include "rosbag2_storage/bag_metadata.hpp"
#include "rosbag2_storage/serialized_bag_message.hpp"
#include "rosbag2_storage/topic_metadata.hpp"

namespace rosbag_sensor_trimmer
{

std::filesystem::path normalize_bag_uri(const std::filesystem::path & input);

class BagReader
{
public:
  BagReader() = default;
  ~BagReader();

  BagReader(const BagReader &) = delete;
  BagReader & operator=(const BagReader &) = delete;

  void open(const std::filesystem::path & input, const std::string & storage_id = "");
  void close();

  bool is_open() const noexcept;
  bool has_next();
  std::shared_ptr<rosbag2_storage::SerializedBagMessage> read_next();
  void seek(std::int64_t timestamp_ns);

  const std::filesystem::path & uri() const noexcept;
  const std::string & storage_id() const noexcept;
  const rosbag2_storage::BagMetadata & metadata() const;
  const std::vector<rosbag2_storage::TopicMetadata> & topics() const;

private:
  std::filesystem::path uri_;
  std::string storage_id_;
  rosbag2_storage::BagMetadata metadata_;
  std::vector<rosbag2_storage::TopicMetadata> topics_;
  std::vector<std::filesystem::path> temporary_decompressed_files_;
  std::unique_ptr<rosbag2_cpp::Reader> reader_;
};

}  // namespace rosbag_sensor_trimmer

#endif  // ROSBAG_SENSOR_TRIMMER__BAG_READER_HPP_
