#include "rosbag_sensor_trimmer/bag_writer.hpp"

#include <stdexcept>

#include "rosbag2_compression/sequential_compression_writer.hpp"
#include "rosbag2_cpp/converter_options.hpp"
#include "rosbag2_cpp/writers/sequential_writer.hpp"
#include "rosbag2_storage/storage_options.hpp"

namespace rosbag_sensor_trimmer
{

BagWriter::~BagWriter()
{
  try {
    close();
  } catch (const std::exception &) {
    writer_.reset();
    opened_ = false;
  }
}

void BagWriter::open(const TrimJob & job)
{
  close();
  std::unique_ptr<rosbag2_cpp::writer_interfaces::BaseWriterInterface> implementation;
  if (job.enable_compression) {
    rosbag2_compression::CompressionOptions compression_options;
    compression_options.compression_format = job.compression_format;
    compression_options.compression_mode =
      rosbag2_compression::compression_mode_from_string(job.compression_mode);
    compression_options.compression_threads = 1;
    implementation = std::make_unique<rosbag2_compression::SequentialCompressionWriter>(
      compression_options);
  } else {
    implementation = std::make_unique<rosbag2_cpp::writers::SequentialWriter>();
  }

  writer_ = std::make_unique<rosbag2_cpp::Writer>(std::move(implementation));
  rosbag2_storage::StorageOptions storage_options;
  storage_options.uri = job.output_uri.string();
  storage_options.storage_id = job.output_storage_id;
  try {
    writer_->open(storage_options, rosbag2_cpp::ConverterOptions());
    opened_ = true;
  } catch (const std::exception & exception) {
    writer_.reset();
    throw std::runtime_error(
            "无法创建输出 bag: " + job.output_uri.string() + "，storage_id=" +
            job.output_storage_id + ". " + exception.what() +
            ". 请确认输出 storage plugin 已安装且目录不存在。");
  }
}

void BagWriter::create_topic(const rosbag2_storage::TopicMetadata & topic)
{
  if (!writer_ || !opened_) {
    throw std::runtime_error("bag writer 尚未打开");
  }
  writer_->create_topic(topic);
}

void BagWriter::write(std::shared_ptr<rosbag2_storage::SerializedBagMessage> message)
{
  if (!writer_ || !opened_) {
    throw std::runtime_error("bag writer 尚未打开");
  }
  writer_->write(std::move(message));
}

void BagWriter::close()
{
  if (writer_ && opened_) {
    try {
      writer_->close();
    } catch (const std::exception &) {
      writer_.reset();
      opened_ = false;
      throw;
    }
  }
  writer_.reset();
  opened_ = false;
}

}  // namespace rosbag_sensor_trimmer
