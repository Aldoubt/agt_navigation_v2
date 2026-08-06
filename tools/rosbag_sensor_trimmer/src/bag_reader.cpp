#include "rosbag_sensor_trimmer/bag_reader.hpp"

#include <stdexcept>

#include "rosbag2_compression/sequential_compression_reader.hpp"
#include "rosbag2_cpp/converter_options.hpp"
#include "rosbag2_cpp/readers/sequential_reader.hpp"
#include "rosbag2_storage/metadata_io.hpp"
#include "rosbag2_storage/storage_options.hpp"

namespace rosbag_sensor_trimmer
{

namespace
{

std::string describe_exception(const std::filesystem::path & uri, const std::exception & exception)
{
  return "无法打开 bag: " + uri.string() + ". " + exception.what() +
         ". 请检查 metadata.yaml、数据库/MCAP 文件、storage plugin 和消息存储格式。";
}

bool compressed_metadata(const rosbag2_storage::BagMetadata & metadata)
{
  return !metadata.compression_mode.empty() && metadata.compression_mode != "none" &&
         metadata.compression_mode != "NONE";
}

}  // namespace

std::filesystem::path normalize_bag_uri(const std::filesystem::path & input)
{
  std::error_code error;
  if (!std::filesystem::exists(input, error)) {
    throw std::runtime_error("bag 路径不存在: " + input.string());
  }

  if (std::filesystem::is_directory(input, error)) {
    return input;
  }

  if (std::filesystem::is_regular_file(input, error) && input.filename() == "metadata.yaml") {
    return input.parent_path();
  }

  throw std::runtime_error(
          "输入必须是 rosbag2 目录或 metadata.yaml: " + input.string());
}

BagReader::~BagReader()
{
  try {
    close();
  } catch (const std::exception &) {
    reader_.reset();
  }
}

void BagReader::open(const std::filesystem::path & input, const std::string & requested_storage_id)
{
  close();
  uri_ = normalize_bag_uri(input);

  rosbag2_storage::MetadataIo metadata_io;
  if (!metadata_io.metadata_file_exists(uri_.string())) {
    throw std::runtime_error(
            "metadata.yaml 缺失: " + uri_.string() +
            ". 如果数据库仍然存在，请先使用 ros2 bag reindex 恢复索引。");
  }

  try {
    metadata_ = metadata_io.read_metadata(uri_.string());
    storage_id_ = requested_storage_id.empty() ? metadata_.storage_identifier : requested_storage_id;
    if (storage_id_.empty()) {
      throw std::runtime_error("metadata.yaml 中没有 storage_identifier，且未指定 storage_id");
    }

    const bool file_compressed = metadata_.compression_mode == "file" ||
      metadata_.compression_mode == "FILE";
    if (file_compressed) {
      for (const auto & relative_file : metadata_.relative_file_paths) {
        const std::filesystem::path compressed_path = uri_ / relative_file;
        if (compressed_path.extension() == ".zstd") {
          auto decompressed_path = compressed_path;
          decompressed_path.replace_extension();
          std::error_code error;
          if (!std::filesystem::exists(decompressed_path, error) || error) {
            temporary_decompressed_files_.push_back(decompressed_path);
          }
        }
      }
    }

    std::unique_ptr<rosbag2_cpp::reader_interfaces::BaseReaderInterface> implementation;
    if (compressed_metadata(metadata_)) {
      implementation = std::make_unique<rosbag2_compression::SequentialCompressionReader>();
    } else {
      implementation = std::make_unique<rosbag2_cpp::readers::SequentialReader>();
    }
    reader_ = std::make_unique<rosbag2_cpp::Reader>(std::move(implementation));

    rosbag2_storage::StorageOptions storage_options;
    storage_options.uri = uri_.string();
    storage_options.storage_id = storage_id_;
    reader_->open(storage_options, rosbag2_cpp::ConverterOptions());
    metadata_ = reader_->get_metadata();
    topics_ = reader_->get_all_topics_and_types();
  } catch (const std::exception & exception) {
    reader_.reset();
    for (const auto & temporary_file : temporary_decompressed_files_) {
      std::error_code error;
      std::filesystem::remove(temporary_file, error);
    }
    temporary_decompressed_files_.clear();
    throw std::runtime_error(describe_exception(uri_, exception));
  }
}

void BagReader::close()
{
  if (reader_) {
    try {
      reader_->close();
    } catch (const std::exception &) {
      reader_.reset();
      for (const auto & temporary_file : temporary_decompressed_files_) {
        std::error_code error;
        std::filesystem::remove(temporary_file, error);
      }
      temporary_decompressed_files_.clear();
      throw;
    }
  }
  reader_.reset();
  for (const auto & temporary_file : temporary_decompressed_files_) {
    std::error_code error;
    std::filesystem::remove(temporary_file, error);
  }
  temporary_decompressed_files_.clear();
  topics_.clear();
  storage_id_.clear();
}

bool BagReader::is_open() const noexcept
{
  return static_cast<bool>(reader_);
}

bool BagReader::has_next()
{
  if (!reader_) {
    throw std::runtime_error("bag reader 尚未打开");
  }
  return reader_->has_next();
}

std::shared_ptr<rosbag2_storage::SerializedBagMessage> BagReader::read_next()
{
  if (!reader_) {
    throw std::runtime_error("bag reader 尚未打开");
  }
  return reader_->read_next();
}

void BagReader::seek(std::int64_t timestamp_ns)
{
  if (!reader_) {
    throw std::runtime_error("bag reader 尚未打开");
  }
  reader_->seek(timestamp_ns);
}

const std::filesystem::path & BagReader::uri() const noexcept
{
  return uri_;
}

const std::string & BagReader::storage_id() const noexcept
{
  return storage_id_;
}

const rosbag2_storage::BagMetadata & BagReader::metadata() const
{
  if (!reader_) {
    throw std::runtime_error("bag reader 尚未打开");
  }
  return metadata_;
}

const std::vector<rosbag2_storage::TopicMetadata> & BagReader::topics() const
{
  if (!reader_) {
    throw std::runtime_error("bag reader 尚未打开");
  }
  return topics_;
}

}  // namespace rosbag_sensor_trimmer
