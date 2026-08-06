#include "rosbag_sensor_trimmer/trim_job.hpp"

#include <algorithm>
#include <cctype>
#include <iomanip>
#include <sstream>
#include <stdexcept>

#include "rosbag_sensor_trimmer/bag_reader.hpp"

namespace rosbag_sensor_trimmer
{

namespace
{

std::string lower(std::string value)
{
  std::transform(value.begin(), value.end(), value.begin(),
    [](unsigned char character) {return static_cast<char>(std::tolower(character));});
  return value;
}

bool same_path(const std::filesystem::path & left, const std::filesystem::path & right)
{
  std::error_code error;
  if (std::filesystem::exists(left, error) && std::filesystem::exists(right, error)) {
    return std::filesystem::equivalent(left, right, error) && !error;
  }
  return std::filesystem::absolute(left).lexically_normal() ==
         std::filesystem::absolute(right).lexically_normal();
}

}  // namespace

void validate_trim_job(const TrimJob & job)
{
  if (job.input_uri.empty()) {
    throw std::invalid_argument("输入 bag 路径不能为空");
  }
  if (job.output_uri.empty()) {
    throw std::invalid_argument("输出 bag 路径不能为空");
  }
  if (!std::filesystem::exists(job.input_uri)) {
    throw std::invalid_argument("输入 bag 路径不存在: " + job.input_uri.string());
  }
  if (same_path(normalize_bag_uri(job.input_uri), job.output_uri)) {
    throw std::invalid_argument("禁止把输出目录设置为输入 bag 本身");
  }
  if (std::filesystem::exists(job.output_uri) && !job.overwrite_output) {
    throw std::invalid_argument(
            "输出目录已存在: " + job.output_uri.string() + "，请更换路径或显式使用 --overwrite");
  }
  if (job.output_storage_id.empty()) {
    throw std::invalid_argument("输出 storage_id 不能为空");
  }
  const auto storage_id = lower(job.output_storage_id);
  if (storage_id != "sqlite3" && storage_id != "mcap") {
    throw std::invalid_argument("输出 storage_id 只支持 sqlite3 或 mcap: " + job.output_storage_id);
  }
  TimeRange{job.start_time_ns, job.end_time_ns}.validate();

  if (job.enable_compression) {
    const auto mode = lower(job.compression_mode);
    const auto format = lower(job.compression_format);
    if (mode != "file" && mode != "message") {
      throw std::invalid_argument("压缩模式只支持 file 或 message: " + job.compression_mode);
    }
    if (format != "zstd") {
      throw std::invalid_argument("当前构建只启用 zstd 压缩: " + job.compression_format);
    }
  }
}

std::filesystem::path default_output_uri(
  const std::filesystem::path & input_uri, double start_seconds, double end_seconds)
{
  const auto normalized = input_uri.filename() == "metadata.yaml" ? input_uri.parent_path() : input_uri;
  std::ostringstream suffix;
  suffix << std::fixed << std::setprecision(1)
         << "_trimmed_" << start_seconds << "_" << end_seconds;
  return normalized.parent_path() / (normalized.filename().string() + suffix.str());
}

}  // namespace rosbag_sensor_trimmer
