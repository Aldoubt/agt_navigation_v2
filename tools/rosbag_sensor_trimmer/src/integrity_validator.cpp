#include "rosbag_sensor_trimmer/integrity_validator.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <limits>
#include <map>
#include <sstream>
#include <stdexcept>
#include <unordered_map>

#include "rosbag_sensor_trimmer/bag_index.hpp"
#include "rosbag_sensor_trimmer/bag_reader.hpp"
#include "rosbag_sensor_trimmer/topic_filter.hpp"
#include "rosbag2_storage/metadata_io.hpp"

namespace rosbag_sensor_trimmer
{

namespace
{

std::string json_escape(const std::string & value)
{
  std::ostringstream output;
  for (const char character : value) {
    switch (character) {
      case '\\': output << "\\\\"; break;
      case '"': output << "\\\""; break;
      case '\n': output << "\\n"; break;
      case '\r': output << "\\r"; break;
      case '\t': output << "\\t"; break;
      default: output << character; break;
    }
  }
  return output.str();
}

void write_string_array(std::ostream & output, const std::vector<std::string> & values)
{
  output << "[";
  for (std::size_t index = 0; index < values.size(); ++index) {
    if (index != 0) {
      output << ", ";
    }
    output << "\"" << json_escape(values[index]) << "\"";
  }
  output << "]";
}

const TopicStatistics * find_statistics(
  const BagStatistics & statistics, const std::string & name)
{
  const auto it = std::find_if(statistics.topics.begin(), statistics.topics.end(),
    [&name](const auto & topic) {return topic.metadata.name == name;});
  return it == statistics.topics.end() ? nullptr : &(*it);
}

struct SensorWindow
{
  bool has_messages{false};
  std::int64_t first_ns{std::numeric_limits<std::int64_t>::max()};
  std::int64_t last_ns{std::numeric_limits<std::int64_t>::min()};
  std::int64_t maximum_gap_ns{0};
  std::uint64_t message_count{0};
};

SensorWindow aggregate_sensor_window(const BagStatistics & statistics, TopicKind kind)
{
  SensorWindow window;
  for (const auto & topic : statistics.topics) {
    if (classify_topic(topic.metadata) != kind || !topic.has_messages) {
      continue;
    }
    window.has_messages = true;
    window.first_ns = std::min(window.first_ns, topic.first_timestamp_ns);
    window.last_ns = std::max(window.last_ns, topic.last_timestamp_ns);
    window.maximum_gap_ns = std::max(window.maximum_gap_ns, topic.maximum_gap_ns);
    window.message_count += topic.message_count;
  }
  return window;
}

void add_sensor_checks(
  IntegrityReport & report, const IntegrityOptions & options)
{
  const auto lidar = aggregate_sensor_window(report.output_statistics, TopicKind::Lidar);
  const auto imu = aggregate_sensor_window(report.output_statistics, TopicKind::Imu);
  if (!lidar.has_messages || !imu.has_messages) {
    report.warnings.push_back("输出中没有同时找到带消息的 LiDAR 和 IMU 话题，跳过重叠覆盖判断");
    return;
  }

  report.lidar_imu_overlap = lidar.first_ns <= imu.last_ns && imu.first_ns <= lidar.last_ns;
  report.imu_covers_lidar_start = imu.first_ns <= lidar.first_ns;
  if (options.require_lidar_imu_overlap && !report.lidar_imu_overlap) {
    report.errors.push_back("LiDAR 与 IMU 的消息时间区间没有重叠");
  }
  if (!report.imu_covers_lidar_start) {
    report.warnings.push_back("IMU 没有覆盖 LiDAR 的起始消息时间");
  }

  if (static_cast<double>(imu.maximum_gap_ns) / 1.0e9 > options.maximum_imu_gap_sec) {
    report.warnings.push_back("IMU 存在超过配置阈值的消息间隔");
  }
  if (static_cast<double>(lidar.maximum_gap_ns) / 1.0e9 > options.maximum_lidar_gap_sec) {
    report.warnings.push_back("LiDAR 存在超过配置阈值的消息间隔");
  }
}

void check_topic_set(
  IntegrityReport & report,
  const std::vector<rosbag2_storage::TopicMetadata> & actual_topics,
  const std::vector<rosbag2_storage::TopicMetadata> & expected_topics)
{
  std::map<std::string, rosbag2_storage::TopicMetadata> actual;
  std::map<std::string, rosbag2_storage::TopicMetadata> expected;
  for (const auto & topic : actual_topics) {
    actual[topic.name] = topic;
  }
  for (const auto & topic : expected_topics) {
    expected[topic.name] = topic;
  }

  report.topic_set_correct = actual.size() == expected.size();
  for (const auto & [name, topic] : expected) {
    const auto it = actual.find(name);
    if (it == actual.end() || it->second.type != topic.type ||
      it->second.serialization_format != topic.serialization_format) {
      report.topic_set_correct = false;
      report.errors.push_back("输出话题集合或消息类型不匹配: " + name);
    }
  }
  for (const auto & [name, unused] : actual) {
    if (expected.count(name) == 0) {
      report.topic_set_correct = false;
      report.errors.push_back("输出包含未选择的话题: " + name);
    }
    (void)unused;
  }
}

void check_common_fields(
  IntegrityReport & report,
  const std::vector<IndexEntry> & entries,
  const TimeRange * range)
{
  report.timestamps_monotonic = true;
  report.time_range_correct = true;
  bool first = true;
  std::int64_t previous = 0;
  for (const auto & entry : entries) {
    if (!first && entry.timestamp_ns < previous) {
      report.timestamps_monotonic = false;
    }
    if (range && !range->contains(entry.timestamp_ns)) {
      report.time_range_correct = false;
    }
    previous = entry.timestamp_ns;
    first = false;
  }
  if (!report.timestamps_monotonic) {
    report.errors.push_back("输出消息时间戳不是单调不下降");
  }
  if (range && !report.time_range_correct) {
    report.errors.push_back("输出存在时间范围外消息");
  }
}

}  // namespace

IntegrityReport IntegrityValidator::validate(
  const TrimJob & job,
  const std::vector<rosbag2_storage::TopicMetadata> & expected_topics,
  const std::unordered_map<std::string, std::uint64_t> & expected_counts,
  const IntegrityOptions & options)
{
  IntegrityReport report;
  try {
    const auto normalized = normalize_bag_uri(job.output_uri);
    std::error_code error;
    report.metadata_present = std::filesystem::exists(
      normalized / rosbag2_storage::MetadataIo::metadata_filename, error) && !error;
    if (!report.metadata_present) {
      report.errors.push_back("输出 bag 缺少 metadata.yaml");
    }

    BagReader reader;
    reader.open(job.output_uri, job.output_storage_id);
    const auto actual_topics = reader.topics();
    const auto index = BagIndex::build(reader);
    report.output_statistics = index.statistics();

    check_topic_set(report, actual_topics, expected_topics);
    const TimeRange expected_range{job.start_time_ns, job.end_time_ns};
    expected_range.validate();
    check_common_fields(report, index.entries(), &expected_range);
    report.storage_consistent = report.output_statistics.storage_id == job.output_storage_id;
    if (!report.storage_consistent) {
      report.errors.push_back(
        "输出 storage_id 不匹配，期望 " + job.output_storage_id + "，实际 " +
        report.output_statistics.storage_id);
    }

    for (const auto & topic : expected_topics) {
      const auto * statistics = find_statistics(report.output_statistics, topic.name);
      const auto expected = expected_counts.count(topic.name) == 0 ? 0 : expected_counts.at(topic.name);
      if (!statistics || statistics->message_count != expected) {
        report.errors.push_back(
          "话题消息数量不匹配: " + topic.name + "，期望 " + std::to_string(expected) +
          "，实际 " + std::to_string(statistics ? statistics->message_count : 0));
      }
      if (!statistics || !statistics->has_messages) {
        report.has_zero_message_topics = true;
        report.warnings.push_back("输出话题没有消息: " + topic.name);
      }
    }
    add_sensor_checks(report, options);
  } catch (const std::exception & exception) {
    report.errors.push_back(exception.what());
  }
  report.ok = report.errors.empty();
  return report;
}

IntegrityReport IntegrityValidator::validate_basic(
  const std::filesystem::path & uri, const std::string & storage_id, const IntegrityOptions & options)
{
  IntegrityReport report;
  try {
    const auto normalized = normalize_bag_uri(uri);
    std::error_code error;
    report.metadata_present = std::filesystem::exists(
      normalized / rosbag2_storage::MetadataIo::metadata_filename, error) && !error;
    if (!report.metadata_present) {
      report.errors.push_back("bag 缺少 metadata.yaml");
    }
    BagReader reader;
    reader.open(uri, storage_id);
    const auto index = BagIndex::build(reader);
    report.output_statistics = index.statistics();
    report.topic_set_correct = true;
    report.time_range_correct = true;
    check_common_fields(report, index.entries(), nullptr);
    report.storage_consistent = storage_id.empty() || index.statistics().storage_id == storage_id;
    if (!report.storage_consistent) {
      report.errors.push_back("bag storage_id 与指定值不一致");
    }
    add_sensor_checks(report, options);
  } catch (const std::exception & exception) {
    report.errors.push_back(exception.what());
  }
  report.ok = report.errors.empty();
  return report;
}

void IntegrityValidator::write_json(const std::filesystem::path & path, const IntegrityReport & report)
{
  if (!path.parent_path().empty()) {
    std::filesystem::create_directories(path.parent_path());
  }
  std::ofstream output(path);
  if (!output) {
    throw std::runtime_error("无法写入 JSON 报告: " + path.string());
  }
  output << std::boolalpha << "{\n"
         << "  \"ok\": " << report.ok << ",\n"
         << "  \"metadata_present\": " << report.metadata_present << ",\n"
         << "  \"topic_set_correct\": " << report.topic_set_correct << ",\n"
         << "  \"time_range_correct\": " << report.time_range_correct << ",\n"
         << "  \"timestamps_monotonic\": " << report.timestamps_monotonic << ",\n"
         << "  \"storage_consistent\": " << report.storage_consistent << ",\n"
         << "  \"lidar_imu_overlap\": " << report.lidar_imu_overlap << ",\n"
         << "  \"imu_covers_lidar_start\": " << report.imu_covers_lidar_start << ",\n"
         << "  \"has_zero_message_topics\": " << report.has_zero_message_topics << ",\n"
         << "  \"storage_id\": \"" << json_escape(report.output_statistics.storage_id) << "\",\n"
         << "  \"message_count\": " << report.output_statistics.message_count << ",\n"
         << "  \"file_size_bytes\": " << report.output_statistics.file_size_bytes << ",\n"
         << "  \"start_time_ns\": " << report.output_statistics.start_time_ns << ",\n"
         << "  \"end_time_ns\": " << report.output_statistics.end_time_ns << ",\n"
         << "  \"duration_ns\": " << report.output_statistics.duration_ns << ",\n"
         << "  \"errors\": ";
  write_string_array(output, report.errors);
  output << ",\n  \"warnings\": ";
  write_string_array(output, report.warnings);
  output << ",\n  \"topics\": [\n";
  for (std::size_t index = 0; index < report.output_statistics.topics.size(); ++index) {
    const auto & topic = report.output_statistics.topics[index];
    output << "    {\"name\": \"" << json_escape(topic.metadata.name) << "\", \"type\": \""
           << json_escape(topic.metadata.type) << "\", \"message_count\": "
           << topic.message_count << ", \"serialized_bytes\": " << topic.serialized_bytes
           << ", \"first_timestamp_ns\": " << topic.first_timestamp_ns
           << ", \"last_timestamp_ns\": " << topic.last_timestamp_ns
           << ", \"average_frequency_hz\": " << std::setprecision(12)
           << topic.average_frequency_hz << ", \"maximum_gap_ns\": " << topic.maximum_gap_ns
           << "}" << (index + 1 == report.output_statistics.topics.size() ? "\n" : ",\n");
  }
  output << "  ]\n}\n";
}

void IntegrityValidator::write_markdown(
  const std::filesystem::path & path, const IntegrityReport & report)
{
  if (!path.parent_path().empty()) {
    std::filesystem::create_directories(path.parent_path());
  }
  std::ofstream output(path);
  if (!output) {
    throw std::runtime_error("无法写入 Markdown 报告: " + path.string());
  }
  output << "# rosbag_sensor_trimmer 验证报告\n\n"
         << "- 结果: **" << (report.ok ? "通过" : "失败") << "**\n"
         << "- storage_id: `" << report.output_statistics.storage_id << "`\n"
         << "- 消息数: " << report.output_statistics.message_count << "\n"
         << "- 文件大小: " << report.output_statistics.file_size_bytes << " bytes\n"
         << "- 时间范围: `" << report.output_statistics.start_time_ns << " .. "
         << report.output_statistics.end_time_ns << "`\n\n"
         << "## 检查项\n\n"
         << "| 检查 | 结果 |\n| --- | --- |\n"
         << "| metadata.yaml | " << (report.metadata_present ? "通过" : "失败") << " |\n"
         << "| 话题集合 | " << (report.topic_set_correct ? "通过" : "失败") << " |\n"
         << "| 时间范围 | " << (report.time_range_correct ? "通过" : "失败") << " |\n"
         << "| 时间单调性 | " << (report.timestamps_monotonic ? "通过" : "失败") << " |\n"
         << "| storage_id | " << (report.storage_consistent ? "通过" : "失败") << " |\n"
         << "| LiDAR/IMU 重叠 | " << (report.lidar_imu_overlap ? "通过" : "未满足") << " |\n"
         << "| IMU 覆盖 LiDAR 起点 | " << (report.imu_covers_lidar_start ? "通过" : "未满足") << " |\n\n"
         << "## 话题统计\n\n"
         << "| 话题 | 类型 | 消息数 | 序列化字节 | 平均频率 (Hz) | 最大间隔 (s) |\n"
         << "| --- | --- | ---: | ---: | ---: | ---: |\n";
  output << std::fixed << std::setprecision(3);
  for (const auto & topic : report.output_statistics.topics) {
    output << "| `" << topic.metadata.name << "` | `" << topic.metadata.type << "` | "
           << topic.message_count << " | " << topic.serialized_bytes << " | "
           << topic.average_frequency_hz << " | "
           << static_cast<double>(topic.maximum_gap_ns) / 1.0e9 << " |\n";
  }
  if (!report.errors.empty()) {
    output << "\n## 错误\n\n";
    for (const auto & error : report.errors) {
      output << "- " << error << "\n";
    }
  }
  if (!report.warnings.empty()) {
    output << "\n## 警告\n\n";
    for (const auto & warning : report.warnings) {
      output << "- " << warning << "\n";
    }
  }
}

}  // namespace rosbag_sensor_trimmer
