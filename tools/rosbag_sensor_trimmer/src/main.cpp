#include <algorithm>
#include <chrono>
#include <cstdlib>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include "rosbag_sensor_trimmer/bag_index.hpp"
#include "rosbag_sensor_trimmer/bag_reader.hpp"
#include "rosbag_sensor_trimmer/integrity_validator.hpp"
#include "rosbag_sensor_trimmer/ros1_livox_converter.hpp"
#include "rosbag_sensor_trimmer/time_range.hpp"
#include "rosbag_sensor_trimmer/topic_filter.hpp"
#include "rosbag_sensor_trimmer/trim_job.hpp"
#include "rosbag_sensor_trimmer/trim_worker.hpp"

namespace rst = rosbag_sensor_trimmer;

namespace
{

struct Arguments
{
  std::filesystem::path input;
  std::filesystem::path output;
  std::string input_storage;
  std::string output_storage;
  std::string report;
  std::vector<std::string> topics;
  std::vector<std::string> exclude_topics;
  double start_seconds{0.0};
  double end_seconds{0.0};
  std::int64_t start_ns{0};
  std::int64_t end_ns{0};
  bool has_start_seconds{false};
  bool has_end_seconds{false};
  bool has_start_ns{false};
  bool has_end_ns{false};
  bool compression{false};
  std::string compression_mode{"file"};
  std::string compression_format{"zstd"};
  bool overwrite{false};
  bool verify{false};
  bool verify_only{false};
  bool dry_run{false};
  bool info{false};
  bool convert_ros1_livox{false};
  std::int64_t max_lidar_messages{0};
  std::string input_lidar_topic;
  std::string input_imu_topic;
  std::string output_lidar_topic;
  std::string output_imu_topic;
};

void print_help()
{
  std::cout << R"HELP(rosbag_sensor_trimmer_cli - rosbag2 LiDAR/IMU 统计、裁剪与验证工具

用法:
  ros2 run rosbag_sensor_trimmer rosbag_sensor_trimmer_cli --input BAG [选项]

信息:
  --input PATH                 rosbag2 目录或 metadata.yaml
  --input-storage ID           覆盖 metadata 中的输入 storage_id
  --info                       读取并打印 bag metadata、话题统计和频率

裁剪:
  --output PATH                输出 bag 目录；省略时只做信息读取
  --start SEC --end SEC        相对 bag 起始记录时间，规则为 [start, end)
  --start-ns NS --end-ns NS    绝对记录时间戳，不能与 --start/--end 混用
  --topics TOPIC ...           话题白名单，直到下一个 -- 选项
  --exclude-topics TOPIC ...  话题黑名单，黑名单优先
  --output-storage ID          sqlite3 或 mcap，默认跟随输入
  --compression FORMAT         none 或 zstd，启用 rosbag2 压缩
  --compression-mode MODE      file 或 message，默认 file
  --overwrite                  允许删除并重新创建已有输出目录
  --dry-run                    扫描并估算裁剪结果，不写输出

验证:
  --verify                     裁剪后重新打开输出并写 JSON/Markdown 报告
  --verify-only                只验证 --input，不执行裁剪
  --report PATH                JSON 报告路径；Markdown 使用同名 .md

ROS1 Livox 转换:
  --convert-ros1-livox         将 ROS1 Livox 分片 bag 转成 ROS2 bag
  --max-lidar-messages N       只转换前 N 个 LiDAR 帧，0 表示全部
  --input-lidar-topic TOPIC    默认 /livox/lidar
  --input-imu-topic TOPIC      默认 /livox/imu
  --output-lidar-topic TOPIC   默认 /agt/sensors/lidar/custom
  --output-imu-topic TOPIC     默认 /agt/sensors/imu/data

其他:
  --help                       显示帮助
  --version                    显示版本
  --topics /lidar /imu 示例:
    ros2 run rosbag_sensor_trimmer rosbag_sensor_trimmer_cli \
      --input /data/original_bag --output /data/trimmed \
      --start 5 --end 30 --topics /lidar /imu --output-storage sqlite3 --verify
)HELP";
}

std::string require_value(int & index, int argc, char ** argv, const std::string & option)
{
  if (index + 1 >= argc) {
    throw std::invalid_argument("选项缺少参数: " + option);
  }
  ++index;
  return argv[index];
}

std::vector<std::string> require_list(int & index, int argc, char ** argv, const std::string & option)
{
  std::vector<std::string> values;
  while (index + 1 < argc && std::string(argv[index + 1]).rfind("--", 0) != 0) {
    values.emplace_back(argv[++index]);
  }
  if (values.empty()) {
    throw std::invalid_argument("选项至少需要一个值: " + option);
  }
  return values;
}

double parse_double(const std::string & value, const std::string & option)
{
  std::size_t consumed = 0;
  const auto parsed = std::stod(value, &consumed);
  if (consumed != value.size()) {
    throw std::invalid_argument("无法解析 " + option + ": " + value);
  }
  return parsed;
}

std::int64_t parse_int64(const std::string & value, const std::string & option)
{
  std::size_t consumed = 0;
  const auto parsed = std::stoll(value, &consumed);
  if (consumed != value.size()) {
    throw std::invalid_argument("无法解析 " + option + ": " + value);
  }
  return parsed;
}

Arguments parse_arguments(int argc, char ** argv)
{
  Arguments arguments;
  for (int index = 1; index < argc; ++index) {
    const std::string option = argv[index];
    if (option == "--help" || option == "-h") {
      print_help();
      std::exit(0);
    } else if (option == "--version") {
      std::cout << "rosbag_sensor_trimmer 0.1.0 (P0/P1)\n";
      std::exit(0);
    } else if (option == "--input") {
      arguments.input = require_value(index, argc, argv, option);
    } else if (option == "--output") {
      arguments.output = require_value(index, argc, argv, option);
    } else if (option == "--input-storage") {
      arguments.input_storage = require_value(index, argc, argv, option);
    } else if (option == "--output-storage") {
      arguments.output_storage = require_value(index, argc, argv, option);
    } else if (option == "--topics") {
      arguments.topics = require_list(index, argc, argv, option);
    } else if (option == "--exclude-topics") {
      arguments.exclude_topics = require_list(index, argc, argv, option);
    } else if (option == "--start") {
      arguments.start_seconds = parse_double(require_value(index, argc, argv, option), option);
      arguments.has_start_seconds = true;
    } else if (option == "--end") {
      arguments.end_seconds = parse_double(require_value(index, argc, argv, option), option);
      arguments.has_end_seconds = true;
    } else if (option == "--start-ns") {
      arguments.start_ns = parse_int64(require_value(index, argc, argv, option), option);
      arguments.has_start_ns = true;
    } else if (option == "--end-ns") {
      arguments.end_ns = parse_int64(require_value(index, argc, argv, option), option);
      arguments.has_end_ns = true;
    } else if (option == "--compression") {
      const auto format = require_value(index, argc, argv, option);
      if (format == "none" || format == "NONE") {
        arguments.compression = false;
      } else {
        arguments.compression = true;
        arguments.compression_format = format;
      }
    } else if (option == "--compression-mode") {
      arguments.compression_mode = require_value(index, argc, argv, option);
    } else if (option == "--overwrite") {
      arguments.overwrite = true;
    } else if (option == "--verify") {
      arguments.verify = true;
    } else if (option == "--verify-only") {
      arguments.verify_only = true;
    } else if (option == "--dry-run") {
      arguments.dry_run = true;
    } else if (option == "--info") {
      arguments.info = true;
    } else if (option == "--convert-ros1-livox") {
      arguments.convert_ros1_livox = true;
    } else if (option == "--max-lidar-messages") {
      arguments.max_lidar_messages = parse_int64(
        require_value(index, argc, argv, option), option);
      if (arguments.max_lidar_messages < 0) {
        throw std::invalid_argument("--max-lidar-messages 不能为负数");
      }
    } else if (option == "--input-lidar-topic") {
      arguments.input_lidar_topic = require_value(index, argc, argv, option);
    } else if (option == "--input-imu-topic") {
      arguments.input_imu_topic = require_value(index, argc, argv, option);
    } else if (option == "--output-lidar-topic") {
      arguments.output_lidar_topic = require_value(index, argc, argv, option);
    } else if (option == "--output-imu-topic") {
      arguments.output_imu_topic = require_value(index, argc, argv, option);
    } else if (option == "--report") {
      arguments.report = require_value(index, argc, argv, option);
    } else {
      throw std::invalid_argument("未知选项: " + option + "，使用 --help 查看用法");
    }
  }
  if (arguments.input.empty()) {
    throw std::invalid_argument("必须指定 --input");
  }
  if (arguments.has_start_seconds != arguments.has_end_seconds) {
    throw std::invalid_argument("--start 和 --end 必须同时指定");
  }
  if (arguments.has_start_ns != arguments.has_end_ns) {
    throw std::invalid_argument("--start-ns 和 --end-ns 必须同时指定");
  }
  if ((arguments.has_start_seconds || arguments.has_end_seconds) &&
    (arguments.has_start_ns || arguments.has_end_ns)) {
    throw std::invalid_argument("--start/--end 与 --start-ns/--end-ns 不能混用");
  }
  if (arguments.convert_ros1_livox && (arguments.info || arguments.verify_only)) {
    throw std::invalid_argument("--convert-ros1-livox 不能与 --info 或 --verify-only 混用");
  }
  if (arguments.convert_ros1_livox && !arguments.dry_run && arguments.output.empty()) {
    throw std::invalid_argument("ROS1 转换必须指定 --output，除非使用 --dry-run");
  }
  if (arguments.verify_only && !arguments.output.empty()) {
    throw std::invalid_argument("--verify-only 不能同时使用 --output");
  }
  return arguments;
}

std::filesystem::path markdown_report_path(const std::filesystem::path & json_path)
{
  auto path = json_path;
  path.replace_extension(".md");
  return path;
}

void print_statistics(const rst::BagStatistics & statistics, const std::filesystem::path & uri)
{
  std::cout << "bag 路径: " << uri << "\n"
            << "storage_id: " << statistics.storage_id << "\n"
            << "压缩: " << (statistics.compression_mode.empty() ? "none" : statistics.compression_mode)
            << (statistics.compression_format.empty() ? "" : " / " + statistics.compression_format) << "\n"
            << "开始时间(ns): " << statistics.start_time_ns << "\n"
            << "结束时间(ns): " << statistics.end_time_ns << "\n"
            << "总时长: " << rst::format_duration_seconds(statistics.duration_ns) << "\n"
            << "消息总数: " << statistics.message_count << "\n"
            << "文件总大小: " << statistics.file_size_bytes << " bytes\n"
            << "话题数量: " << statistics.topics.size() << "\n\n"
            << "话题统计:\n";
  std::cout << std::left << std::setw(34) << "话题" << std::setw(43) << "类型"
            << std::right << std::setw(12) << "消息数" << std::setw(16) << "估算字节"
            << std::setw(14) << "平均 Hz" << "  分类\n";
  for (const auto & topic : statistics.topics) {
    std::cout << std::left << std::setw(34) << topic.metadata.name
              << std::setw(43) << topic.metadata.type << std::right
              << std::setw(12) << topic.message_count << std::setw(16) << topic.serialized_bytes
              << std::setw(14) << std::fixed << std::setprecision(3) << topic.average_frequency_hz
              << "  " << rst::topic_kind_to_string(rst::classify_topic(topic.metadata)) << "\n";
  }
  const auto recommended = rst::recommended_topics(
    [&statistics]() {
      std::vector<rosbag2_storage::TopicMetadata> topics;
      for (const auto & topic : statistics.topics) {
        topics.push_back(topic.metadata);
      }
      return topics;
    }());
  std::cout << "\n推荐保留话题: ";
  if (recommended.empty()) {
    std::cout << "无";
  } else {
    for (std::size_t index = 0; index < recommended.size(); ++index) {
      if (index != 0) {
        std::cout << ", ";
      }
      std::cout << recommended[index].name;
    }
  }
  std::cout << "\n";
}

std::filesystem::path choose_report_path(
  const Arguments & arguments, const std::filesystem::path & output_uri)
{
  if (!arguments.report.empty()) {
    return arguments.report;
  }
  return output_uri / "trim_report.json";
}

int run_info(const Arguments & arguments)
{
  rst::BagReader reader;
  reader.open(arguments.input, arguments.input_storage);
  auto index = rst::BagIndex::build(reader);
  print_statistics(index.statistics(), reader.uri());
  return 0;
}

int run_verify_only(const Arguments & arguments)
{
  const auto report = rst::IntegrityValidator::validate_basic(arguments.input, arguments.input_storage);
  print_statistics(report.output_statistics, rst::normalize_bag_uri(arguments.input));
  if (!arguments.report.empty()) {
    rst::IntegrityValidator::write_json(arguments.report, report);
    rst::IntegrityValidator::write_markdown(markdown_report_path(arguments.report), report);
    std::cout << "报告: " << arguments.report << " 和 "
              << markdown_report_path(arguments.report) << "\n";
  }
  for (const auto & warning : report.warnings) {
    std::cout << "警告: " << warning << "\n";
  }
  for (const auto & error : report.errors) {
    std::cerr << "验证错误: " << error << "\n";
  }
  return report.ok ? 0 : 3;
}

int run_convert_ros1_livox(const Arguments & arguments)
{
  rst::Ros1LivoxConversionOptions options;
  options.inputs.push_back(arguments.input);
  options.output_uri = arguments.output;
  options.output_storage_id = arguments.output_storage.empty() ?
    "sqlite3" : arguments.output_storage;
  options.overwrite_output = arguments.overwrite;
  options.max_lidar_messages = static_cast<std::uint64_t>(arguments.max_lidar_messages);
  if (!arguments.input_lidar_topic.empty()) {
    options.input_lidar_topic = arguments.input_lidar_topic;
  }
  if (!arguments.input_imu_topic.empty()) {
    options.input_imu_topic = arguments.input_imu_topic;
  }
  if (!arguments.output_lidar_topic.empty()) {
    options.output_lidar_topic = arguments.output_lidar_topic;
  }
  if (!arguments.output_imu_topic.empty()) {
    options.output_imu_topic = arguments.output_imu_topic;
  }

  if (arguments.dry_run) {
    const auto stats = rst::scan_ros1_livox_bag(options);
    std::cout << rst::format_ros1_livox_conversion_summary(stats, false);
    return 0;
  }

  const auto stats = rst::convert_ros1_livox_bag_to_ros2(
    options,
    [](const rst::Ros1LivoxConversionProgress & progress) {
      std::cout << "\r读取 " << progress.read_messages
                << "，写入 LiDAR " << progress.written_lidar
                << "，写入 IMU " << progress.written_imu
                << "，跳过 " << progress.skipped_messages << std::flush;
    });
  std::cout << "\n" << rst::format_ros1_livox_conversion_summary(stats, true);
  return 0;
}

int run_trim(const Arguments & arguments)
{
  rst::BagReader input_reader;
  input_reader.open(arguments.input, arguments.input_storage);
  const auto input_index = rst::BagIndex::build(input_reader);
  const auto & input_statistics = input_index.statistics();

  rst::TimeRange range;
  double start_seconds = 0.0;
  double end_seconds = 0.0;
  if (arguments.has_start_ns) {
    range = rst::make_absolute_time_range(arguments.start_ns, arguments.end_ns);
  } else if (arguments.has_start_seconds) {
    start_seconds = arguments.start_seconds;
    end_seconds = arguments.end_seconds;
    range = rst::make_relative_time_range(input_statistics.start_time_ns, start_seconds, end_seconds);
  } else {
    throw std::invalid_argument("裁剪必须指定 --start/--end 或 --start-ns/--end-ns");
  }

  rst::TopicFilter filter;
  filter.set_include_topics(arguments.topics);
  filter.set_exclude_topics(arguments.exclude_topics);
  const auto selected_topics = filter.select(input_reader.topics());
  if (arguments.dry_run) {
    std::uint64_t count = 0;
    std::uint64_t bytes = 0;
    for (const auto & entry : input_index.entries()) {
      if (filter.matches(entry.topic_name) && range.contains(entry.timestamp_ns)) {
        ++count;
        bytes += entry.serialized_size;
      }
    }
    std::cout << "dry-run\n裁剪时间范围: " << rst::time_range_to_string(range)
              << "\n选中话题数: " << selected_topics.size()
              << "\n消息数量估算: " << count
              << "\n序列化数据大小估算: " << bytes << " bytes\n";
    return 0;
  }

  rst::TrimJob job;
  job.input_uri = arguments.input;
  job.output_uri = arguments.output.empty() ?
    rst::default_output_uri(arguments.input, start_seconds, end_seconds) : arguments.output;
  job.input_storage_id = arguments.input_storage;
  job.output_storage_id = arguments.output_storage.empty() ? input_statistics.storage_id : arguments.output_storage;
  job.start_time_ns = range.start_time_ns;
  job.end_time_ns = range.end_time_ns;
  job.selected_topics = arguments.topics;
  job.excluded_topics = arguments.exclude_topics;
  job.enable_compression = arguments.compression;
  job.compression_mode = arguments.compression_mode;
  job.compression_format = arguments.compression_format;
  job.overwrite_output = arguments.overwrite;

  std::cout << "开始裁剪，时间基准: rosbag2 记录接收时间，区间: "
            << rst::time_range_to_string(range) << "\n输出: " << job.output_uri << "\n";
  const auto result = rst::TrimWorker::run(job,
    [](const rst::TrimProgress & progress) {
      std::cout << "\r读取 " << progress.read_messages << "，写入 " << progress.written_messages
                << "，跳过 " << progress.skipped_messages << "，进度 "
                << std::fixed << std::setprecision(1) << progress.progress * 100.0 << "%"
                << std::flush;
    });
  std::cout << "\n裁剪完成: 读取 " << result.read_messages << "，写入 " << result.written_messages
            << "，跳过 " << result.skipped_messages << "，输出大小 "
            << result.output_size_bytes << " bytes\n";

  const bool should_verify = arguments.verify || !arguments.report.empty();
  if (should_verify) {
    const auto report = rst::IntegrityValidator::validate(
      job, selected_topics, result.topic_message_counts);
    const auto json_path = choose_report_path(arguments, job.output_uri);
    const auto markdown_path = markdown_report_path(json_path);
    rst::IntegrityValidator::write_json(json_path, report);
    rst::IntegrityValidator::write_markdown(markdown_path, report);
    std::cout << "验证报告: " << json_path << " 和 " << markdown_path << "\n";
    for (const auto & warning : report.warnings) {
      std::cout << "警告: " << warning << "\n";
    }
    for (const auto & error : report.errors) {
      std::cerr << "验证错误: " << error << "\n";
    }
    return report.ok ? 0 : 3;
  }
  return 0;
}

}  // namespace

int main(int argc, char ** argv)
{
  try {
    const auto arguments = parse_arguments(argc, argv);
    if (arguments.convert_ros1_livox) {
      return run_convert_ros1_livox(arguments);
    }
    if (arguments.verify_only) {
      return run_verify_only(arguments);
    }
    if (arguments.info || (arguments.output.empty() && !arguments.dry_run)) {
      return run_info(arguments);
    }
    return run_trim(arguments);
  } catch (const std::exception & exception) {
    std::cerr << "错误: " << exception.what() << "\n使用 --help 查看用法。\n";
    return 2;
  }
}
