#include "rosbag_sensor_trimmer/ros1_livox_converter.hpp"

#include <rclcpp/time.hpp>
#include <rosbag2_cpp/converter_options.hpp>
#include <rosbag2_cpp/writer.hpp>
#include <rosbag2_cpp/writers/sequential_writer.hpp>
#include <rosbag2_storage/storage_options.hpp>
#include <rosbag2_storage/topic_metadata.hpp>

#include <bzlib.h>
#include <livox_ros_driver2/msg/custom_msg.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include <algorithm>
#include <array>
#include <cctype>
#include <cstring>
#include <fstream>
#include <limits>
#include <iterator>
#include <sstream>
#include <stdexcept>
#include <tuple>
#include <unordered_map>
#include <unordered_set>
#include <utility>

namespace rosbag_sensor_trimmer
{

const char * const kDefaultRos1LivoxLidarTopic = "/livox/lidar";
const char * const kDefaultRos1LivoxImuTopic = "/livox/imu";
const char * const kDefaultRos2LivoxLidarTopic = "/agt/sensors/lidar/custom";
const char * const kDefaultRos2LivoxImuTopic = "/agt/sensors/imu/data";

namespace
{

constexpr const char * kRos1LivoxCustomType = "livox_ros_driver2/CustomMsg";
constexpr const char * kRos1ImuType = "sensor_msgs/Imu";
constexpr const char * kRos2LivoxCustomType = "livox_ros_driver2/msg/CustomMsg";
constexpr const char * kRos2ImuType = "sensor_msgs/msg/Imu";

constexpr std::uint8_t kOpMsgData = 0x02;
constexpr std::uint8_t kOpFileHeader = 0x03;
constexpr std::uint8_t kOpChunk = 0x05;
constexpr std::uint8_t kOpConnection = 0x07;

class Ros1BagError : public std::runtime_error
{
public:
  explicit Ros1BagError(const std::string & message)
  : std::runtime_error(message)
  {
  }
};

using FieldMap = std::unordered_map<std::string, std::string>;

struct Record
{
  FieldMap header;
  std::vector<std::uint8_t> data;
};

struct ConnectionInfo
{
  std::uint32_t conn_id{0};
  std::string topic;
  std::string datatype;
  std::string md5sum;
};

struct Ros1Message
{
  std::filesystem::path path;
  std::string topic;
  std::string datatype;
  std::int64_t timestamp_ns{0};
  std::vector<std::uint8_t> payload;
};

bool ends_with(const std::string & text, const std::string & suffix)
{
  return text.size() >= suffix.size() &&
    text.compare(text.size() - suffix.size(), suffix.size(), suffix) == 0;
}

std::uint32_t read_u32_le(const std::uint8_t * data)
{
  std::uint32_t value = 0;
  std::memcpy(&value, data, sizeof(value));
  return value;
}

std::uint64_t read_u64_le(const std::uint8_t * data)
{
  std::uint64_t value = 0;
  std::memcpy(&value, data, sizeof(value));
  return value;
}

double read_f64_le(const std::uint8_t * data)
{
  double value = 0.0;
  std::memcpy(&value, data, sizeof(value));
  return value;
}

float read_f32_le(const std::uint8_t * data)
{
  float value = 0.0F;
  std::memcpy(&value, data, sizeof(value));
  return value;
}

void read_exact(std::istream & stream, char * data, std::size_t size, const std::string & label)
{
  stream.read(data, static_cast<std::streamsize>(size));
  if (stream.gcount() != static_cast<std::streamsize>(size)) {
    throw Ros1BagError("truncated ROS 1 record while reading " + label);
  }
}

bool read_stream_u32(std::istream & stream, std::uint32_t & value)
{
  std::array<char, 4> bytes{};
  stream.read(bytes.data(), static_cast<std::streamsize>(bytes.size()));
  if (stream.gcount() == 0 && stream.eof()) {
    return false;
  }
  if (stream.gcount() != static_cast<std::streamsize>(bytes.size())) {
    throw Ros1BagError("truncated ROS 1 record header length");
  }
  value = read_u32_le(reinterpret_cast<const std::uint8_t *>(bytes.data()));
  return true;
}

FieldMap parse_fields(const std::uint8_t * data, std::size_t size)
{
  FieldMap fields;
  std::size_t offset = 0;
  while (offset + sizeof(std::uint32_t) <= size) {
    const auto field_len = read_u32_le(data + offset);
    offset += sizeof(std::uint32_t);
    if (offset + field_len > size) {
      throw Ros1BagError("truncated ROS 1 record header field");
    }
    std::string raw(reinterpret_cast<const char *>(data + offset), field_len);
    offset += field_len;
    const auto separator = raw.find('=');
    if (separator != std::string::npos) {
      fields.emplace(raw.substr(0, separator), raw.substr(separator + 1));
    }
  }
  return fields;
}

FieldMap parse_fields(const std::vector<std::uint8_t> & data)
{
  return parse_fields(data.data(), data.size());
}

bool read_record(std::istream & stream, Record & record)
{
  std::uint32_t header_len = 0;
  if (!read_stream_u32(stream, header_len)) {
    return false;
  }
  std::vector<std::uint8_t> header(header_len);
  if (!header.empty()) {
    read_exact(
      stream, reinterpret_cast<char *>(header.data()), header.size(), "record header");
  }

  std::uint32_t data_len = 0;
  if (!read_stream_u32(stream, data_len)) {
    throw Ros1BagError("truncated ROS 1 record data length");
  }
  record.data.resize(data_len);
  if (!record.data.empty()) {
    read_exact(
      stream, reinterpret_cast<char *>(record.data.data()), record.data.size(), "record data");
  }
  record.header = parse_fields(header);
  return true;
}

std::optional<std::uint8_t> record_op(const FieldMap & header)
{
  const auto iter = header.find("op");
  if (iter == header.end() || iter->second.empty()) {
    return std::nullopt;
  }
  return static_cast<std::uint8_t>(iter->second.front());
}

std::optional<std::uint32_t> read_u32_field(const FieldMap & header, const std::string & name)
{
  const auto iter = header.find(name);
  if (iter == header.end() || iter->second.size() < sizeof(std::uint32_t)) {
    return std::nullopt;
  }
  return read_u32_le(reinterpret_cast<const std::uint8_t *>(iter->second.data()));
}

std::optional<std::int64_t> read_time_field_ns(
  const FieldMap & header,
  const std::string & name)
{
  const auto iter = header.find(name);
  if (iter == header.end() || iter->second.size() < 8) {
    return std::nullopt;
  }
  const auto * data = reinterpret_cast<const std::uint8_t *>(iter->second.data());
  const auto sec = read_u32_le(data);
  const auto nsec = read_u32_le(data + 4);
  return static_cast<std::int64_t>(sec) * 1000000000LL + static_cast<std::int64_t>(nsec);
}

std::string field_text(
  const FieldMap & header,
  const std::string & name,
  const std::string & fallback = "")
{
  const auto iter = header.find(name);
  return iter == header.end() ? fallback : iter->second;
}

std::vector<std::uint8_t> decompress_chunk(
  const FieldMap & header,
  const std::vector<std::uint8_t> & data,
  const std::filesystem::path & path)
{
  const auto compression = field_text(header, "compression", "none");
  if (compression == "none" || compression == "NONE" || compression.empty()) {
    return data;
  }
  if (compression == "bz2" || compression == "BZ2") {
    bz_stream stream{};
    stream.next_in = const_cast<char *>(reinterpret_cast<const char *>(data.data()));
    stream.avail_in = static_cast<unsigned int>(data.size());
    if (BZ2_bzDecompressInit(&stream, 0, 0) != BZ_OK) {
      throw Ros1BagError(path.string() + ": failed to initialize bz2 decompressor");
    }

    std::vector<std::uint8_t> output;
    const auto reserve_size = read_u32_field(header, "size");
    if (reserve_size) {
      output.reserve(*reserve_size);
    }
    std::array<char, 32768> buffer{};
    int status = BZ_OK;
    do {
      stream.next_out = buffer.data();
      stream.avail_out = static_cast<unsigned int>(buffer.size());
      status = BZ2_bzDecompress(&stream);
      if (status != BZ_OK && status != BZ_STREAM_END) {
        BZ2_bzDecompressEnd(&stream);
        throw Ros1BagError(path.string() + ": failed to decompress bz2 ROS 1 chunk");
      }
      const auto produced = buffer.size() - stream.avail_out;
      output.insert(output.end(),
        reinterpret_cast<const std::uint8_t *>(buffer.data()),
        reinterpret_cast<const std::uint8_t *>(buffer.data()) + produced);
    } while (status != BZ_STREAM_END);
    BZ2_bzDecompressEnd(&stream);
    return output;
  }
  throw Ros1BagError(
          path.string() + ": unsupported ROS 1 chunk compression '" + compression +
          "' in the Qt converter; use the Python converter or decompress the ROS 1 bag first");
}

std::vector<Record> iter_chunk_records(
  const std::vector<std::uint8_t> & data,
  const std::filesystem::path & path)
{
  std::vector<Record> records;
  std::size_t offset = 0;
  while (offset < data.size()) {
    if (offset + sizeof(std::uint32_t) > data.size()) {
      throw Ros1BagError(path.string() + ": truncated nested record header length");
    }
    const auto header_len = read_u32_le(data.data() + offset);
    offset += sizeof(std::uint32_t);
    if (offset + header_len + sizeof(std::uint32_t) > data.size()) {
      throw Ros1BagError(path.string() + ": truncated nested record header");
    }
    Record record;
    record.header = parse_fields(data.data() + offset, header_len);
    offset += header_len;
    const auto data_len = read_u32_le(data.data() + offset);
    offset += sizeof(std::uint32_t);
    if (offset + data_len > data.size()) {
      throw Ros1BagError(path.string() + ": truncated nested record data");
    }
    record.data.assign(data.begin() + static_cast<std::ptrdiff_t>(offset),
      data.begin() + static_cast<std::ptrdiff_t>(offset + data_len));
    offset += data_len;
    records.push_back(std::move(record));
  }
  return records;
}

std::optional<ConnectionInfo> parse_connection(
  const FieldMap & header,
  const std::vector<std::uint8_t> & data,
  const std::optional<std::uint32_t> & fallback_conn = std::nullopt)
{
  auto conn_id = read_u32_field(header, "conn");
  if (!conn_id) {
    conn_id = fallback_conn;
  }
  if (!conn_id) {
    return std::nullopt;
  }
  const auto fields = parse_fields(data);
  std::string topic = field_text(header, "topic");
  if (topic.empty()) {
    topic = field_text(fields, "topic");
  }
  return ConnectionInfo{
    *conn_id,
    topic,
    field_text(fields, "type"),
    field_text(fields, "md5sum")};
}

class Ros1Buffer
{
public:
  explicit Ros1Buffer(const std::vector<std::uint8_t> & data)
  : data_(data)
  {
  }

  std::uint8_t read_u8()
  {
    return read_bytes(1)[0];
  }

  std::uint32_t read_u32()
  {
    return read_u32_le(read_bytes(4));
  }

  std::uint64_t read_u64()
  {
    return read_u64_le(read_bytes(8));
  }

  float read_f32()
  {
    return read_f32_le(read_bytes(4));
  }

  double read_f64()
  {
    return read_f64_le(read_bytes(8));
  }

  std::string read_string()
  {
    const auto length = read_u32();
    const auto * raw = read_bytes(length);
    return std::string(reinterpret_cast<const char *>(raw), length);
  }

  std::tuple<std::uint32_t, std::uint32_t, std::string> read_header()
  {
    (void)read_u32();
    const auto sec = read_u32();
    const auto nsec = read_u32();
    return {sec, nsec, read_string()};
  }

private:
  const std::uint8_t * read_bytes(std::size_t size)
  {
    if (offset_ + size > data_.size()) {
      throw Ros1BagError("truncated ROS 1 message payload");
    }
    const auto * value = data_.data() + offset_;
    offset_ += size;
    return value;
  }

  const std::vector<std::uint8_t> & data_;
  std::size_t offset_{0};
};

livox_ros_driver2::msg::CustomMsg parse_ros1_custom_msg(
  const std::vector<std::uint8_t> & data)
{
  Ros1Buffer source(data);
  auto [sec, nsec, frame_id] = source.read_header();

  livox_ros_driver2::msg::CustomMsg output;
  output.header.stamp.sec = static_cast<std::int32_t>(sec);
  output.header.stamp.nanosec = nsec;
  output.header.frame_id = frame_id;
  output.timebase = source.read_u64();
  const auto ros1_point_num = source.read_u32();
  output.lidar_id = source.read_u8();
  output.rsvd = {source.read_u8(), source.read_u8(), source.read_u8()};
  const auto point_count = source.read_u32();
  output.points.reserve(point_count);
  for (std::uint32_t index = 0; index < point_count; ++index) {
    livox_ros_driver2::msg::CustomPoint point;
    point.offset_time = source.read_u32();
    point.x = source.read_f32();
    point.y = source.read_f32();
    point.z = source.read_f32();
    point.reflectivity = source.read_u8();
    point.tag = source.read_u8();
    point.line = source.read_u8();
    output.points.push_back(point);
  }
  output.point_num = static_cast<std::uint32_t>(output.points.size());
  if (ros1_point_num == output.point_num) {
    output.point_num = ros1_point_num;
  }
  return output;
}

sensor_msgs::msg::Imu parse_ros1_imu(const std::vector<std::uint8_t> & data)
{
  Ros1Buffer source(data);
  auto [sec, nsec, frame_id] = source.read_header();

  sensor_msgs::msg::Imu output;
  output.header.stamp.sec = static_cast<std::int32_t>(sec);
  output.header.stamp.nanosec = nsec;
  output.header.frame_id = frame_id;
  output.orientation.x = source.read_f64();
  output.orientation.y = source.read_f64();
  output.orientation.z = source.read_f64();
  output.orientation.w = source.read_f64();
  for (auto & value : output.orientation_covariance) {
    value = source.read_f64();
  }
  output.angular_velocity.x = source.read_f64();
  output.angular_velocity.y = source.read_f64();
  output.angular_velocity.z = source.read_f64();
  for (auto & value : output.angular_velocity_covariance) {
    value = source.read_f64();
  }
  output.linear_acceleration.x = source.read_f64();
  output.linear_acceleration.y = source.read_f64();
  output.linear_acceleration.z = source.read_f64();
  for (auto & value : output.linear_acceleration_covariance) {
    value = source.read_f64();
  }
  return output;
}

void throw_if_cancelled(const std::atomic_bool * cancel_requested)
{
  if (cancel_requested && cancel_requested->load()) {
    throw Ros1BagError("任务已取消");
  }
}

void add_seen(
  Ros1LivoxConversionStats & stats,
  const std::string & topic,
  const std::string & datatype,
  std::int64_t timestamp_ns)
{
  ++stats.read_messages;
  auto topic_iter = std::find_if(stats.topics.begin(), stats.topics.end(),
    [&](const Ros1LivoxTopicStats & item) {
      return item.topic == topic && item.datatype == datatype;
    });
  if (topic_iter == stats.topics.end()) {
    stats.topics.push_back(Ros1LivoxTopicStats{topic, datatype});
    topic_iter = std::prev(stats.topics.end());
  }
  ++topic_iter->count;
  if (!topic_iter->first_timestamp_ns || timestamp_ns < *topic_iter->first_timestamp_ns) {
    topic_iter->first_timestamp_ns = timestamp_ns;
  }
  if (!topic_iter->last_timestamp_ns || timestamp_ns > *topic_iter->last_timestamp_ns) {
    topic_iter->last_timestamp_ns = timestamp_ns;
  }
  if (!stats.start_timestamp_ns || timestamp_ns < *stats.start_timestamp_ns) {
    stats.start_timestamp_ns = timestamp_ns;
  }
  if (!stats.end_timestamp_ns || timestamp_ns > *stats.end_timestamp_ns) {
    stats.end_timestamp_ns = timestamp_ns;
  }
}

using Ros1MessageCallback = std::function<bool(const Ros1Message &)>;

bool read_ros1_messages(
  const std::filesystem::path & path,
  const std::unordered_set<std::string> & wanted_topics,
  std::vector<std::string> & warnings,
  const Ros1MessageCallback & callback,
  const std::atomic_bool * cancel_requested)
{
  std::ifstream stream(path, std::ios::binary);
  if (!stream) {
    throw Ros1BagError("failed to open ROS 1 bag: " + path.string());
  }
  std::string magic;
  std::getline(stream, magic);
  if (!magic.empty() && magic.back() == '\r') {
    magic.pop_back();
  }
  if (magic != "#ROSBAG V2.0") {
    throw Ros1BagError(path.string() + ": unsupported bag magic '" + magic + "'");
  }

  Record first;
  if (!read_record(stream, first)) {
    throw Ros1BagError(path.string() + ": missing file header");
  }
  const auto first_op = record_op(first.header);
  if (!first_op || *first_op != kOpFileHeader) {
    throw Ros1BagError(path.string() + ": first record is not a ROS 1 file header");
  }

  std::unordered_map<std::uint32_t, ConnectionInfo> connections;
  Record record;
  while (read_record(stream, record)) {
    throw_if_cancelled(cancel_requested);
    const auto op = record_op(record.header);
    if (op && *op == kOpConnection) {
      auto connection = parse_connection(record.header, record.data);
      if (connection) {
        connections[connection->conn_id] = *connection;
      }
      continue;
    }
    if (!op || *op != kOpChunk) {
      continue;
    }

    try {
      const auto chunk_data = decompress_chunk(record.header, record.data, path);
      for (const auto & nested : iter_chunk_records(chunk_data, path)) {
        throw_if_cancelled(cancel_requested);
        const auto nested_op = record_op(nested.header);
        if (nested_op && *nested_op == kOpConnection) {
          auto connection = parse_connection(nested.header, nested.data);
          if (connection) {
            connections[connection->conn_id] = *connection;
          }
          continue;
        }
        if (!nested_op || *nested_op != kOpMsgData) {
          continue;
        }
        const auto conn_id = read_u32_field(nested.header, "conn");
        const auto timestamp_ns = read_time_field_ns(nested.header, "time");
        if (!conn_id || !timestamp_ns) {
          continue;
        }
        const auto connection_iter = connections.find(*conn_id);
        if (connection_iter == connections.end()) {
          continue;
        }
        const auto & connection = connection_iter->second;
        if (wanted_topics.count(connection.topic) == 0) {
          continue;
        }
        if (!callback(Ros1Message{
            path,
            connection.topic,
            connection.datatype,
            *timestamp_ns,
            nested.data}))
        {
          return false;
        }
      }
    } catch (const Ros1BagError & error) {
      warnings.push_back(error.what());
      if (!ends_with(path.filename().string(), ".active")) {
        throw;
      }
      warnings.push_back(path.string() + ": stopped at a truncated active chunk");
      break;
    }
  }
  return true;
}

std::pair<int, std::string> natural_bag_key(const std::filesystem::path & path)
{
  const auto name = path.filename().string();
  constexpr const char * prefix = "data_";
  constexpr const char * bag_suffix = ".bag";
  constexpr const char * active_suffix = ".bag.active";
  if (name.compare(0, std::char_traits<char>::length(prefix), prefix) == 0) {
    const auto bag_pos = name.find(bag_suffix, std::char_traits<char>::length(prefix));
    if (bag_pos != std::string::npos &&
      (bag_pos + std::char_traits<char>::length(bag_suffix) == name.size() ||
      name.compare(bag_pos, std::char_traits<char>::length(active_suffix), active_suffix) == 0))
    {
      const auto number_text = name.substr(std::char_traits<char>::length(prefix),
        bag_pos - std::char_traits<char>::length(prefix));
      if (!number_text.empty() &&
        std::all_of(number_text.begin(), number_text.end(),
          [](unsigned char ch) { return std::isdigit(ch) != 0; }))
      {
        return {std::stoi(number_text), name};
      }
    }
  }
  return {std::numeric_limits<int>::max(), name};
}

void validate_options(const Ros1LivoxConversionOptions & options, bool require_output)
{
  if (options.inputs.empty()) {
    throw Ros1BagError("请选择 ROS1 bag 文件或分片目录");
  }
  if (require_output && options.output_uri.empty()) {
    throw Ros1BagError("请选择 ROS2 bag 输出目录");
  }
  if (options.output_storage_id.empty()) {
    throw Ros1BagError("输出 storage ID 不能为空");
  }
  if (options.input_lidar_topic.empty() || options.input_imu_topic.empty() ||
    options.output_lidar_topic.empty() || options.output_imu_topic.empty())
  {
    throw Ros1BagError("输入和输出话题不能为空");
  }
  if (options.start_time_ns && options.end_time_ns &&
    *options.end_time_ns <= *options.start_time_ns)
  {
    throw Ros1BagError("结束时间必须大于开始时间");
  }
}

bool in_time_window(
  std::int64_t timestamp_ns,
  const Ros1LivoxConversionOptions & options)
{
  if (options.start_time_ns && timestamp_ns < *options.start_time_ns) {
    return false;
  }
  if (options.end_time_ns && timestamp_ns >= *options.end_time_ns) {
    return false;
  }
  return true;
}

std::unique_ptr<rosbag2_cpp::Writer> create_writer(const Ros1LivoxConversionOptions & options)
{
  auto writer = std::make_unique<rosbag2_cpp::Writer>(
    std::make_unique<rosbag2_cpp::writers::SequentialWriter>());
  rosbag2_storage::StorageOptions storage_options;
  storage_options.uri = options.output_uri.string();
  storage_options.storage_id = options.output_storage_id;
  writer->open(storage_options, rosbag2_cpp::ConverterOptions{"cdr", "cdr"});
  writer->create_topic(rosbag2_storage::TopicMetadata{
    options.output_lidar_topic, kRos2LivoxCustomType, "cdr", ""});
  writer->create_topic(rosbag2_storage::TopicMetadata{
    options.output_imu_topic, kRos2ImuType, "cdr", ""});
  return writer;
}

void remove_output_if_allowed(const std::filesystem::path & output, bool overwrite)
{
  if (!std::filesystem::exists(output)) {
    return;
  }
  if (!overwrite) {
    throw Ros1BagError("输出目录已存在：" + output.string());
  }
  std::filesystem::remove_all(output);
}

void sort_topic_stats(Ros1LivoxConversionStats & stats)
{
  std::sort(stats.topics.begin(), stats.topics.end(),
    [](const Ros1LivoxTopicStats & left, const Ros1LivoxTopicStats & right) {
      if (left.topic != right.topic) {
        return left.topic < right.topic;
      }
      return left.datatype < right.datatype;
    });
}

}  // namespace

std::vector<std::filesystem::path> resolve_ros1_livox_input_files(
  const std::vector<std::filesystem::path> & inputs)
{
  std::vector<std::filesystem::path> files;
  for (const auto & input : inputs) {
    if (std::filesystem::is_directory(input)) {
      std::vector<std::filesystem::path> matches;
      for (const auto & entry : std::filesystem::directory_iterator(input)) {
        if (!entry.is_regular_file()) {
          continue;
        }
        const auto name = entry.path().filename().string();
        if (ends_with(name, ".bag") || ends_with(name, ".bag.active")) {
          matches.push_back(entry.path());
        }
      }
      if (matches.empty()) {
        throw Ros1BagError("目录中没有 ROS1 .bag 文件：" + input.string());
      }
      std::sort(matches.begin(), matches.end(),
        [](const std::filesystem::path & left, const std::filesystem::path & right) {
          return natural_bag_key(left) < natural_bag_key(right);
        });
      files.insert(files.end(), matches.begin(), matches.end());
    } else if (std::filesystem::is_regular_file(input)) {
      files.push_back(input);
    } else {
      throw Ros1BagError("输入路径不存在：" + input.string());
    }
  }

  std::vector<std::filesystem::path> unique;
  std::unordered_set<std::string> seen;
  for (const auto & file : files) {
    const auto resolved = std::filesystem::canonical(file).string();
    if (seen.insert(resolved).second) {
      unique.push_back(file);
    }
  }
  return unique;
}

Ros1LivoxConversionStats scan_ros1_livox_bag(
  const Ros1LivoxConversionOptions & options,
  const std::atomic_bool * cancel_requested)
{
  validate_options(options, false);
  Ros1LivoxConversionStats stats;
  stats.input_files = resolve_ros1_livox_input_files(options.inputs);
  stats.output_uri = options.output_uri;
  const std::unordered_set<std::string> wanted{
    options.input_lidar_topic,
    options.input_imu_topic};

  for (const auto & path : stats.input_files) {
    throw_if_cancelled(cancel_requested);
    const bool keep_going = read_ros1_messages(
      path, wanted, stats.warnings,
      [&](const Ros1Message & message) {
        add_seen(stats, message.topic, message.datatype, message.timestamp_ns);
        return true;
      },
      cancel_requested);
    if (!keep_going) {
      break;
    }
  }
  sort_topic_stats(stats);
  return stats;
}

Ros1LivoxConversionStats convert_ros1_livox_bag_to_ros2(
  const Ros1LivoxConversionOptions & options,
  const Ros1LivoxProgressCallback & progress_callback,
  const std::atomic_bool * cancel_requested)
{
  validate_options(options, true);

  Ros1LivoxConversionStats stats;
  stats.input_files = resolve_ros1_livox_input_files(options.inputs);
  stats.output_uri = options.output_uri;
  const std::unordered_set<std::string> wanted{
    options.input_lidar_topic,
    options.input_imu_topic};

  remove_output_if_allowed(options.output_uri, options.overwrite_output);
  auto writer = create_writer(options);

  std::optional<std::int64_t> last_timestamp_ns;
  std::optional<std::int64_t> stop_after_ns;
  auto emit_progress = [&]() {
      if (progress_callback) {
        progress_callback(Ros1LivoxConversionProgress{
          stats.read_messages,
          stats.skipped_messages,
          stats.written_lidar,
          stats.written_imu});
      }
    };

  for (const auto & path : stats.input_files) {
    throw_if_cancelled(cancel_requested);
    const bool keep_going = read_ros1_messages(
      path, wanted, stats.warnings,
      [&](const Ros1Message & message) {
        throw_if_cancelled(cancel_requested);
        if (stop_after_ns && message.timestamp_ns > *stop_after_ns) {
          return false;
        }
        add_seen(stats, message.topic, message.datatype, message.timestamp_ns);
        if (!in_time_window(message.timestamp_ns, options)) {
          ++stats.skipped_messages;
          return true;
        }
        if (last_timestamp_ns && message.timestamp_ns < *last_timestamp_ns) {
          stats.warnings.push_back(
            "non-monotonic bag timestamp: " + std::to_string(message.timestamp_ns) +
            " after " + std::to_string(*last_timestamp_ns) + " in " +
            message.path.string());
        }
        last_timestamp_ns = message.timestamp_ns;

        if (message.topic == options.input_lidar_topic) {
          if (message.datatype != kRos1LivoxCustomType) {
            stats.warnings.push_back(
              "skip lidar topic with unexpected type " + message.datatype);
            ++stats.skipped_messages;
            return true;
          }
          const auto ros2_msg = parse_ros1_custom_msg(message.payload);
          writer->write(
            ros2_msg,
            options.output_lidar_topic,
            rclcpp::Time(message.timestamp_ns, RCL_SYSTEM_TIME));
          ++stats.written_lidar;
          if (options.max_lidar_messages > 0 &&
            stats.written_lidar >= options.max_lidar_messages && !stop_after_ns)
          {
            stop_after_ns = message.timestamp_ns;
          }
        } else if (message.topic == options.input_imu_topic) {
          if (message.datatype != kRos1ImuType) {
            stats.warnings.push_back("skip IMU topic with unexpected type " + message.datatype);
            ++stats.skipped_messages;
            return true;
          }
          const auto ros2_msg = parse_ros1_imu(message.payload);
          writer->write(
            ros2_msg,
            options.output_imu_topic,
            rclcpp::Time(message.timestamp_ns, RCL_SYSTEM_TIME));
          ++stats.written_imu;
        } else {
          ++stats.skipped_messages;
        }

        if (stats.read_messages % 1000 == 0 || message.topic == options.input_lidar_topic) {
          emit_progress();
        }
        return true;
      },
      cancel_requested);
    if (!keep_going) {
      break;
    }
  }

  emit_progress();
  writer->close();
  sort_topic_stats(stats);
  return stats;
}

std::string format_ros1_livox_conversion_summary(
  const Ros1LivoxConversionStats & stats,
  bool include_output)
{
  std::ostringstream output;
  output << "输入分片: " << stats.input_files.size() << "\n";
  if (include_output) {
    output << "输出: " << stats.output_uri.string() << "\n";
  }
  output << "读取消息: " << stats.read_messages << "\n";
  output << "写入 LiDAR: " << stats.written_lidar << "\n";
  output << "写入 IMU: " << stats.written_imu << "\n";
  output << "跳过消息: " << stats.skipped_messages << "\n";
  if (stats.start_timestamp_ns && stats.end_timestamp_ns) {
    const auto duration_ns = *stats.end_timestamp_ns - *stats.start_timestamp_ns;
    output << "时间范围: " << *stats.start_timestamp_ns << " -> " <<
      *stats.end_timestamp_ns << " (" << static_cast<double>(duration_ns) / 1.0e9 <<
      " s)\n";
  }
  output << "\n话题:\n";
  for (const auto & topic : stats.topics) {
    output << "  " << topic.topic << " [" << topic.datatype << "]: " <<
      topic.count << "\n";
  }
  if (!stats.warnings.empty()) {
    output << "\n警告:\n";
    for (const auto & warning : stats.warnings) {
      output << "  " << warning << "\n";
    }
  }
  return output.str();
}

}  // namespace rosbag_sensor_trimmer
