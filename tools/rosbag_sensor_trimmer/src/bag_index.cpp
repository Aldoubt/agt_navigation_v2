#include "rosbag_sensor_trimmer/bag_index.hpp"

#include <algorithm>
#include <unordered_map>

namespace rosbag_sensor_trimmer
{

BagIndex BagIndex::build(BagReader & reader, const TopicFilter & filter)
{
  BagIndex index;
  index.statistics_.uri = reader.uri();
  index.statistics_.storage_id = reader.storage_id();
  index.statistics_.compression_mode = reader.metadata().compression_mode;
  index.statistics_.compression_format = reader.metadata().compression_format;
  index.statistics_.file_size_bytes = directory_size_bytes(reader.uri());

  std::unordered_map<std::string, std::size_t> topic_indices;
  for (const auto & topic : reader.topics()) {
    topic_indices.emplace(topic.name, index.statistics_.topics.size());
    TopicStatistics statistics;
    statistics.metadata = topic;
    index.statistics_.topics.push_back(statistics);
  }

  bool has_global_timestamp = false;
  while (reader.has_next()) {
    auto message = reader.read_next();
    if (!message) {
      continue;
    }
    const auto metadata_it = std::find_if(
      reader.topics().begin(), reader.topics().end(),
      [&message](const auto & topic) {return topic.name == message->topic_name;});
    if (metadata_it == reader.topics().end() || !filter.matches(*metadata_it)) {
      continue;
    }

    const auto timestamp = static_cast<std::int64_t>(message->time_stamp);
    const auto size = message->serialized_data ? message->serialized_data->buffer_length : 0U;
    index.entries_.push_back(IndexEntry{
      timestamp, message->topic_name, index.entries_.size(), static_cast<std::uint64_t>(size)});
    ++index.statistics_.message_count;
    index.statistics_.empty = false;
    if (!has_global_timestamp) {
      index.statistics_.start_time_ns = timestamp;
      has_global_timestamp = true;
    }
    index.statistics_.end_time_ns = timestamp;
    if (!has_global_timestamp || timestamp < index.statistics_.start_time_ns) {
      index.statistics_.start_time_ns = timestamp;
    }
    const auto topic_index = topic_indices.find(message->topic_name);
    if (topic_index == topic_indices.end()) {
      continue;
    }
    auto & statistics = index.statistics_.topics[topic_index->second];
    ++statistics.message_count;
    statistics.serialized_bytes += size;
    if (!statistics.has_messages) {
      statistics.first_timestamp_ns = timestamp;
      statistics.last_timestamp_ns = timestamp;
      statistics.has_messages = true;
    } else {
      if (timestamp >= statistics.last_timestamp_ns) {
        statistics.maximum_gap_ns = std::max(
          statistics.maximum_gap_ns, timestamp - statistics.last_timestamp_ns);
      }
      statistics.last_timestamp_ns = timestamp;
    }
  }

  if (index.statistics_.message_count > 0) {
    index.statistics_.duration_ns = index.statistics_.end_time_ns - index.statistics_.start_time_ns;
  }
  for (auto & statistics : index.statistics_.topics) {
    if (statistics.message_count > 1 && statistics.last_timestamp_ns > statistics.first_timestamp_ns) {
      statistics.average_frequency_hz = static_cast<double>(statistics.message_count - 1) /
        (static_cast<double>(statistics.last_timestamp_ns - statistics.first_timestamp_ns) / 1.0e9);
    }
  }
  return index;
}

const std::vector<IndexEntry> & BagIndex::entries() const noexcept
{
  return entries_;
}

const BagStatistics & BagIndex::statistics() const noexcept
{
  return statistics_;
}

std::uint64_t BagIndex::count_in_range(const TimeRange & range, const TopicFilter & filter) const
{
  std::uint64_t count = 0;
  for (const auto & entry : entries_) {
    if (filter.matches(entry.topic_name) && range.contains(entry.timestamp_ns)) {
      ++count;
    }
  }
  return count;
}

}  // namespace rosbag_sensor_trimmer
