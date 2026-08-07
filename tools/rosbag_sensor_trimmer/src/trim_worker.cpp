#include "rosbag_sensor_trimmer/trim_worker.hpp"

#include <chrono>
#include <filesystem>
#include <stdexcept>

#include "rosbag_sensor_trimmer/bag_reader.hpp"
#include "rosbag_sensor_trimmer/bag_metadata.hpp"
#include "rosbag_sensor_trimmer/bag_writer.hpp"
#include "rosbag_sensor_trimmer/topic_filter.hpp"

namespace rosbag_sensor_trimmer
{

TrimResult TrimWorker::run(
  const TrimJob & job, const TrimProgressCallback & progress_callback,
  const std::atomic_bool * cancel_requested)
{
  validate_trim_job(job);
  const auto started = std::chrono::steady_clock::now();

  if (job.overwrite_output && std::filesystem::exists(job.output_uri)) {
    std::error_code error;
    std::filesystem::remove_all(job.output_uri, error);
    if (error) {
      throw std::runtime_error(
              "无法清理已有输出目录: " + job.output_uri.string() + "，" + error.message());
    }
  }
  if (!job.output_uri.parent_path().empty()) {
    std::filesystem::create_directories(job.output_uri.parent_path());
  }

  BagReader reader;
  reader.open(job.input_uri, job.input_storage_id);
  const auto range = make_absolute_time_range(job.start_time_ns, job.end_time_ns);

  TopicFilter filter;
  filter.set_include_topics(job.selected_topics);
  filter.set_exclude_topics(job.excluded_topics);
  const auto output_topics = filter.select(reader.topics());

  BagWriter writer;
  writer.open(job);
  for (const auto & topic : output_topics) {
    writer.create_topic(topic);
  }

  TrimResult result;
  result.job = job;
  result.range = range;
  result.output_topics = output_topics;
  for (const auto & topic : output_topics) {
    result.topic_message_counts[topic.name] = 0;
  }

  TrimProgress progress;
  progress.input_message_count = reader.metadata().message_count;
  auto last_update = started;
  while (reader.has_next()) {
    if (cancel_requested && cancel_requested->load()) {
      throw std::runtime_error("裁剪任务已取消");
    }
    auto message = reader.read_next();
    if (!message) {
      continue;
    }

    ++result.read_messages;
    ++progress.read_messages;
    progress.current_timestamp_ns = static_cast<std::int64_t>(message->time_stamp);
    const auto topic_it = std::find_if(
      reader.topics().begin(), reader.topics().end(),
      [&message](const auto & topic) {return topic.name == message->topic_name;});
    const bool selected = topic_it != reader.topics().end() && filter.matches(*topic_it);
    if (!selected || !range.contains(progress.current_timestamp_ns)) {
      ++result.skipped_messages;
      ++progress.skipped_messages;
    } else {
      const auto size = message->serialized_data ? message->serialized_data->buffer_length : 0U;
      writer.write(message);
      ++result.written_messages;
      result.serialized_bytes += size;
      ++progress.written_messages;
      progress.serialized_bytes += size;
      ++result.topic_message_counts[message->topic_name];
    }

    if (progress.input_message_count > 0) {
      progress.progress = static_cast<double>(progress.read_messages) /
        static_cast<double>(progress.input_message_count);
    }
    const auto now = std::chrono::steady_clock::now();
    if (progress_callback &&
      std::chrono::duration_cast<std::chrono::milliseconds>(now - last_update).count() >= 100) {
      progress_callback(progress);
      last_update = now;
    }
  }
  writer.close();
  reader.close();

  result.output_size_bytes = directory_size_bytes(job.output_uri);
  result.elapsed_seconds = std::chrono::duration<double>(
    std::chrono::steady_clock::now() - started).count();
  progress.progress = 1.0;
  if (progress_callback) {
    progress_callback(progress);
  }
  return result;
}

}  // namespace rosbag_sensor_trimmer
