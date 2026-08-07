#ifndef ROSBAG_SENSOR_TRIMMER__TRIM_WORKER_HPP_
#define ROSBAG_SENSOR_TRIMMER__TRIM_WORKER_HPP_

#include <atomic>
#include <cstdint>
#include <functional>
#include <string>
#include <unordered_map>
#include <vector>

#include "rosbag_sensor_trimmer/time_range.hpp"
#include "rosbag_sensor_trimmer/trim_job.hpp"
#include "rosbag2_storage/topic_metadata.hpp"

namespace rosbag_sensor_trimmer
{

struct TrimProgress
{
  std::uint64_t read_messages{0};
  std::uint64_t written_messages{0};
  std::uint64_t skipped_messages{0};
  std::uint64_t serialized_bytes{0};
  std::uint64_t input_message_count{0};
  std::int64_t current_timestamp_ns{0};
  double progress{0.0};
};

struct TrimResult
{
  TrimJob job;
  TimeRange range;
  std::uint64_t read_messages{0};
  std::uint64_t written_messages{0};
  std::uint64_t skipped_messages{0};
  std::uint64_t serialized_bytes{0};
  std::uint64_t output_size_bytes{0};
  double elapsed_seconds{0.0};
  std::vector<rosbag2_storage::TopicMetadata> output_topics;
  std::unordered_map<std::string, std::uint64_t> topic_message_counts;
};

using TrimProgressCallback = std::function<void(const TrimProgress &)>;

class TrimWorker
{
public:
  static TrimResult run(
    const TrimJob & job,
    const TrimProgressCallback & progress_callback = TrimProgressCallback(),
    const std::atomic_bool * cancel_requested = nullptr);
};

}  // namespace rosbag_sensor_trimmer

#endif  // ROSBAG_SENSOR_TRIMMER__TRIM_WORKER_HPP_
