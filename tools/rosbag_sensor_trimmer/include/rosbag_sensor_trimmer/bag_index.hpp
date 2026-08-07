#ifndef ROSBAG_SENSOR_TRIMMER__BAG_INDEX_HPP_
#define ROSBAG_SENSOR_TRIMMER__BAG_INDEX_HPP_

#include <cstdint>
#include <vector>

#include "rosbag_sensor_trimmer/bag_metadata.hpp"
#include "rosbag_sensor_trimmer/bag_reader.hpp"
#include "rosbag_sensor_trimmer/time_range.hpp"
#include "rosbag_sensor_trimmer/topic_filter.hpp"

namespace rosbag_sensor_trimmer
{

struct IndexEntry
{
  std::int64_t timestamp_ns{0};
  std::string topic_name;
  std::uint64_t sequence{0};
  std::uint64_t serialized_size{0};
};

class BagIndex
{
public:
  static BagIndex build(BagReader & reader, const TopicFilter & filter = TopicFilter());

  const std::vector<IndexEntry> & entries() const noexcept;
  const BagStatistics & statistics() const noexcept;

  std::uint64_t count_in_range(
    const TimeRange & range, const TopicFilter & filter = TopicFilter()) const;

private:
  std::vector<IndexEntry> entries_;
  BagStatistics statistics_;
};

}  // namespace rosbag_sensor_trimmer

#endif  // ROSBAG_SENSOR_TRIMMER__BAG_INDEX_HPP_
