#ifndef ROSBAG_SENSOR_TRIMMER__BAG_WRITER_HPP_
#define ROSBAG_SENSOR_TRIMMER__BAG_WRITER_HPP_

#include <memory>

#include "rosbag_sensor_trimmer/trim_job.hpp"
#include "rosbag2_cpp/writer.hpp"

namespace rosbag_sensor_trimmer
{

class BagWriter
{
public:
  BagWriter() = default;
  ~BagWriter();

  BagWriter(const BagWriter &) = delete;
  BagWriter & operator=(const BagWriter &) = delete;

  void open(const TrimJob & job);
  void create_topic(const rosbag2_storage::TopicMetadata & topic);
  void write(std::shared_ptr<rosbag2_storage::SerializedBagMessage> message);
  void close();

private:
  std::unique_ptr<rosbag2_cpp::Writer> writer_;
  bool opened_{false};
};

}  // namespace rosbag_sensor_trimmer

#endif  // ROSBAG_SENSOR_TRIMMER__BAG_WRITER_HPP_
