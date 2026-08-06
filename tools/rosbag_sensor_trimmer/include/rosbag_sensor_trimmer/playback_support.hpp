#ifndef ROSBAG_SENSOR_TRIMMER__PLAYBACK_SUPPORT_HPP_
#define ROSBAG_SENSOR_TRIMMER__PLAYBACK_SUPPORT_HPP_

#include <string>

namespace rosbag_sensor_trimmer
{

struct PlaybackTypeSupport
{
  bool available{false};
  std::string package_name;
  std::string error;
};

std::string message_package_name(const std::string & message_type);
PlaybackTypeSupport check_playback_type_support(const std::string & message_type) noexcept;

}  // namespace rosbag_sensor_trimmer

#endif  // ROSBAG_SENSOR_TRIMMER__PLAYBACK_SUPPORT_HPP_
