#include "rosbag_sensor_trimmer/playback_support.hpp"

#include <exception>
#include <stdexcept>
#include <string>

#include "rosbag2_cpp/typesupport_helpers.hpp"

namespace rosbag_sensor_trimmer
{

std::string message_package_name(const std::string & message_type)
{
  const auto separator = message_type.find('/');
  if (separator == std::string::npos || separator == 0) {
    throw std::invalid_argument("无效的 ROS 消息类型: " + message_type);
  }
  return message_type.substr(0, separator);
}

PlaybackTypeSupport check_playback_type_support(const std::string & message_type) noexcept
{
  PlaybackTypeSupport result;
  try {
    result.package_name = message_package_name(message_type);
    constexpr auto typesupport_identifier = "rosidl_typesupport_cpp";
    const auto library = rosbag2_cpp::get_typesupport_library(
      message_type, typesupport_identifier);
    if (!library || !rosbag2_cpp::get_typesupport_handle(
        message_type, typesupport_identifier, library))
    {
      result.error = "消息类型支持句柄为空";
      return result;
    }
    result.available = true;
  } catch (const std::exception & exception) {
    result.error = exception.what();
  } catch (...) {
    result.error = "加载消息类型支持时发生未知错误";
  }
  return result;
}

}  // namespace rosbag_sensor_trimmer
