#include <QApplication>
#include <QCommandLineOption>
#include <QCommandLineParser>
#include <QTimer>

#include <dlfcn.h>
#include "rclcpp/rclcpp.hpp"
#include "rosbag_sensor_trimmer/gui_main_window.hpp"

namespace
{

void load_system_pthread_for_snap_qt()
{
  // Some Snap-hosted terminals make Qt's xcb plugin resolve libpthread from core20.
  // Loading the host library first keeps the GUI executable name and launch command unchanged.
  dlopen("/lib/x86_64-linux-gnu/libpthread.so.0", RTLD_NOW | RTLD_GLOBAL);
}

}  // namespace

int main(int argc, char ** argv)
{
  load_system_pthread_for_snap_qt();
  rclcpp::init(argc, argv);
  QApplication application(argc, argv);
  application.setApplicationName("rosbag_sensor_trimmer");
  application.setApplicationVersion("0.2.0");
  QCommandLineParser parser;
  parser.setApplicationDescription("Qt GUI for rosbag2 trimming and timeline visualization");
  parser.addHelpOption();
  parser.addVersionOption();
  QCommandLineOption input_option({"i", "input"}, "预填输入 bag 路径", "path");
  QCommandLineOption output_option({"o", "output"}, "预填输出目录", "path");
  parser.addOption(input_option);
  parser.addOption(output_option);
  parser.process(application);
  rosbag_sensor_trimmer::GuiMainWindow window;
  const auto input = parser.value(input_option);
  const auto output = parser.value(output_option);
  if (!input.isEmpty() || !output.isEmpty()) {
    window.set_initial_paths(input, output);
  }
  window.show();
  if (!input.isEmpty()) {
    QTimer::singleShot(0, &window, &rosbag_sensor_trimmer::GuiMainWindow::read_initial_bag);
  }
  const auto result = application.exec();
  rclcpp::shutdown();
  return result;
}
