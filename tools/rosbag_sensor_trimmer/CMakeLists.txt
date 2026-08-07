cmake_minimum_required(VERSION 3.8)
project(rosbag_sensor_trimmer)

set(CMAKE_AUTOMOC ON)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(rosbag2_transport REQUIRED)
find_package(rosbag2_storage REQUIRED)
find_package(rosbag2_compression REQUIRED)
find_package(rosbag2_compression_zstd REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets OpenGL)
find_package(rclcpp REQUIRED)
find_package(BZip2 REQUIRED)
find_package(livox_ros_driver2 REQUIRED)
find_package(rosgraph_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_msgs REQUIRED)

set(CORE_SOURCES
  src/bag_metadata.cpp
  src/bag_reader.cpp
  src/bag_writer.cpp
  src/bag_index.cpp
  src/topic_filter.cpp
  src/time_range.cpp
  src/trim_job.cpp
  src/trim_worker.cpp
  src/integrity_validator.cpp
  src/imu_analysis.cpp
  src/gap_analysis.cpp
  src/playback_support.cpp
  src/ros1_livox_converter.cpp
)

add_library(${PROJECT_NAME}_core ${CORE_SOURCES})
target_include_directories(${PROJECT_NAME}_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(${PROJECT_NAME}_core
  rosbag2_cpp
  rosbag2_storage
  rosbag2_compression
  rosbag2_compression_zstd
  rclcpp
  livox_ros_driver2
  sensor_msgs
)
target_link_libraries(${PROJECT_NAME}_core ${BZIP2_LIBRARIES})
target_include_directories(${PROJECT_NAME}_core PRIVATE ${BZIP2_INCLUDE_DIRS})

add_executable(rosbag_sensor_trimmer_cli src/main.cpp)
target_link_libraries(rosbag_sensor_trimmer_cli ${PROJECT_NAME}_core)
ament_target_dependencies(rosbag_sensor_trimmer_cli
  rosbag2_cpp
  rosbag2_storage
)

add_executable(rosbag_sensor_trimmer_gui
  include/rosbag_sensor_trimmer/gui_main_window.hpp
  include/rosbag_sensor_trimmer/gui_tasks.hpp
  include/rosbag_sensor_trimmer/imu_data.hpp
  include/rosbag_sensor_trimmer/imu_plot_widget.hpp
  include/rosbag_sensor_trimmer/gap_analysis.hpp
  include/rosbag_sensor_trimmer/playback_monitor_task.hpp
  include/rosbag_sensor_trimmer/playback_task.hpp
  include/rosbag_sensor_trimmer/pointcloud_preview_widget.hpp
  include/rosbag_sensor_trimmer/telemetry_widget.hpp
  include/rosbag_sensor_trimmer/timeline_widget.hpp
  src/gui_main.cpp
  src/gui_main_window.cpp
  src/gui_tasks.cpp
  src/imu_plot_widget.cpp
  src/playback_monitor_task.cpp
  src/playback_task.cpp
  src/pointcloud_preview_widget.cpp
  src/telemetry_widget.cpp
  src/timeline_widget.cpp
)
target_link_libraries(rosbag_sensor_trimmer_gui
  ${PROJECT_NAME}_core
  Qt5::Widgets
  Qt5::OpenGL
  ${CMAKE_DL_LIBS}
)
ament_target_dependencies(rosbag_sensor_trimmer_gui
  rosbag2_cpp
  rosbag2_transport
  rosbag2_storage
  rclcpp
  livox_ros_driver2
  rosgraph_msgs
  nav_msgs
  sensor_msgs
  tf2_msgs
)

install(TARGETS ${PROJECT_NAME}_core rosbag_sensor_trimmer_cli rosbag_sensor_trimmer_gui
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})
install(FILES test_data/README.md DESTINATION share/${PROJECT_NAME}/test_data)
install(PROGRAMS
  scripts/inspect_bag.py
  scripts/verify_trim_result.py
  scripts/create_test_bag.py
  scripts/convert_ros1_livox_bag_to_ros2.py
  scripts/smoke_test.sh
  scripts/run_gui.sh
  DESTINATION share/${PROJECT_NAME}/scripts
)
install(FILES README.md LICENSE DESTINATION share/${PROJECT_NAME})

ament_export_include_directories(include)
ament_export_dependencies(
  rosbag2_cpp
  rosbag2_transport
  rosbag2_storage
  rosbag2_compression
  rosbag2_compression_zstd
  rclcpp
  livox_ros_driver2
  rosgraph_msgs
  nav_msgs
  sensor_msgs
  tf2_msgs
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_rosbag_sensor_trimmer
    test/test_bag_index.cpp
    test/test_integrity_validator.cpp
    test/test_imu_analysis.cpp
    test/test_gap_analysis.cpp
    test/test_playback_support.cpp
    test/test_time_range.cpp
    test/test_topic_filter.cpp
    test/test_trim_job.cpp
  )
  if(TARGET test_rosbag_sensor_trimmer)
    target_link_libraries(test_rosbag_sensor_trimmer ${PROJECT_NAME}_core)
    ament_target_dependencies(test_rosbag_sensor_trimmer
      rosbag2_cpp
      rosbag2_storage
      rosbag2_compression
      rosbag2_compression_zstd
    )
  endif()
endif()

ament_package()
