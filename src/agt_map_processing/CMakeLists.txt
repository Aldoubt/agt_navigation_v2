cmake_minimum_required(VERSION 3.8)
project(agt_map_processing)

find_package(ament_cmake REQUIRED)

install(PROGRAMS
  scripts/apply_swept_footprint_to_map.py
  scripts/static_obstacle_evidence.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_octomap_projection test/test_octomap_projection.py)
  ament_add_pytest_test(
    test_static_obstacle_evidence test/test_static_obstacle_evidence.py
  )
endif()

ament_package()
