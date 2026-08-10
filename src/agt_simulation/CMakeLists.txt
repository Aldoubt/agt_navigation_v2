cmake_minimum_required(VERSION 3.8)
project(agt_simulation)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  launch
  config
  models
  worlds
  rviz
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/gazebo_odom_adapter.py
  scripts/gazebo_sensor_adapter.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(
    test_v25_11_simulation_contract
    test/test_v25_11_simulation_contract.py
  )
endif()

ament_package()
