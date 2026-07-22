cmake_minimum_required(VERSION 3.8)
project(agt_map_manager)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})
install(PROGRAMS scripts/map_registry.py scripts/map_optimizer.py DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_map_registry test/test_map_registry.py)
  ament_add_pytest_test(test_optimizer test/test_optimizer.py)
endif()

ament_package()
