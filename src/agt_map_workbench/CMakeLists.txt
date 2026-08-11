cmake_minimum_required(VERSION 3.8)
project(agt_map_workbench)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  scripts/agt_map_workbench.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_workbench_model test/test_workbench_model.py)
endif()

ament_package()
