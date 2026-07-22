cmake_minimum_required(VERSION 3.8)
project(agt_web_console)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})
install(PROGRAMS scripts/web_console.py DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config static DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_web_console test/test_web_console.py)
endif()

ament_package()
