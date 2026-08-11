cmake_minimum_required(VERSION 3.8)
project(agt_simulation)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  launch
  config
  models
  routes
  worlds
  rviz
  DESTINATION share/${PROJECT_NAME}
)

# Keep ROS entrypoints executable under colcon --symlink-install even when the
# repository checkout stores Python sources with mode 0644.
set(AGT_SIMULATION_GENERATED_SCRIPT_DIR
  "${CMAKE_CURRENT_BINARY_DIR}/generated_scripts"
)
file(MAKE_DIRECTORY "${AGT_SIMULATION_GENERATED_SCRIPT_DIR}")

set(AGT_SIMULATION_ENTRYPOINTS
  gazebo_odom_adapter.py
  gazebo_sensor_adapter.py
  keyboard_teleop.py
  synthetic_localization_evidence.py
  v25_11a_baseline_acceptance.py
  v25_11b_localization_acceptance.py
  v25_11c_route_runner.py
  v25_11c_navigation_acceptance.py
  v25_11d_fault_injector.py
  v25_11d_failure_acceptance.py
  v25_11e_observability.py
  v25_11e_observability_acceptance.py
  v25_11e_fault_metrics.py
  v25_11e_comparison_acceptance.py
  v25_11e_compare_reports.py
)

foreach(_script IN LISTS AGT_SIMULATION_ENTRYPOINTS)
  file(COPY
    "${CMAKE_CURRENT_SOURCE_DIR}/scripts/${_script}"
    DESTINATION "${AGT_SIMULATION_GENERATED_SCRIPT_DIR}"
    FILE_PERMISSIONS
      OWNER_READ OWNER_WRITE OWNER_EXECUTE
      GROUP_READ GROUP_EXECUTE
      WORLD_READ WORLD_EXECUTE
  )
endforeach()

set(AGT_SIMULATION_GENERATED_ENTRYPOINTS "")
foreach(_script IN LISTS AGT_SIMULATION_ENTRYPOINTS)
  list(APPEND AGT_SIMULATION_GENERATED_ENTRYPOINTS
    "${AGT_SIMULATION_GENERATED_SCRIPT_DIR}/${_script}"
  )
endforeach()

install(PROGRAMS
  ${AGT_SIMULATION_GENERATED_ENTRYPOINTS}
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(
    test_v25_11_simulation_contract
    test/test_v25_11_simulation_contract.py
  )
  ament_add_pytest_test(
    test_v25_11c_navigation_contract
    test/test_v25_11c_navigation_contract.py
  )
  ament_add_pytest_test(
    test_v25_11d_failure_contract
    test/test_v25_11d_failure_contract.py
  )
  ament_add_pytest_test(
    test_v25_11e_observability_contract
    test/test_v25_11e_observability_contract.py
  )
  ament_add_pytest_test(
    test_v25_11e_comparison_contract
    test/test_v25_11e_comparison_contract.py
  )
  ament_add_pytest_test(
    test_v25_11e_latency_contract
    test/test_v25_11e_latency_contract.py
  )
endif()

ament_package()