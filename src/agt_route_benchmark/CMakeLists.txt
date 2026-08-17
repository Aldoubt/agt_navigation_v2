cmake_minimum_required(VERSION 3.8)
project(agt_route_benchmark)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  scripts/route_benchmark_run.py
  scripts/route_benchmark_batch.py
  scripts/route_csv_to_path.py
  scripts/route_benchmark_accept_site.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config scenarios launch rviz
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_contracts test/test_contracts.py)
  ament_add_pytest_test(test_scenario test/test_scenario.py)
  ament_add_pytest_test(test_path_io test/test_path_io.py)
  ament_add_pytest_test(test_metrics test/test_metrics.py)
  ament_add_pytest_test(test_profile test/test_profile.py)
  ament_add_pytest_test(test_graph_io test/test_graph_io.py)
  ament_add_pytest_test(test_proposed test/test_proposed.py)
  ament_add_pytest_test(test_adapters test/test_adapters.py)
  ament_add_pytest_test(test_v25_route_asset_adapter test/test_v25_route_asset_adapter.py)
  ament_add_pytest_test(test_manual_waypoint_io test/test_manual_waypoint_io.py)
  ament_add_pytest_test(test_experiment test/test_experiment.py)
  ament_add_pytest_test(test_renderer test/test_renderer.py)
  ament_add_pytest_test(test_batch test/test_batch.py)
  ament_add_pytest_test(test_batch_export test/test_batch_export.py)
  ament_add_pytest_test(test_site_snapshot test/test_site_snapshot.py)
  ament_add_pytest_test(test_map_io test/test_map_io.py)
  ament_add_pytest_test(test_validation test/test_validation.py)
  ament_add_pytest_test(test_semantic_path_validation test/test_semantic_path_validation.py)
  ament_add_pytest_test(test_nav2_params test/test_nav2_params.py)
  ament_add_pytest_test(test_coverage_bridge test/test_coverage_bridge.py)
endif()

ament_package()
