#pragma once
#include <nlohmann/json.hpp>
#include "topology_map.h"
struct TaskChain {
  std::vector<TopologyMap::PointInfo> points;
};
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(TaskChain, points);

// Project-owned execution boundary carried through the Qt message bus.  The
// ROS2 channel converts this request to ExecuteWaypointTask; the widget must
// not decide success from pose-distance polling.
struct TaskExecutionRequest {
  std::vector<TopologyMap::PointInfo> points;
  uint32_t loop_count{1};
  // Saved task identity resolved by the robot-side Task Registry. Qt must not
  // send frontend-local absolute task file paths for formal execution.
  std::string map_id;
  std::string map_version_id;
  std::string task_group_id;
  uint32_t task_revision{0};
  std::string expected_content_sha256;
  std::string task_json;
  std::string client_request_id;
};

struct TaskExecutionStatus {
  std::string session_id;
  std::string client_request_id;
  std::string map_id;
  std::string map_version_id;
  std::string task_group_id;
  std::string state;
  uint32_t current_waypoint{0};
  uint32_t total_waypoints{0};
  std::string message;
  std::string blocker_code;
  std::string technical_message;
  bool terminal{false};
  bool success{false};
  std::vector<int32_t> missed_waypoints;
};
