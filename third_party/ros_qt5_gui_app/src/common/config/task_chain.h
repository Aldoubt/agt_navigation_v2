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
  // A saved task group is preferred so the project Action server can repeat
  // its map-binding and content-hash checks at execution time.
  std::string task_file;
};

struct TaskExecutionStatus {
  std::string state;
  uint32_t current_waypoint{0};
  uint32_t total_waypoints{0};
  std::string message;
  bool terminal{false};
  bool success{false};
  std::vector<int32_t> missed_waypoints;
};
