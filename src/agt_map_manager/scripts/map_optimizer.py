#!/usr/bin/env python3

import rclpy
from agt_interfaces.action import OptimizeMap
from rclpy.action import ActionServer, GoalResponse
from rclpy.node import Node

from agt_map_manager.optimizer import reject_optimization


class MapOptimizer(Node):
    def __init__(self) -> None:
        super().__init__("agt_map_optimizer")
        self._server = ActionServer(self, OptimizeMap, "/agt/map/optimize", self._execute, goal_callback=self._goal)

    @staticmethod
    def _goal(goal: OptimizeMap.Goal):
        return GoalResponse.ACCEPT if goal.backend in {"pose_graph", "factor_graph", "visual_ba", "noop"} else GoalResponse.REJECT

    async def _execute(self, goal_handle):
        result = OptimizeMap.Result()
        goal_handle.publish_feedback(OptimizeMap.Feedback(stage="VALIDATING", progress=0.0, message="reserved backend check"))
        success, error_code, message = reject_optimization(goal_handle.request.backend)
        result.success = success
        result.error_code = error_code
        result.message = message
        goal_handle.abort()
        return result


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MapOptimizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
