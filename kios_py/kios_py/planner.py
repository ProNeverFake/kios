"""
The planner node of kios_py:
    1. an action server for MakePlan action, which is used by coach node to make a plan.
"""

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.qos import qos_profile_sensor_data

import json

from kios_interface.srv import MakePlanRequest

class Planner(Node):
    def __init__(self):
        super().__init__("planner")

        self.hasNewPlan = False

        self.plan = ""

        self.failure_count = 0

        server_callback_group = MutuallyExclusiveCallbackGroup()

        self.make_plan_server_ = self.create_service(
            MakePlanRequest,
            "make_plan_service",
            self.make_plan_server_callback,
            callback_group=server_callback_group,
        )

    def make_plan_server_callback(self, request, response):
        current_state = request.current_state
        response.is_success = False

        result = self.make_plan(current_state)

        self.get_logger().info("make plan test")

        if result:
            self.hasNewPlan = True
            response.is_success = True
            response.plan = self.plan
            response.error_message = "test msg"
            return response
        else:
            response.error_message = "no feasible plan can be found!"
            return response

    def make_plan(current_state, goal_state) -> (bool, str):
        # TODO
        return True

    def validate_plan(self, plan) -> bool:
        """validate the plan in logic and structure level

        Args:
            plan (_type_): _description_

        Returns:
            bool: _description_
        """
        # TODO
        pass


def main(args=None):
    rclpy.init(args=args)

    planner = Planner()

    executor = MultiThreadedExecutor()
    executor.add_node(planner)

    executor.spin()

    planner.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
