import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.qos import qos_profile_sensor_data

from rclpu.action import ActionServer

import json

from kios_interface.srv import GetObjectRequest
from kios_interface.srv import GenerateTreeRequest
from kios_interface.srv import MakePlanRequest

from kios_interface.action import MakePlan


class Coach(Node):
    def __init__(self):
        super().__init__("coach")

        self.hasNewPlan = False

        self.plan = ""

        self.failure_count = 0

        # declare parameters
        self.declare_parameter("power", True)

        timer_callback_group = ReentrantCallbackGroup()

        # initialize timer
        # self.timer_ = self.create_timer(
        #     1, self.timer_callback, callback_group=timer_callback_group  # sec
        # )

        self.planning_server_ = self.create_service(
            MakePlanRequest,
            "make_plan_service",
            self.make_plan_server_callback,
            callback_group=timer_callback_group,
        )

        self.generate_tree_client_ = slef.create_client(
            GenerateTreeRequest,
            "generate_tree_service",
            callback_group=timer_callback_group,
        )

        self.make_plan_action_server_ = ActionServer(
            self, MakePlan, "make_plan_action", self.make_plan_action_callback
        )

    def make_plan_action_callback(self, goal_handle):
        request = goal_handle.request
        feedback_msg = MakePlan.Feedback()
        result = MakePlan.Result()

        if self.is_running:
            result, self.plan = self.make_plan(
                request.current_state, request.goal_state
            )
            if result:
                self.hasNewPlan = True
                goal_handle.succeed()
                return
            else:
                self.get_logger().error("no feasible plan can be found!")
                goal_handle.abort()
                return
        else:
            self.get_logger().error("planner is not runnning!")
            goal_handle.abort()
            return

    # def timer_callback(self):
    #     if self.is_running:
    #         if self.hasNewPlan:
    #             request = GenerateTreeRequest.Request()
    #             request.behavior_tree = self.plan

    #             response = self.generate_tree_client_.call(request)
    #             if response.is_accepted:
    #                 self.hasNewPlan = False
    #                 self.failure_count = 0
    #             else:
    #                 self.get_logger().error("generate tree failed!")
    #                 self.failure_count += 1

    #     else:
    #         self.get_logger().error("not runnning, timer pass ...")
    #         pass

    # def make_plan_server_callback(self, request, response):
    #     current_state = request.current_state
    #     response.is_accepted = False

    #     if self.is_running:
    #         result, self.plan = self.make_plan(current_state)

    #         if result:
    #             self.hasNewPlan = True
    #             response.is_accepted = True
    #             response.plan = self.plan
    #             return response
    #         else:
    #             response.error_message = "no feasible plan can be found!"
    #             return response

    #     else:
    #         self.get_logger().error("planner is not runnning!")
    #         response.error_message = "is_running == False"
    #         return response


def main(args=None):
    rclpy.init(args=args)

    coach = Coach()

    executor = MultiThreadedExecutor()
    executor.add_node(coach)

    executor.spin()

    planner.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
