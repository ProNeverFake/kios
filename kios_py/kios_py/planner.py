"""
The planner node of kios_py:
    1. an action server for MakePlan action, which is used by coach node to make a plan.
"""


import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.qos import qos_profile_sensor_data

from rclpy.action import ActionServer

import json

from kios_interface.srv import MakePlanRequest # ! discarded

from kios_interface.action import MakePlan


class Planner(Node):
    def __init__(self):
        super().__init__("planner")

        # declare flag
        self.is_running = True

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

        self.generate_tree_client_ = slef.create_client(
            GenerateTreeRequest,
            "generate_tree_service",
            callback_group=timer_callback_group,
        )

        self.make_plan_action_server_ = ActionServer(
            self, 
            MakePlan,
            "make_plan_action",
            self.make_plan_action_callback
        )

    
    def make_plan_action_callback(self, goal_handle) -> MakePlan.Result:
        """ the callback function of make_plan_action_server

        Args:
            goal_handle (): the ros2 action goal handle of make_plan_action_server

        Returns:
            result (MakePlan.Result): the result of make_plan_action_server
        """
        request = goal_handle.request
        feedback_msg = MakePlan.Feedback()
        result = MakePlan.Result()

        # initialize failure count and max
        feedback_msg.failure_count = 0
        failure_max = 5
        
        while self.is_running and feedback_msg.failure_count <= failure_max:
            # try to make the plan
            self.get_logger().info("try to make a plan...")
            result, self.plan = self.make_plan(
                request.current_state, request.goal_state
            )
            
            if result: # if planning succeeds
                self.hasNewPlan = True
                result.is_success = True
                result.error_message = ""
                result.plan = self.plan
                goal_handle.succeed()
                return result
            
            else: # if planning fails: add failure count and try again
                feedback_msg.failure_count += 1
                self.get_logger().warn("planning failed: no feasible plan can be found!")
                goal_handle.publish_feedback(feedback_msg)
                continue
            
        # if failure count exceeds the max, abort
        self.get_logger().error("planner is not runnning!")
        result.is_success = False
        result.error_message = "planning failed: Failed to find a feasible plan in maximal number of tries!"
        result.plan = ""
        goal_handle.abort()
        return result

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

    def make_plan(current_state, goal_state) -> (bool, str):
        # TODO
        pass

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
