"""
The code for the coach node, which realizes the entire learning and planning process.

"The main function of the planning/grounding process"

"""

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.qos import qos_profile_sensor_data

from rclpy.action import ActionClient

import json

from kios_interface.srv import GetObjectRequest
from kios_interface.srv import GenerateTreeRequest
from kios_interface.srv import TuneActionRequest

from kios_interface.action import MakePlan


class Coach(Node):
    def __init__(self):
        super().__init__("coach")

        self.plan = ""

        self.failure_count = 0

        # declare parameters
        self.declare_parameter("power", True)

        coach_callback_group = ReentrantCallbackGroup()

        # initialize timer
        # self.timer_ = self.create_timer(
        #     1, self.timer_callback, callback_group=timer_callback_group  # sec
        # )

        self.tune_skill_service_client_ = self.create_client(
            TuneSkillRequest,
            "tune_skill_service",
            callback_group=coach_callback_group,
        )
        
        self.make_plan_action_client_ = ActionClient(
            self, 
            MakePlan,
            "make_plan_action",
            callback_group=coach_callback_group,
        )
        
    # ! test
####################################################### MakePlan ###################################################
    def send_goal(self, current_state, goal_state):
        goal_msg = MakePlan.Goal()
        goal_msg.current_state = current_state
        goal_msg.goal_state = goal_state
        
        self.make_plan_action_client_.wait_for_server()
        
        self._send_goal_future = self.make_plan_action_client_.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        
        self._send_goal_future.add_done_callback(self.goal_response_callback)        
    
    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            # ! handle: goal rejected!
            rclpy.logging.get_logger("coach").error("Goal rejected!")
            return
        
        self.get_logger("coach").info("Goal accepted!")
        
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)
        
    def get_result_callback(self, future):
        result = future.result().result
        if result.is_success:
            # ! handle: goal succeeded!
            rclpy.logging.get_logger("coach").info("Goal succeeded!")
            self.plan = result.plan

    def feedback_callback(self, feedback_msg):
        # * print failure count feedback
        self.get_logger().warn('Feedback: Planning has failed {0} time(s)'.format(feedback_msg.feedback.failure_count))
####################################################################################################################


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
