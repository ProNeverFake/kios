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

from kios_interface.srv import TuneSkillRequest

from kios_interface.srv import MakePlanRequest

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
    def send_tune_skill_request(self, ready_deadline=0.1, response_deadline=10) -> bool:
        """wrapper method for the TuneSkill service client

        Args:
            ready_deadline (float, optional): time limit for waiting for service ready. Defaults to 0.1.
            response_deadline (int, optional): time limit for server response. Defaults to 10 secs.

        Returns:
            bool: True if service call succeeded, False otherwise.
        """
        request = TuneSkillRequest.Request()
        request.plan = self.plan # ! dummy request

        if not self.tune_skill_service_client_.wait_for_service(timeout_sec=ready_deadline):
            self.get_logger("coach").error(f"Service {self.tune_skill_service_client_.srv_name} not available.")
            return False

        future = self.tune_skill_service_client_.call_async(request)

        try:
            rclpy.spin_until_future_complete(self, future, timeout_sec=response_deadline)
        except ROSInterruptException:
            self.get_logger("coach").error(f"Service call interrupted.")
            return False

        if future.done():
            response = future.result()
            if response.is_success:
                self.get_logger("coach").info(f"Service {self.tune_skill_service_client_.srv_name}: succeeded.")
                return True
            else:
                self.get_logger("coach").error(f"Service {self.tune_skill_service_client_.srv_name}: failed!")
                self.get_logger("coach").error(f"Error message: {response.error_msg}")
                return False
        else:
            self.get_logger("coach").error(f"UNKNOWN ERROR: Service {self.tune_skill_service_client_.srv_name} future not ready (timeout!).")
            return False
            
    def send_tune_skill_request(self, ready_deadline=0.1, response_deadline=10) -> bool:
        """wrapper method for the MakePlan service client

        Args:
            ready_deadline (float, optional): time limit for waiting for service ready. Defaults to 0.1.
            response_deadline (int, optional): time limit for server response. Defaults to 10 secs.

        Returns:
            bool: True if service call succeeded, False otherwise.
        """
        request = MakePlanRequest.Request()
        request.plan = self.plan # ! dummy request

        if not self.make_plan_request_client_.wait_for_service(timeout_sec=ready_deadline):
            self.get_logger("coach").error(f"Service {self.switch_action_client_.srv_name} not available.")
            return False

        future = self.switch_action_client_.call_async(request)

        try:
            rclpy.spin_until_future_complete(self, future, timeout_sec=response_deadline)
        except ROSInterruptException:
            self.get_logger("coach").error(f"Service call interrupted.")
            return False

        if future.done():
            response = future.result()
            if response.is_success:
                self.get_logger("coach").info(f"Service {self.tune_skill_service_client_.srv_name}: succeeded.")
                return True
            else:
                self.get_logger("coach").error(f"Service {self.tune_skill_service_client_.srv_name}: failed!")
                self.get_logger("coach").error(f"Error message: {response.error_msg}")
                return False
        else:
            self.get_logger("coach").error(f"UNKNOWN ERROR: Service {self.tune_skill_service_client_.srv_name} future not ready (timeout!).")
            return False
        

def main(args=None):
    # should run the experiment here.
    rclpy.init(args=args)

    coach = Coach()

    executor = MultiThreadedExecutor()
    executor.add_node(coach)

    executor.spin()

    planner.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
