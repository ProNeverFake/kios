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

from kios_interface.action import ExecuteTree

from .resource.kios_utils import ResultCode

from threading import Thread


class ExecuteTreeRecord:
    def __init__(self):
        self.result_code = ""
        self.node_result = ""

    def feedback_from_json(self, json_str):
        pass

    def write_result(result):
        pass


class Coach(Node):
    def __init__(self):
        super().__init__("coach")

        self.plan = ""

        self.failure_count = 0

        coach_callback_group = ReentrantCallbackGroup()

        # initialize timer
        # self.timer_ = self.create_timer(
        #     1, self.timer_callback, callback_group=timer_callback_group  # sec
        # )
        self.execute_tree_action_client_ = ActionClient(
            self,
            ExecuteTree,
            "execute_tree_action",
            callback_group=coach_callback_group,
        )

        self.tune_skill_service_client_ = self.create_client(
            TuneSkillRequest,
            "tune_skill_service",
            callback_group=coach_callback_group,
        )

        self.make_plan_service_client_ = self.create_client(
            MakePlanRequest,
            "make_plan_service",
            callback_group=coach_callback_group,
        )

    #################################### action ####################################
    def execute_tree_request(self, tree: str, isRetry: bool = False):
        goal_msg = ExecuteTree.Goal()
        goal_msg.tree = tree
        goal_msg.is_retry = isRetry

        self.execute_tree_action_client_.wait_for_server()

        self._send_goal_future = self.execute_tree_action_client_.send_goal_async(
            goal_msg, feedback_callback=self.execute_tree_feedback_callback
        )
        # go to the response callback
        # self._send_goal_future.add_done_callback(self.execute_tree_response_callback)
        rclpy.spin_until_future_complete(self, self._send_goal_future)

        try:
            send_goal_response = self._send_goal_future.result()
            if send_goal_response is None:
                self.get_logger().info("Goal response is None")
                return
            if not send_goal_response.accepted:
                self.get_logger().info("Goal rejected :(")
                return
        except Exception as e:
            self.get_logger().error(f"Exception while sending goal: {e}")
            return

        self.get_logger().info("Execute tree goal accepted :)")
        self.get_result_future = send_goal_response.get_result_async()
        self.get_result_future.add_done_callback(self.execute_tree_result_callback)
        return self.get_result_future

    def execute_tree_send_goal(self, tree: str, isRetry: bool = False) -> None:
        goal_msg = ExecuteTree.Goal()
        goal_msg.tree = tree

        self.execute_tree_action_client_.wait_for_server()

        self._send_goal_future = self.execute_tree_action_client_.send_goal_async(
            goal_msg, feedback_callback=self.execute_tree_feedback_callback
        )
        # go to the response callback
        self._send_goal_future.add_done_callback(self.execute_tree_response_callback)

    def execute_tree_response_callback(self, future):
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().info("Goal rejected :(")
            # ! should not be rejected
            return

        self.get_logger().info("Execute tree goal accepted :)")

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.execute_tree_result_callback)

    def execute_tree_result_callback(self, future):
        """default result handler for the execute_tree action

        Args:
            future (_type_): _description_
        """
        result = future.result().result
        # default handler, can be overwritten, here do nothing.

    def execute_tree_feedback_callback(self, feedback_msg):
        """default feedback handler for the execute_tree action

        Args:
            feedback_msg (_type_): _description_
        """
        feedback = feedback_msg.feedback
        rclpy.logging.get_logger("coach").info(f"Feedback: {feedback.update}")
        # handle the feedback. default handler, can be overwritten, here do nothing.

    def wait_for_result(self, future):
        """a blocking method to wait for the result of a future, can
        be used for service or action.

        Args:
            future (_type_): _description_

        Returns:
            _type_: _description_
        """
        try:
            # This will block until the future is done.
            rclpy.spin_until_future_complete(self, future)
            result = future.result()
            return result.result
        except Exception as e:
            print(f"An error occurred: {e}")
            return None

    ################################################################################

    def send_tune_skill_request(self, plan: str):
        """wrapper method for the TuneSkill service client

        Args:
            ready_deadline (float, optional): time limit for waiting for service ready. Defaults to 0.1.
            response_deadline (int, optional): time limit for server response. Defaults to 10 secs.

        Returns:
            bool: True if service call succeeded, False otherwise.
        """
        request = TuneSkillRequest.Request()
        request.plan = self.plan  # ! dummy request

        if not self.tune_skill_service_client_.wait_for_service(timeout_sec=5):
            self.get_logger().error(
                f"Service {self.tune_skill_service_client_.srv_name} not available."
            )

            return None

        future = self.tune_skill_service_client_.call_async(request)

        try:
            rclpy.spin_until_future_complete(self, future, timeout_sec=5)
        except ROSInterruptException:
            self.get_logger("coach").error(f"Service call interrupted.")
            return None

        if future.done():
            response = future.result()
            if response.is_success:
                self.get_logger().info(
                    f"Service {self.tune_skill_service_client_.srv_name}: succeeded."
                )
                return response
            else:
                self.get_logger().error(
                    f"Service {self.tune_skill_service_client_.srv_name}: failed!"
                )
                self.get_logger().error(f"Error message: {response.error_message}")
                return response
        else:
            self.get_logger().error(
                f"UNKNOWN ERROR: Service {self.tune_skill_service_client_.srv_name} future not ready (timeout!)."
            )
            return None

    def send_make_plan_request(self, current_state: str):
        """wrapper method for the MakePlan service client

        Args:
            ready_deadline (float, optional): time limit for waiting for service ready. Defaults to 0.1.
            response_deadline (int, optional): time limit for server response. Defaults to 10 secs.

        Returns:
            bool: True if service call succeeded, False otherwise.
        """
        request = MakePlanRequest.Request()
        request.current_state = current_state  # ! dummy request

        if not self.make_plan_service_client_.wait_for_service(timeout_sec=5):
            self.get_logger().error(
                f"Service {self.make_plan_service_client_.srv_name} not available."
            )
            return None

        future = self.make_plan_service_client_.call_async(request)

        try:
            rclpy.spin_until_future_complete(self, future, timeout_sec=50)
        except ROSInterruptException:
            self.get_logger("coach").error(f"Service call interrupted.")
            return None

        if future.done():
            response = future.result()
            if response.is_success:
                self.get_logger().info(
                    f"Service {self.make_plan_service_client_.srv_name}: succeeded."
                )
                return response
            else:
                self.get_logger().error(
                    f"Service {self.make_plan_service_client_.srv_name}: failed!"
                )
                self.get_logger().error(f"Error message: {response.error_message}")
                return response
        else:
            self.get_logger().error(
                f"UNKNOWN ERROR: Service {self.make_plan_service_client_.srv_name} future not ready (timeout!)."
            )
            return None


def main(args=None):
    # should run the experiment here.
    rclpy.init(args=args)

    coach = Coach()

    print("Robot learning process is starting...")

    future = coach.execute_tree_request("test_tree", isRetry=False)

    result = coach.wait_for_result(future)

    # result = coach.send_tune_skill_request("test_plan")
    result = coach.send_make_plan_request("test_state")

    print(result.error_message)

    rclpy.shutdown()


if __name__ == "__main__":
    main()
