"""
The Commander node of kios_py:
    1. an action server for MakePlan action, which is used by coach node to make a plan.
"""

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.qos import qos_profile_sensor_data

from rclpy.action import ActionServer, CancelResponse, GoalResponse

import json

from kios_interface.srv import MakePlanRequest

from kios_interface.action import ExecuteSkill

from .resource.task import *

import multiprocessing
import atexit

MIOS = "127.0.0.1"


def mios_monitor(
    task: Task, pipe_connection: multiprocessing.connection.Connection
) -> None:
    task.wait()
    idle = True
    try:
        result = task.wait()
        pipe_connection.send([result])
    except KeyboardInterrupt:
        pass


class Commander(Node):
    def __init__(self):
        super().__init__("commander")
        self.child_connection = None
        self.parent_connection = None

        # server_callback_group = MutuallyExclusiveCallbackGroup()
        action_callback_group = ReentrantCallbackGroup()

        # self.make_plan_server_ = self.create_service(
        #     MakePlanRequest,
        #     "make_plan_service",
        #     self.make_plan_server_callback,
        #     callback_group=server_callback_group,
        # )

        self._action_server = ActionServer(
            self,
            ExecuteSkill,
            "mios_execution_action",
            execute_callback=self.execute_callback,
            callback_group=action_callback_group,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
        )

    def destroy(self):
        self._action_server.destroy()
        super().destroy_node()

    def cancel_callback(self, goal_handle):
        """Accept or reject a client request to cancel an action."""
        self.get_logger().info("Received cancel request")

        return CancelResponse.ACCEPT

    async def execute_callback(self, goal_handle):
        self.get_logger().info("Executing goal...")

        feedback_msg = ExecuteSkill.Feedback()
        request = ExecuteSkill.Goal

        skill_type = request.skill_type
        skill_context = request.command_context

        # start the task in mios here

        new_task = Task(MIOS)
        new_task.add_skill("skill", skill_type, skill_context)
        new_task.start()
        # check if start task is successful
        if bool(new_task.task_start_response["result"]["result"]) == False:  # !
            # if not, return error
            self.get_logger().info("Task start failed")
            goal_handle.abort()
            result = ExecuteSkill.Result()
            result.error_msg = "Task start failed"
            return result

        # wait for task to finish
        self.get_logger().info("Task start success")

        # start a subprocess to monitor the task
        self.parent_connection, self.child_connection = multiprocessing.Pipe()
        self.mios_monitor_subprocess = multiprocessing.Process(
            target=mios_monitor, args=(self.child_connection,)
        )
        atexit.register(self.mios_monitor_subprocess.terminate)
        self.mios_monitor_subprocess.start()

        # wait for task to finish
        isExecuting = True
        self.wait_response = None
        while isExecuting:
            # check if cancel requested
            if (
                goal_handle.is_cancel_requested
            ):  # cancel requested, stop task, return error msg
                goal_handle.canceled()
                self.mios_monitor_subprocess.terminate()
                new_task.stop()
                result = ExecuteSkill.Result()
                result.error_msg = "Task cancelled"
                return result
            # check if task finished
            if self.parent_connection.poll():
                self.wait_response = (
                    self.parent_connection.recv().pop()
                )  # use this or newtask.task_wait_response?
                isExecuting = False

        goal_handle.succeed()  # ! currently only success

        result = ExecuteSkill.Result()
        result.result = bool(new_task.task_wait_response["result"]["result"])
        result.error_msg = new_task.task_wait_response["result"]["error_msg"]
        # ? error code?
        return result

    def goal_callback(self, goal_request):
        """Accept or reject a client request to begin an action."""
        # This server allows multiple goals in parallel
        self.get_logger().info("Received goal request")
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        """Accept or reject a client request to cancel an action."""
        self.get_logger().info("Received cancel request")

        return CancelResponse.ACCEPT

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


def main(args=None):
    rclpy.init(args=args)

    commander = Commander()

    executor = MultiThreadedExecutor()
    executor.add_node(commander)

    executor.spin()

    commander.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
