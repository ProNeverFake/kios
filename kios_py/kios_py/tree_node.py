""" tree node python implementation.
this may be integrated with the coach node in the future.
"""

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup

from rclpy.action import ActionClient

from kios_interface.msg import TaskState
from kios_interface.action import ExecuteSkill

from resource.action_nodes import ToolPick


class TreeNode(Node):
    def __init__(self):
        super().__init__("TreeNode")

        action_callback_group = ReentrantCallbackGroup()

        self.execute_skill_action_client_ = ActionClient(
            self,
            ExecuteSkill,
            "execute_tree_action",
            callback_group=action_callback_group,
        )

    #################################### action ####################################
    def cancel_done(self, future):
        cancel_response = future.result()
        if len(cancel_response.goals_canceling) > 0:
            self.get_logger().info("Goal successfully canceled")
        else:
            self.get_logger().info("Goal failed to cancel")

    def execute_skill_request(self, skill_context: str):
        goal = ExecuteSkill.Goal()
        goal.command_context = skil_context
        goal.skill_type = skill_type

        self.execute_skill_action_client_.wait_for_server()

        self._send_goal_future = self.execute_skill_action_client_.send_goal_async(
            goal, feedback_callback=self.execute_skill_feedback_callback
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
        self.get_result_future.add_done_callback(self.execute_skill_result_callback)
        return self.get_result_future

    def execute_skill_send_goal(self, tree: str, isRetry: bool = False) -> None:
        goal_msg = ExecuteSkill.Goal()
        goal_msg.tree = tree

        self.execute_skill_action_client_.wait_for_server()

        self._send_goal_future = self.execute_skill_action_client_.send_goal_async(
            goal_msg, feedback_callback=self.execute_skill_feedback_callback
        )
        # go to the response callback
        self._send_goal_future.add_done_callback(self.execute_skill_goal_response_callback)

    def execute_skill_goal_response_callback(self, future):
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().info("Goal rejected :(")
            # ! should not be rejected
            return

        self.get_logger().info("Execute tree goal accepted :)")

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.execute_skill_result_callback)

    def execute_skill_result_callback(self, future):
        """default result handler for the execute_tree action
        """
        result = future.result().result
        # default handler, can be overwritten, here do nothing.

    # ! not in use
    def execute_skill_feedback_callback(self, feedback_msg):
        """default feedback handler for the execute_tree action
        """
        feedback = feedback_msg.feedback
        self.get_logger().info(f"Feedback: {feedback.update}")
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


def main(args=None):
    rclpy.init(args=args)

    tree_node = TreeNode()

    executor = MultiThreadedExecutor()
    executor.add_node(tree_node)

    executor.spin()

    tree_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
