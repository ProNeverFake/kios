""" tree node python implementation.
this may be integrated with the coach node in the future.
"""

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup

from kios_interface.msg import TaskState




class TreeNode(Node):
    def __init__(self):
        super().__init__("tree_node")

        # declare parameters
        self.declare_parameter("power", True)

        self.task_state_sub_ = self.create_subscription(
            TaskState, "task_state_topic", self.task_state_sub_callback, 10
        )

        timer_callback_group = ReentrantCallbackGroup()

        self.timer = self.create_timer(
            0.5, self.timer_callback, callback_group=timer_callback_group  # sec
        )

    def timer_callback(self):
        pass

    def task_state_sub_callback(self, msg: TaskState):
        pass


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
