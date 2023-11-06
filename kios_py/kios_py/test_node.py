import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup

import time
from datetime import datetime


class TestNode(Node):

    def __init__(self):
        super().__init__('test_node')

        # declare parameters
        self.declare_parameter('power', True)

        

        timer_callback_group = ReentrantCallbackGroup()

        self.timer = self.create_timer(
            0.5,  # sec
            self.timer_callback,
            callback_group=timer_callback_group)

    def timer_callback(self):
        if self.check_power():
            self.get_logger().info('Power on, timer hit ...')
        else:
            self.get_logger().error('Power off, timer pass ...')
            pass

    def check_power(self):
        if self.has_parameter('power'):
            self.get_logger().error('CHECK POWER')
            return self.get_parameter('power').get_parameter_value().bool_value
        else:
            self.get_logger().error('PARAM MISSING: POWER!')
            return False

    def switch_power(self, turn_on: bool):
        power = rclpy.parameter.Parameter(
            'power',
            rclpy.Parameter.Type.BOOL,
            turn_on
        )
        all_new_parameters = [power]
        self.set_parameters(all_new_parameters)


def main(args=None):
    rclpy.init(args=args)

    test_node = TestNode()

    executor = MultiThreadedExecutor()
    executor.add_node(test_node)

    executor.spin()

    test_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
