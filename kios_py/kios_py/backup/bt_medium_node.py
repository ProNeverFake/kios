import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup

import json

from ament_index_python.packages import get_package_share_directory

from kios_interface.msg import RobotState

from kios_interface.srv import RequestState


class BTMediumNode(Node):
    # robot state variable dictionary.
    robot_state = {"tf_f_ext_k": [0.1, 0.1, 0.1, 0.1, 0.1, 0.1]}

    def __init__(self):
        super().__init__('BTMediumPoint')
        # register flag parameter for updating robot
        server_callback_group = ReentrantCallbackGroup()
        subscriber_callback_group = server_callback_group
        # timer_callback_group = server_callback_group

        # ! timer discarded
        # timer_period = 0.05  # seconds
        # self.timer = self.create_timer(
        #     timer_period,
        #     self.timer_callback,
        #     callback_group=timer_callback_group)

        # initialize the service
        self.service = self.create_service(
            RequestState,
            'request_state',
            self.server_callback,
            callback_group=server_callback_group)

        # initialize the subscriber
        self.subscriber = self.create_subscription(
            RobotState,
            'mios_state_topic',
            self.subscriber_callback,
            10,
            callback_group=subscriber_callback_group)

    def server_callback(self, request, response):
        self.get_logger().info('server hit. the request is : %s' % request.object)
        response.tf_f_ext_k = self.robot_state['tf_f_ext_k']
        self.get_logger().info('server send response.')
        return response

    # def timer_callback(self):
    #     if self.is_update():
    #         self.udp_update_parameter()
    #     else:
    #         self.get_logger().info('timer pass.')
    #         pass

    def subscriber_callback(self, msg):
        self.get_logger().info('subscriber hit. setting the state.')
        self.robot_state["tf_f_ext_k"] = msg.tf_f_ext_k


def main(args=None):
    rclpy.init(args=args)

    bt_medium_node = BTMediumNode()

    executor = MultiThreadedExecutor()
    executor.add_node(bt_medium_node)

    executor.spin()

    bt_medium_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
