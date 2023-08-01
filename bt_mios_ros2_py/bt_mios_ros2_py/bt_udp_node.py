import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup

import socket
import time
import json
import os

from ament_index_python.packages import get_package_share_directory

from .resource.ws_client import *

from bt_mios_ros2_interface.srv import RequestState

from example_interfaces.srv import AddTwoInts


class BTUdpNode(Node):

    subscriber = ''
    subscriber_ip = "127.0.0.1"
    subscriber_port = 12346
    # robot state variable dictionary.
    robot_state = {"tf_f_ext_k": [0.1, 0.1, 0.1, 0.1, 0.1, 0.1]}

    def __init__(self):
        super().__init__('bt_udp_node')
        # register flag parameter for updating robot
        self.declare_parameter('is_update', False)
        client_callback_group = ReentrantCallbackGroup()
        timer_callback_group = client_callback_group

        # set param with json file
        pkg_share_dir = get_package_share_directory('bt_mios_ros2_py')
        parameter_file_path = os.path.join(
            pkg_share_dir, 'resource', 'parameter.json')
        with open(parameter_file_path, 'r') as file:
            parameters = json.load(file)
            for name, value in parameters.items():
                self.declare_parameter(name, value)
        # setup the udp
        self.udp_setup()
        #  self.publisher_ = self.create_publisher(String, 'topic', 10)
        timer_period = 0.05  # seconds
        self.timer = self.create_timer(
            timer_period, self.timer_callback, callback_group=timer_callback_group)
        self.i = 0
        # ! the service should be in different callback group
        self.srv = self.create_service(
            RequestState, 'request_state', self.request_state_callback, callback_group=client_callback_group)
        # self.srv = self.create_service(
        #     AddTwoInts, 'add_two_ints', self.add_two_ints_callback, callback_group=client_callback_group)

    def request_state_callback(self, request, response):
        self.get_logger().info('the request is : %s' % request.object)
        response.tf_f_ext_k = self.robot_state['tf_f_ext_k']
        self.get_logger().info('service triggered')
        return response

    # def add_two_ints_callback(self, request, response):
    #     response.sum = request.a + request.b
    #     print('service triggered, the response is :', response.sum)
    #     return response

    def is_update(self):
        flag = self.get_parameter('is_update').get_parameter_value().bool_value
        self.get_logger().info('flag: %s' % flag)
        return flag

    def timer_callback(self):
        if self.is_update():
            #
            pass
        else:
            self.get_logger().info('timer pass.')
            pass
        # msg = String()
        # msg.data = 'Hello World: %d' % self.i
        # self.publisher_.publish(msg)
        # self.get_logger().info('Publishing: "%s"' % msg.data)
        # self.i += 1

    def udp_get_package(self):
        udp_pkg = []
        # ! the frequency is too low.
        data, adrr = self.subscriber.recvfrom(8192)
        udp_pkg.append(json.loads(data.decode("utf-8")))

        if len(udp_pkg) > 0:
            pkg = udp_pkg[-1]
            return pkg
        else:
            self.get_logger().info('udp_get_package: no pkg received.')
            return {}

    def udp_update_parameter(self):
        pkg = self.udp_get_package()
        if len(pkg) > 0:
            new_parameters = []
            new_param = rclpy.parameter.Parameter(
                'TF_F_ext_K',
                rclpy.Parameter.Type.DOUBLE_ARRAY,
                pkg.get('TF_F_ext_K')
            )
            new_parameters.append(new_param)
            self.set_parameters(new_parameters)
            self.get_logger().info('udp_update_parameter done.')
            self.robot_state['tf_f_ext_k'] = pkg.get('TF_F_ext_K')
            # print(pkg.get('TF_F_ext_K'))
        else:
            self.get_logger().info('udp_update_parameter pass.')

    def udp_setup(self):
        self.subscriber = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.subscriber.bind((self.subscriber_ip, self.subscriber_port))


def main(args=None):
    rclpy.init(args=args)

    bt_udp_node = BTUdpNode()

    # rclpy.spin(bt_udp_node)
    executor = MultiThreadedExecutor()
    executor.add_node(bt_udp_node)

    executor.spin()

    bt_udp_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
