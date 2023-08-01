import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup

import socket
import errno
import time
import json
import os

from ament_index_python.packages import get_package_share_directory

from .resource.ws_client import *

from bt_mios_ros2_interface.srv import RequestState

class BTMediumPoint(Node):

    subscriber = ''
    subscriber_ip = "127.0.0.1"
    subscriber_port = 12346
    # robot state variable dictionary.
    robot_state = {"tf_f_ext_k": [0.1, 0.1, 0.1, 0.1, 0.1, 0.1]}
    data = ""
    adrr = ""

    def __init__(self):
        super().__init__('BTMediumPoint')
        # register flag parameter for updating robot
        server_callback_group = ReentrantCallbackGroup()
        subscriber_callback_group = server_callback_group
        
        # self.publisher_ = self.create_publisher(String, 'topic', 10)
        timer_period = 0.05  # seconds
        self.timer = self.create_timer(
            timer_period, self.timer_callback, callback_group=timer_callback_group)
        self.i = 0
        # ! the service should be in different callback group
        self.srv = self.create_service(
            RequestState, 'request_state', self.request_state_callback, callback_group=client_callback_group)

    def request_state_callback(self, request, response):
        self.get_logger().info('the request is : %s' % request.object)
        response.tf_f_ext_k = self.robot_state['tf_f_ext_k']
        self.get_logger().info('service triggered')
        return response

    def is_update(self):
        flag = self.get_parameter('is_update').get_parameter_value().bool_value
        self.get_logger().info('flag: %s' % flag)
        return flag

    def timer_callback(self):
        if self.is_update():
            self.udp_update_parameter()
        else:
            self.get_logger().info('timer pass.')
            pass

    def udp_get_package(self):

        try:
            self.subscriber.setblocking(False)
            # data, adrr = self.subscriber.recvfrom(8192)
            pkg = json.loads(data.decode("utf-8"))
            return pkg
        except socket.error as e:
            if e.errno != errno.EWOULDBLOCK:
                self.get_logger().error('Socket error: {}'.format(e))
            else:
                self.get_logger().info('No data available, non-blocking socket')
            return {}
        except json.JSONDecodeError:
            self.get_logger().error('Failed to decode JSON from received data')
            return {}
        # udp_pkg = []
        # # ! the frequency is too low.
        # data, adrr = self.subscriber.recvfrom(8192)
        # udp_pkg.append(json.loads(data.decode("utf-8")))

        # if len(udp_pkg) > 0:
        #     pkg = udp_pkg[-1]
        #     return pkg
        # else:
        #     self.get_logger().info('udp_get_package: no pkg received.')
        #     return {}

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
            print(pkg.get('TF_F_ext_K')[2])
            # print(pkg.get('TF_F_ext_K'))
        else:
            self.get_logger().info('udp_update_parameter pass.')

    def udp_setup(self):
        self.subscriber = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.subscriber.bind((self.subscriber_ip, self.subscriber_port))
        self.subscriber.setblocking(False)
        self.data, self.adrr = self.subscriber.recvfrom(8192)


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
