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

# from bt_mios_ros2_interface.srv import RequestState

from bt_mios_ros2_interface.msg import RobotState


class BTUdpNode(Node):

    udp_subscriber = ''
    udp_ip = "127.0.0.1"
    udp_port = 12346
    # robot state variable dictionary.
    robot_state = {
        {"tf_f_ext_k": [0.1, 0.1, 0.1, 0.1, 0.1, 0.1]}
    }

    def __init__(self):
        super().__init__('bt_udp_node')
        # register flag parameter for updating robot
        self.declare_parameter('is_update', False)
        # server_callback_group = ReentrantCallbackGroup()
        timer_callback_group = ReentrantCallbackGroup()
        publisher_callback_group = timer_callback_group

        # set param with json file
        pkg_share_dir = get_package_share_directory('bt_mios_ros2_py')
        parameter_file_path = os.path.join(
            pkg_share_dir, 'resource', 'parameter.json')
        with open(parameter_file_path, 'r') as file:
            parameters = json.load(file)
            for name, value in parameters.items():
                self.declare_parameter(name, value)

        timer_period = 0.05  # seconds
        self.timer = self.create_timer(
            timer_period,
            self.timer_callback,
            callback_group=timer_callback_group)

        # ! server discarded
        # self.service = self.create_service(
        #     RequestState,
        #     'request_state',
        #     self.request_state_callback,
        #     callback_group=client_callback_group)

        self.publisher = self.create_publisher(
            RobotState,
            'mios_state_topic',
            10,
            callback_group=publisher_callback_group
        )
        # setup the udp (listen to the port)
        self.udp_setup()

    # # ! server discarded
    # def request_state_callback(self, request, response):
    #     self.get_logger().info('the request is : %s' % request.object)
    #     response.tf_f_ext_k = self.robot_state['tf_f_ext_k']
    #     self.get_logger().info('service triggered')
    #     return response

    def is_update(self):
        flag = self.get_parameter('is_update').get_parameter_value().bool_value
        self.get_logger().info('udp update state context: %s' % flag)
        return flag

    def timer_callback(self):
        # udp update state
        if self.is_update():
            self.udp_update_parameter()
        else:
            pass

        # publisher publish msg
        self.get_logger().info('timer hit.')
        msg = RobotState()
        msg.tf_f_ext_k = self.robot_state["tf_f_ext_k"]
        self.publisher.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)

    def udp_get_package(self):
        try:
            data, adrr = self.udp_subscriber.recvfrom(8192)
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

    def udp_update_parameter(self):
        pkg = self.udp_get_package()
        if len(pkg) > 0:
            self.get_logger().info('udp_update_parameter hit.')
            self.robot_state['tf_f_ext_k'] = pkg.get('TF_F_ext_K')
            print(pkg.get('TF_F_ext_K')[2])
        else:
            self.get_logger().info('udp_update_parameter pass.')

    def udp_setup(self):
        self.udp_subscriber = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.udp_subscriber.bind((self.udp_ip, self.udp_port))
        # ! enable block
        # self.udp_subscriber.setblocking(False)


def main(args=None):
    rclpy.init(args=args)

    bt_udp_node = BTUdpNode()

    executor = MultiThreadedExecutor()
    executor.add_node(bt_udp_node)

    executor.spin()

    bt_udp_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
