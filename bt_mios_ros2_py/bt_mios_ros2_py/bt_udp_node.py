import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.qos import qos_profile_sensor_data

import socket
import errno
import time
import json
import os

import threading

from ament_index_python.packages import get_package_share_directory

from .resource.ws_client import *

from bt_mios_ros2_interface.msg import RobotState


class BTUdpNode(Node):

    is_udp_on = False
    udp_subscriber = ''
    # robot state variable dictionary.
    robot_state_default = [0.1, 0.1, 0.1, 0.1, 0.1, 0.1]

    def __init__(self):
        super().__init__('bt_udp_node')
        # register flag parameter for updating robot
        self.declare_parameter('is_update', False)
        # server_callback_group = ReentrantCallbackGroup()
        timer_callback_group = ReentrantCallbackGroup()
        publisher_callback_group = timer_callback_group

        self.udp_ip = "localhost"
        self.udp_port = 12346

        # ! loop
        while not self.is_update():
            self.get_logger().info('waiting for udp register ...')
            time.sleep(0.5)
            pass

        self.udp_setup()

        timer_period = 0.05  # seconds
        self.timer = self.create_timer(
            timer_period,
            self.timer_callback,
            callback_group=timer_callback_group)

        self.publisher = self.create_publisher(
            RobotState,
            'mios_state_topic',
            10,
            callback_group=publisher_callback_group
        )

    def is_update(self):
        flag = self.get_parameter('is_update').get_parameter_value().bool_value
        self.get_logger().info('udp update state context: %s' % flag)
        return flag

    def timer_callback(self):
        if self.is_udp_on:
            data, addr = self.udp_subscriber.recvfrom(1024)
            self.get_logger().info("Received message: %s" % data.decode())
            robot_state = json.loads(data.decode())
            msg = RobotState()
            msg.tf_f_ext_k = robot_state["tf_f_ext_k"]
            self.publisher.publish(msg)
            self.get_logger().info("Published RobotState to topic")
        else:
            self.get_logger().info('is_update off, timer pass ...')
            pass

    # def udp_get_package(self):
    #     try:
    #         data, adrr = self.udp_subscriber.recvfrom(8192)
    #         pkg = json.loads(data.decode("utf-8"))
    #         return pkg
    #     except socket.error as e:
    #         if e.errno != errno.EWOULDBLOCK:
    #             self.get_logger().error('Socket error: {}'.format(e))
    #         else:
    #             self.get_logger().info('No data available, non-blocking socket')
    #         return {}
    #     except json.JSONDecodeError:
    #         self.get_logger().error('Failed to decode JSON from received data')
    #         return {}

    # def udp_update_parameter(self):
    #     pkg = self.udp_get_package()
    #     if len(pkg) > 0:
    #         self.get_logger().info('udp_update_parameter hit.')
    #         self.robot_state['tf_f_ext_k'] = pkg.get('TF_F_ext_K')
    #         print(pkg.get('TF_F_ext_K')[2])
    #     else:
    #         self.get_logger().info('udp_update_parameter pass.')

    def udp_setup(self):
        if self.is_udp_on == False:
            self.udp_subscriber = socket.socket(
                socket.AF_INET, socket.SOCK_DGRAM)
            self.udp_subscriber.bind((self.udp_ip, self.udp_port))
            self.is_udp_on == True
            self.get_logger().info('udp setup hit.')
        else:
            pass

        # ! enable block
        # self.udp_subscriber.setblocking(False)


def main(args=None):
    rclpy.init(args=args)

    bt_udp_node = BTUdpNode()

    executor = MultiThreadedExecutor()
    executor.add_node(bt_udp_node)

    executor.spin()

    # udp_thread = threading.Thread()
    # ros2_thread = threading.Thread(target=executor.spin)

    bt_udp_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

# ! something wrong:
# ! the corrupted package: almost all the packages are corrupted
# ! in the telemetry test. why?
# ! multithread: is it possible to run the node and the udp in different threads?
# ! QOS problem: ...
