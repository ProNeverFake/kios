import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.qos import qos_profile_sensor_data

import socket
import time
from datetime import datetime
import json

from .resource.ws_client import *

from bt_mios_ros2_interface.msg import RobotState


class StateReader(Node):

    udp_subscriber = ''
    # robot state variable dictionary.
    robot_state_default = [0.1, 0.1, 0.1, 0.1, 0.1, 0.1]

    def __init__(self):
        self.is_udp_on = False
        super().__init__('bt_state_reader')

        # register flag parameter for updating robot
        self.declare_parameter('is_update', False)

        timer_callback_group = ReentrantCallbackGroup()
        publisher_callback_group = timer_callback_group

        self.udp_ip = "localhost"
        self.udp_port = 12346

        self.timer = self.create_timer(
            0.002,  # sec
            self.timer_callback,
            callback_group=timer_callback_group)

        self.publisher = self.create_publisher(
            RobotState,
            'mios_state_topic',
            10,
            callback_group=publisher_callback_group
        )

        self.udp_setup()

    def timer_callback(self):
        if self.is_udp_on:
            data, addr = self.udp_subscriber.recvfrom(1024)
            self.get_logger().info("Received message: %s" % data.decode())
            robot_state = json.loads(data.decode())
            msg = RobotState()
            msg.tf_f_ext_k = robot_state["TF_F_ext_K"]
            self.publisher.publish(msg)
            self.get_logger().info("Published RobotState to topic")
            print("check: ", robot_state["TF_F_ext_K"][2],
                  " sender time: ", robot_state["system_time"],
                  "receiver time: ", datetime.now())
        else:
            self.get_logger().info('is_udp_on off, timer pass ...')
            pass

    def udp_setup(self):

        self.udp_subscriber = socket.socket(
            socket.AF_INET, socket.SOCK_DGRAM)
        self.udp_subscriber.bind((self.udp_ip, self.udp_port))
        self.is_udp_on = True
        self.get_logger().info('udp setup hit.')


def main(args=None):
    rclpy.init(args=args)

    bt_state_reader = StateReader()

    executor = MultiThreadedExecutor()
    executor.add_node(bt_state_reader)

    executor.spin()

    bt_state_reader.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
