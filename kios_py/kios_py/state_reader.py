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

from kios_interface.msg import MiosState


class StateReader(Node):

    udp_subscriber = ''
    # robot state variable dictionary.
    mios_state_default = [0.1, 0.1, 0.1, 0.1, 0.1, 0.1]

    def __init__(self):
        super().__init__('bt_state_reader')

        # declare flag
        self.is_running = False

        # declare parameters
        self.declare_parameter('power_on', True)

        timer_callback_group = ReentrantCallbackGroup()
        publisher_callback_group = timer_callback_group

        # udp settings
        self.udp_ip = "localhost"
        self.udp_port = 12346

        self.timer = self.create_timer(
            0.5,  # sec
            self.timer_callback,
            callback_group=timer_callback_group)

        self.publisher = self.create_publisher(
            MiosState,
            'mios_state_topic',
            10,
            callback_group=publisher_callback_group
        )

        self.udp_setup()

    def timer_callback(self):
        if self.is_running:
            data, addr = self.udp_subscriber.recvfrom(1024)
            self.get_logger().info("Received message: %s" % data.decode())
            mios_state = json.loads(data.decode())
            # publish msg
            msg = MiosState()
            msg.tf_f_ext_k = mios_state["TF_F_ext_K"]
            self.publisher.publish(msg)
            self.get_logger().info("Published RobotState to topic")
            print("check: ", mios_state["TF_F_ext_K"][2],
                  "sender time: ", mios_state["system_time"],
                  "receiver time: ", datetime.now())
        else:
            self.get_logger().info('not runnning, timer pass ...')
            pass

    def udp_setup(self):
        self.get_logger().info('udp setup hit.')
        self.udp_subscriber = socket.socket(
            socket.AF_INET, socket.SOCK_DGRAM)
        self.udp_subscriber.bind((self.udp_ip, self.udp_port))
        self.is_running = True


def main(args=None):
    rclpy.init(args=args)

    state_reader = StateReader()

    executor = MultiThreadedExecutor()
    executor.add_node(state_reader)

    executor.spin()

    state_reader.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
