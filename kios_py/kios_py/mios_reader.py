import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.qos import qos_profile_sensor_data

from rclpy import logging


import asyncio
import websockets
import socket
import time
from datetime import datetime
import json

import queue

from .resource.udp_receiver import UDPReceiver
from .resource.ws_client import *

from kios_interface.msg import MiosState


class MiosReader(Node):
    udp_subscriber = ""
    # robot state variable dictionary.
    mios_state_default = {
        "TF_F_ext_K": [0.1, 0.1, 0.1, 0.1, 0.1, 0.1],
        "T_T_EE": [
            0.1,
            0.1,
            0.1,
            0.1,
            0.1,
            0.1,
            0.1,
            0.1,
            0.1,
            0.1,
            0.1,
            0.1,
            0.1,
            0.1,
            0.1,
            0.1,
        ],
    }

    def __init__(self):
        super().__init__("mios_reader")

        # set logger severity
        logging.set_logger_level(self.get_logger().name, logging.LoggingSeverity.WARN)

        # declare parameters
        self.declare_parameter("power", True)

        # * new udp receiver!
        self.udp_receiver_ = UDPReceiver()
        self.time_out_count_ = 0
        self.udpOn = False

        timer_callback_group = ReentrantCallbackGroup()
        publisher_callback_group = timer_callback_group

        self.timer = self.create_timer(
            0.1, self.timer_callback, callback_group=timer_callback_group  # sec
        )

        self.publisher = self.create_publisher(
            MiosState, "mios_state_topic", 10, callback_group=publisher_callback_group
        )

        time.sleep(2)

    def __del__(self):
        self.udp_receiver_.stop()

    def timer_callback(self):
        if self.check_power():
            if self.udpOn == False:
                self.udp_receiver_.start()
                self.udpOn = True
            else:
                pass

            pub_msg = MiosState()
            udp_msg = self.udp_receiver_.get_last_message()
            if udp_msg:
                # * msg received, publish new.
                self.time_out_count_ = 0
                self.get_logger().info("Received message: %s" % udp_msg.decode())
                mios_state = json.loads(udp_msg.decode())
                pub_msg = self.update_mios_state_message(pub_msg, mios_state)
                # check time
                # self.get_logger().info(f'CHECK: '+ mios_state)
                # print('mios time: ', mios_state["system_time"])
                # print('kios time: ', datetime.now())
                self.publisher.publish(pub_msg)
            else:
                # * no message, count timeout
                self.time_out_count_ = self.time_out_count_ + 1
                self.get_logger().error("UDP TIME OUT TRIGGERED ONCE")
                if self.time_out_count_ >= 10:
                    self.get_logger().error("UDP TIME OUT FOR TOO LONG! POWER OFF!")
                    self.switch_power(turn_on=False)
                # pub_msg = self.update_mios_state_message(
                #     pub_msg, self.mios_state_default)
            # publish msg

        else:
            # * set the udp with the same state as the node.
            if self.udpOn == True:
                self.udp_receiver_.stop()
                self.udpOn = False
            else:
                pass
            self.get_logger().error("Power off, timer pass ...")

    def check_power(self):
        if self.has_parameter("power"):
            # ! BBDEBUG
            return self.get_parameter("power").get_parameter_value().bool_value
        else:
            self.get_logger().error("PARAM MISSING: POWER!")
            return False

    def switch_power(self, turn_on: bool):
        power = rclpy.parameter.Parameter("power", rclpy.Parameter.Type.BOOL, turn_on)
        all_new_parameters = [power]
        self.set_parameters(all_new_parameters)

    # method to update the msg to publish
    def update_mios_state_message(self, msg: MiosState, mios_state: str):
        msg.tf_f_ext_k = mios_state["TF_F_ext_K"]
        msg.t_t_ee = mios_state["T_T_EE"]
        return msg


def main(args=None):
    rclpy.init(args=args)

    mios_reader = MiosReader()

    executor = MultiThreadedExecutor()
    executor.add_node(mios_reader)

    executor.spin()

    mios_reader.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
