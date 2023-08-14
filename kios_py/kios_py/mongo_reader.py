import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.qos import qos_profile_sensor_data

import socket
import time
from datetime import datetime
import json

from .resource.mongodb_client import MongoDBClient

# from kios_interface.msg import MiosState


class MongoReader(Node):

    udp_subscriber = ''
    # robot state variable dictionary.
    mios_state_default = [0.1, 0.1, 0.1, 0.1, 0.1, 0.1]

    def __init__(self):
        super().__init__('mongo_reader')

        # declare flag
        self.is_running = True

        # declare parameters
        self.declare_parameter('power_on', True)

        # intialize mongoDB client
        self.mongo_client_ = MongoDBClient(port=27017)

        timer_callback_group = ReentrantCallbackGroup()
        publisher_callback_group = timer_callback_group

        self.timer = self.create_timer(
            0.5,  # sec
            self.timer_callback,
            callback_group=timer_callback_group)

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


def main(args=None):
    rclpy.init(args=args)

    mongo_reader = MongoReader()

    executor = MultiThreadedExecutor()
    executor.add_node(mongo_reader)

    executor.spin()

    mongo_reader.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
