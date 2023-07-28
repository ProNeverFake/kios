import rclpy
from rclpy.node import Node

import socket
import time
import json
import os

from ament_index_python.packages import get_package_share_directory

from .resource.ws_client import *


class BTUdpNode(Node):

    subsriber = ''
    # ! COMPLETE
    subscriber_ip = ""
    subscriber_port = 12346

    def __init__(self):
        super().__init__('bt_upd_node')
        # register flag parameter for updating robot
        self.declare_parameter('is_update', 'false')

        # declare parameters with a json file
        # Get the path to the 'resources' directory
        # package_path = os.path.dirname(__file__)
        # resources_path = os.path.join(package_path, 'resource')
        # # Get the path to the 'parameter.json' file
        # parameter_file_path = os.path.join(resources_path, 'parameter.json')
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
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def is_update(self):
        flag = self.get_parameter('is_update').get_parameter_value().bool_value
        return flag

    def timer_callback(self):
        if self.is_update():
            self.udp_update_parameter()
        else:
            pass
        # msg = String()
        # msg.data = 'Hello World: %d' % self.i
        # self.publisher_.publish(msg)
        # self.get_logger().info('Publishing: "%s"' % msg.data)
        # self.i += 1

    def udp_get_package(self):
        udp_pkg = []
        start_time = time.time()
        while True:
            data, adrr = self.subscriber.recvfrom(8192)
            udp_pkg.append(json.loads(data.decode("utf-8")))
            if time.time() - start_time > 0.04:
                break
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
        else:
            self.get_logger().info('udp_update_parameter pass.')

    def udp_setup(self):
        self.subscriber = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.subscriber.bind((self.subscriber_ip, self.subscriber_port))

    def test_telemetry_udp(self, address: str, subscriber_addr: str, subscriber_port: int = 12346):
        # ! discarded
        # # This line makes a call to the server to subscribe to the telemetry topics "tau_ext" and "q"
        # result_1 = call_method(address, 12000, "subscribe_telemetry",
        #                        {"ip": subscriber_addr, "port": subscriber_port, "subscribe": ["tau_ext", "q"]}, silent=False, timeout=7)
        # # Checking if the subscription was successful, if true print a success message, else print the error
        # if result_1["result"]["result"]:
        #     print("successfull subscribed.")
        # else:
        #     print("Error while subscribing: ", result_1)

        # Sets up a UDP socket and binds it to the subscriber's address and port
        self.subscriber = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.subscriber.bind((self.subscriber_ip, self.subscriber_port))

        received_pkgs = []
        start_time = time.time()
        try:
            print("\n    --Interrupt with ctrl+c--\n")

            # Receives packages for 10 seconds or until interrupted by a KeyboardInterrupt (ctrl+c)
            while True:
                data, adrr = self.subscriber.recvfrom(8192)
                received_pkgs.append(json.loads(data.decode("utf-8")))
                if time.time() - start_time > 10:
                    break
        # If a KeyboardInterrupt happens, the loop is broken
        except KeyboardInterrupt:
            pass

        # Record the end time
        end_time = time.time()

        # Check the received packages for validation
        pkg_validation_cnt = 0
        for pkg in received_pkgs:
            if pkg.get("tau_ext", False) != False and pkg.get("q", False) != False:
                pkg_validation_cnt += 1

        # Makes a call to the server to unsubscribe from the telemetry
        result_2 = call_method(address, 12000, "unsubscribe_telemetry", {
                               "ip": subscriber_addr})

        # Check if unsubscription was successful
        if result_2["result"]["result"]:
            print("successfull unsibscribed.")


def main(args=None):
    rclpy.init(args=args)

    bt_udp_node = BTUdpNode()

    rclpy.spin(bt_udp_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    bt_udp_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
