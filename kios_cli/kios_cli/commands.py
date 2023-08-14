import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.qos import qos_profile_sensor_data

from datetime import datetime
import json

from kios_interface.srv import TeachObjectService


class CLINode(Node):

    def __init__(self):
        super().__init__('CLI')
        # ! DISCARDED
        # self.cli = self.create_client(AddTwoInts, 'add_two_ints')
        # while not self.cli.wait_for_service(timeout_sec=1.0):
        #     self.get_logger().info('service not available, waiting again...')
        # self.req = AddTwoInts.Request()

    # ! DISCARDED
    # def send_request(self, a, b):
    #     self.req.a = a
    #     self.req.b = b
    #     self.future = self.cli.call_async(self.req)
    #     rclpy.spin_until_future_complete(self, self.future)
    #     return self.future.result()

    def teach_object(self, object_name: str):
        self.client = self.create_client(
            TeachObjectService, 'teach_object_cli')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().error('service not available, waiting again...')
        request = TeachObjectService.Request()
        request.object_name = object_name
        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        return future.result()


rclpy.init()
# * Initialize the node
CLI_client = CLINode()

# ! DISCARDED
# def main():

#     response = CLI_client.send_request(int(sys.argv[1]), int(sys.argv[2]))
#     minimal_client.get_logger().info(
#         'Result of add_two_ints: for %d + %d = %d' %
#         (int(sys.argv[1]), int(sys.argv[2]), response.sum))

#     minimal_client.destroy_node()
#     rclpy.shutdown()


def say(args):
    if args.message:
        CLI_client.get_logger().info(args.message)
    else:
        CLI_client.get_logger().error("Usage: ros2 kios say \"<message>\"")
    rclpy.shutdown()


def teach_object(args):
    if args.object_name:
        result = CLI_client.teach_object(args.object_name)
        if result.is_success == True:
            CLI_client.get_logger().info("teach_object: succeed.")
        else:
            CLI_client.get_logger().error("teach_object: FAILED.")
    else:
        CLI_client.get_logger().error("Usage: ros2 kios teach_object \"<object_name>\"")
    rclpy.shutdown


def update_object(args):
    pass


def modify_object(args):
    pass


def turn_on(args):
    pass


def turn_off(args):
    pass
