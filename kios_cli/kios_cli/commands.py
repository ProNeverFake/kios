from rcl_interfaces.srv import SetParameters
from rclpy.parameter import Parameter
import rclpy
from rclpy.node import Node
import json
from time import sleep

import sys

from kios_interface.srv import TeachObjectService

from rcl_interfaces.srv import GetParameters, SetParameters, SetParametersAtomically
from rcl_interfaces.msg import Parameter, ParameterValue

from ros2param.api import call_set_parameters
from ros2param.api import get_parameter_value


class CLINode(Node):

    def __init__(self):
        super().__init__('CLI',
                         allow_undeclared_parameters=True,
                         automatically_declare_parameters_from_overrides=True)

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

    def get_node_parameter(self, node_name, param_name):
        get_param_cli_name = "/" + node_name + "/" + "get_parameters"
        self.client = self.create_client(
            GetParameters, get_param_cli_name
        )
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().error(
                f'FAILED: node {node_name} is alive but get_parameters srv is not avaliable!')
            rclpy.shutdown()
        request = GetParameters.Request()
        request.names.append(param_name)
        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        return future.result().values[0]

    def set_node_parameter(self, node_name, param_name, param_value):
        set_param_cli_name = "/" + node_name + "/" + "set_parameters_atomically"
        self.client = self.create_client(
            SetParametersAtomically, set_param_cli_name
        )
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().error(
                f'FAILED: node {node_name} is alive but set_parameters_atomically srv is not avaliable!')
            rclpy.shutdown()
        request = SetParametersAtomically.Request()

        parameter_value_msg = ParameterValue()
        parameter_value_msg.bool_value = param_value

        parameter_msg = Parameter()
        parameter_msg.name = param_name
        parameter_msg.value = parameter_value_msg

        request.parameters = [parameter_msg]
        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

    def switch_power(self, node_name: str, turn_on: bool):

        param = self.get_node_parameter(node_name, "power")

        if param.bool_value == turn_on:  # to turn on/off a node that is already on/off
            if turn_on:
                state = "on"
            else:
                state = "off"
            self.get_logger().info(f'The node {node_name} is already {state}.')
            return
        elif param.bool_value != turn_on:  # need to switch on/off

            if turn_on == True:  # to switch on
                response = self.set_node_parameter(node_name, "power", True)
                result = response.result
                if result.successful == True:
                    self.get_logger().info(
                        f'The node {node_name} is successfully turned on.')
                    return
                else:
                    self.get_logger().error(
                        f'Failed when turning on the node {node_name}.')
                    self.get_logger().error(
                        f'Failure reason: {result.reason}.')
                    return

            if turn_on == False:  # to switch off
                response = self.set_node_parameter(node_name, "power", False)
                result = response.result
                if result.successful == True:
                    self.get_logger().info(
                        f'The node {node_name} is successfully turned off.')
                    return
                else:
                    self.get_logger().error(
                        f'Failed when turning off the node {node_name}.')
                    self.get_logger().error(
                        f'Failure reason: {result.reason}.')
                    return
        else:
            self.get_logger().error(
                f'The parameter [power] of {node_name} was not read correctly. (TODO change the msg)')
            return

    def just_turn_off(self, node_name: str):

        set_param_cli_name = "/" + node_name + "/" + "set_parameters_atomically"
        self.client = self.create_client(
            SetParametersAtomically, set_param_cli_name
        )
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().error(
                f'FAILED: node {node_name} is alive but set_parameters_atomically srv is not avaliable!')
            rclpy.shutdown()
        request = SetParametersAtomically.Request()

        value = ParameterValue()
        value.bool_value = False

        parameter_msg = Parameter()
        parameter_msg.name = "power"
        parameter_msg.value = value

        request.parameters = [parameter_msg]
        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        result = future.result().result

        if result.successful == True:
            self.get_logger().info(
                f'The node {node_name} is successfully turned off.')
            return
        else:
            self.get_logger().error(
                f'Failed when turning off the node {node_name}.')
            self.get_logger().error(
                f'Failure reason: {result.reason}.')
            return


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
    rclpy.shutdown()


def update_object(args):
    pass


def modify_object(args):
    pass


# def turn_on(args):
    # sleep(1)
    # if args.node_name:
    #     # get alive node list
    #     node_list = CLI_client.get_node_names()
    #     is_node_alive = False

    #     # check if the node is alive in ros2
    #     for node_name in node_list:
    #         if args.node_name == node_name:
    #             is_node_alive = True
    #             break

    #     if is_node_alive:
    #         CLI_client.switch_power(args.node_name, turn_on=True)
    #     else:
    #         CLI_client.get_logger().error(
    #             f'Failure in turn_on: The node {args.node_name} is not alive in ros2 node list!')
    # else:
    #     CLI_client.get_logger().error("Usage: ros2 kios turn_on \"<node_name>\"")
    # rclpy.shutdown()


# def turn_off(args):
#     sleep(1)
#     if args.node_name:
#         # get alive node list
#         node_list = CLI_client.get_node_names()
#         is_node_alive = False
#         # check if the node is alive
#         CLI_client.get_logger().info(f'node exist: {node_list}')
#         for node_name in node_list:
#             if args.node_name == node_name:
#                 is_node_alive = True
#                 break

#         if is_node_alive:
#             CLI_client.switch_power(args.node_name, turn_on=False)
#         else:
#             CLI_client.get_logger().error(
#                 f'Failure in turn_off: The node {args.node_name} is not alive in ros2 node list!')
#     else:
#         CLI_client.get_logger().error("Usage: ros2 kios turn_off \"<node_name>\"")
#     rclpy.shutdown()

# def turn_off(args):
#     sleep(1)
#     if args.node_name:
#         # get alive node list
#         node_list = CLI_client.get_node_names()
#         is_node_alive = False
#         # check if the node is alive
#         CLI_client.get_logger().info(f'node exist: {node_list}')
#         for node_name in node_list:
#             if args.node_name == node_name:
#                 is_node_alive = True
#                 break

#         if is_node_alive:
#             CLI_client.just_turn_off(args.node_name)
#         else:
#             CLI_client.get_logger().error(
#                 f'Failure in turn_off: The node {args.node_name} is not alive in ros2 node list!')
#     else:
#         CLI_client.get_logger().error("Usage: ros2 kios turn_off \"<node_name>\"")
#     rclpy.shutdown()


def turn_off(args):
    if args.node_name:

        parameter = Parameter()
        parameter.name = "power"
        parameter.value = get_parameter_value(string_value="false")

        response = call_set_parameters(
            node=CLI_client, node_name=args.node_name, parameters=[parameter])

        assert len(response.results) == 1
        result = response.results[0]
        if result.successful:
            msg = 'Set parameter successful'
            if result.reason:
                msg += ': ' + result.reason
            print(msg)
        else:
            msg = 'Setting parameter failed'
            if result.reason:
                msg += ': ' + result.reason
            print(msg, file=sys.stderr)
    else:
        CLI_client.get_logger().error("Usage: ros2 kios turn_off \"<node_name>\"")
    rclpy.shutdown()


def turn_on(args):
    if args.node_name:

        parameter = Parameter()
        parameter.name = "power"
        parameter.value = get_parameter_value(string_value="true")

        response = call_set_parameters(
            node=CLI_client, node_name=args.node_name, parameters=[parameter])

        assert len(response.results) == 1
        result = response.results[0]
        if result.successful:
            msg = 'Set parameter successful'
            if result.reason:
                msg += ': ' + result.reason
            print(msg)
        else:
            msg = 'Setting parameter failed'
            if result.reason:
                msg += ': ' + result.reason
            print(msg, file=sys.stderr)
    else:
        CLI_client.get_logger().error("Usage: ros2 kios turn_on \"<node_name>\"")
    rclpy.shutdown()
