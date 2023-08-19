from rcl_interfaces.srv import SetParameters
from rclpy.parameter import Parameter
import rclpy
from rclpy.node import Node
import json
from time import sleep

from kios_interface.srv import TeachObjectService

from rcl_interfaces.srv import GetParameters, SetParameters, SetParametersAtomically
from rcl_interfaces.msg import Parameter, ParameterValue

from ros2param.api import call_set_parameters
from ros2param.api import get_parameter_value
from ros2node.api import get_node_names
from ros2node.api import get_absolute_node_name


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

    def find_node(self, node_name):
        node_names = get_node_names(
            node=CLI_client, include_hidden_nodes=False)
        full_node_name = get_absolute_node_name(node_name)
        if full_node_name not in {n.full_name for n in node_names}:
            return False
        else:
            return True

    def set_parameter(self, node_name, param_name, param_value: str):
        """method to set parameter making use of ros2 api

        Args:
            node_name (_type_): _description_
            param_name (_type_): _description_
            param_value (str): _description_

        Returns:
            _type_: _description_
        """
        parameter = Parameter()
        parameter.name = param_name
        parameter.value = get_parameter_value(string_value=param_value)

        response = call_set_parameters(
            node=self, node_name=node_name, parameters=[parameter])

        assert len(response.results) == 1
        result = response.results[0]
        return result


rclpy.init()
# * Initialize the node
CLI_client = CLINode()


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


def turn_off(args):
    if args.node_name:
        if CLI_client.find_node(args.node_name):
            CLI_client.get_logger().error(
                f'FAILED: Node {args.node_name} not found!')
        else:
            result = CLI_client.set_parameter(
                node_name=args.node_name, param_name="power", param_value="false")
            if result.successful:
                CLI_client.get_logger().info(
                    f'Successfully turned off {args.node_name}.')
            else:
                msg = f'Failed when turning off {args.node_name}'
                if result.reason:
                    msg += ': ' + result.reason
                CLI_client.get_logger().error(f'{msg}!')
    else:
        CLI_client.get_logger().error("Usage: ros2 kios turn_off \"<node_name>\"")
    rclpy.shutdown()


def turn_on(args):
    if args.node_name:
        if CLI_client.find_node(args.node_name):
            CLI_client.get_logger().error(
                f'FAILED: Node {args.node_name} not found!')
        else:
            result = CLI_client.set_parameter(
                node_name=args.node_name, param_name="power", param_value="true")
            if result.successful:
                CLI_client.get_logger().info(
                    f'Successfully turned on {args.node_name}.')
            else:
                msg = f'Failed when turning on {args.node_name}'
                if result.reason:
                    msg += ': ' + result.reason
                CLI_client.get_logger().error(f'{msg}!')
    else:
        CLI_client.get_logger().error("Usage: ros2 kios turn_on \"<node_name>\"")
    rclpy.shutdown()
