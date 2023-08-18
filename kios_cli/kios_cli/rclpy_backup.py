 # # ! BUG DISCARDED
    # def get_node_parameter(self, node_name, param_name):
    #     get_param_cli_name = "/" + node_name + "/" + "get_parameters"
    #     self.client = self.create_client(
    #         GetParameters, get_param_cli_name
    #     )
    #     while not self.client.wait_for_service(timeout_sec=1.0):
    #         self.get_logger().error(
    #             f'FAILED: node {node_name} is alive but get_parameters srv is not avaliable!')
    #         rclpy.shutdown()
    #     request = GetParameters.Request()
    #     request.names.append(param_name)
    #     future = self.client.call_async(request)
    #     rclpy.spin_until_future_complete(self, future)
    #     return future.result().values[0]
    # # ! BUG DISCARDED
    # def set_node_parameter(self, node_name, param_name, param_value):
    #     set_param_cli_name = "/" + node_name + "/" + "set_parameters_atomically"
    #     self.client = self.create_client(
    #         SetParametersAtomically, set_param_cli_name
    #     )
    #     while not self.client.wait_for_service(timeout_sec=1.0):
    #         self.get_logger().error(
    #             f'FAILED: node {node_name} is alive but set_parameters_atomically srv is not avaliable!')
    #         rclpy.shutdown()
    #     request = SetParametersAtomically.Request()

    #     parameter_value_msg = ParameterValue()
    #     parameter_value_msg.bool_value = param_value

    #     parameter_msg = Parameter()
    #     parameter_msg.name = param_name
    #     parameter_msg.value = parameter_value_msg

    #     request.parameters = [parameter_msg]
    #     future = self.client.call_async(request)
    #     rclpy.spin_until_future_complete(self, future)
    #     return future.result()
    # ! BUG DISCARDED
    # def switch_power(self, node_name: str, turn_on: bool):

    #     param = self.get_node_parameter(node_name, "power")

    #     if param.bool_value == turn_on:  # to turn on/off a node that is already on/off
    #         if turn_on:
    #             state = "on"
    #         else:
    #             state = "off"
    #         self.get_logger().info(f'The node {node_name} is already {state}.')
    #         return
    #     elif param.bool_value != turn_on:  # need to switch on/off

    #         if turn_on == True:  # to switch on
    #             response = self.set_node_parameter(node_name, "power", True)
    #             result = response.result
    #             if result.successful == True:
    #                 self.get_logger().info(
    #                     f'The node {node_name} is successfully turned on.')
    #                 return
    #             else:
    #                 self.get_logger().error(
    #                     f'Failed when turning on the node {node_name}.')
    #                 self.get_logger().error(
    #                     f'Failure reason: {result.reason}.')
    #                 return

    #         if turn_on == False:  # to switch off
    #             response = self.set_node_parameter(node_name, "power", False)
    #             result = response.result
    #             if result.successful == True:
    #                 self.get_logger().info(
    #                     f'The node {node_name} is successfully turned off.')
    #                 return
    #             else:
    #                 self.get_logger().error(
    #                     f'Failed when turning off the node {node_name}.')
    #                 self.get_logger().error(
    #                     f'Failure reason: {result.reason}.')
    #                 return
    #     else:
    #         self.get_logger().error(
    #             f'The parameter [power] of {node_name} was not read correctly. (TODO change the msg)')
    #         return
    # ! BUG DISCARDED
    # def just_turn_off(self, node_name: str):

    #     set_param_cli_name = "/" + node_name + "/" + "set_parameters_atomically"
    #     self.client = self.create_client(
    #         SetParametersAtomically, set_param_cli_name
    #     )
    #     while not self.client.wait_for_service(timeout_sec=1.0):
    #         self.get_logger().error(
    #             f'FAILED: node {node_name} is alive but set_parameters_atomically srv is not avaliable!')
    #         rclpy.shutdown()
    #     request = SetParametersAtomically.Request()

    #     value = ParameterValue()
    #     value.bool_value = False

    #     parameter_msg = Parameter()
    #     parameter_msg.name = "power"
    #     parameter_msg.value = value

    #     request.parameters = [parameter_msg]
    #     future = self.client.call_async(request)
    #     rclpy.spin_until_future_complete(self, future)
    #     result = future.result().result

    #     if result.successful == True:
    #         self.get_logger().info(
    #             f'The node {node_name} is successfully turned off.')
    #         return
    #     else:
    #         self.get_logger().error(
    #             f'Failed when turning off the node {node_name}.')
    #         self.get_logger().error(
    #             f'Failure reason: {result.reason}.')
    #         return
