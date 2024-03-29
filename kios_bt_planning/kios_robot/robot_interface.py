"""
a robot interface for robot behavior control.
some functionaltiy is tailored for mios.
"""

from kios_utils.task import *
import numpy as np
from typing import Any, List, Dict

from kios_bt.data_types import Action

from kios_robot.robot_proprioceptor import RobotProprioceptor
from kios_robot.mios_task_factory import MiosTaskFactory
from kios_robot.data_types import TaskScene, MiosInterfaceResponse


# # * use localhost when running mios locally.
# MIOS = "127.0.0.1"
# # * use docker ip when running mios in docker.
# MIOS_DOCKER = "10.157.175.17"


class RobotInterface:
    robot_address: str = None
    robot_port: int = None

    proprioceptor: RobotProprioceptor = None
    mios_task_factory: MiosTaskFactory = None
    # actuator: RobotActuator = None

    # robot_status: RobotStatus = None
    task_scene: TaskScene = None

    def __init__(self, robot_address: str = None, robot_port: int = None):
        if robot_address is not None:
            self.robot_address = robot_address
        else:
            self.robot_address = "127.0.0.1"

        if robot_port is not None:
            self.robot_port = robot_port
        else:
            self.robot_port = 12000

        # assert self.test_connection() == True

        self.initialize()

    def initialize(self):
        self.proprioceptor = RobotProprioceptor(self.robot_address, self.robot_port)
        self.mios_task_factory = MiosTaskFactory(self.task_scene, robot_interface=self)

    def mios_setup(self):
        pass
        # dummy_object = self.proprioceptor.get_dummy_object()
        # set the tool objects

    def setup_scene(self, task_scene: TaskScene):
        self.task_scene = task_scene
        self.mios_task_factory.setup_scene(task_scene)
        # teach the scene to mios

    def test_connection(self) -> bool:
        response = call_method(self.robot_address, self.robot_port, "test_connection")
        mios_response = MiosInterfaceResponse.from_json(response["result"])
        print(mios_response)
        return mios_response.has_finished

    # * BBCORE
    def generate_robot_command(self, action: Action, shared_data: Any):
        """
        shard data is shared between the action node and the robot command thread.
        """
        from kios_robot.robot_command import RobotCommand

        robot_command = RobotCommand(
            robot_address=self.robot_address,
            robot_port=self.robot_port,
            shared_data=shared_data,
            task_scene=self.task_scene,
            robot_interface=self,  # ! LET'S HACK!
        )
        """core method. 
        generate a robot command from an action. load the shared data into the command for possible use.

        Raises:
            Exception: ...

        Returns:
            RobotCommand: the robot command for the action node to execute.
        """
        # ! hack
        tasks = self.mios_task_factory.generate_tasks(
            action=action, shared_data=shared_data
        )
        if tasks is not None and len(tasks) > 0:
            robot_command.add_tasks(tasks)
        else:
            raise Exception("Action to robot command: None task is generated!")

        return robot_command

    # # * robot command tests
    # def load_tool(self, robot: str, tool_name: str) -> RobotCommand:
    #     print("todo: check the tool in the scene.")
    #     robot_command = RobotCommand(
    #         robot_address=self.robot_address,
    #         robot_port=self.robot_port,
    #         robot_scene=self.task_scene,
    #     )
    #     robot_command.add_mios_task(
    #         self.mios_task_factory.generate_load_tool(tool_name)
    #     )
    #     print("todo: add change robot TCP")
    #     return robot_command

    # def unload_tool(self, robot: str, tool_name: str):
    #     print("todo: check the tool in the scene.")
    #     robot_command = RobotCommand(
    #         robot_address=self.robot_address,
    #         robot_port=self.robot_port,
    #         robot_scene=self.task_scene,
    #     )
    #     robot_command.add_mios_task(
    #         self.mios_task_factory.generate_unload_tool(tool_name)
    #     )
    #     print("todo: change robot status TCP")
    #     return robot_command

    # def pick(self, object_name: str):
    #     robot_command = RobotCommand(
    #         robot_address=self.robot_address,
    #         robot_port=self.robot_port,
    #         robot_scene=self.task_scene,
    #     )
    #     robot_command.add_mios_task(self.mios_task_factory.generate_pick(object_name))
    #     robot_command.add_mios_task(self.mios_task_factory.generate_move_to_object())
    #     robot_command.add_mios_task(self.mios_task_factory.generate_gripper_grasp())
    #     robot_command.add_mios_task(self.mios_task_factory.generate_())
    #     return robot_command
