"""
a robot interface for robot behavior control.
some functionaltiy is tailored for mios.
"""

from kios_utils.task import *
import numpy as np

from kios_bt.data_types import Action

from kios_robot.robot_proprioceptor import RobotProprioceptor
from kios_robot.mios_task_factory import MiosTaskFactory
from kios_robot.data_types import TaskScene, MiosInterfaceResponse


class RobotInterface:
    robot_address: str = None
    robot_port: int = None

    proprioceptor: RobotProprioceptor = None
    mios_task_factory: MiosTaskFactory = None

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

    def setup_scene(self, task_scene: TaskScene):
        self.task_scene = task_scene
        self.mios_task_factory.setup_scene(task_scene)

    def test_connection(self) -> bool:
        response = call_method(self.robot_address, self.robot_port, "test_connection")
        mios_response = MiosInterfaceResponse.from_json(response["result"])
        print(mios_response)
        return mios_response.has_finished

    # * BBCORE
    def generate_robot_command(self, action: Action, shared_data):
        """
        shard data is shared between the action node and the robot command thread.
        """
        from kios_robot.robot_command import RobotCommand

        robot_command = RobotCommand(
            robot_address=self.robot_address,
            robot_port=self.robot_port,
            shared_data=shared_data,
            task_scene=self.task_scene,
            robot_interface=self,  # TODO someone comes to refactor this part plz!
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
