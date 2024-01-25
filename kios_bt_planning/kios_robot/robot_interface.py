"""
a robot interface for robot behavior control.
some functionaltiy is tailored for mios.
"""

from kios_utils.task import *
import numpy as np

from kios_robot.robot_proprioceptor import RobotProprioceptor
from kios_robot.robot_skill_engine import RobotSkillEngine
from kios_robot.robot_actuator import RobotActuator
from kios_robot.data_types import TaskScene


# # * use localhost when running mios locally.
# MIOS = "127.0.0.1"
# # * use docker ip when running mios in docker.
# MIOS_DOCKER = "10.157.175.17"


class RobotInterface:
    robot_address: str = None
    robot_port: int = None

    proprioceptor: RobotProprioceptor = None
    skill_engine: RobotSkillEngine = None
    actuator: RobotActuator = None

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

        self.initialize()

    def initialize(self):
        self.proprioceptor = RobotProprioceptor(self.robot_address, self.robot_port)
        self.actuator = RobotActuator(self.robot_address, self.robot_port)

        self.skill_engine = RobotSkillEngine(self.actuator)

    def setup_scene(self, task_scene: TaskScene):
        self.task_scene = task_scene
        # teach the scene to mios

    def test_connection(self):
        return call_method(self.robot_address, self.robot_port, "test_connection")
