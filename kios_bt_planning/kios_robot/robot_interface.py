"""
a robot interface for robot behavior control.
some functionaltiy is tailored for mios.
"""

from kios_utils.task import *
import numpy as np
from typing import Any, List, Dict

from kios_robot.robot_proprioceptor import RobotProprioceptor
from kios_robot.robot_skill_engine import RobotSkillEngine
from kios_robot.robot_actuator import RobotActuator


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

    def test_connection(self):
        return call_method(self.robot_address, self.robot_port, "test_connection")

    def map_action(self, action: Any):
        '''
        map the bt action to an corresponding robot action in the skill engine.
        '''
        raise NotImplementedError



    ###################################################################################
    # TODO the following functions should be sorted out later. most of them should be moved to proprioceptor.
    def get_robot_state(self):
        return call_method(self.robot_address, self.robot_port, "get_state")

    def get_robot_q(self):
        robot_state = self.get_robot_state()
        return robot_state["result"]["q"]

    def get_robot_O_T_EE(self):
        robot_state = self.get_robot_state()
        return np.reshape(np.array(robot_state["result"]["O_T_EE"]), (4, 4)).T

    def get_robot_pose_R(self):
        return self.get_robot_O_T_EE()[0:3, 0:3]

    def get_robot_pose_T(self):
        return self.get_robot_O_T_EE()[0:3, 3]

    def modify_object_position(self, name: str, DeltaT: np.ndarray):
        position = self.get_robot_pose_T()
        new_x = position[0] + DeltaT[0]
        new_y = position[1] + DeltaT[1]
        new_z = position[2] + DeltaT[2]

        R = self.get_robot_pose_R().T.flatten().tolist()

        payload = {
            "object": name,
            "data": {
                "x": new_x,
                "y": new_y,
                "z": new_z,
                "R": R,
            },
        }
        return call_method(
            self.robot_address, self.robot_port, "set_partial_object_data", payload
        )

    def get_robot_state(self):
        return call_method(self.robot_address, self.robot_port, "get_state")

    def get_robot_q(self):
        robot_state = self.get_robot_state()
        return robot_state["result"]["q"]

    def get_robot_O_T_EE(self):
        robot_state = self.get_robot_state()
        return np.reshape(np.array(robot_state["result"]["O_T_EE"]), (4, 4)).T

    def modify_object(self, name: str, HT: np.ndarray):
        payload = {
            "object": name,
            "data": {
                "x": HT[0, 3],
                "y": HT[1, 3],
                "z": HT[2, 3],
                "R": HT[0:3, 0:3].T.flatten().tolist(),
            },
        }
        return call_method(
            self.robot_address, self.robot_port, "set_partial_object_data", payload
        )

    def teach_object(robot: str, object_name: str):
        call_method(robot, 12000, "teach_object", {"object": object_name})
