from kios_utils.task import *
import numpy as np


class RobotProprioceptor:
    robot_address: str = None
    robot_port: int = None

    def __init__(self, robot_address: str, robot_port: int):
        if robot_address is not None:
            self.robot_address = robot_address
        else:
            raise Exception("robot_address is not set")

        if robot_port is not None:
            self.robot_port = robot_port
        else:
            raise Exception("robot_port is not set")

    def test_connection(self):
        return call_method(self.robot_address, self.robot_port, "test_connection")

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

    def modify_object_position(self, object_name: str, DeltaT: np.ndarray):
        position = self.get_robot_pose_T()
        new_x = position[0] + DeltaT[0]
        new_y = position[1] + DeltaT[1]
        new_z = position[2] + DeltaT[2]

        R = self.get_robot_pose_R().T.flatten().tolist()

        payload = {
            "object": object_name,
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
