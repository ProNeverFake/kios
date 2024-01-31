from kios_utils.task import *
import numpy as np
from typing import Any, List, Dict

from kios_robot.data_types import (
    MiosObject,
    MiosInterfaceResponse,
    RobotState,
    TaskScene,
)

from kios_scene.mongodb_interface import MongoDBInterface


# from kios_robot.robot_status import RobotStatus


class RobotProprioceptor:
    robot_address: str = None
    robot_port: int = None

    mongodb_interface: MongoDBInterface = None
    # robot_status: RobotStatus = None

    def __init__(self, robot_address: str, robot_port: int):
        if robot_address is not None:
            self.robot_address = robot_address
        else:
            raise Exception("robot_address is not set")

        if robot_port is not None:
            self.robot_port = robot_port
        else:
            raise Exception("robot_port is not set")

        self.initialize()

    def initialize(self):
        pass
        self.mongodb_interface = MongoDBInterface()
        # self.robot_status = RobotStatus()
        # self.refresh_robot_status()

    def test_connection(self) -> bool:
        response = call_method(self.robot_address, self.robot_port, "test_connection")
        mios_response = MiosInterfaceResponse.from_json(response["result"])
        print(mios_response)
        return mios_response.has_finished

    def get_robot_state(self) -> RobotState:
        response = call_method(self.robot_address, self.robot_port, "get_state")
        mios_response = MiosInterfaceResponse.from_json(response["result"])
        if mios_response.has_finished:
            return RobotState.from_json(response["result"])
        else:
            raise Exception("Robot state is not ready yet.")

    def update_scene_object_from_mios(self, scene: TaskScene, object_name: str):
        mios_object = self.mongodb_interface.query_mios_object(object_name)
        scene.object_map[object_name] = scene.object_map[object_name].from_mios_object(
            mios_object
        )

    def update_scene(self, scene: TaskScene):
        # * tool map does not need to be updated
        # * check the obejct map first
        for object_name, kios_object in scene.object_map.items():
            if kios_object.source == "mios":
                mios_object = self.mongodb_interface.query_mios_object(object_name)
                kios_object = kios_object.from_mios_object(mios_object)
            elif kios_object.source == "pre-defined":
                pass
            elif kios_object.source == "tag_detection":
                raise NotImplementedError
            elif kios_object.source == "segmentation":
                raise NotImplementedError
            else:
                raise Exception("Undefined source!")

        # * then the relation object map. the strategy is to generate them again
        pass

    def teach_object(self, object_name: str):
        return call_method(
            self.robot_address, self.robot_port, "teach_object", {"object": object_name}
        )

    # ! this will lead to a jerk in the robot. dont use it.
    def change_EE_T_TCP(self, new_EE_T_TCP: np.ndarray):
        payload = {
            "EE_T_TCP": new_EE_T_TCP.T.flatten().tolist(),
        }
        return call_method(
            self.robot_address,
            self.robot_port,
            "change_EE_T_TCP",
            payload,
        )

    def get_object_O_T_OB(self, object_name: str):
        """
        based on the investigation, O_T_OB = O_T_EE, EE_T_OB = I
        """
        response = call_method(
            self.robot_address,
            self.robot_port,
            "get_object",
            {"object": object_name},
        )

        return np.reshape(np.array(response["result"]["O_T_OB"]), (4, 4)).T

    def get_dummy_object(self) -> MiosObject or None:
        response = call_method(
            self.robot_address,
            self.robot_port,
            "get_object",
            {"object_name": "NoneObject"},
        )

        mios_response = MiosInterfaceResponse.from_json(response["result"])
        print(mios_response)
        if mios_response.has_finished:
            dummy_object = MiosObject.from_json(response["result"])
            print(dummy_object)
            return dummy_object
        else:
            return None

    def set_object(self, name: str, HT: np.ndarray):
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
