from kios_utils.task import *
import numpy as np
import logging
from termcolor import colored
from pprint import pprint
from typing import Any

from kios_robot.data_types import (
    MiosObject,
    KiosObject,
    MiosInterfaceResponse,
    RobotState,
    TaskScene,
)

from kios_scene.mongodb_interface import MongoDBInterface

from kios_utils.bblab_utils import bb_deprecated, setup_logger

rp_logger = setup_logger(__name__, logging.DEBUG)


class RobotProprioceptor:
    robot_address: str = None
    robot_port: int = None

    mongodb_interface: MongoDBInterface = None

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

    def test_connection(self) -> bool:
        response = call_method(self.robot_address, self.robot_port, "test_connection")
        mios_response = MiosInterfaceResponse.from_json(response["result"])
        pprint(mios_response)
        return mios_response.has_finished

    def get_robot_state(self) -> RobotState:
        response = call_method(self.robot_address, self.robot_port, "get_state")
        mios_response = MiosInterfaceResponse.from_json(response["result"])
        if mios_response.has_finished:
            robot_state = RobotState.from_json(response["result"])
            pprint(robot_state)
            return robot_state
        else:
            raise Exception("Robot state is not ready yet.")

    def update_scene_object_from_mios(
        self, scene: TaskScene, object_name: str | list[str]
    ) -> bool:
        if isinstance(object_name, str):
            object_names = [object_name]
        try:
            for object_name in object_names:
                # rp_logger.debug(
                #     f"scene content {object_name}.x before: {scene.object_map[object_name].O_T_TCP[0][3]}"
                # )
                mios_object = self.mongodb_interface.query_mios_object(object_name)

                if scene.object_map.get(object_name) is None:
                    rp_logger.warn(
                        f"object {object_name} is not in the scene. Add it now."
                    )

                scene.object_map[object_name] = KiosObject.from_mios_object(mios_object)
                # rp_logger.debug(
                #     f"scene content {object_name}.x after: {scene.object_map[object_name].O_T_TCP[0][3]}"
                # )

        except Exception as e:
            rp_logger.error(f"Error occurred in the update_scene_object_from_mios: {e}")
            raise e

        return True

    def update_scene_from_vision(self, scene: TaskScene):
        raise NotImplementedError

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

    def teach_object_TCP(self, object_name: str):
        """
        teach an mios object to mios mongodb, which has a O_T_TCP that is calcalated w.r.t. the equipped tool.
        """
        # if you teach the robot directly, the O_T_EE (base to hand) will be recorded under this object name.
        # this method tell mios to take the equipped tool into account
        # the O_T_TCP (base to tool) will be calculated and recorded under this object name.
        # ! after that, don't forget to reset the tool to the default tool.
        response = call_method(
            self.robot_address,
            self.robot_port,
            "teach_object_TCP",
            {"object": object_name},
        )
        mios_response = MiosInterfaceResponse.from_json(response["result"])
        print(mios_response)

    @bb_deprecated(reason="this will lead to a jerk in the robot. dont use it.")
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

    def get_object(self, object: str) -> MiosObject | None:
        response = call_method(
            self.robot_address,
            self.robot_port,
            "get_object",
            {"object_name": object},
        )
        mios_response = MiosInterfaceResponse.from_json(response["result"])
        pprint(mios_response)
        if mios_response.has_finished:
            mios_object = MiosObject.from_json(response["result"])
            pprint(mios_object)
            return mios_object

    def align_object(self, object_name: str, **kwargs: dict[str, Any]):
        """
        cheat method for BB usecase
        """
        self.modify_object_position(
            object_name=object_name,
            x=kwargs["x"] if "x" in kwargs.keys() else 0,
            y=kwargs["y"] if "y" in kwargs.keys() else 0,
            z=kwargs["z"] if "z" in kwargs.keys() else 0,
            R=[1, 0, 0, 0, -1, 0, 0, 0, -1],
        )

    def modify_object_position(self, object_name: str, **kwargs: dict[str, Any]):
        this_object = self.get_object(object_name)
        O_T_EE = this_object.O_T_OB
        O_T_TCP = this_object.O_T_TCP
        if "x" in kwargs.keys():
            O_T_EE[0, 3] += kwargs["x"]
            O_T_TCP[0, 3] += kwargs["x"]
        if "y" in kwargs.keys():
            O_T_EE[1, 3] += kwargs["y"]
            O_T_TCP[1, 3] += kwargs["y"]
        if "z" in kwargs.keys():
            O_T_EE[2, 3] += kwargs["z"]
            O_T_TCP[2, 3] += kwargs["z"]
        if "R" in kwargs.keys():
            R = np.reshape(kwargs["R"], (3, 3)).T
            O_T_EE[0:3, 0:3] = R
            O_T_TCP[0:3, 0:3] = R

        this_object_json = this_object.to_json(this_object)
        this_object_json["object"] = object_name

        response = call_method(
            self.robot_address,
            self.robot_port,
            "set_object",
            this_object_json,
        )

        mios_response = MiosInterfaceResponse.from_json(response["result"])
        pprint(colored(f"mios replied:\n {mios_response}", "green"))

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

    def get_dummy_object(self) -> MiosObject | None:
        response = call_method(
            self.robot_address,
            self.robot_port,
            "get_object",
            {"object_name": "NoneObject"},
        )

        mios_response = MiosInterfaceResponse.from_json(response["result"])
        pprint(mios_response)
        if mios_response.has_finished:
            dummy_object = MiosObject.from_json(response["result"])
            pprint(dummy_object)
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
