from typing import Any
import re
import numpy as np
import logging

from kios_scene.mongodb_interface import MongoDBInterface
from kios_utils.bblab_utils import bb_deprecated

from kios_robot.data_types import (
    MiosCall,
    MiosSkill,
    KiosCall,
    Toolbox,
    TaskScene,
    MiosObject,
)

# * only for demo
from kios_robot.demo_solution import assembly_dictionary

from kios_bt.data_types import Action
from kios_utils.bblab_utils import setup_logger
from kios_utils.exceptions import FormatException, ParsingException, CallException

"""
BB knows this is not a good design to create generate method for each task/call. 
A better implementation can be a common interface for generating mios tasks,
which takes formatted node data as input. 
Anyway, the current implementation is straightforward and easy to understand.
"""

mti_logger = setup_logger(
    __name__,
    # logging.DEBUG,
)


class MiosTaskFactory:
    task_scene: TaskScene
    robot_interface: Any

    def __init__(
        self,
        task_scene: TaskScene = None,
        robot_interface: Any = None,
    ):
        self.task_scene = task_scene
        self.robot_interface = robot_interface

    def initialize(self):
        pass

    def setup_scene(self, task_scene: TaskScene):
        self.task_scene = task_scene

    def parse_action(self, action: Action) -> dict[str, Any]:
        """parse the action string to get the action name and arguments.

        Args:
            action(Action).name: action_name(arg1, arg2, ...)

        Returns:
            Dict[str, Any]:
                action_name: str
                arges: [arg1, arg2, arg3, ...]

        """
        # use re to parse the action
        pattern = r"(?:action:\s)?(\w+):?\s*\(([\w\s,]+)\)"
        action_string = action.name
        match = re.match(pattern, action_string)
        if match:
            action_name = match.group(1)  # Extract the action name from the match
            args = [
                arg.strip() for arg in match.group(2).split(",")
            ]  # Extract and clean up the arguments
            return {
                "name": action_name,
                "args": args,
            }  # Return a dictionary with the action name and arguments
        else:
            raise ParsingException(
                f'Action "{action_string}" is not in the correct format.'
            )  # Raise an exception if the action format is incorrect
            # return {"action_name": None, "args": []}

    # * BBCORE
    def generate_tasks(
        self, action: Action, shared_data: Any = None
    ) -> list[MiosCall | MiosSkill]:
        """core function.
        generate mios tasks from a kios action and return them in a list.

        Args:
            action (Action): _description_

        Returns:
            List[MiosCall or MiosSkill]: _description_
        """
        parsed_action = self.parse_action(action)

        if parsed_action["name"] == "load_tool":
            return self.generate_load_tool_skill(parsed_action)
        elif parsed_action["name"] == "unload_tool":
            return self.generate_unload_tool_skill(parsed_action)
        elif parsed_action["name"] == "change_tool":
            return self.generate_change_tool_skill(parsed_action)
        elif parsed_action["name"] == "pick_up":
            return self.generate_pick_up_skill(parsed_action)
        elif parsed_action["name"] == "screw":
            return self.generate_screw_skill(parsed_action)
        elif parsed_action["name"] == "put_down":
            return self.generate_put_down_skill(parsed_action)
        elif parsed_action["name"] == "place":
            return self.generate_place_skill(parsed_action)
        elif parsed_action["name"] == "insert":
            return self.generate_insert_skill(parsed_action)
        else:
            raise ParsingException(
                f"action {parsed_action['name']} is not supported yet."
            )

    @bb_deprecated(reason="preserved but not used")
    def generate_fake_mios_tasks(self, action: Action) -> list[MiosCall | MiosSkill]:
        """
        test function.
        """
        return [
            MiosCall(
                method_name="test_method",
                method_payload={"test": "test"},
            )
        ]

    ###################################################################
    # * kios call methods
    def generate_kios_dummy_call(self) -> KiosCall:
        return KiosCall(
            method=print,
            args=["this is a dummy call from kios!"],
        )

    def generate_update_object_from_mios_call(self, object_name: str) -> KiosCall:
        """update the object in kios from mios.

        Args:
            object_name (str): _description_

        Returns:
            KiosCall: _description_
        """
        return KiosCall(
            method=self.robot_interface.proprioceptor.update_scene_object_from_mios,
            args=[self.task_scene, object_name],
        )

    ###################################################################
    # * methods to generate one mios task
    def generate_teach_O_T_TCP_call(self, object_name: str) -> MiosCall:
        """teach the O_T_TCP to mios (well, O_T_EE and q as well).
        This is different from the teach_object_call because the O_T_TCP can change when the robot is using a toolcube.

        Args:
            object_name (str): _description_

        Returns:
            MiosCall: _description_
        """
        payload = {
            "object": object_name,
        }
        return MiosCall(method_name="teach_object_TCP", method_payload=payload)

    def generate_teach_object_call(self, object_name: str) -> MiosCall:
        """teach the current cartesian and joint pose to mios.
        this includes q and O_T_EE.

        Args:
            object_name (str): _description_

        Returns:
            MiosCall: _description_
        """
        payload = {
            "object": object_name,
        }
        return MiosCall(method_name="teach_object", method_payload=payload)

    @bb_deprecated(reason="only for test. mios should take over this", can_run=True)
    def generate_set_load_call(self, load_mass: float) -> MiosCall:
        """MIOS_CALL
        change the load of the object the robot is holding.
        using the libfranka API.
        used here for changing toolcube mass.

        Args:
            load_mass: the mass of the object in kg.

        Returns:
            _type_: _description_
        """
        payload = {
            "load_mass": load_mass,
        }
        return MiosCall(method_name="set_load", method_payload=payload)

    def generate_update_mios_memory_environment_call(self) -> MiosCall:
        """call mios to update the memory "environment" from the mongodb.

        Returns:
            MiosCall: the call to update the mios memory environment
        """
        payload = {}
        return MiosCall(
            method_name="update_memory_environment",
            method_payload=payload,
            isTrivial=True,
        )

    def generate_move_above_mp(self, object_name: str) -> MiosSkill:
        """
        move to 15cm above the object in the z axis of the object coordinate system.
        """
        # default: 15cm above the object
        # get the object from the scene
        # mti_logger.warn(
        #     f"scene content {object_name} x: {self.task_scene.object_map[object_name].O_T_TCP[2][0]}"
        # )
        kios_object = self.task_scene.get_object(object_name)

        # get the O_T_EE and modify it to be 15cm above the object
        object_O_T_TCP = kios_object.O_T_TCP
        above_O_T_TCP = object_O_T_TCP @ np.array(
            [
                [1, 0, 0, 0],
                [0, 1, 0, 0],
                [0, 0, 1, -0.15],
                [0, 0, 0, 1],
            ]
        )

        return self.generate_cartesian_move_mp(O_T_TCP=above_O_T_TCP)

    def generate_reach_mp(self, object_name: str) -> MiosSkill:
        """
        move to the object.
        """
        # get the object from the scene
        kios_object = self.task_scene.get_object(object_name)

        # get the O_T_EE and modify it to be 5cm above the object
        object_O_T_TCP = kios_object.O_T_TCP

        # generate the cartesian move
        return self.generate_cartesian_move_mp(O_T_TCP=object_O_T_TCP)

    def generate_cartesian_move_mp(
        self, object_name: str = None, O_T_TCP: np.ndarray = None, dx_d=None, ddx_d=None
    ) -> MiosSkill:
        """make the TCP of the robot move to the pose defined by the mios object or the O_T_TCP.
        IMPORTANT: Here the robot is asked to move to the TCP pose.
        the O_T_EE for cartesian motion will be calculated by mios.

        Args:
            object_name (str, optional): _description_. Defaults to None.
            O_T_TCP (np.ndarray, optional): _description_. Defaults to None.

        Raises:
            Exception: _description_

        Returns:
            MiosSkill: _description_
        """
        if object_name is None and O_T_TCP is None:
            raise CallException(
                f"Neither object_name nor O_T_TCP is set for function {self.generate_cartesian_move_mp.__name__}!"
            )
        if object_name:
            context = {
                "skill": {
                    "p0": {
                        "dX_d": dx_d if dx_d is not None else [0.1, 1],
                        "ddX_d": ddx_d if ddx_d is not None else [0.5, 1],
                        "K_x": [1500, 1500, 1000, 150, 150, 150],
                    },
                    "objects": {"GoalPose": object_name},
                },
                "control": {"control_mode": 0},
            }
        else:
            context = {
                "skill": {
                    "p0": {
                        "dX_d": dx_d if dx_d is not None else [0.5, 1],
                        "ddX_d": ddx_d if ddx_d is not None else [0.3, 1],
                        "K_x": [1500, 1500, 1500, 150, 150, 150],
                        "T_T_EE_g": O_T_TCP.T.flatten().tolist(),  # ! T_T_EE IS ACTUALLY O_T_TCP!
                    },
                },
                "control": {"control_mode": 0},
            }

        return MiosSkill(
            skill_name="cartesian_move",
            skill_type="KiosCartesianMove",
            skill_parameters=context,
            # retry=True,
        )

    @bb_deprecated(reason="not for task space operation", can_run=True)
    def generate_joint_move_mp(self, joint_location: str) -> MiosSkill:
        """
            * available. debug: task context error, "skill", "control" and "user" are on the same level.
            move to a joint position.
        Args:
            joint_location (str): the stored Taskframe_HomogeneousT_EE_goal in mongo DB
        """
        context = {
            "skill": {
                "speed": 1,
                "acc": 0.8,
                "q_g": [0, 0, 0, 0, 0, 0, 0],
                "q_g_offset": [0, 0, 0, 0, 0, 0, 0],
                "objects": {"goal_pose": joint_location},
            },
            "control": {"control_mode": 3},
            "user": {"env_X": [0.005, 0.005, 0.005, 0.0175, 0.0175, 0.0175]},
        }

        return MiosSkill(
            skill_name="joint_move",
            skill_type="MoveToPoseJoint",
            skill_parameters=context,
        )

    def generate_gripper_grasp_mp(
        self, width=0.01, speed=0.2, force=50, epsilon_inner=0.05, epsilon_outer=0.05
    ) -> MiosCall:
        payload = {
            "width": width,
            "speed": speed,
            "force": force,
            "epsilon_inner": epsilon_inner,
            "epsilon_outer": epsilon_outer,
        }
        return MiosCall(method_name="grasp", method_payload=payload)

    @bb_deprecated(reason="meaningless", can_run=True)
    def generate_gripper_release_mp(self, width=0.08, speed=0.2) -> MiosCall:
        """
        the gripper limit is set in mios. when a gripper is equipped, it should be 0.4
        """
        return self.generate_gripper_move_mp(width=width, speed=speed)

    def generate_gripper_move_mp(
        self,
        width: float,
        speed: float = 0.05,
    ) -> MiosCall:
        payload = {
            "width": width,
            "speed": speed,
        }
        return MiosCall(method_name="move_gripper", method_payload=payload)

    def generate_gripper_home_mp(self) -> MiosCall:
        payload = {}
        return MiosCall(method_name="home_gripper", method_payload=payload)

    # @bb_deprecated(reason="screw is not ready now in mios")
    # def generate_screw_in_mp(self, object_name: str = None, O_T_OB=None) -> MiosSkill:
    #     if object_name is None and O_T_OB is None:
    #         raise Exception("Object target is not set!")
    #     if object_name:
    #         payload = {
    #             "skill": {
    #                 "p0": {
    #                     "dX_d": [0.1, 0.5],
    #                     "ddX_d": [0.5, 1],
    #                     # "O_T_OB":
    #                     "K_x": [1500, 1500, 1500, 150, 150, 150],
    #                     "F_ff": [0, 0, 0, 0, 0, 2],
    #                 },
    #                 "objects": {"Container": object_name},
    #             },
    #             "control": {"control_mode": 0},
    #         }
    #     else:
    #         payload = {
    #             "skill": {
    #                 "p0": {
    #                     "dX_d": [0.1, 0.3],
    #                     "ddX_d": [0.5, 0.5],
    #                     "K_x": [
    #                         1000,
    #                         1000,
    #                         1000,
    #                         500,
    #                         500,
    #                         100,
    #                     ],  # ! is EE_k_x if you don't set frame!!!
    #                     "O_T_OB": O_T_OB.T.flatten().tolist(),
    #                     "F_ff": [0, 0, 3, 0, 0, -10],
    #                 },
    #                 # "objects": {"GoalPose": "NullObject"},
    #             },
    #             "control": {"control_mode": 0},
    #         }

    #     return MiosSkill(
    #         skill_name="screw_in",
    #         skill_type="KiosScrewIn",
    #         skill_parameters=payload,
    #     )

    @bb_deprecated(reason="rapid implimentation, DONT USE THIS", can_run=False)
    def generate_screw_in_mp(self, object_name: str = None, O_T_OB=None) -> MiosSkill:
        if object_name is None and O_T_OB is None:
            raise Exception("Object target is not set!")
        if object_name:
            payload = {
                "skill": {
                    "p0": {
                        "dX_d": [0.1, 0.5],
                        "ddX_d": [0.5, 1],
                        # "O_T_OB":
                        "K_x": [1500, 1500, 1500, 150, 150, 150],
                        "F_ff": [0, 0, 0, 0, 0, 2],
                    },
                    "objects": {"Container": object_name},
                },
                "control": {"control_mode": 0},
            }
        else:
            payload = {
                "skill": {
                    "p0": {
                        "dX_d": [0.1, 0.3],
                        "ddX_d": [0.5, 0.5],
                        "K_x": [
                            1000,
                            1000,
                            1000,
                            500,
                            500,
                            100,
                        ],  # ! is EE_k_x if you don't set frame!!!
                        "O_T_OB": O_T_OB.T.flatten().tolist(),
                        "F_ff": [0, 0, 3, 0, 0, -10],
                    },
                    # "objects": {"GoalPose": "NullObject"},
                },
                "control": {"control_mode": 0},
            }

        return MiosSkill(
            skill_name="screw_in",
            skill_type="KiosScrewIn",
            skill_parameters=payload,
        )

    @bb_deprecated(reason="demo only")
    def generate_insert_mp_mod(
        self, insertable: str, container: str, param: list[str, Any] = None
    ) -> MiosCall:
        if container is None:
            raise Exception("container is not set!")

        # get the container from the scene
        kios_object = self.task_scene.get_object(container)

        if param is None:
            param = {
                "search_a": [2, 2, 1, 1, 1, 0],
                "search_f": [1, 1, 1, 1.5, 1.5, 0],
                "F_ext_contact": [13.0, 2.0],
                "f_push": [0, 0, 5, 0, 0, 0],
                "K_x": [300, 300, 500, 500, 500, 800],
                "D_x": [0.7, 0.7, 0.7, 0.7, 0.7, 0.7],
            }

        search_a = (
            param["search_a"] if "search_a" in param.keys() else [2, 2, 1, 1, 1, 0]
        )
        search_f = (
            param["search_f"] if "search_f" in param.keys() else [1, 1, 1, 1.5, 1.5, 0]
        )
        F_ext_contact = (
            param["F_ext_contact"] if "F_ext_contact" in param.keys() else [13.0, 2.0]
        )
        f_push = param["f_push"] if "f_push" in param.keys() else [0, 0, 5, 0, 0, 0]
        K_x = param["K_x"] if "K_x" in param.keys() else [300, 300, 500, 500, 500, 800]
        D_x = param["D_x"] if "D_x" in param.keys() else [0.7, 0.7, 0.7, 0.7, 0.7, 0.7]

        O_T_TCP = kios_object.O_T_TCP
        payload = {
            "skill": {
                "objects": {
                    # "Container": container,
                },
                # "time_max": 35,
                "p0": {
                    "O_T_TCP": O_T_TCP.T.flatten().tolist(),
                    "dX_d": [0.05, 0.3],
                    "ddX_d": [0.5, 1],
                    "DeltaX": [0, 0, 0, 0, 0, 0],
                    "K_x": [1500, 1500, 1500, 600, 600, 600],
                },
                "p1": {
                    "dX_d": [0.05, 0.1],
                    "ddX_d": [0.1, 0.05],
                    "K_x": [1500, 1500, 500, 800, 800, 800],
                },
                "p2": {
                    # "search_a": [10, 10, 0, 2, 2, 0],
                    # "search_f": [1, 1, 0, 1.2, 1.2, 0],
                    "search_a": search_a,
                    "search_f": search_f,
                    "search_phi": (
                        param["search_phi"]
                        if "search_phi" in param.keys()
                        else [
                            0,
                            3.14159265358979323846 / 2,
                            0,
                            3.14159265358979323846 / 2,
                            0,
                            0,
                        ]
                    ),
                    "K_x": K_x,
                    "D_x": D_x,
                    "f_push": f_push,
                    # "dX_d": [0.1, 0.5],
                    # "ddX_d": [0.5, 1],
                    "dX_d": [0.03, 0.3],
                    "ddX_d": [0.3, 1],
                },
                "p3": {
                    "dX_d": [0.1, 0.5],
                    "ddX_d": [0.5, 1],
                    "f_push": 7,
                    "K_x": [500, 500, 0, 800, 800, 800],
                },
            },
            "control": {"control_mode": 0},
            "user": {
                "env_X": (
                    param["env_X"]
                    if "env_X" in param.keys()
                    else [0.01, 0.01, 0.002, 0.05, 0.05, 0.05]
                ),
                "env_dX": [0.001, 0.001, 0.001, 0.005, 0.005, 0.005],
                "F_ext_contact": F_ext_contact,
                "tau_ext_max": [100, 100, 100, 100, 40, 40, 40],
            },
        }

        insert = MiosSkill(
            skill_name="insert",
            skill_type="KiosInsertion",
            skill_parameters=payload,
        )

        return insert

    def generate_update_tool_call(self, tool_name: str = None) -> MiosCall:
        """let mios know the tool is loaded and it need to change EE_T_TCP.

        Args:
            tool_name (str, optional): _description_. Defaults to None.

        Returns:
            MiosCall: _description_
        """
        # get the tool from the scene
        if tool_name is None:
            raise CallException(
                f"tool name is not set in {self.generate_update_tool_call.__name__}!"
            )
        else:
            tool = self.task_scene.get_tool(tool_name)
        # get the EE_T_TCP
        EE_T_TCP = tool.EE_T_TCP
        EE_finger_width_max = tool.EE_finger_width_max
        EE_finger_width_min = tool.EE_finger_width_min
        tool_mass = tool.tool_mass

        payload = {
            "EE_T_TCP": EE_T_TCP.T.flatten().tolist(),
            "EE_finger_width_max": EE_finger_width_max,
            "EE_finger_width_min": EE_finger_width_min,
            "tool_mass": tool_mass,
        }
        return MiosCall(method_name="change_tool", method_payload=payload)

    ###################################################################
    # * methods to generate a sequence of mios tasks

    @bb_deprecated(
        reason="Now we know the bug. This is not used anymore", can_run=False
    )
    def memory_scene_object_synchronize(
        self, object_name: str
    ) -> list[MiosCall | KiosCall]:
        """synchronize the indicated object in both sides.

        Args:
            object_name (str): the name of the object to be synchronized.

        Returns:
            list[MiosCall | KiosCall]: _description_
        """
        return [
            self.generate_update_mios_memory_environment_call(),
            self.generate_update_object_from_mios_call(object_name=object_name),
        ]

    def generate_change_tool_skill(
        self, parsed_action: dict[str, Any]
    ) -> list[MiosCall | MiosSkill]:
        tool1 = parsed_action["args"][1]
        tool2 = parsed_action["args"][2]

        if tool1 is None or tool2 is None:
            raise ParsingException(
                f'tool names are missing in {parsed_action["name"]}! tool1: {tool1}, tool2: {tool2}'
            )

        action1 = {"args": ["xxxx", tool1]}
        unload_skill = self.generate_unload_tool_skill(action1)

        action2 = {"args": ["xxxx", tool2]}
        load_skill = self.generate_load_tool_skill(action2)

        return unload_skill + load_skill

    def generate_load_tool_skill(
        self, parsed_action: dict[str, Any]
    ) -> list[MiosSkill | MiosCall]:

        tool_name = parsed_action["args"][1]
        if tool_name is None:
            raise Exception("tool_name is not set!")

        if tool_name == "defaultgripper":
            return [self.generate_update_tool_call(tool_name)]

        payload = {
            "skill": {
                "objects": {
                    "Tool": tool_name,
                },
                # "time_max": 30,
                "MoveAbove": {
                    "dX_d": [0.5, 1.2],
                    "ddX_d": [0.3, 1],
                    "DeltaX": [0, 0, 0, 0, 0, 0],
                    "K_x": [1500, 1500, 1500, 600, 600, 600],
                },
                "MoveIn": {
                    "dX_d": [0.2, 0.4],
                    "ddX_d": [0.1, 0.1],
                    "DeltaX": [0, 0, 0, 0, 0, 0],
                    "K_x": [1500, 1500, 1500, 600, 600, 600],
                },
                "GripperMove": {
                    "width": 0.04,
                    "speed": 0.1,
                    "force": 70,
                    "K_x": [1500, 1500, 1500, 100, 100, 100],
                    "eps_in": 0.05,
                    "eps_out": 0.05,
                },
                "Retreat": {
                    "dX_d": [0.4, 0.4],
                    "ddX_d": [0.1, 0.1],
                    "DeltaX": [0, 0, 0, 0, 0, 0],
                    "K_x": [1500, 1500, 1500, 600, 600, 600],
                },
            },
            "control": {"control_mode": 0},
        }

        load_tool = MiosSkill(
            skill_name="load_tool",
            skill_type="KiosLoadTool",
            skill_parameters=payload,
        )

        update_tool = self.generate_update_tool_call(tool_name)

        return [
            load_tool,
            update_tool,
        ]

    def generate_unload_tool_skill(
        self, parsed_action: dict[str, Any]
    ) -> list[MiosSkill | MiosCall]:
        tool_name = parsed_action["args"][1]
        if tool_name is None:
            raise Exception("tool_name is not set!")

        payload = {
            "skill": {
                "objects": {
                    "Tool": tool_name,
                },
                "time_max": 30,
                "MoveAbove": {
                    "dX_d": [0.5, 1.2],
                    "ddX_d": [0.3, 1],
                    "DeltaX": [0, 0, 0, 0, 0, 0],
                    "K_x": [1500, 1500, 1500, 600, 600, 600],
                },
                "MoveIn": {
                    "dX_d": [0.4, 0.4],
                    "ddX_d": [0.1, 0.1],
                    "DeltaX": [0, 0, 0, 0, 0, 0],
                    "K_x": [1500, 1500, 1500, 600, 600, 600],
                },
                "GripperMove": {
                    "width": 0.08,
                    "speed": 0.1,
                    "force": 70,
                    "K_x": [1500, 1500, 1500, 100, 100, 100],
                    "eps_in": 0.05,
                    "eps_out": 0.05,
                },
                "Retreat": {
                    "dX_d": [0.4, 0.4],
                    "ddX_d": [0.1, 0.1],
                    "DeltaX": [0, 0, 0, 0, 0, 0],
                    "K_x": [1500, 1500, 1500, 600, 600, 600],
                },
            },
            "control": {"control_mode": 0},
        }

        unload_tool = MiosSkill(
            skill_name="unload_tool",
            skill_type="KiosLoadTool",
            skill_parameters=payload,
        )

        update_tool = self.generate_update_tool_call(tool_name="defaultgripper")

        task = []

        if tool_name != "defaultgripper":
            task.append(unload_tool)

        task.append(update_tool)

        return task

    def generate_pick_up_skill(
        self, parsed_action: dict[str, Any]
    ) -> list[MiosCall | MiosSkill]:

        object_name = parsed_action["args"][2]
        if object_name is None:
            raise ParsingException(
                f'object name is not set in {parsed_action["name"]}!'
            )

        task_list = []
        # move above
        move_above = self.generate_move_above_mp(object_name)
        task_list.append(move_above)
        # move in
        reach = self.generate_reach_mp(object_name)
        task_list.append(reach)
        # grasp
        grasp = self.generate_gripper_grasp_mp()
        task_list.append(grasp)
        # retreat
        retreat = self.generate_move_above_mp(object_name)
        task_list.append(retreat)
        # update object in mios
        update_object_in_mios = self.generate_teach_object_call(object_name)
        task_list.append(update_object_in_mios)
        # update object in kios
        update_object_in_kios = self.generate_update_object_from_mios_call(object_name)
        task_list.append(update_object_in_kios)

        return task_list

    def generate_put_down_skill(
        self, parsed_action: dict[str, Any]
    ) -> list[MiosCall | MiosSkill]:
        object_name = parsed_action["args"][2]
        place = "free_space"
        if object_name is None:
            raise ParsingException(
                f'object name is not set in {parsed_action["name"]}!'
            )

        # move above
        move_above = self.generate_move_above_mp(place)
        # move in
        reach = self.generate_reach_mp(place)
        # release
        release = self.generate_gripper_release_mp()
        # retreat
        retreat = self.generate_move_above_mp(place)
        # update object in mios
        update_object_in_mios = self.generate_teach_object_call(object_name)
        # update object in kios
        update_object_in_kios = self.generate_update_object_from_mios_call(object_name)

        return [
            move_above,
            reach,
            release,
            retreat,
            update_object_in_mios,
            update_object_in_kios,
        ]

    def generate_place_skill(
        self, parsed_action: dict[str, Any]
    ) -> list[MiosCall | MiosSkill]:
        return [self.generate_kios_dummy_call()]

    def generate_screw_skill(
        self, parsed_action: dict[str, Any]
    ) -> list[MiosCall | MiosSkill]:
        screwable = parsed_action["args"][2]
        container = parsed_action["args"][3]
        if container is None:
            raise Exception("container is not set!")

        # kios_object = self.task_scene.get_object(container)

        move_above = self.generate_move_above_mp(container)

        # insert first
        insert = self.generate_insert_mp(insertable=screwable, container=container)

        # screw in
        drive_in = self.generate_drive_in_mp(screwable, container)

        teach_object_in_mios = self.generate_teach_O_T_TCP_call(screwable)

        update_object_in_mios = self.generate_update_mios_memory_environment_call()
        update_object_in_kios = self.generate_update_object_from_mios_call(screwable)

        release = self.generate_gripper_release_mp(width=0.08)

        retreat = self.generate_move_above_mp(container)

        return [
            move_above,
            insert,
            drive_in,
            teach_object_in_mios,
            update_object_in_mios,
            update_object_in_kios,
            release,
            retreat,
        ]

    # def generate_drive_skill(
    #     self, parsed_action: Dict[str, Any]
    # ) -> List[MiosCall | MiosSkill]:
    #     drivable = parsed_action["args"][2]
    #     container = parsed_action["args"][3]

    #     if drivable is None or container is None:
    #         raise Exception(
    #             f'drivable "{drivable}" or container "{container}" is not set!'
    #         )

    #     kios_object = self.task_scene.get_object(container)
    #     if kios_object is None:
    #         raise Exception(f'object "{container}" is not found in the scene!')
    @bb_deprecated(reason="not used")
    def generate_loose_gripper_call(self) -> MiosCall:
        payload = {}
        return MiosCall(method_name="loose_gripper", method_payload=payload)

    @bb_deprecated(reason="TESTING", can_run=True)
    def generate_drive_in_mp(self, drivable, container) -> MiosSkill:
        # ! BUG: the torque end condition doesn't work well. A connection error will be raised at the websocket side.
        if container is None:
            raise Exception("container is not set!")

        # get the container from the scene
        kios_object = self.task_scene.get_object(container)
        if kios_object is not None:
            payload = {
                "skill": {
                    "objects": {
                        # "Container": container,
                    },
                    # "time_max": 60,
                    "p0": {
                        "K_x": [1500, 1500, 1500, 600, 600, 600],
                        "dq_max": 1.5,
                        "ddq_max": 1.5,
                        "tighten_torque": 1.5,
                        "f_push": [
                            0,
                            0,
                            8,
                            0,
                            0,
                            0,
                        ],  # ! well, this won't work since it is inconsistent with the control mode
                        "max_rounds": 4,
                    },
                    "p1": {
                        "K_x": [3000, 3000, 3000, 1200, 1200, 1200],
                    },
                    "p2": {
                        "K_x": [3000, 3000, 3000, 1200, 1200, 1200],
                        "dq_max": 1.5,
                        "ddq_max": 1.5,
                    },
                    "p3": {
                        "grasp_force": 50,
                        "K_x": [3000, 3000, 3000, 800, 800, 800],
                    },
                },
                "control": {"control_mode": 1},
            }

        return MiosSkill(
            skill_name="drivein",
            skill_type="KiosDriveIn",
            skill_parameters=payload,
        )

    @bb_deprecated(reason="not sure if this is ready")
    def generate_drive_out_mp(self, drivable, container) -> MiosSkill:

        if container is None:
            raise Exception("container is not set!")

        # get the container from the scene
        kios_object = self.task_scene.get_object(container)
        if kios_object is not None:
            payload = {
                "skill": {
                    "objects": {
                        # "Container": container,
                    },
                    # "time_max": 60,
                    "p0": {
                        "K_x": [1500, 1500, 3, 600, 600, 0.5],
                        "D_x": [0.7, 0.7, 0.7, 0.7, 0.7, 1000],
                        # "dX_max": 0.3,
                        # "ddX_max": 0.3,
                        "f_drive": [0, 0, 0, 0, 0, 1],
                        "limit_tighten_torque": 60,
                        "f_pull": [0, 0, 8, 0, 0, 0],
                        "z_offset": 0.05,
                    },
                    "p1": {
                        "K_x": [500, 500, 500, 600, 600, 600],
                    },
                    "p2": {
                        "K_x": [500, 500, 500, 800, 800, 800],
                        "dX_max": [0.1, 0.3],
                        "ddX_max": [0.5, 1],
                    },
                    "p3": {
                        "grasp_force": 30,
                        "K_x": [500, 500, 500, 800, 800, 800],
                    },
                },
                "control": {"control_mode": 0},
                "user": {
                    "env_X": [0.01, 0.01, 0.002, 0.05, 0.05, 0.05],
                    "env_dX": [0.001, 0.001, 0.001, 0.005, 0.005, 0.005],
                },
            }

        return MiosSkill(
            skill_name="driveout",
            skill_type="KiosDriveOut",
            skill_parameters=payload,
        )

    # @bb_deprecated(reason="not ready")
    # def generate_drive_in_mp(
    #     self,
    #     drivable,
    #     container,
    #     clockwise: bool = True,
    # ) -> MiosSkill:

    #     if container is None:
    #         raise Exception("container is not set!")

    #     # get the container from the scene
    #     kios_object = self.task_scene.get_object(container)
    #     if kios_object is not None:
    #         payload = {
    #             "skill": {
    #                 "objects": {
    #                     # "Container": container,
    #                 },
    #                 "time_max": 60,
    #                 "p0": {
    #                     "K_x": [1500, 1500, 1500, 600, 600, 600],
    #                     "dq_max": 0.5,
    #                     "ddq_max": 0.6,
    #                     "clockwise": clockwise,
    #                     "tighten_torque": 3,
    #                 },
    #             },
    #             "control": {"control_mode": 1},
    #             "user": {
    #                 "env_X": [0.01, 0.01, 0.002, 0.05, 0.05, 0.05],
    #                 "env_dX": [0.001, 0.001, 0.001, 0.005, 0.005, 0.005],
    #             },
    #         }

    #     return MiosSkill(
    #         skill_name="drivein",
    #         skill_type="KiosDriveIn",
    #         skill_parameters=payload,
    #     )

    """
    The final presentation has been postponed. 
    I'm kind of pissed off. No suffer anymore from now.
    """

    def generate_insert_skill(
        self, parsed_action: dict[str, Any]
    ) -> list[MiosCall | MiosSkill]:
        insertable = parsed_action["args"][2]
        container = parsed_action["args"][3]
        if container is None:
            raise ParsingException(f'container is not set in {parsed_action["name"]}!')
        test_container = self.task_scene.get_object(container)  # check its validity
        # * this only for demo video. cease this to make the action align with the true action context.
        container = assembly_dictionary.get(insertable, None)
        if container is None:
            raise ParsingException(
                f"Cannot find the container for {insertable} in the assembly dictionary!"
            )

        # get the container from the scene
        kios_object = self.task_scene.get_object(container)
        O_T_TCP = kios_object.O_T_TCP

        payload = {
            "skill": {
                "objects": {
                    # "Container": container,
                },
                # "time_max": 40,
                "p0": {
                    "O_T_TCP": O_T_TCP.T.flatten().tolist(),
                    "dX_d": [0.2, 0.3],
                    "ddX_d": [0.5, 1],
                    "DeltaX": [0, 0, 0, 0, 0, 0],
                    "K_x": [1500, 1500, 1500, 600, 600, 600],
                },
                "p1": {
                    "dX_d": [0.08, 0.1],
                    "ddX_d": [0.1, 0.05],
                    "K_x": [1500, 1500, 500, 800, 800, 800],
                },
                "p2": {
                    # "search_a": [10, 10, 0, 2, 2, 0],
                    # "search_f": [1, 1, 0, 1.2, 1.2, 0],
                    "search_a": [10, 10, 0, 1.5, 1.5, 20],
                    "search_f": [1, 1, 0, 1.2, 1.2, 0.8],
                    "search_phi": [
                        0,
                        3.14159265358979323846 / 2,
                        0,
                        3.14159265358979323846 / 2,
                        0,
                        0,
                    ],
                    "K_x": [300, 300, 500, 500, 500, 100],
                    "f_push": [0, 0, 8, 0, 0, 0],
                    "dX_d": [0.03, 0.3],
                    "ddX_d": [0.3, 1],
                },
                "p3": {
                    "dX_d": [0.1, 0.5],
                    "ddX_d": [0.5, 1],
                    "f_push": 7,
                    "K_x": [500, 500, 0, 800, 800, 800],
                },
            },
            "control": {"control_mode": 0},
            "user": {
                "env_X": [0.01, 0.01, 0.002, 0.05, 0.05, 0.05],
                "env_dX": [0.001, 0.001, 0.001, 0.005, 0.005, 0.005],
                "F_ext_contact": [10.0, 2.0],
            },
        }

        insert = MiosSkill(
            skill_name="insert",
            skill_type="KiosInsertion",
            skill_parameters=payload,
        )
        teach_object_in_mios = self.generate_teach_O_T_TCP_call(insertable)

        update_object_in_mios = self.generate_update_mios_memory_environment_call()
        update_object_in_kios = self.generate_update_object_from_mios_call(insertable)

        release = self.generate_gripper_release_mp(width=0.08)

        retreat = self.generate_move_above_mp(container)

        return [
            insert,
            teach_object_in_mios,
            update_object_in_mios,
            update_object_in_kios,
            release,
            retreat,
        ]

    @bb_deprecated(reason="demo only")
    def generate_insert_skill_mod(
        self, parsed_action: dict[str, Any], param: dict[str, Any] = None
    ) -> list[MiosCall | MiosSkill]:
        insertable = parsed_action["args"][2]
        container = parsed_action["args"][3]
        if container is None:
            raise Exception("container is not set!")

        # get the container from the scene
        kios_object = self.task_scene.get_object(container)

        if param is None:
            param = {
                "search_a": [2, 2, 1, 1, 1, 0],
                "search_f": [1, 1, 1, 1.5, 1.5, 0],
                "F_ext_contact": [13.0, 2.0],
                "f_push": [0, 0, 5, 0, 0, 0],
                "K_x": [300, 300, 500, 500, 500, 800],
                "D_x": [0.7, 0.7, 0.7, 0.7, 0.7, 0.7],
            }

        search_a = (
            param["search_a"] if "search_a" in param.keys() else [2, 2, 1, 1, 1, 0]
        )
        search_f = (
            param["search_f"] if "search_f" in param.keys() else [1, 1, 1, 1.5, 1.5, 0]
        )
        F_ext_contact = (
            param["F_ext_contact"] if "F_ext_contact" in param.keys() else [13.0, 2.0]
        )
        f_push = param["f_push"] if "f_push" in param.keys() else [0, 0, 5, 0, 0, 0]
        K_x = param["K_x"] if "K_x" in param.keys() else [300, 300, 500, 500, 500, 800]
        D_x = param["D_x"] if "D_x" in param.keys() else [0.7, 0.7, 0.7, 0.7, 0.7, 0.7]

        if kios_object is not None:
            O_T_TCP = kios_object.O_T_TCP
            payload = {
                "skill": {
                    "objects": {
                        # "Container": container,
                    },
                    # "time_max": 35,
                    "p0": {
                        "O_T_TCP": O_T_TCP.T.flatten().tolist(),
                        "dX_d": [0.05, 0.3],
                        "ddX_d": [0.5, 1],
                        "DeltaX": [0, 0, 0, 0, 0, 0],
                        "K_x": [1500, 1500, 1500, 600, 600, 600],
                    },
                    "p1": {
                        "dX_d": [0.05, 0.1],
                        "ddX_d": [0.1, 0.05],
                        "K_x": [1500, 1500, 500, 800, 800, 800],
                    },
                    "p2": {
                        # "search_a": [10, 10, 0, 2, 2, 0],
                        # "search_f": [1, 1, 0, 1.2, 1.2, 0],
                        "search_a": search_a,
                        "search_f": search_f,
                        "search_phi": [
                            0,
                            3.14159265358979323846 / 2,
                            0,
                            3.14159265358979323846 / 2,
                            0,
                            0,
                        ],
                        "K_x": K_x,
                        "D_x": D_x,
                        "f_push": f_push,
                        # "dX_d": [0.1, 0.5],
                        # "ddX_d": [0.5, 1],
                        "dX_d": [0.03, 0.3],
                        "ddX_d": [0.3, 1],
                    },
                    "p3": {
                        "dX_d": [0.1, 0.5],
                        "ddX_d": [0.5, 1],
                        "f_push": 7,
                        "K_x": [500, 500, 0, 800, 800, 800],
                    },
                },
                "control": {"control_mode": 0},
                "user": {
                    "env_X": (
                        param["env_X"]
                        if "env_X" in param.keys()
                        else [0.01, 0.01, 0.002, 0.05, 0.05, 0.05]
                    ),
                    "env_dX": [0.001, 0.001, 0.001, 0.005, 0.005, 0.005],
                    "F_ext_contact": F_ext_contact,
                },
            }
        # else:
        #     payload = {
        #         "skill": {
        #             "objects": {
        #                 "Container": container,
        #             },
        #             "time_max": 20,
        #             "p0": {  # ! approach should be move above in mios skill. mod it
        #                 "dX_d": [0.1, 0.3],
        #                 "ddX_d": [0.5, 4],
        #                 "DeltaX": [0, 0, 0, 0, 0, 0],
        #                 "K_x": [1500, 1500, 1500, 600, 600, 600],
        #             },
        #             "p1": {
        #                 "dX_d": [0.03, 0.1],
        #                 "ddX_d": [0.05, 0.01],
        #                 "K_x": [500, 500, 500, 600, 600, 600],
        #             },
        #             "p2": {
        #                 # "search_a": [10, 10, 0, 2, 2, 0],
        #                 # "search_f": [1, 1, 0, 1.2, 1.2, 0],
        #                 "search_a": [5, 5, 0, 2, 2, 0],
        #                 "search_f": [1, 1, 0, 1.2, 1.2, 0],
        #                 "search_phi": [
        #                     0,
        #                     3.14159265358979323846 / 2,
        #                     0,
        #                     3.14159265358979323846 / 2,
        #                     0,
        #                     0,
        #                 ],
        #                 "K_x": [500, 500, 500, 800, 800, 800],
        #                 "f_push": [0, 0, 3, 0, 0, 0],
        #                 # "dX_d": [0.1, 0.5],
        #                 # "ddX_d": [0.5, 1],
        #                 "dX_d": [0.08, 0.5],
        #                 "ddX_d": [0.3, 1],
        #             },
        #             "p3": {
        #                 "dX_d": [0.1, 0.5],
        #                 "ddX_d": [0.5, 1],
        #                 "f_push": 7,
        #                 "K_x": [500, 500, 0, 800, 800, 800],
        #             },
        #         },
        #         "control": {"control_mode": 0},
        #         "user": {
        #             "env_X": [0.01, 0.01, 0.002, 0.05, 0.05, 0.05],
        #             "env_dX": [0.001, 0.001, 0.001, 0.005, 0.005, 0.005],
        #             "F_ext_contact": [10.0, 2.0], # ! this doesn't work. change the param in the mongodb!
        #         },
        #     }
        insert = MiosSkill(
            skill_name="insert",
            skill_type="KiosInsertion",
            skill_parameters=payload,
        )
        update_object_in_mios = self.generate_teach_object_call(insertable)
        update_object_in_kios = self.generate_update_object_from_mios_call(insertable)

        release = self.generate_gripper_release_mp(
            width=param["width"] if "width" in param.keys() else 0.042
        )

        retreat = self.generate_move_above_mp(container)

        return [
            insert,
            update_object_in_mios,
            update_object_in_kios,
            release,
            retreat,
        ]

    @bb_deprecated(reason="demo only", can_run=True)
    def generate_insert_mp(
        self,
        insertable: str,
        container: str,
        param: dict[str, Any] = None,
    ) -> MiosSkill:
        if container is None:
            raise Exception("container is not set!")

        # get the container from the scene
        kios_object = self.task_scene.get_object(container)

        if param is None:
            param = {
                "search_a": [10, 10, 1, 1, 1, 0],
                "search_f": [1, 1, 1, 1.5, 1.5, 0],
                "F_ext_contact": [13.0, 2.0],
                "f_push": [0, 0, 5, 0, 0, 0],
                "K_x": [300, 300, 500, 500, 500, 800],
                "D_x": [0.7, 0.7, 0.7, 0.7, 0.7, 0.7],
            }

        search_a = (
            param["search_a"] if "search_a" in param.keys() else [2, 2, 1, 1, 1, 0]
        )
        search_f = (
            param["search_f"] if "search_f" in param.keys() else [1, 1, 1, 1.5, 1.5, 0]
        )
        F_ext_contact = (
            param["F_ext_contact"] if "F_ext_contact" in param.keys() else [13.0, 2.0]
        )
        f_push = param["f_push"] if "f_push" in param.keys() else [0, 0, 5, 0, 0, 0]
        K_x = param["K_x"] if "K_x" in param.keys() else [300, 300, 500, 500, 500, 800]
        D_x = param["D_x"] if "D_x" in param.keys() else [0.7, 0.7, 0.7, 1.4, 1.4, 1.4]

        if kios_object is not None:
            O_T_TCP = kios_object.O_T_TCP
            payload = {
                "skill": {
                    "objects": {
                        # "Container": container,
                    },
                    "time_max": 60,
                    "p0": {
                        "O_T_TCP": O_T_TCP.T.flatten().tolist(),
                        "dX_d": [0.15, 0.5],
                        "ddX_d": [0.5, 1],
                        "DeltaX": [0, 0, 0, 0, 0, 0],
                        "K_x": [1500, 1500, 1500, 600, 600, 600],
                    },
                    "p1": {
                        "dX_d": [0.05, 0.5],
                        "ddX_d": [0.1, 0.05],
                        "K_x": [1500, 1500, 500, 800, 800, 800],
                    },
                    "p2": {
                        "search_a": search_a,
                        "search_f": search_f,
                        "search_phi": [
                            0,
                            3.14159265358979323846 / 2,
                            0,
                            3.14159265358979323846 / 2,
                            0,
                            0,
                        ],
                        "K_x": K_x,
                        "D_x": D_x,
                        "f_push": f_push,
                        "dX_d": [0.03, 0.3],
                        "ddX_d": [0.3, 1],
                    },
                    "p3": {
                        "dX_d": [0.1, 0.5],
                        "ddX_d": [0.5, 1],
                        "f_push": 7,
                        "K_x": [500, 500, 0, 800, 800, 800],
                    },
                },
                "control": {"control_mode": 0},
                "user": {
                    "env_X": (
                        param["env_X"]
                        if "env_X" in param.keys()
                        else [0.01, 0.01, 0.002, 0.05, 0.05, 0.05]
                    ),
                    "env_dX": [0.001, 0.001, 0.001, 0.005, 0.005, 0.005],
                    "F_ext_contact": F_ext_contact,
                },
            }

        return MiosSkill(
            skill_name="insert",
            skill_type="KiosInsertion",
            skill_parameters=payload,
        )

    @bb_deprecated(reason="demo only")
    def generate_insert_skill_standalone_mod(
        self, parsed_action: dict[str, Any], param: dict[str, Any] = None
    ) -> list[MiosCall | MiosSkill]:
        insertable = parsed_action["args"][2]
        container = parsed_action["args"][3]
        if container is None:
            raise Exception("container is not set!")

        # get the container from the scene
        kios_object = self.task_scene.get_object(container)

        if param is None:
            param = {
                "search_a": [2, 2, 1, 1, 1, 0],
                "search_f": [1, 1, 1, 1.5, 1.5, 0],
                "F_ext_contact": [13.0, 2.0],
                "f_push": [0, 0, 5, 0, 0, 0],
                "K_x": [300, 300, 500, 500, 500, 800],
                "D_x": [0.7, 0.7, 0.7, 0.7, 0.7, 0.7],
            }

        search_a = (
            param["search_a"] if "search_a" in param.keys() else [2, 2, 1, 1, 1, 0]
        )
        search_f = (
            param["search_f"] if "search_f" in param.keys() else [1, 1, 1, 1.5, 1.5, 0]
        )
        F_ext_contact = (
            param["F_ext_contact"] if "F_ext_contact" in param.keys() else [13.0, 2.0]
        )
        f_push = param["f_push"] if "f_push" in param.keys() else [0, 0, 5, 0, 0, 0]
        K_x = param["K_x"] if "K_x" in param.keys() else [300, 300, 500, 500, 500, 800]
        D_x = param["D_x"] if "D_x" in param.keys() else [0.7, 0.7, 0.7, 0.7, 0.7, 0.7]

        if kios_object is not None:
            O_T_TCP = kios_object.O_T_TCP
            payload = {
                "skill": {
                    "objects": {
                        # "Container": container,
                    },
                    "time_max": 25,
                    "p0": {
                        "O_T_TCP": O_T_TCP.T.flatten().tolist(),
                        "dX_d": [0.05, 0.3],
                        "ddX_d": [0.5, 1],
                        "DeltaX": [0, 0, 0, 0, 0, 0],
                        "K_x": [1500, 1500, 1500, 600, 600, 600],
                    },
                    "p1": {
                        "dX_d": [0.05, 0.1],
                        "ddX_d": [0.1, 0.05],
                        "K_x": [1500, 1500, 500, 800, 800, 800],
                    },
                    "p2": {
                        "search_a": search_a,
                        "search_f": search_f,
                        "search_phi": [
                            0,
                            3.14159265358979323846 / 2,
                            0,
                            3.14159265358979323846 / 2,
                            0,
                            0,
                        ],
                        "K_x": K_x,
                        "D_x": D_x,
                        "f_push": f_push,
                        "dX_d": [0.03, 0.3],
                        "ddX_d": [0.3, 1],
                    },
                    "p3": {
                        "dX_d": [0.1, 0.5],
                        "ddX_d": [0.5, 1],
                        "f_push": 7,
                        "K_x": [500, 500, 0, 800, 800, 800],
                    },
                },
                "control": {"control_mode": 0},
                "user": {
                    "env_X": (
                        param["env_X"]
                        if "env_X" in param.keys()
                        else [0.01, 0.01, 0.002, 0.05, 0.05, 0.05]
                    ),
                    "env_dX": [0.001, 0.001, 0.001, 0.005, 0.005, 0.005],
                    "F_ext_contact": F_ext_contact,
                },
            }

        insert = MiosSkill(
            skill_name="insert",
            skill_type="KiosInsertion",
            skill_parameters=payload,
        )
        update_object_in_mios = self.generate_teach_O_T_TCP_call(insertable)
        update_object_in_kios = self.generate_update_object_from_mios_call(insertable)

        return [
            insert,
            # update_object_in_mios,
            # update_object_in_kios,
        ]
