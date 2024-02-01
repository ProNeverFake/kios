from typing import List, Any, Dict
import re
import numpy as np

from kios_scene.mongodb_interface import MongoDBInterface

# from kios_robot.robot_interface import RobotInterface
# ! circular import

# from kios_robot.robot_proprioceptor import RobotProprioceptor
from kios_robot.data_types import (
    MiosCall,
    MiosSkill,
    KiosCall,
    Toolbox,
    TaskScene,
    MiosObject,
)

# from kios_robot.robot_status import RobotStatus
from kios_bt.data_types import Action

"""
BB knows this is not a good design to create generate method for each task/call. 
A better implementation can be a common interface for generating mios tasks,
which takes formatted node data as input. 
Anyway, the current implementation is straightforward and easy to understand.
"""


class MiosTaskFactory:
    task_scene: TaskScene
    robot_interface: Any
    # robot_proprioceptor: RobotProprioceptor

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

    def parse_action(self, action: Action) -> Dict[str, Any]:
        """parse the action string to get the action name and arguments.

        Args:
            action (Action): _description_

        Raises:
            Exception: _description_

        Returns:
            Dict[str, Any]: action_name: str, args: List[str]
        """
        # use re to parse the action
        pattern = r"(\w+)\(([\w\s,]+)\)"
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
            raise Exception(
                "Action format error!"
            )  # Raise an exception if the action format is incorrect
            # return {"action_name": None, "args": []}

    # * BBCORE
    def generate_mios_tasks(self, action: Action) -> List[MiosCall or MiosSkill]:
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
        if parsed_action["name"] == "unload_tool":
            return self.generate_unload_tool_skill(parsed_action)
        if parsed_action["name"] == "pick_up":
            return self.generate_pick_up_skill(parsed_action)
        if parsed_action["name"] == "insert":
            return self.generate_insert_skill(parsed_action)

        # raise NotImplementedError

    def generate_fake_mios_tasks(self, action: Action) -> List[MiosCall or MiosSkill]:
        """
        test function.
        """
        print("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
        print("generate_fake_mios_tasks, skipped for test.")
        # print(action)
        print(self.parse_action(action))
        return [
            MiosCall(
                method_name="test_method",
                method_payload={"test": "test"},
            )
        ]

    ###################################################################
    # * kios call methods
    def generate_update_object_from_mios_call(self, object_name: str) -> KiosCall:
        """update the object in kios from mios.

        Args:
            object_name (str): _description_

        Returns:
            KiosCall: _description_
        """
        return KiosCall(
            method=self.robot_interface.proprioceptor.update_scene_object_from_mios,
            method_args=[self.task_scene, object_name],
        )

    ###################################################################
    # * methods to generate one mios task

    def generate_move_above_mp(self, object_name: str) -> MiosSkill:
        # default: 15cm above the object
        # get the object from the scene
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

        # generate the cartesian move
        return self.generate_cartesian_move_mp(O_T_TCP=above_O_T_TCP)

    def generate_cartesian_move_mp(
        self, object_name: str = None, O_T_TCP: np.ndarray = None
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
            raise Exception("cartesian target is not set!")
        if object_name:
            context = {
                "skill": {
                    "p0": {
                        "dX_d": [0.1, 0.5],
                        "ddX_d": [0.5, 1],
                        "K_x": [1500, 1500, 1500, 150, 150, 150],
                    },
                    "objects": {"GoalPose": object_name},
                },
                "control": {"control_mode": 0},
            }
        else:
            context = {
                "skill": {
                    "p0": {
                        "dX_d": [0.1, 0.5],
                        "ddX_d": [0.5, 1],
                        "K_x": [1500, 1500, 1500, 150, 150, 150],
                        "T_T_EE_g": O_T_TCP.T.flatten().tolist(),
                    },
                    # "objects": {"GoalPose": "NullObject"},
                },
                "control": {"control_mode": 0},
            }

        return MiosSkill(
            skill_name="cartesian_move",
            skill_type="KiosCartesianMove",
            skill_parameters=context,
        )

    # def generate_cartesian_move_mp_HT(self, object: str, tool: Toolbox) -> MiosSkill:
    #     context = {
    #         "skill": {
    #             "p0": {
    #                 "dX_d": [0.1, 0.5],
    #                 "ddX_d": [0.5, 1],
    #                 "K_x": [1500, 1500, 1500, 150, 150, 150],
    #             },
    #             "time_max": 15,
    #             "objects": {"GoalPose": object},
    #             "tool": {
    #                 "EE_T_TCP": tool.EE_T_TCP.T.flatten().tolist(),
    #             },
    #         },
    #         "control": {"control_mode": 0},
    #         "frames": {
    #             "EE_T_TCP": tool.EE_T_TCP.T.flatten().tolist(),
    #         },
    #     }

    #     return MiosSkill(
    #         skill_name="cartesian_move",
    #         skill_type="KiosCartesianMove",
    #         skill_parameters=context,
    #     )

    # ! don't use this
    def generate_joint_move(self, joint_location: str) -> MiosSkill:
        """
            * available. debug: task context error, "skill", "control" and "user" are on the same level.
            move to a joint position.
        Args:
            joint_location (str): the stored Taskframe_HomogeneousT_EE_goal in mongo DB
        """
        context = {
            "skill": {
                "speed": 0.5,
                "acc": 1,
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
        # return call_method(self.robot_address, self.robot_port, "grasp", payload)
        return MiosCall(method_name="grasp", method_payload=payload)

    def generate_gripper_release_mp(self, width=0.08, speed=0.2) -> MiosCall:
        payload = {
            "width": width,
            "speed": speed,
        }
        # return call_method(self.robot_address, self.robot_port, "release", payload)
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

    # preserved for future use
    def generate_equip_tool(self) -> MiosCall:
        # move finger to the right position
        return self.generate_gripper_move_mp(0.042)

    # preserved for future use
    def generate_lift_tool(self) -> MiosCall:
        # move finger to the right position
        return self.generate_gripper_move_mp(0.08)

    def generate_teach_object_call(self, object_name: str) -> MiosCall:
        payload = {
            "object": object_name,
        }
        return MiosCall(method_name="teach_object", method_payload=payload)

    def generate_update_tool_call(self, tool_name: str = None) -> MiosCall:
        """let mios know the tool is loaded and it need to change EE_T_TCP.

        Args:
            tool_name (str, optional): _description_. Defaults to None.

        Returns:
            MiosCall: _description_
        """
        # get the tool from the scene
        if tool_name is None:
            tool = self.task_scene.get_tool("no_tool")
        else:
            tool = self.task_scene.get_tool(tool_name)
        # get the EE_T_TCP
        EE_T_TCP = tool.EE_T_TCP
        assert isinstance(EE_T_TCP, np.ndarray)
        payload = {"EE_T_TCP": EE_T_TCP.T.flatten().tolist()}
        return MiosCall(method_name="change_tool_EE_T_TCP", method_payload=payload)

    ###################################################################

    ###################################################################
    # * methods to generate a sequence of mios tasks

    def generate_load_tool_skill(
        self, parsed_action: Dict[str, Any]
    ) -> List[MiosSkill or MiosCall]:
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
                    "dX_d": [0.2, 0.2],
                    "ddX_d": [0.2, 0.2],
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
                    "width": 0.043,
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
            "user": {
                "env_X": [0.01, 0.01, 0.002, 0.05, 0.05, 0.05],
                "env_dX": [0.001, 0.001, 0.001, 0.005, 0.005, 0.005],
            },
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
        self, parsed_action: Dict[str, Any]
    ) -> List[MiosSkill or MiosCall]:
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
                    "dX_d": [0.2, 0.2],
                    "ddX_d": [0.2, 0.2],
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
            "user": {
                "env_X": [0.01, 0.01, 0.002, 0.05, 0.05, 0.05],
                "env_dX": [0.001, 0.001, 0.001, 0.005, 0.005, 0.005],
            },
        }

        unload_tool = MiosSkill(
            skill_name="unload_tool",
            skill_type="KiosLoadTool",
            skill_parameters=payload,
        )

        update_tool = self.generate_update_tool_call(tool_name=None)

        return [
            unload_tool,
            update_tool,
        ]

    def generate_pick_up_skill(
        self, parsed_action: Dict[str, Any]
    ) -> List[MiosCall or MiosSkill]:
        # tool_name = parsed_action["args"][1]
        # if tool_name is None:
        #     raise Exception("tool_name is not set!")
        object_name = parsed_action["args"][2]
        if object_name is None:
            raise Exception("object_name is not set!")

        task_list = []
        # move above
        move_above = self.generate_move_above_mp(object_name)
        task_list.append(move_above)
        # move in
        reach = self.generate_cartesian_move_mp(object_name=object_name)
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

    def generate_insert_skill(
        self, parsed_action: Dict[str, Any]
    ) -> List[MiosCall or MiosSkill]:
        insertable = parsed_action["args"][2]
        container = parsed_action["args"][3]
        if container is None:
            raise Exception("container is not set!")

        # get the container from the scene
        kios_object = self.task_scene.get_object(container)
        if kios_object is not None:
            O_T_TCP = kios_object.O_T_TCP
            payload = {
                "skill": {
                    "objects": {
                        # "Container": container,
                    },
                    "time_max": 20,
                    "p0": {  # ! approach should be move above in mios skill. mod it
                        "O_T_TCP": O_T_TCP.T.flatten().tolist(),
                        "dX_d": [0.1, 1],
                        "ddX_d": [0.5, 4],
                        "DeltaX": [0, 0, 0, 0, 0, 0],
                        "K_x": [1500, 1500, 1500, 600, 600, 600],
                    },
                    "p1": {
                        "dX_d": [0.03, 0.1],
                        "ddX_d": [0.5, 0.1],
                        "K_x": [500, 500, 500, 600, 600, 600],
                    },
                    "p2": {
                        # "search_a": [10, 10, 0, 2, 2, 0],
                        # "search_f": [1, 1, 0, 1.2, 1.2, 0],
                        "search_a": [5, 5, 0, 2, 2, 0],
                        "search_f": [1, 1, 0, 1.2, 1.2, 0],
                        "search_phi": [
                            0,
                            3.14159265358979323846 / 2,
                            0,
                            3.14159265358979323846 / 2,
                            0,
                            0,
                        ],
                        "K_x": [500, 500, 500, 800, 800, 800],
                        "f_push": [0, 0, 7, 0, 0, 0],
                        # "dX_d": [0.1, 0.5],
                        # "ddX_d": [0.5, 1],
                        "dX_d": [0.08, 0.5],
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
                    "F_ext_contact": [3.0, 2.0],
                },
            }
        else:
            payload = {
                "skill": {
                    "objects": {
                        "Container": container,
                    },
                    "time_max": 20,
                    "p0": {  # ! approach should be move above in mios skill. mod it
                        "dX_d": [0.1, 1],
                        "ddX_d": [0.5, 4],
                        "DeltaX": [0, 0, 0, 0, 0, 0],
                        "K_x": [1500, 1500, 1500, 600, 600, 600],
                    },
                    "p1": {
                        "dX_d": [0.03, 0.1],
                        "ddX_d": [0.5, 0.1],
                        "K_x": [500, 500, 500, 600, 600, 600],
                    },
                    "p2": {
                        # "search_a": [10, 10, 0, 2, 2, 0],
                        # "search_f": [1, 1, 0, 1.2, 1.2, 0],
                        "search_a": [5, 5, 0, 2, 2, 0],
                        "search_f": [1, 1, 0, 1.2, 1.2, 0],
                        "search_phi": [
                            0,
                            3.14159265358979323846 / 2,
                            0,
                            3.14159265358979323846 / 2,
                            0,
                            0,
                        ],
                        "K_x": [500, 500, 500, 800, 800, 800],
                        "f_push": [0, 0, 7, 0, 0, 0],
                        # "dX_d": [0.1, 0.5],
                        # "ddX_d": [0.5, 1],
                        "dX_d": [0.08, 0.5],
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
                    "F_ext_contact": [3.0, 2.0],
                },
            }
        insert = MiosSkill(
            skill_name="insert",
            skill_type="KiosInsertion",
            skill_parameters=payload,
        )
        update_object_in_mios = self.generate_teach_object_call(insertable)
        update_object_in_kios = self.generate_update_object_from_mios_call(insertable)
        release = self.generate_gripper_release_mp(width=0.042)

        return [
            insert,
            update_object_in_mios,
            update_object_in_kios,
        ]
