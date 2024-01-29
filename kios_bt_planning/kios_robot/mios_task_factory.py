from typing import List, Any

from kios_robot.data_types import MiosCall, MiosSkill, Toolbox
from kios_bt.data_types import Action


class MiosTaskFactory:
    def __init__(self):
        pass

    # * BBCORE
    def generate_mios_tasks(self, action: Action) -> List[MiosCall or MiosSkill]:
        """core function.
        generate mios tasks from a kios action and return them in a list.

        Args:
            action (Action): _description_

        Returns:
            List[MiosCall or MiosSkill]: _description_
        """
        
        # raise NotImplementedError

    def generate_fake_mios_tasks(self, action: Action) -> List[MiosCall or MiosSkill]:
        """
        test function.
        """
        print("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
        print("generate_fake_mios_tasks, skipped for test.")
        return [
            MiosCall(
                method_name="test_method",
                method_payload={"test": "test"},
            )
        ]

    def generate_cartesian_move(self, object: str) -> MiosSkill:
        context = {
            "skill": {
                "p0": {
                    "dX_d": [0.1, 0.5],
                    "ddX_d": [0.5, 1],
                    "K_x": [1500, 1500, 1500, 150, 150, 150],
                },
                "objects": {"GoalPose": object},
            },
            "control": {"control_mode": 0},
        }

        return MiosSkill(
            is_general_skill=True,
            skill_name="cartesian_move",
            skill_type="TaxMove",
            skill_parameters=context,
        )

    def generate_cartesian_move_HT(self, object: str, tool: Toolbox) -> MiosSkill:
        context = {
            "skill": {
                "p0": {
                    "dX_d": [0.1, 0.5],
                    "ddX_d": [0.5, 1],
                    "K_x": [1500, 1500, 1500, 150, 150, 150],
                },
                "time_max": 15,
                "objects": {"GoalPose": object},
                "tool": {
                    "EE_HT_TCP": tool.EE_HT_TCP.T.flatten().tolist(),
                },
            },
            "control": {"control_mode": 0},
            "frames": {
                "EE_T_TCP": tool.EE_HT_TCP.T.flatten().tolist(),
            },
        }

        return MiosSkill(
            skill_name="cartesian_move",
            skill_type="KiosCartesianMove",
            skill_parameters=context,
        )

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

    def generate_gripper_grasp(
        self, width=0.01, speed=0.05, force=50, epsilon_inner=0.05, epsilon_outer=0.05
    ) -> MiosCall:
        payload = {
            "width": 0.01,
            "speed": 0.01,
            "force": force,
            "epsilon_inner": 0.05,
            "epsilon_outer": 0.05,
        }
        # return call_method(self.robot_address, self.robot_port, "grasp", payload)
        return MiosCall(method_name="grasp", method_payload=payload)

    def generate_gripper_move(
        self,
        width: float,
        speed: float = 0.05,
    ) -> MiosCall:
        payload = {
            "width": width,
            "speed": speed,
        }
        return MiosCall(method_name="move_gripper", method_payload=payload)

    # # ? how should we make use of the object in the system?
    # def gripper_grasp_object(self, force=60):
    #     payload = {
    #         "object": "NoneObject",
    #         "width": 0.0,
    #         "speed": 0.05,
    #         "force": force,
    #         "check_widtch": False,
    #     }
    #     return call_method(self.robot_address, self.robot_port, "grasp_object", payload)

    def generate_gripper_home(self) -> MiosCall:
        payload = {}
        return MiosCall(method_name="home_gripper", method_payload=payload)

    # preserved for future use
    def generate_equip_tool(self) -> MiosCall:
        # move finger to the right position
        return self.generate_gripper_move(0.042)

    # preserved for future use
    def generate_lift_tool(self) -> MiosCall:
        # move finger to the right position
        return self.generate_gripper_move(0.08)

    # # * modify this later
    # def insertion(self):
    #     call_method(
    #         self.robot_address,
    #         self.robot_port,
    #         "set_grasped_object",
    #         {"object": "ring"},
    #     )
    #     content = {
    #         "skill": {
    #             "objects": {
    #                 "Container": "housing",
    #                 "Approach": "app1",
    #                 "Insertable": "ring",
    #             },
    #             "time_max": 17,
    #             "p0": {
    #                 "dX_d": [0.1, 1],
    #                 "ddX_d": [0.5, 4],
    #                 "DeltaX": [0, 0, 0, 0, 0, 0],
    #                 "K_x": [1500, 1500, 1500, 600, 600, 600],
    #             },
    #             "p1": {
    #                 "dX_d": [0.03, 0.1],
    #                 "ddX_d": [0.5, 0.1],
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
    #                 "f_push": [0, 0, 7, 0, 0, 0],
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
    #             "F_ext_contact": [3.0, 2.0],
    #         },
    #     }
    #     t = Task(self.robot_address)
    #     t.add_skill("insertion", "TaxInsertion", content)
    #     t.start()
    #     time.sleep(0.5)
    #     result = t.wait()
    #     print("Result: " + str(result))

    def generate_load_tool(self, tool_name: str):
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
                "F_ext_contact": [3.0, 2.0],
            },
        }

        return MiosSkill(
            skill_name="load_tool",
            skill_type="KiosLoadTool",
            skill_parameters=payload,
        )

    def generate_unload_tool(self, tool_name: str):
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
                "F_ext_contact": [3.0, 2.0],
            },
        }

        return MiosSkill(
            skill_name="unload_tool",
            skill_type="KiosLoadTool",
            skill_parameters=payload,
        )
