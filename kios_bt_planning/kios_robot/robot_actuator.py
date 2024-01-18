from kios_utils.task import *
import time
import json


class RobotActuator:
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

    def gripper_move(self, width: float, speed: float = 0.05):
        payload = {
            "width": width,
            "speed": speed,
        }
        return call_method(self.robot_address, self.robot_port, "move_gripper", payload)

    # move to pose cartesian
    def cartesian_move(self, object: str):
        context = {
            "skill": {
                "p0": {
                    "dX_d": [0.1, 0.5],
                    "ddX_d": [0.5, 1],
                    "K_x": [1500, 1500, 1500, 150, 150, 150],
                },
                "objects": {"GoalPose": object},
            },
            "control": {"control_mode": 2},
        }
        t = Task(self.robot_address)
        t.add_skill("move", "TaxMove", context)
        t.start()
        result = t.wait()
        print("Result: " + str(result))

    # move to pose joint
    def joint_move(self, joint_location: str):
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
        t = Task(self.robot_address)
        t.add_skill("move", "MoveToPoseJoint", context)
        t.start()
        result = t.wait()
        print("Result: " + str(result))

    def gripper_grasp(self, force=20):
        payload = {
            "width": 0.01,
            "speed": 0.01,
            "force": force,
            "epsilon_inner": 0.05,
            "epsilon_outer": 0.05,
        }
        return call_method(self.robot_address, self.robot_port, "grasp", payload)

    # ? how should we make use of the object in the system?
    def gripper_grasp_object(self, force=60):
        payload = {
            "object": "NoneObject",
            "width": 0.0,
            "speed": 0.05,
            "force": force,
            "check_widtch": False,
        }
        return call_method(self.robot_address, self.robot_port, "grasp_object", payload)

    def home_gripper(self):
        payload = {}
        return call_method(self.robot_address, self.robot_port, "home_gripper", payload)

    def move_gripper(self, width):
        payload = {
            "width": width,
            "speed": 0.05,
        }
        return call_method(self.robot_address, self.robot_port, "move_gripper", payload)

    def grasp_tool(self):
        payload = {
            "width": 0.04,
            "speed": 0.05,
        }
        return call_method(self.robot_address, self.robot_port, "move_gripper", payload)

    def release_tool(self):
        payload = {
            "width": 0.08,
            "speed": 0.05,
        }
        return call_method(self.robot_address, self.robot_port, "move_gripper", payload)

    # ! don't use this.
    def moveJ(self, q_g):
        """
        call mios for moving the lefthand to desired joint position

        Paramter
        --------
        q_g: list, len(7)
        """
        parameters = {
            "parameters": {
                "pose": "NoneObject",
                "q_g": q_g,
                "speed": 0.5,
                "acc": 0.7,
            }
        }
        return start_task_and_wait(
            self.robot_address, "MoveToJointPose", parameters, False
        )

    # * modify this later
    def insertion(self):
        call_method(
            self.robot_address,
            self.robot_port,
            "set_grasped_object",
            {"object": "ring"},
        )
        content = {
            "skill": {
                "objects": {
                    "Container": "housing",
                    "Approach": "app1",
                    "Insertable": "ring",
                },
                "time_max": 17,
                "p0": {
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
        t = Task(self.robot_address)
        t.add_skill("insertion", "TaxInsertion", content)
        t.start()
        time.sleep(0.5)
        result = t.wait()
        print("Result: " + str(result))
