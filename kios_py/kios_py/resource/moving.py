#!/usr/bin/python3 -u
from task import *
from ws_client import *


def teach_location(robot: str, location: str):
    call_method(robot, 12000, "teach_object", {"object": location})

def move_gripper(robot, width):
    payload = {
        "width": width,
        "speed": 0.05,
    }
    return call_method(robot, 12000, "move_gripper", payload)

# move to pose cartesian
def move_to_cartesian_location(robot: str, location: str):
    context = {
        "skill": {
            "p0":{
                "dX_d": [0.1, 0.5],
                "ddX_d": [0.5, 1],
                "K_x": [1500, 1500, 1500, 150, 150, 150],

            },
            "objects": {
                    "GoalPose": location
                }
        },
        "control": {
            "control_mode": 2
        }
    }
    t = Task(robot)
    t.add_skill("move", "TaxMove", context)
    t.start()
    result = t.wait()
    print("Result: " + str(result))

# move to pose joint 
def move_to_joint_location(robot: str, joint_location: str):
    """
        * available. debug: task context error, "skill", "control" and "user" are on the same level.
        move to a joint position.
    Args:
        robot (str): default robot ip address
        joint_location (str): the stored Taskframe_HomogeneousT_EE_goal in mongo DB
    """
    context = {
        "skill": {
            "speed": 0.5,
            "acc": 1,
            "q_g": [0, 0, 0, 0, 0, 0, 0],
            "q_g_offset": [0, 0, 0, 0, 0, 0, 0],
            "objects": {
                "goal_pose": joint_location
                }
            },
        "control": {
            "control_mode": 3
            },
        "user": {
            "env_X": [0.005, 0.005, 0.005, 0.0175, 0.0175, 0.0175]
            }
        }
    t = Task(robot)
    t.add_skill("move", "MoveToPoseJoint", context)
    t.start()
    result = t.wait()
    print("Result: " + str(result))
