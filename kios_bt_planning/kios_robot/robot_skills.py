from kios_utils.task import *

import time

import json

def teach_location(robot: str, location: str):
    call_method(robot, 12000, "teach_object", {"object": location})

def teach_object(robot: str, object_name: str):
    call_method(robot, 12000, "teach_object", {"object": object_name})

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


def gripper(robot, width):
    payload = {
        "width": width,
        "speed": 0.05,
        "force": 0.1,
        "epsilon_inner": 1,
        "epsilon_outer": 1,
    }
    return call_method(robot, 12000, "grasp", payload)


def grasp(robot, force=20):
    payload = {
        "width": 0.01,
        "speed": 0.01,
        "force": force,
        "epsilon_inner": 0.05,
        "epsilon_outer": 0.05,
    }
    return call_method(robot, 12000, "grasp", payload)


def grasp_object(robot, force=60):
    payload = {
        "object": "NoneObject",
        "width": 0.0,
        "speed": 0.05,
        "force": force,
        "check_widtch": False,
    }
    return call_method(robot, 12000, "grasp_object", payload)


def home_gripper(robot):
    payload = {}
    return call_method(robot, 12000, "home_gripper", payload)


def move_gripper(robot, width):
    payload = {
        "width": width,
        "speed": 0.05,
    }
    return call_method(robot, 12000, "move_gripper", payload)


def release(robot):
    payload = {
        "width": 0.08,
        "speed": 0.05,
    }
    return call_method(robot, 12000, "move_gripper", payload)


def modify_taught_pose(robot, name: str, x, y, z, R=[1, 0, 0, 0, -1, 0, 0, 0, 1]):
    """change the stored Taskframe_homogT_EE in mongo DB.
        ! attention: input changed!
    Args:
        name (str): the name of the object stored in mongo DB
        x (double): the x of the position vector
        y ():
        z ():
        R : rotation matrix in vector form

    Returns:
        call the "set_partial_object_data" method in mios.
    """
    payload = {
        "object": name,
        "data": {
            "x": x,
            "y": y,
            "z": z,
            "R": [0, 1, 0, 1, 0, 0, 0, 0, -1],
            # "R": R,
        },
    }
    return call_method(robot, 12000, "set_partial_object_data", payload)


def moveJ(robot, q_g):
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
    return start_task_and_wait(robot, "MoveToJointPose", parameters, False)


def insertion(robot):
    call_method(robot, 12000, "set_grasped_object", {"object": "ring"})
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
    t = Task(robot)
    t.add_skill("insertion", "TaxInsertion", content)
    t.start()
    time.sleep(0.5)
    result = t.wait()
    print("Result: " + str(result))


def modify_housing(x=0.417140386890134, y=0.4047956531460382):
    return modify_taught_pose(x, y, 0.015098423457252558, "housing")


def extract(robot):
    move_gripper(robot, 0.002)
    time.sleep(0.5)
    gripper(robot, 0.0005)
    teach_location(robot, "above")
    a = call_method(robot, 12000, "get_state")
    x = a["result"]["O_T_EE"][-4]
    y = a["result"]["O_T_EE"][-3]
    z = a["result"]["O_T_EE"][-2] + 0.08
    modify_taught_pose(robot, "above", x, y, z)
    print("Housing pose:", [x, y])
    move_to_cartesian_location(robot, "above")
