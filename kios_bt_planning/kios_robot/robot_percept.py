from kios_utils.task import *
import numpy as np

# * use localhost when running mios locally.
MIOS = "127.0.0.1"
# * use docker ip when running mios in docker.
MIOS_DOCKER = "10.157.175.17"

'''
#! BB, you should integrate this into the robot_interface!
'''

def test_connection(robot=MIOS):
    return call_method(robot, 12000, "test_connection")


def get_robot_state(robot=MIOS):
    return call_method(robot, 12000, "get_state")


def get_robot_q(robot=MIOS):
    robot_state = get_robot_state(robot)
    return robot_state["result"]["q"]


def get_robot_O_T_EE(robot=MIOS):
    robot_state = get_robot_state(robot)
    return np.reshape(np.array(robot_state["result"]["O_T_EE"]), (4, 4)).T


def get_robot_R(robot=MIOS):
    return get_robot_O_T_EE(robot)[0:3, 0:3]


def get_robot_position(robot=MIOS):
    return get_robot_O_T_EE()[0:3, 3]


# modify the orientation
# TODO:
def modify_object_orientation(
    robot, name: str, x, y, z, R=[1, 0, 0, 0, -1, 0, 0, 0, 1]
):
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
            # "R": [0, 1, 0, 1, 0, 0, 0, 0, -1],
            "R": R,
        },
    }
    return call_method(robot, 12000, "set_partial_object_data", payload)


def modify_object(robot, name: str, T: np.ndarray):
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
            "x": T[0, 3],
            "y": T[1, 3],
            "z": T[2, 3],
            "R": T[0:3, 0:3].T.flatten().tolist(),
        },
    }
    return call_method(robot, 12000, "set_partial_object_data", payload)


# modify_object_orientation(
#     MIOS,
#     "test",
#     -0.46902237,
#     0.27293867,
#     0.51091767,
#     [
#         0.07152728,  0.35314508, -0.93283,
#         0.71677906, -0.66854478, -0.1981323,
#         -0.69360827, -0.65446034, -0.30094942
#     ],
# )


# modify the position (relative change)
def modify_object_position(robot, name: str, x, y, z):
    """change the stored Taskframe_homogT_EE in mongo DB.
        ! attention: input changed!
        todo comment update
    Args:
        name (str): the name of the object stored in mongo DB
        x (double): the x of the position vector
        y ():
        z ():
        R : rotation matrix in vector form

    Returns:
        call the "set_partial_object_data" method in mios.
    """

    # relative movement
    position = get_robot_position()
    new_x = position[0] + x
    new_y = position[0] + y
    new_z = position[0] + z

    R = get_robot_R().T.flatten().tolist()
    print(R)

    payload = {
        "object": name,
        "data": {
            "x": new_x,
            "y": new_y,
            "z": new_z,
            # "R": [0, 1, 0, 1, 0, 0, 0, 0, -1],
            "R": R,
        },
    }
    return call_method(robot, 12000, "set_partial_object_data", payload)


# TODO:
def get_object():
    pass
