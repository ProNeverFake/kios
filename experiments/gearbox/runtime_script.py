import numpy as np
import os
import json
from spatialmath import *
from spatialmath.base import trnorm
from pprint import pprint

from kios_robot.mios_task_factory import MiosTaskFactory
from kios_robot.robot_command import RobotCommand
from kios_robot.robot_interface import RobotInterface
from kios_robot.data_types import Toolbox
from kios_scene.scene_factory import SceneFactory

from kios_scene.mios_ltm_manipulator import LangTermMemoryManipulator
from kios_utils.bblab_utils import execution_timer

ri = RobotInterface()
sf = SceneFactory()

current_dir = os.path.dirname(os.path.realpath(__file__))
scene_file = os.path.join(current_dir, "scene.json")

with open(scene_file, "r") as f:
    scene_string = f.read()
    scene_json = json.loads(scene_string)

scene = sf.create_scene_from_json(scene_json)
ri.setup_scene(scene)

ltm_manipulator = LangTermMemoryManipulator()


def show_scene():
    pprint(scene)


# def teach_object(object: str):
#     ri.proprioceptor.teach_object(object)
def backup_mios_environment(backup_name: str):
    ltm_manipulator.backup_mios_environment(backup_name)


def clear_mios_environment():
    ltm_manipulator.clear_mios_environment()


def show_backups():
    ltm_manipulator.show_backups()


def restore_to_mios_environment(backup_name: str):
    ltm_manipulator.restore_to_mios_environment(backup_name)
    # ! you need to update the memory of mios.
    update_mios_memory_environment()
    # ! you also need to update the scene object in robot_interface.
    scene = sf.create_scene_from_json(scene_json)
    ri.setup_scene(scene)


def teach_object_TCP(object_name: str):
    ri.proprioceptor.teach_object_TCP(object_name)
    ri.proprioceptor.update_scene_object_from_mios(scene=scene, object_name=object_name)


def align_object(object_name: str, **kwargs):
    ri.proprioceptor.align_object(object_name, **kwargs)
    ri.proprioceptor.update_scene_object_from_mios(scene=scene, object_name=object_name)


def update_mios_memory_environment():
    robot_command = RobotCommand(
        robot_address="127.0.0.1",
        robot_port=12000,
        shared_data=None,
        task_scene=scene,
        robot_interface=ri,
    )
    robot_command.add_task(
        ri.mios_task_factory.generate_update_mios_memory_environment_call()
    )
    robot_command.execute_task_list_sync()


def get_object(object: str):
    ri.proprioceptor.get_object(object)


def set_tool(toolbox_name: str):
    if "robot_command" in locals():
        print("robot command is set")
        print(f"the robot command's tasks are {robot_command.task_list}")
    else:
        print("robot command is not set")

    robot_command = RobotCommand(
        robot_address="127.0.0.1",
        robot_port=12000,
        shared_data=None,
        task_scene=scene,
        robot_interface=ri,
    )
    print(f"the number of tasks in the task list is {len(robot_command.task_list)}")
    robot_command.add_task(ri.mios_task_factory.generate_update_tool_call(toolbox_name))
    robot_command.execute_task_list_sync()
    robot_command.clear_tasks()


def test_cartesian_joint_move():
    # need object "test"
    robot_command = RobotCommand(
        robot_address="127.0.0.1",
        robot_port=12000,
        shared_data=None,
        task_scene=scene,
        robot_interface=ri,
    )

    O_T_TCP = scene.get_object("test").O_T_TCP
    print(O_T_TCP)
    print(type(O_T_TCP))
    print(O_T_TCP.shape)

    robot_command.add_task(
        ri.mios_task_factory.generate_cartesian_move_mp(O_T_TCP=O_T_TCP)
    )

    # rotate in z axis by 90 degrees
    # rotation_90 = SE3.Rz(np.deg2rad(90))

    # rotate in x axis by 90 degrees
    rotation_90 = SE3.Rx(np.deg2rad(90))

    se_O_T_TCP = SE3(trnorm(O_T_TCP))
    result = se_O_T_TCP * rotation_90
    O_T_TCP = result.A

    robot_command.add_task(
        ri.mios_task_factory.generate_cartesian_move_mp(O_T_TCP=O_T_TCP)
    )

    robot_command.execute_task_list_sync()


def test_screw_in(object_name: str):
    # need object "test"
    robot_command = RobotCommand(
        robot_address="127.0.0.1",
        robot_port=12000,
        shared_data=None,
        task_scene=scene,
        robot_interface=ri,
    )

    O_T_OB = scene.get_object(object_name).O_T_TCP

    print(O_T_OB)

    robot_command.add_task(ri.mios_task_factory.generate_screw_in_mp(O_T_OB=O_T_OB))

    robot_command.execute_task_list_sync()


def tool_test():
    robot_command = RobotCommand(
        robot_address="127.0.0.1",
        robot_port=12000,
        shared_data=None,
        task_scene=scene,
        robot_interface=ri,
    )

    robot_command.add_mios_task(
        ri.mios_task_factory.generate_cartesian_move_mp("test_location")
    )
    robot_command.add_mios_task(
        ri.mios_task_factory.generate_update_tool_call("parallel_box1")
    )
    robot_command.add_mios_task(
        ri.mios_task_factory.generate_cartesian_move_mp("test_location")
    )

    robot_command.execute_task_list_sync()


def load_tool_test(tool_name: str):
    parsed_action = {
        "action_name": "load_tool",
        "args": ["left_hand", tool_name],
    }

    robot_command = RobotCommand(
        robot_address="127.0.0.1",
        robot_port=12000,
        task_scene=scene,
        shared_data=None,
        robot_interface=ri,
    )

    robot_command.add_tasks(
        ri.mios_task_factory.generate_load_tool_skill(parsed_action)
    )

    robot_command.execute_task_list_sync()


def pick_test(object_name: str):

    parsed_action = {
        "action_name": "pick",
        "args": [None, None, object_name],
    }

    robot_command = RobotCommand(
        robot_address="127.0.0.1",
        robot_port=12000,
        shared_data=None,
        task_scene=scene,
        robot_interface=ri,
    )

    # robot_command.add_mios_task(ri.mios_task_factory.generate_gripper_home_mp())

    robot_command.add_tasks(ri.mios_task_factory.generate_pick_up_skill(parsed_action))

    robot_command.execute_task_list_sync()


def move_gripper(width: float):
    robot_command = RobotCommand(
        robot_address="127.0.0.1",
        robot_port=12000,
        task_scene=scene,
        shared_data=None,
        robot_interface=ri,
    )

    robot_command.add_task(ri.mios_task_factory.generate_gripper_move_mp(width=width))

    robot_command.execute_task_list_sync()


def home_gripper():
    robot_command = RobotCommand(
        robot_address="127.0.0.1",
        robot_port=12000,
        task_scene=scene,
        shared_data=None,
        robot_interface=ri,
    )

    robot_command.add_task(ri.mios_task_factory.generate_gripper_home_mp())

    robot_command.execute_task_list_sync()


def cartesian_move(object: str):
    robot_command = RobotCommand(
        robot_address="127.0.0.1",
        robot_port=12000,
        task_scene=scene,
        shared_data=None,
        robot_interface=ri,
    )

    robot_command.add_task(ri.mios_task_factory.generate_cartesian_move_mp(object))

    robot_command.execute_task_list_sync()


def joint_move(object: str):
    robot_command = RobotCommand(
        robot_address="127.0.0.1",
        robot_port=12000,
        task_scene=scene,
        shared_data=None,
        robot_interface=ri,
    )

    robot_command.add_task(ri.mios_task_factory.generate_joint_move_mp(object))

    robot_command.execute_task_list_sync()


def insert_test(object_name: str):
    robot_command = RobotCommand(
        robot_address="127.0.0.1",
        robot_port=12000,
        task_scene=scene,
        shared_data=None,
        robot_interface=ri,
    )

    insert_action = {
        "action_name": "insert",
        "args": [None, None, "gear3", object_name],
    }

    robot_command.add_tasks(ri.mios_task_factory.generate_insert_skill(insert_action))

    robot_command.execute_task_list_sync()


def drive_test():
    robot_command = RobotCommand(
        robot_address="127.0.0.1",
        robot_port=12000,
        task_scene=scene,
        shared_data=None,
        robot_interface=ri,
    )

    drive_action = {
        "action_name": "drive",
        "args": [None, None, None, None],
    }

    robot_command.add_tasks(ri.mios_task_factory.generate_drive_skill(drive_action))

    robot_command.execute_task_list_sync()


@execution_timer
def change_gripper(current_tool: str, new_tool: str):

    robot_command = RobotCommand(
        robot_address="127.0.0.1",
        robot_port=12000,
        task_scene=scene,
        shared_data=None,
        robot_interface=ri,
    )

    if current_tool != "defaultgripper":
        unload_tool_parsed_action = {
            "action_name": "unload_tool",
            "args": ["left_hand", current_tool],
        }

        robot_command.add_tasks(
            ri.mios_task_factory.generate_unload_tool_skill(unload_tool_parsed_action)
        )

    if new_tool != "defaultgripper":

        load_tool_parsed_action = {
            "action_name": "load_tool",
            "args": ["left_hand", new_tool],
        }

        robot_command.add_tasks(
            ri.mios_task_factory.generate_load_tool_skill(load_tool_parsed_action)
        )

    result = robot_command.execute_task_list_sync()

    if result == False:
        pause = input("failed to change gripper")


@execution_timer
def initialize_gripper():

    robot_command = RobotCommand(
        robot_address="127.0.0.1",
        robot_port=12000,
        shared_data=None,
        task_scene=scene,
        robot_interface=ri,
    )

    robot_command.add_task(
        ri.mios_task_factory.generate_joint_move_mp("initialposition")
    )

    robot_command.execute_task_list_sync()


@execution_timer
def insert_task(part1: str, part2: str):
    """
    part1: the location name of the part to be inserted
    part2: the location name of the part (usually a feature name) to be inserted into
    """
    # * first pickup, then insert

    robot_command = RobotCommand(
        robot_address="127.0.0.1",
        robot_port=12000,
        shared_data=None,
        task_scene=scene,
        robot_interface=ri,
    )

    pick_up_parsed_action = {
        "action_name": "pick",  # ! maybe this should be "pick_up"
        "args": [None, None, part1],
    }

    robot_command.add_tasks(
        ri.mios_task_factory.generate_pick_up_skill(pick_up_parsed_action)
    )

    insert_parsed_action = {
        "action_name": "insert",
        "args": [None, None, part1, part2],
    }

    # ! warning: the location of part1 will be refreshed after the insertion!
    robot_command.add_tasks(
        ri.mios_task_factory.generate_insert_skill_mod(insert_parsed_action)
    )

    robot_command.execute_task_list_sync()


@execution_timer
def insert_outerring():
    robot_command = RobotCommand(
        robot_address="127.0.0.1",
        robot_port=12000,
        shared_data=None,
        task_scene=scene,
        robot_interface=ri,
    )

    pick_up_parsed_action = {
        "action_name": "pick",  # ! maybe this should be "pick_up"
        "args": [None, None, "outerring"],
    }

    robot_command.add_tasks(
        ri.mios_task_factory.generate_pick_up_skill(pick_up_parsed_action)
    )

    insert_parsed_action = {
        "action_name": "insert",
        "args": [None, None, "outerring", "housingringhole"],
    }

    param = {
        "search_a": [2, 2, 2, 1, 1, 0],
        "search_f": [1, 1, 1, 1.5, 1.5, 0],
        "F_ext_contact": [10.0, 2.0],
        "f_push": [0, 0, 3, 0, 0, 0],
        "K_x": [300, 300, 0, 500, 500, 800],
        "env_X": [0.01, 0.01, 0.001, 0.05, 0.05, 0.05],
    }

    # ! warning: the location of part1 will be refreshed after the insertion!
    robot_command.add_tasks(
        ri.mios_task_factory.generate_insert_skill_mod(
            insert_parsed_action, param=param
        )
    )

    robot_command.execute_task_list_sync()


@execution_timer
def insert_cone():
    robot_command = RobotCommand(
        robot_address="127.0.0.1",
        robot_port=12000,
        shared_data=None,
        task_scene=scene,
        robot_interface=ri,
    )

    pick_up_parsed_action = {
        "action_name": "pick",  # ! maybe this should be "pick_up"
        "args": [None, None, "cone"],
    }

    robot_command.add_tasks(
        ri.mios_task_factory.generate_pick_up_skill(pick_up_parsed_action)
    )

    insert_parsed_action = {
        "action_name": "insert",
        "args": [None, None, "cone", "outerring"],
    }

    param = {
        "search_a": [2, 2, 1, 1, 1, 0],
        "search_f": [1, 1, 1, 1.5, 1.5, 0],
        "F_ext_contact": [10.0, 2.0],
        "f_push": [0, 0, 5, 0, 0, 0],
        "K_x": [500, 500, 0, 500, 500, 800],
    }

    # ! warning: the location of part1 will be refreshed after the insertion!
    robot_command.add_tasks(
        ri.mios_task_factory.generate_insert_skill_mod(
            insert_parsed_action, param=param
        )
    )

    robot_command.execute_task_list_sync()


@execution_timer
def insert_outputshaftandgearstage2():

    robot_command = RobotCommand(
        robot_address="127.0.0.1",
        robot_port=12000,
        shared_data=None,
        task_scene=scene,
        robot_interface=ri,
    )

    pick_up_parsed_action = {
        "action_name": "pick",  # ! maybe this should be "pick_up"
        "args": [None, None, "outputshaftandgearstage2"],
    }

    robot_command.add_tasks(
        ri.mios_task_factory.generate_pick_up_skill(pick_up_parsed_action)
    )

    insert_parsed_action = {
        "action_name": "insert",
        "args": [None, None, "outputshaftandgearstage2", "housinginternalgear"],
    }

    param = {
        "release_width": 0.04,
        "search_a": [10, 10, 2, 3, 3, 1],
        "search_f": [1, 1, 1, 1.5, 1.5, 1],
        "F_ext_contact": [13.0, 2.0],
        "f_push": [0, 0, 7, 0, 0, 0],
        "K_x": [500, 500, 0, 500, 500, 800],
    }

    # ! warning: the location of part1 will be refreshed after the insertion!
    robot_command.add_tasks(
        ri.mios_task_factory.generate_insert_skill_mod(
            parsed_action=insert_parsed_action,
            param=param,
        )
    )

    result = robot_command.execute_task_list_sync()
    if result == False:
        pause = input("failed to insert outputshaftandgearstage2")

    move_gripper(0.08)

    change_gripper("defaultgripper", "inwardgripper")

    robot_command.clear_tasks()

    insert_parsed_action = {
        "action_name": "insert",
        "args": [
            None,
            None,
            "outputshaftandgearstage2",
            "outputshaftandgearstage2push",
        ],
    }

    param = {
        "search_a": [1, 1, 0, 2, 2, 40],
        "search_f": [1, 1, 0, 1, 1, 0.25],
        "F_ext_contact": [10.0, 2.0],
        "f_push": [0, 0, 20, 0, 0, 0],
        "K_x": [500, 500, 0, 800, 800, 70],
        "D_x": [0.7, 0.7, 1.2, 0.7, 0.7, 1.2],
    }

    # ! warning: the location of part1 will be refreshed after the insertion!
    robot_command.add_tasks(
        ri.mios_task_factory.generate_insert_skill_mod(
            parsed_action=insert_parsed_action,
            param=param,
        )
    )

    robot_command.execute_task_list_sync()

    change_gripper("inwardgripper", "defaultgripper")


def push_outputshaftandgearstage2():
    robot_command = RobotCommand(
        robot_address="127.0.0.1",
        robot_port=12000,
        shared_data=None,
        task_scene=scene,
        robot_interface=ri,
    )

    insert_parsed_action = {
        "action_name": "insert",
        "args": [
            None,
            None,
            "outputshaftandgearstage2",
            "outputshaftandgearstage2push",
        ],
    }

    param = {
        "width": 0.035,
        "search_a": [1, 1, 0, 2, 2, 40],
        "search_f": [1, 1, 0, 1, 1, 0.25],
        "F_ext_contact": [10.0, 2.0],
        "f_push": [0, 0, 18, 0, 0, 0],
        "K_x": [500, 500, 0, 800, 800, 70],
        "D_x": [0.7, 0.7, 1.2, 0.7, 0.7, 1.2],
    }

    # ! warning: the location of part1 will be refreshed after the insertion!
    robot_command.add_tasks(
        ri.mios_task_factory.generate_insert_skill_mod(
            parsed_action=insert_parsed_action,
            param=param,
        )
    )

    robot_command.execute_task_list_sync()


@execution_timer
def insert_ringgear():
    robot_command = RobotCommand(
        robot_address="127.0.0.1",
        robot_port=12000,
        shared_data=None,
        task_scene=scene,
        robot_interface=ri,
    )

    robot_command.add_task(
        ri.mios_task_factory.generate_gripper_grasp_mp(),
    )

    robot_command.add_task(
        ri.mios_task_factory.generate_move_above_mp("ringgear"),
    )
    # !
    # insert_parsed_action = {
    #     "action_name": "insert",
    #     "args": [
    #         None,
    #         None,
    #         "ringgear",
    #         "ringgear",
    #     ],
    # }

    # param = {
    #     "search_a": [8, 8, 0, 0, 0, 0],
    #     "search_f": [1, 1, 0, 0, 0, 0],
    #     "F_ext_contact": [13.0, 2.0],
    #     "f_push": [0, 0, 8, 0, 0, 0],
    #     "K_x": [300, 300, 0, 600, 600, 500],
    #     "D_x": [0.8, 0.8, 0.7, 0.7, 0.7, 0.7],
    #     # "env_X": [0.01, 0.01, 0.001, 0.05, 0.05, 0.05],
    # }

    # # ! warning: the location of part1 will be refreshed after the insertion!
    # robot_command.add_tasks(
    #     ri.mios_task_factory.generate_insert_skill_standalone_mod(
    #         parsed_action=insert_parsed_action,
    #         param=param,
    #     )
    # )

    robot_command.add_task(
        ri.mios_task_factory.generate_reach_mp("ringgear"),
    )

    robot_command.add_task(
        ri.mios_task_factory.generate_gripper_release_mp(width=0.023),
    )

    robot_command.add_task(
        ri.mios_task_factory.generate_move_above_mp("ringgear"),
    )

    # * skip teach new pose for ringgear

    insert_parsed_action = {
        "action_name": "insert",
        "args": [
            None,
            None,
            "ringgear",
            "housingringgearhole",
        ],
    }

    param = {
        "search_a": [8, 8, 1, 2, 2, 0],
        "search_f": [1, 1, 1, 1, 1, 0],
        "F_ext_contact": [13.0, 2.0],
        "f_push": [0, 0, 3, 0, 0, 0],
        "K_x": [500, 500, 0, 300, 300, 500],
        "D_x": [0.7, 0.7, 0, 0.7, 0.7, 0.7],
        "env_X": [0.01, 0.01, 0.001, 0.05, 0.05, 0.05],
    }

    # ! warning: the location of part1 will be refreshed after the insertion!
    robot_command.add_tasks(
        ri.mios_task_factory.generate_insert_skill_standalone_mod(
            parsed_action=insert_parsed_action,
            param=param,
        )
    )

    robot_command.add_task(
        ri.mios_task_factory.generate_gripper_grasp_mp(),
    )

    robot_command.add_task(
        ri.mios_task_factory.generate_move_above_mp("housingringgearhole"),
    )

    robot_command.add_task(
        ri.mios_task_factory.generate_gripper_release_mp(width=0.042),
    )

    robot_command.execute_task_list_sync()

    robot_command.clear_tasks()


@execution_timer
def insert_designring():
    robot_command = RobotCommand(
        robot_address="127.0.0.1",
        robot_port=12000,
        shared_data=None,
        task_scene=scene,
        robot_interface=ri,
    )

    pick_up_parsed_action = {
        "action_name": "pick",  # ! maybe this should be "pick_up"
        "args": [None, None, "designring"],
    }

    robot_command.add_tasks(
        ri.mios_task_factory.generate_pick_up_skill(pick_up_parsed_action)
    )

    insert_parsed_action = {
        "action_name": "insert",
        "args": [None, None, "designring", "designringhole"],
    }

    param = {
        "search_a": [12, 12, 1, 2, 2, 0],
        "search_f": [1, 1, 1, 1.5, 1.5, 0],
        "F_ext_contact": [10.0, 2.0],
        "f_push": [0, 0, 10, 0, 0, 0],
        "K_x": [500, 500, 0, 500, 500, 800],
    }

    # ! warning: the location of part1 will be refreshed after the insertion!
    robot_command.add_tasks(
        ri.mios_task_factory.generate_insert_skill_mod(
            insert_parsed_action, param=param
        )
    )

    robot_command.execute_task_list_sync()


@execution_timer
def insert_gearstage1():
    robot_command = RobotCommand(
        robot_address="127.0.0.1",
        robot_port=12000,
        shared_data=None,
        task_scene=scene,
        robot_interface=ri,
    )

    pick_up_parsed_action = {
        "action_name": "pick",  # ! maybe this should be "pick_up"
        "args": [None, None, "gearstage1"],
    }

    robot_command.add_tasks(
        ri.mios_task_factory.generate_pick_up_skill(pick_up_parsed_action)
    )

    insert_parsed_action = {
        "action_name": "insert",
        "args": [None, None, "gearstage1", "gearstage1hole"],
    }

    param = {
        "release_width": 0.03,
        "search_a": [1, 1, 2, 1, 1, 40],
        "search_f": [1, 1, 1, 1, 1, 0.25],
        "F_ext_contact": [10.0, 2.0],
        "f_push": [0, 0, 20, 0, 0, 0],
        "K_x": [500, 500, 0, 800, 800, 70],
        "D_x": [0.7, 0.7, 0, 0.7, 0.7, 1.2],
    }

    # ! warning: the location of part1 will be refreshed after the insertion!
    robot_command.add_tasks(
        ri.mios_task_factory.generate_insert_skill_mod(
            parsed_action=insert_parsed_action,
            param=param,
        )
    )

    result = robot_command.execute_task_list_sync()
    if result == False:
        pause = input("failed to insert gearstage1")

    robot_command.clear_tasks()

    robot_command.add_task(
        ri.mios_task_factory.generate_gripper_move_mp(width=0.03),
    )

    insert_parsed_action = {
        "action_name": "insert",
        "args": [
            None,
            None,
            "gearstage1",
            "gearstage1push",
        ],
    }

    param = {
        "search_a": [1, 1, 3, 2, 2, 40],
        "search_f": [1, 1, 1, 1, 1, 0.25],
        "F_ext_contact": [10.0, 2.0],
        "f_push": [0, 0, 26, 0, 0, 0],
        "K_x": [500, 500, 0, 800, 800, 70],
        "D_x": [0.7, 0.7, 1.2, 0.7, 0.7, 1.2],
    }

    # ! warning: the location of part1 will be refreshed after the insertion!
    robot_command.add_tasks(
        ri.mios_task_factory.generate_insert_skill_mod(
            parsed_action=insert_parsed_action,
            param=param,
        )
    )

    robot_command.execute_task_list_sync()


@execution_timer
def insert_driveflange():
    robot_command = RobotCommand(
        robot_address="127.0.0.1",
        robot_port=12000,
        shared_data=None,
        task_scene=scene,
        robot_interface=ri,
    )

    pick_up_parsed_action = {
        "action_name": "pick",  # ! maybe this should be "pick_up"
        "args": [None, None, "driveflange"],
    }

    robot_command.add_tasks(
        ri.mios_task_factory.generate_pick_up_skill(pick_up_parsed_action)
    )

    insert_parsed_action = {
        "action_name": "insert",
        "args": [None, None, "driveflange", "driveflangehole"],
    }

    param = {
        "width": 0.08,
        "search_a": [8, 8, 1, 2, 2, 0],
        "search_f": [1, 1, 1, 1.5, 1.5, 0],
        "F_ext_contact": [10.0, 2.0],
        "f_push": [0, 0, 10, 0, 0, 0],
        "K_x": [500, 500, 0, 500, 500, 800],
    }

    # ! warning: the location of part1 will be refreshed after the insertion!
    robot_command.add_tasks(
        ri.mios_task_factory.generate_insert_skill_mod(
            insert_parsed_action, param=param
        )
    )

    robot_command.execute_task_list_sync()


@execution_timer
def insert_inputpinion():
    robot_command = RobotCommand(
        robot_address="127.0.0.1",
        robot_port=12000,
        shared_data=None,
        task_scene=scene,
        robot_interface=ri,
    )

    pick_up_parsed_action = {
        "action_name": "pick",  # ! maybe this should be "pick_up"
        "args": [None, None, "inputpinion"],
    }

    robot_command.add_tasks(
        ri.mios_task_factory.generate_pick_up_skill(pick_up_parsed_action)
    )

    insert_parsed_action = {
        "action_name": "insert",
        "args": [None, None, "inputpinion", "inputpinionhole"],
    }

    param = {
        "search_a": [3, 3, 0, 2, 2, 20],
        "search_f": [1.5, 1.5, 0.1, 1.5, 1.5, 0.25],
        "F_ext_contact": [10.0, 2.0],
        "f_push": [0, 0, 20, 0, 0, 0],
        "K_x": [300, 300, 0, 500, 500, 100],
        "D_x": [0.7, 0.7, 1.2, 0.7, 0.7, 1.2],
        "env_X": [0.01, 0.01, -0.002, 0.05, 0.05, 0.05],
    }

    # ! warning: the location of part1 will be refreshed after the insertion!
    robot_command.add_tasks(
        ri.mios_task_factory.generate_insert_skill_mod(
            insert_parsed_action, param=param
        )
    )

    robot_command.execute_task_list_sync()
