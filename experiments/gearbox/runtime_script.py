import numpy as np
import os
import json
from spatialmath import *
from spatialmath.base import trnorm


from kios_robot.mios_task_factory import MiosTaskFactory
from kios_robot.robot_command import RobotCommand
from kios_robot.robot_interface import RobotInterface
from kios_robot.data_types import Toolbox
from kios_scene.scene_factory import SceneFactory

from kios_scene.mios_ltm_manipulator import LangTermMemoryManipulator

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

    robot_command.execute_task_list_sync()


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
        "search_a": [5, 5, 0, 2, 2, 0],
        "search_f": [1, 1, 0, 1.5, 1.5, 0],
        "F_ext_contact": [13.0, 2.0],
        "f_push": [0, 0, 5, 0, 0, 0],
        "K_x": [500, 500, 0, 800, 800, 800],
    }

    # ! warning: the location of part1 will be refreshed after the insertion!
    robot_command.add_tasks(
        ri.mios_task_factory.generate_insert_skill_mod(
            parsed_action=insert_parsed_action,
            param=param,
        )
    )

    robot_command.execute_task_list_sync()
