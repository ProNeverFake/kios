import numpy as np
import os

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
    scene_json = f.read()

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


# ! BUG
def restore_to_mios_environment(backup_name: str):
    ltm_manipulator.restore_to_mios_environment(backup_name)


def teach_object_TCP(object_name: str):
    ri.proprioceptor.teach_object_TCP(object_name)
    ri.proprioceptor.update_scene_object_from_mios(scene=scene, object_name=object_name)


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


def pick_test():

    parsed_action = {
        "action_name": "pick",
        "args": [None, None, "test_object"],
    }

    robot_command = RobotCommand(
        robot_address="127.0.0.1",
        robot_port=12000,
        shared_data=None,
        task_scene=scene,
        robot_interface=ri,
    )

    robot_command.add_mios_task(ri.mios_task_factory.generate_gripper_home_mp())

    pick_up_tasks = ri.mios_task_factory.generate_pick_up_skill(
        parsed_action=parsed_action
    )
    for task in pick_up_tasks:
        robot_command.add_task(task)

    robot_command.task_list.append(ri.mios_task_factory.generate_gripper_release_mp())

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

    unload_tool_parsed_action = {
        "action_name": "unload_tool",
        "args": ["left_hand", current_tool],
    }

    robot_command.add_tasks(
        ri.mios_task_factory.generate_unload_tool_skill(unload_tool_parsed_action)
    )

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
        ri.mios_task_factory.generate_insert_skill(insert_parsed_action)
    )

    robot_command.execute_task_list_sync()
