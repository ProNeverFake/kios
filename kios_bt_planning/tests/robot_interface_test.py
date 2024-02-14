import numpy as np
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
scene = sf.create_test_scene()
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


def test_screw_in():
    # need object "test"
    robot_command = RobotCommand(
        robot_address="127.0.0.1",
        robot_port=12000,
        shared_data=None,
        task_scene=scene,
        robot_interface=ri,
    )

    O_T_OB = scene.get_object("test").O_T_TCP

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


def pick_test():
    sf = SceneFactory()
    scene = sf.create_test_scene()
    ri.setup_scene(scene)

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


if __name__ == "__main__":
    # tool_test()
    # pick_test()
    pass
