import numpy as np

from kios_robot.mios_task_factory import MiosTaskFactory
from kios_robot.robot_command import RobotCommand
from kios_robot.robot_interface import RobotInterface
from kios_robot.data_types import Toolbox
from kios_scene.scene_factory import SceneFactory

ri = RobotInterface()
sf = SceneFactory()
scene = sf.create_test_scene()
ri.setup_scene(scene)


def teach_object(object: str):
    ri.proprioceptor.teach_object(object)


def teach_object_TCP(object: str):
    ri.proprioceptor.teach_object_TCP(object)


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


# def transformation_test():
#     HT = np.array(
#         [
#             [1, 0, 0, 0.0],
#             [0, 1, 0, 0.0],
#             [0, 0, 1, 0.1],
#             [0, 0, 0, 1],
#         ]
#     )
#     print(HT)
#     HT_list = HT.flatten().tolist()
#     print(HT_list)
#     HT_2 = np.reshape(np.array(HT_list), (4, 4)).T
#     print(HT_2)


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


if __name__ == "__main__":
    # tool_test()
    # pick_test()
    pass
