import numpy as np

from kios_robot.mios_task_factory import MiosTaskFactory
from kios_robot.robot_command import RobotCommand
from kios_robot.robot_interface import RobotInterface
from kios_robot.data_types import Toolbox
from kios_scene.scene_factory import SceneFactory

ri = RobotInterface()


def test_robot_interface():
    print(ri.test_connection())
    ri.proprioceptor.teach_object("Grab")


def test_kios_load_and_unload_tool():
    print(
        "please make sure your has taught the tool position in mios! press enter to continue."
    )
    input("Press enter to continue.")
    ri.skill_engine.load_tool("parallel_box1")
    ri.actuator.cartesian_move(object="middle_place")
    ri.skill_engine.unload_tool("parallel_box1")


def teach_location(location: str):
    ri.proprioceptor.teach_object(location)


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
    sf = SceneFactory()
    scene = sf.create_test_scene()
    ri.setup_scene(scene)

    robot_command = RobotCommand(
        robot_address="127.0.0.1",
        robot_port=12000,
        shared_data=None,
        task_scene=None,
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
    )

    robot_command.task_list = ri.mios_task_factory.generate_pick_up_skill(
        parsed_action=parsed_action
    )

    robot_command.execute_task_list_sync()


# def tool_test1():
#     toolbox = Toolbox(
#         name="parallel_box1",
#         EE_T_TCP=np.array(
#             [
#                 [1, 0, 0, 0.0],
#                 [0, 1, 0, 0.0],
#                 [0, 0, 1, 0.1],
#                 [0, 0, 0, 1],
#             ]
#         ),
#     )

#     # print(toolbox.EE_T_TCP)
#     # print(toolbox.EE_T_TCP.flatten().tolist())
#     # return

#     robot_command = RobotCommand(
#         robot_address="127.0.0.1",
#         robot_port=12000,
#         robot_scene=None,
#     )

#     robot_command.add_mios_task(
#         MiosTaskFactory().generate_cartesian_move_HT("test_location1", toolbox)
#     )

#     robot_command.execute_task_list_sync()


if __name__ == "__main__":
    # tool_test()
    pick_test()
    pass
