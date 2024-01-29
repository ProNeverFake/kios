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


def raw_test():
    pass


def teach_location(location: str):
    ri.proprioceptor.teach_object(location)


def transformation_test():
    HT = np.array(
        [
            [1, 0, 0, 0.0],
            [0, 1, 0, 0.0],
            [0, 0, 1, 0.1],
            [0, 0, 0, 1],
        ]
    )
    print(HT)
    HT_list = HT.flatten().tolist()
    print(HT_list)
    HT_2 = np.reshape(np.array(HT_list), (4, 4)).T
    print(HT_2)


def tool_test():
    # ee_t_tcp = np.array(
    #     [
    #         [1, 0, 0, 0.0],
    #         [0, 1, 0, 0.0],
    #         [0, 0, 1, 0.1],
    #         [0, 0, 0, 1],
    #     ]
    # )

    # ri.proprioceptor.change_EE_T_TCP(new_EE_T_TCP=ee_t_tcp)

    toolbox = Toolbox(
        name="parallel_box1",
        EE_HT_TCP=np.array(
            [
                [1, 0, 0, 0.0],
                [0, 1, 0, 0.0],
                [0, 0, 1, 0.0],
                [0, 0, 0, 1],
            ]
        ),
    )

    # print(toolbox.EE_HT_TCP)
    # print(toolbox.EE_HT_TCP.flatten().tolist())
    # return

    robot_command = RobotCommand(
        robot_address="127.0.0.1",
        robot_port=12000,
        robot_scene=None,
    )

    robot_command.add_mios_task(
        MiosTaskFactory().generate_cartesian_move_HT("test_location", toolbox)
    )

    robot_command.execute_task_list_sync()


def tool_test1():
    toolbox = Toolbox(
        name="parallel_box1",
        EE_HT_TCP=np.array(
            [
                [1, 0, 0, 0.0],
                [0, 1, 0, 0.0],
                [0, 0, 1, 0.1],
                [0, 0, 0, 1],
            ]
        ),
    )

    # print(toolbox.EE_HT_TCP)
    # print(toolbox.EE_HT_TCP.flatten().tolist())
    # return

    robot_command = RobotCommand(
        robot_address="127.0.0.1",
        robot_port=12000,
        robot_scene=None,
    )

    robot_command.add_mios_task(
        MiosTaskFactory().generate_cartesian_move_HT("test_location1", toolbox)
    )

    robot_command.execute_task_list_sync()


if __name__ == "__main__":
    # test_robot_interface()
    # test_kios_load_and_unload_tool()
    pass
