from kios_robot.robot_interface import RobotInterface


def test_robot_interface():
    ri = RobotInterface()
    print(ri.test_connection())
    ri.proprioceptor.teach_object("Grab")


def test_kios_load_and_unload_tool():
    print(
        "please make sure your has taught the tool position in mios! press enter to continue."
    )
    input("Press enter to continue.")
    ri = RobotInterface()
    ri.skill_engine.load_tool("parallel_box1")
    ri.actuator.cartesian_move(object="middle_place")
    ri.skill_engine.unload_tool("parallel_box1")


if __name__ == "__main__":
    # test_robot_interface()
    test_kios_load_and_unload_tool()
