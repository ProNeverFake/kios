from kios_robot.robot_interface import RobotInterface

def test_robot_interface():
    ri = RobotInterface()
    print(ri.test_connection())
    ri.proprioceptor.teach_object("Grab")

if __name__ == "__main__":
    test_robot_interface()