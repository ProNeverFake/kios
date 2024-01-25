from kios_robot.robot_actuator import RobotActuator

# ! DISCARD THIS
"""
here to implement the real "skill" --- mp sequence
also the functionality to connect the planned actions to the real movements of the robot
"""


class RobotSkillEngine:
    robot_actuator: RobotActuator = None

    def __init__(self, robot_actuator: RobotActuator):
        if robot_actuator is not None:
            self.robot_actuator = robot_actuator
        else:
            raise Exception("robot_actuator is not set")

    def load_tool(self, tool_name: str):
        print("todo: check the tool in the scene.")
        self.robot_actuator.load_tool(tool_name=tool_name)
        print("todo: change robot status TCP")

    def unload_tool(self, tool_name: str):
        print("todo: check the tool in the scene.")
        self.robot_actuator.unload_tool(tool_name=tool_name)
        print("todo: change robot status TCP")

    # def cartesian_move()

    def pick(self, object_name: str):
        # move onto the object
        self.robot_actuator.cartesian_move()
        #
