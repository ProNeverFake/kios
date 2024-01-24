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

    def pick(self, object_name: str):
        # move onto the object
        self.robot_actuator.cartesian_move()
        #
