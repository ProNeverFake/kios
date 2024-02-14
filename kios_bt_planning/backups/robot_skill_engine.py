# from kios_robot.robot_actuator import RobotActuator
# from kios_robot.robot_proprioceptor import RobotProprioceptor
# import numpy as np

# # ! DISCARDED.
# ! BBREMOVE

# class RobotSkillEngine:
#     robot_actuator: RobotActuator = None
#     robot_proprioceptor: RobotProprioceptor = None

#     def __init__(
#         self,
#         robot_actuator: RobotActuator,
#         robot_proprioceptor: RobotProprioceptor,
#     ):
#         if robot_actuator is not None:
#             self.robot_actuator = robot_actuator
#         else:
#             raise Exception("robot_actuator is not set")

#         if robot_proprioceptor is not None:
#             self.robot_proprioceptor = robot_proprioceptor
#         else:
#             raise Exception("robot_proprioceptor is not set")

#     def create_guidance_pose(self, object_name: str, DeltaHT: np.ndarray):
#         # get the object pose
#         pass

#     def load_tool(self, tool_name: str):
#         print("todo: check the tool in the scene.")
#         self.robot_actuator.load_tool(tool_name=tool_name)
#         print("todo: change robot status TCP")

#     def unload_tool(self, tool_name: str):
#         print("todo: check the tool in the scene.")
#         self.robot_actuator.unload_tool(tool_name=tool_name)
#         print("todo: change robot status TCP")

#     # def cartesian_move()

#     def pick(self, object_name: str):
#         # move onto the object
#         self.robot_actuator.cartesian_move()
#         # move into the object
#         self.robot_actuator.cartesian_move()
#         # grasp the object
#         self.robot_actuator.close_gripper()
#         # retreat back highly
#         self.robot_actuator.cartesian_move()
