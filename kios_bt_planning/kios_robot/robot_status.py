from dataclasses import dataclass
from enum import Enum
import numpy as np

"""
robot status class, tailored for mios interface.
designed for indicating some status of the robot for the adjustments in skills.
for example, whether the robot is holding a tool or not can affects the TCP of the EE.
"""


@dataclass
class Toolbox:
    name: str
    # * for future you should consider using these parameters to invoke the gripper-related skills
    load_width: float = 0.042  # the width the hand to reach in order to load this tool
    unload_width: float = (
        0.08  # the width the hand to reach in order to unload this tool
    )
    grasp_force: float = (
        70  # the force the hand to exert to fully grasp this tool to the end
    )
    grasp_speed: float = 0.5  # not used yet
    grasp_eps_in: float = 0.005
    grasp_eps_out: float = 0.005
    # kinematics parameters
    EE_O_TCP


class RobotStatus:
    Tool: Tool = Tool.NoTool

    def __init__(self):
        pass
