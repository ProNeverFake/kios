from enum import Enum
'''
robot status class, tailored for mios interface.
designed for indicating some status of the robot for the adjustments in skills.
for example, whether the robot is holding a tool or not can affects the TCP of the EE.
'''

class Tool(Enum):
    NoTool = 0
    ParallelTool1 = 1
    InwardCrawer = 2
    OutwardCrawer = 3
    ParallelTool2 = 4


class RobotStatus:

    Tool: Tool = Tool.NoTool

    def __init__(self):
        pass

    

