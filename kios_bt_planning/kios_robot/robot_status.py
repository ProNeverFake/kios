from dataclasses import dataclass, field
from enum import Enum
import numpy as np
from typing import List, Optional, Dict, Any

from kios_robot.data_types import Toolbox

"""
robot status class, tailored for mios interface.
designed for indicating some status of the robot for the adjustments in skills.
for example, whether the robot is holding a tool or not can affects the TCP of the EE.
"""


class RobotStatus:
    tool_list: dict[str, Toolbox]

    def __init__(self):
        raise NotImplementedError
