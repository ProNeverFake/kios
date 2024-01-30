# import numpy as np
# from typing import List, Optional, Dict, Any

# """
# robot status class, tailored for mios interface.
# designed for indicating some status of the robot for the adjustments in skills.
# for example, whether the robot is holding a tool or not can affects the TCP of the EE.
# """

# # ! This design is a bit weird.
# # ! Use mios memory or mongodb instead.


# class RobotStatus:
#     O_T_EE: np.ndarray
#     EE_T_TCP: np.ndarray = np.eye(
#         4
#     )  # * default value is identity matrix, which means no tool is attached.
#     q: List[float] = [0, 0, 0, 0, 0, 0, 0]

#     def __init__(self):
#         pass
