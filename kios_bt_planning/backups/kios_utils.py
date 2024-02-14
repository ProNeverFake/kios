from enum import Enum, auto


class ResultCode(Enum):
    SUCCESS = auto()
    FAILURE = auto()
    ERROR = auto()


class ActionPhase(Enum):
    FINISH = (999,)
    CONDITION = (-9,)

    ERROR = (-1,)
    INITIALIZATION = (0,)
    APPROACH = (1,)

    RECOVER = (10,)
    CARTESIAN_MOVE = (11,)
    JOINT_MOVE = (12,)
    GRIPPER_FORCE = (13,)
    GRIPPER_MOVE = (14,)
    CONTACT = (15,)
    WIGGLE = (16,)

    TOOL_LOAD = (20,)
    TOOL_UNLOAD = (21,)
    TOOL_GRASP = (22,)
    TOOL_RELEASE = (23,)
    TOOL_PICK = (24,)
    TOOL_PLACE = (25,)

    GRIPPER_RELEASE = (26,)
    GRIPPER_GRASP = (27,)
    GRIPPER_PICK = (28,)
    GRIPPER_PLACE = (29,)
