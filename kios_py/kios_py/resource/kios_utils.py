from enum import Enum, auto


class ResultCode(Enum):
    SUCCESS = auto()
    FAILURE = auto()
    ERROR = auto()


class ActionPhase(Enum):
    FINISH = 999
    CONDITION = -9

    ERROR = -1
    INITIALIZATION = 0
    APPROACH = 1

    RECOVER = 10
    CARTESIAN_MOVE = 11
    JOINT_MOVE = 12
    GRIPPER_FORCE = 13
    GRIPPER_MOVE = 14
    CONTACT = 15
    WIGGLE = 16

    TOOL_LOAD = 20
    TOOL_UNLOAD = 21
    TOOL_GRASP = 22
    TOOL_RELEASE = 23
    TOOL_PICK = 24
    TOOL_PLACE = 25

    GRIPPER_RELEASE = 26
    GRIPPER_GRASP = 27
    GRIPPER_PICK = 28
    GRIPPER_PLACE = 29


def ap_to_mios_skill(ap: ActionPhase):
    if ap == ActionPhase.CARTESIAN_MOVE:
        return "BBCartesianMove"
    elif ap == ActionPhase.JOINT_MOVE:
        return "BBJointMove"
    elif ap == ActionPhase.GRIPPER_MOVE:
        return "BBGripperMove"
    elif ap == ActionPhase.GRIPPER_FORCE:
        return "BBGripperForce"
    elif ap == ActionPhase.CONTACT:
        return "BBContact"
    elif ap == ActionPhase.WIGGLE:
        return "BBWiggle"
    elif ap == ActionPhase.TOOL_LOAD:
        return "BBToolLoad"
    elif ap == ActionPhase.TOOL_UNLOAD:
        return "BBToolLoad"
    elif ap == ActionPhase.TOOL_GRASP:
        return "BBGripperForce"
    elif ap == ActionPhase.TOOL_RELEASE:
        return "BBGripperMove"
    elif ap == ActionPhase.GRIPPER_GRASP:
        return "BBGripperForce"
    elif ap == ActionPhase.GRIPPER_RELEASE:
        return "BBGripperMove"
    elif ap == ActionPhase.TOOL_PICK:
        return "BBPick"
    elif ap == ActionPhase.TOOL_PLACE:
        return "BBPlace"
    elif ap == ActionPhase.GRIPPER_PICK:
        return "BBPick"
    elif ap == ActionPhase.GRIPPER_PLACE:
        return "BBPlace"
    else:
        # Handle the default case. Replace with appropriate logging or error handling as needed.
        # Example: print("Error: Cannot ground the action phase to a mios skill!")
        return ""
