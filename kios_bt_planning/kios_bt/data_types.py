"""
typing dataclasses for the kios_bt_planning package
"""


from dataclasses import dataclass
from typing import Dict, List, Any


@dataclass
class ActionInstance:
    """
    an action from the action sequence, with necessary objects provided
    """

    tag: str
    name: str
    variables: dict


@dataclass
class GroundedAction:
    """
    an action from the action template, with necessary objects grounded
    """

    tag: str
    name: str
    variables: Dict[str, Any]
    preconditions: List[Dict[str, List[Any]]]
    effects: List[Dict[str, List[Any]]]

    def self_ground(self, action: ActionInstance):
        """
        ground an action template with an action instance
        """
        self.tag = action.tag
        self.name = action.name
        # ground the extended action
        for key, value in self.variables.items():
            if key in action.variables:
                self.variables[key] = action.variables[key]
            else:
                raise KeyError(
                    f"Key {key} not found in action.variables, action cannot be grounded!"
                )

        for key, value in self.variables.items():
            if value is None:
                raise ValueError("Action is not fully grounded. Value cannot be None.")
