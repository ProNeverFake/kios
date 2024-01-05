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
class GroundedCondition:
    """
    a condition which is grounded from its super class, the grounded action,
    """

    tag: str
    name: str  # reachable
    variables: list  # ["l_from", "l_to"]


@dataclass
class GroundedAction:
    """
    an action from the action template, with necessary objects grounded
    """

    tag: str
    name: str
    variables: Dict[str, Any]
    preconditions: Dict[str, Dict[str, List[Any]]]
    effects: Dict[str, Dict[str, List[Any]]]

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

    def ground_preconditions(self) -> List[GroundedCondition]:
        grounded_preconditions = []
        for key, value_list in self.preconditions["true"].items():
            grounded_precondition = GroundedCondition(
                name=key,
            )
            grounded_precondition.variables = []
            if value_list is None:
                raise ValueError(
                    "Preconditions should always have a value in the precidates! Value cannot be None."
                )
            elif len(value_list) == 0:
                raise ValueError("Preconditions value list cannot be empty!")
            else:
                for item in value_list:
                    if item in self.variables:
                        grounded_precondition.variables.append(self.variables[item])
                    else:
                        raise ValueError(
                            f"variable {item} was not found in the grounded variable lists in the grounded action!!!"
                        )
            # Configure the tag of the grounded precondition
            variables_str = " -".join(
                [f"{value} " for value in grounded_precondition.variables]
            )
            grounded_precondition.tag = (
                grounded_precondition.name + ": " + variables_str
            )
            grounded_preconditions.append(grounded_precondition)
        return grounded_preconditions

    def ground_effects(self) -> List[GroundedCondition]:
        grounded_effects = []
        for key, value_list in self.effects["true"].items():
            grounded_effect = GroundedCondition(
                name=key,
            )
            grounded_effect.variables = []
            if value_list is None:
                raise ValueError(
                    "Effects should always have a value in the precidates! Value cannot be None."
                )
            elif len(value_list) == 0:
                raise ValueError("Effects value list cannot be empty!")
            else:
                for item in value_list:
                    if item in self.variables:
                        grounded_effect.variables.append(self.variables[item])
                    else:
                        raise ValueError(
                            f"variable {item} was not found in the grounded variable lists in the grounded action!!!"
                        )
            # Configure the tag of the grounded precondition
            variables_str = " -".join(
                [f"{value} " for value in grounded_effect.variables]
            )
            grounded_effect.tag = grounded_effect.name + ": " + variables_str
            grounded_effects.append(grounded_effect)
        return grounded_effects
