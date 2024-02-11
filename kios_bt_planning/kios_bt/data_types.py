"""
typing dataclasses for the kios_bt_planning package
"""

# ! most of the data types here are deprecated and not used in the current version of the package

from dataclasses import dataclass
from typing import Dict, List, Any, Optional

import py_trees
import json


# * in use
@dataclass
class TreeResult:
    """
    a result from the simulation
    """

    result: str
    summary: str
    defect_node: Optional[dict]

    def to_json(self):
        return {
            "result": self.result,
            "summary": self.summary,
            "defect_node": self.defect_node,
        }


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
    variables: Dict[str, str]  # ["l_from", "l_to"]


@dataclass
class GroundedAction:
    """
    an action from the action template, with necessary objects grounded
    """

    tag: str
    name: str
    mios_parameters: Dict[str, Any]
    variables: Dict[str, Any]
    preconditions: Dict[str, Dict[str, Dict[str, Any]]]
    effects: Dict[str, Dict[str, Dict[str, Any]]]
    purposes: Dict[str, Dict[str, Dict[str, Any]]]

    def self_ground(self, action: ActionInstance):
        """
        ground an action template with an action instance
        """
        self.tag = action.tag
        self.name = action.name
        # * ground the extended action
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

        # * ground mios parameters
        for key, value in self.mios_parameters["skill_objects"].items():
            if value in self.variables:
                self.mios_parameters["skill_objects"][key] = self.variables[value]
            else:
                raise KeyError(
                    f"Key {value} not found in action.variables, mios_parameters cannot be grounded!"
                )
        # ! the check below may be done at the final generation instead of here.
        # ! this is because the mios_parameters are not used in the BT generation but in robot interactions.
        for key, value in self.mios_parameters["skill_objects"].items():
            if value is None:
                raise ValueError(
                    "mios_parameters is not fully grounded. Value cannot be None."
                )

        self.mios_parameters["skill_parameters"]["skill"]["objects"] = (
            self.mios_parameters["skill_objects"]
        )

    # TODO not condition.
    def ground_preconditions(self) -> List[GroundedCondition]:
        grounded_preconditions = []
        # print(self.preconditions["true"])
        for key, value_dict in self.preconditions["true"].items():
            condition_name = key

            condition_variable_dict = {}
            # print(value_dict)
            if value_dict is None:
                raise ValueError(
                    "Preconditions should always have a value in the precidates! Value cannot be None."
                )
            elif len(value_dict) == 0:
                raise ValueError("Preconditions value list cannot be empty!")
            else:
                for key, value in value_dict.items():
                    if key in self.variables:
                        value_dict[key] = self.variables[key]
                        condition_variable_dict[key] = self.variables[key]
                    else:
                        raise ValueError(
                            f"variable {key} was not found in the grounded variable lists in the grounded action!!!"
                        )
            # Configure the tag of the grounded precondition
            # print(condition_variable_dict)
            variables_str = "".join(
                [f"{key}: {value}, " for key, value in condition_variable_dict.items()]
            )
            condition_tag = condition_name + ": " + variables_str
            grounded_precondition = GroundedCondition(
                name=condition_name,
                variables=condition_variable_dict,
                tag=condition_tag,
            )
            grounded_preconditions.append(grounded_precondition)
        return grounded_preconditions

    # TODO: not condition
    def ground_effects(self) -> List[GroundedCondition]:
        grounded_effects = []
        for key, value_list in self.effects["true"].items():
            condition_name = key
            condition_variable_dict = {}
            if value_list is None:
                raise ValueError(
                    "Effects should always have a value in the precidates! Value cannot be None."
                )
            elif len(value_list) == 0:
                raise ValueError("Effects value list cannot be empty!")
            else:
                for item in value_list:
                    if item in self.variables:
                        condition_variable_dict[item] = self.variables[item]
                    else:
                        raise ValueError(
                            f"variable {item} was not found in the grounded variable lists in the grounded action!!!"
                        )
            # Configure the tag of the grounded precondition
            variables_str = "".join(
                [f"{key}: {value}, " for key, value in condition_variable_dict.items()]
            )
            condition_tag = condition_name + ": " + variables_str
            grounded_effect = GroundedCondition(
                name=condition_name,
                variables=condition_variable_dict,
                tag=condition_tag,
            )
            grounded_effects.append(grounded_effect)
        return grounded_effects


@dataclass
class Fluent:  # not used yet
    """
    a fluent is a condition that is numeric
    """

    name: str
    variables: List[List[float]]


@dataclass
class Predicate:  # not used yet
    """
    a predicate is a condition that takes a list of str and is true by default
    """

    name: str
    variables: List[List[str]]


################################################################## * in use ##################################################################
@dataclass
class ObjectProperty:  # for indicating the conditions and the effects in an object-centric way
    """
    a property of an object
    if it is a property, then the property_value should be None
    if it is a relation, then the property_value should be the name of the target object
    status: True if the property is true, False if the property is false
    """

    object_name: str
    property_name: str
    property_value: str
    status: bool

    def to_json(self):
        return {
            "object_name": self.object_name,
            "property_name": self.property_name,
            "property_value": self.property_value,
            "status": self.status,
        }


@dataclass
class Condition:  # for generating a condition node
    summary: str
    identifier: int
    name: str  # the name of the condition you want this node to check
    conditions: List[ObjectProperty]

    def to_json(self):
        return {
            "summary": self.summary,
            "identifier": self.identifier,
            "name": self.name,
            "conditions": [condition.to_json() for condition in self.conditions],
        }


@dataclass
class Action:  # for generating an action node
    summary: str
    identifier: int
    name: str  # the name of the action you want this node to conduct
    effects: List[ObjectProperty]  # the effects of the action

    def to_json(self):
        return {
            "summary": self.summary,
            "identifier": self.identifier,
            "name": self.name,
            "effects": [effect.to_json() for effect in self.effects],
        }


@dataclass
class ControlFlow:
    identifier: int
    name: str
    children: List[Any]
