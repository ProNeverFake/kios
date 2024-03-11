"""
typing dataclasses for the kios_bt_planning package
"""

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
    final_node: Optional[dict]  # ! alaerm
    world_state: Dict[str, List[Dict[str, str]]]
    # raw_node: Optional[py_trees.behaviour.Behaviour]

    def to_json(self):
        return {
            "result": self.result,
            "summary": self.summary,
            "final_node": self.final_node,
            "world_state": self.world_state,
        }


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

    def to_string(self):
        if self.property_value is None:
            return f"{self.property_name}({self.object_name})"
        else:
            return f"{self.property_name}({self.object_name}, {self.property_value})"


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

    def to_string(self):
        # ! this is a temporary solution
        return self.conditions[0].to_string()


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
