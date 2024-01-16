from kios_bt.data_types import Action, Condition, ObjectProperty, ControlFlow

from kios_bt.behavior_nodes import ActionNode, ConditionNode, ActionNodeTest

from kios_world.world_interface import WorldInterface

from kios_bt.bt_factory import BehaviorTreeFactory

from kios_bt.bt_stewardship import BehaviorTreeStewardship

from kios_utils.pybt_test import (
    generate_bt_stewardship,
    tick_once_test,
    render_dot_tree,
    tick_loop_test,
)

from typing import List, Dict, Any

import json

import py_trees


# class TestClass:
#     roster: Dict[int, Any] = {}

#     def __init__(self) -> None:
#         self.world_interface = WorldInterface()

#     def from_json_to_bt(self, json_data: dict):
#         """
#         generate a behavior tree from a json file
#         """
#         if json_data["type_name"] == "selector":
#             control_flow_node = py_trees.composites.Selector(
#                 name=json_data["name"], memory=False
#             )
#             for child in json_data["children"]:
#                 child_node = self.from_json_to_bt(child)
#                 control_flow_node.add_child(child_node)

#             self.roster[json_data["identifier"]] = control_flow_node
#             return control_flow_node

#         elif json_data["type_name"] == "sequence":
#             control_flow_node = py_trees.composites.Sequence(
#                 name=json_data["name"], memory=False
#             )
#             for child in json_data["children"]:
#                 child_node = self.from_json_to_bt(child)
#                 control_flow_node.add_child(child_node)

#             self.roster[json_data["identifier"]] = control_flow_node
#             return control_flow_node

#         elif json_data["type_name"] == "condition":
#             return self.from_json_to_condition_node(json_data)
#         elif json_data["type_name"] == "action":
#             return self.from_json_to_action_node(json_data)

#     def from_json_to_action_node(self, json_data: dict) -> ActionNode:
#         """
#         from json to action node, and add it to the roster
#         """
#         effects = []
#         for effect in json_data["effects"]:
#             _effect = ObjectProperty(
#                 object_name=effect["object_name"],
#                 property_name=effect["property_name"],
#                 property_value=effect["property_value"],
#                 status=effect["status"],
#             )
#             effects.append(_effect)
#         action = Action(
#             summary=json_data["summary"],
#             identifier=json_data["identifier"],
#             name=json_data["name"],
#             effects=effects,
#         )
#         action_node = ActionNodeTest(action, self.world_interface)
#         self.roster[json_data["identifier"]] = action_node
#         return action_node

#     def from_json_to_condition_node(self, json_data: dict) -> ConditionNode:
#         """
#         from json to condition node, and add it to the roster
#         """
#         conditions = []
#         for condition in json_data["conditions"]:
#             _condition = ObjectProperty(
#                 object_name=condition["object_name"],
#                 property_name=condition["property_name"],
#                 property_value=condition["property_value"],
#                 status=condition["status"],
#             )
#             conditions.append(_condition)
#         condition = Condition(
#             summary=json_data["summary"],
#             identifier=json_data["identifier"],
#             name=json_data["name"],
#             conditions=conditions,
#         )
#         condition_node = ConditionNode(condition, self.world_interface)
#         self.roster[json_data["identifier"]] = condition_node
#         return condition_node


bt_json = {
    "name": "Pick Up Apple",
    "identifier": 0,
    "type_name": "selector",
    "children": [
        {
            "summary": "check if the apple is in hand",
            "name": "check apple in hand",
            "identifier": 1,
            "type_name": "condition",
            "conditions": [
                {
                    "object_name": "apple",
                    "property_name": "in",
                    "property_value": "hand",
                    "status": True,
                }
            ],
        },
        {
            "name": "Pick Up Sequence",
            "identifier": 2,
            "type_name": "sequence",
            "children": [
                {
                    "summary": "check if the apple is on the ground",
                    "name": "check apple on the ground",
                    "identifier": 3,
                    "type_name": "condition",
                    "conditions": [
                        {
                            "object_name": "apple",
                            "property_name": "on_the_ground",
                            "property_value": None,
                            "status": True,
                        }
                    ],
                },
                {
                    "summary": "check if the hand is free",
                    "name": "check hand free",
                    "identifier": 4,
                    "type_name": "condition",
                    "conditions": [
                        {
                            "object_name": "hand",
                            "property_name": "free",
                            "property_value": None,
                            "status": True,
                        }
                    ],
                },
                {
                    "summary": "pick up the apple",
                    "name": "pick_up",
                    "identifier": 5,
                    "type_name": "action",  "object_name": "hand",
                            "property_name": "free",
                            "property_value": None,
                            "status": False,
                    "effects": [
                        {
                            "object_name": "apple",
                            "property_name": "on_the_ground",
                            "property_value": None,
                            "status": False,
                        },
                        {
                            "object_name": "apple",
                            "property_name": "in",
                            "property_value": "hand",
                            "status": True,
                        },
                        {
                            "object_name": "hand",
                            "property_name": "free",
                            "property_value": None,
                            "status": False,
                        },
                    ],
                },
            ],
        },
    ],
}

# be_stewardship = BehaviorTreeStewardship()

test_class = BehaviorTreeFactory()
bt = test_class.from_json_to_bt(bt_json)
bt_stewardship = generate_bt_stewardship(bt)
bt_stewardship.setup(timeout=15)
render_dot_tree(bt_stewardship)

tick_loop_test(bt_stewardship)
