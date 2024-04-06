from kios_bt.data_types import (
    # ActionInstance,
    # GroundedAction,
    Action,
    Condition,
    ObjectProperty,
)
import copy
import py_trees
import json
import re
from pprint import pprint
from typing import List, Dict, Any, Tuple, Optional
from kios_bt.behavior_nodes import ActionNode, ConditionNode, ActionNodeTest

from kios_world.world_interface import WorldInterface
from kios_robot.robot_interface import RobotInterface

from kios_utils.skeleton_parser import (
    parse_node_name,
    ParsedNode,
    ground_action,
    parse_node_type,
)
from kios_utils.pybt_test import fix_node_name


"""
bt_stewardship: the stewardship of the behavior tree, kios class.
behavior_tree: py_trees.trees.BehaviourTree, the wrapper of the tree root
tree_root: py_trees.behaviour.Behaviour, the root of the tree
"""


def id_generator(limit=1000):
    """Generate unique IDs from 0 up to limit-1."""
    for id in range(limit):
        yield id


generator = id_generator()  # Create a generator


class BehaviorTreeFactory:
    visualization_only: bool
    roster: Dict[int, Any] = {}  # * roster hasn't be tested yet.
    node_skeleton_dict: dict[any, str] = {}

    world_interface: WorldInterface = None
    robot_interface: RobotInterface = None

    id_generator = id_generator()

    def __init__(
        self,
        world_interface: WorldInterface = None,
        robot_interface: RobotInterface = None,
        visualization_only: bool = False,
    ):
        """
        initialize the subtree factory with lists of preconditions,
          actions, and effects that are available to the factory
        """

        if world_interface is None:
            self.world_interface = WorldInterface()
            self.world_interface.initialize()
        else:
            self.world_interface = world_interface

        if robot_interface is None:
            self.robot_interface = RobotInterface()
            self.robot_interface.initialize()
        else:
            self.robot_interface = robot_interface

        self.visualization_only = visualization_only

    def initialize(self):
        pass

    ##########################################################
    # ! visualization only, dirty imp.
    def parse_name(self, name: str) -> str:
        """
        get the type od the node from the name
        """
        pattern = r"(selector|sequence|target|precondition|condition|action)"
        match = re.search(pattern, name)
        if match:
            return match.group(0)
        else:
            raise ValueError(f"unable to parse the type of the node from {name}")

    def from_json_to_simple_bt(self, json_data: dict):
        """
        generate a behavior tree from a json file (but you need to parse it first)
        can be a skeleton or a complete bt
        """
        if json_data.get("type_name") is None:
            # parse the type from "name"
            if json_data.get("name") is None:
                raise ValueError(
                    f'both "type_name" and "name" are missing for node {json_data}!!!'
                )

            json_data["type_name"] = self.parse_name(json_data["name"])

        if json_data["type_name"] == "selector":
            control_flow_node = py_trees.composites.Selector(
                name=fix_node_name(json_data["name"]), memory=False
            )
            for child in json_data["children"]:
                child_node = self.from_json_to_simple_bt(child)
                control_flow_node.add_child(child_node)

            return control_flow_node

        elif json_data["type_name"] == "sequence":
            control_flow_node = py_trees.composites.Sequence(
                name=fix_node_name(json_data["name"]), memory=False
            )
            for child in json_data["children"]:
                child_node = self.from_json_to_simple_bt(child)
                control_flow_node.add_child(child_node)

            return control_flow_node

        elif json_data["type_name"] in ["precondition", "condition", "target"]:
            return self.__from_json_to_simple_condition_node(json_data)
        elif json_data["type_name"] == "action":
            return self.__from_json_to_simple_action_node(json_data)
        else:
            raise ValueError(f"unknown type name {json_data['type_name']}")

    def __from_json_to_simple_condition_node(self, json_data: dict) -> ConditionNode:
        """
        from json to condition node, and add it to the roster
        """
        condition = Condition(
            summary=json_data["summary"],
            identifier=0,
            name=json_data["name"],
            conditions=[],
        )
        condition_node = ConditionNode(
            condition,
            self.world_interface,
            self.robot_interface,
        )
        return condition_node

    def __from_json_to_simple_action_node(
        self, json_data: Dict[str, Any]
    ) -> ActionNodeTest:
        """
        from json to action node, and add it to the roster
        """
        effects = []
        action = Action(
            summary=json_data["summary"],
            identifier=0,
            name=json_data["name"],
            effects=[],
        )
        # ! BBHACK
        action_node = ActionNodeTest(
            action,
            self.world_interface,
            self.robot_interface,
        )
        return action_node

    #############################################################
    # * this is for a complete json  behavior tree
    # * json to bt_stw
    def from_json_to_behavior_tree(
        self, json_data: dict
    ) -> Tuple[Dict[str, Any], py_trees.trees.BehaviourTree]:

        tree_root = self.from_json_to_tree_root(json_data)
        behavior_tree = py_trees.trees.BehaviourTree(tree_root)
        return [self.roster, behavior_tree]

    # * json to bt
    def from_json_to_tree_root(self, json_data: dict) -> py_trees.behaviour.Behaviour:
        """
        generate a behavior tree from a json file (but you need to parse it first)
        """
        if json_data["type_name"] == "selector":
            control_flow_node = py_trees.composites.Selector(
                name=json_data["name"], memory=False
            )
            for child in json_data["children"]:
                child_node = self.from_json_to_tree_root(child)
                control_flow_node.add_child(child_node)

            self.roster[json_data["identifier"]] = control_flow_node
            return control_flow_node

        elif json_data["type_name"] == "sequence":
            control_flow_node = py_trees.composites.Sequence(
                name=json_data["name"], memory=False
            )
            for child in json_data["children"]:
                child_node = self.from_json_to_tree_root(child)
                control_flow_node.add_child(child_node)

            self.roster[json_data["identifier"]] = control_flow_node
            return control_flow_node

        elif json_data["type_name"] == "condition":
            return self.from_json_to_condition_node(json_data)
        elif json_data["type_name"] == "action":
            return self.from_json_to_action_node(json_data)

    def from_json_to_action_node(self, json_data: Dict[str, Any]) -> ActionNode:
        """
        from json to action node, and add it to the roster
        """
        effects = []
        for effect in json_data["effects"]:
            _effect = ObjectProperty(
                object_name=effect["object_name"],
                property_name=effect["property_name"],
                property_value=effect["property_value"],
                status=effect["status"],
            )
            effects.append(_effect)
        action = Action(
            summary=json_data["summary"],
            identifier=json_data["identifier"],
            name=json_data["name"],
            effects=effects,
        )
        # ! BBHACK
        action_node = ActionNode(
            action,
            self.world_interface,
            self.robot_interface,
        )
        self.roster[json_data["identifier"]] = action_node
        return action_node

    def from_json_to_condition_node(self, json_data: dict) -> ConditionNode:
        """
        from json to condition node, and add it to the roster
        """
        conditions = []
        for condition in json_data["conditions"]:
            _condition = ObjectProperty(
                object_name=condition["object_name"],
                property_name=condition["property_name"],
                property_value=condition["property_value"],
                status=condition["status"],
            )
            conditions.append(_condition)
        condition = Condition(
            summary=json_data["summary"],
            identifier=json_data["identifier"],
            name=json_data["name"],
            conditions=conditions,
        )
        condition_node = ConditionNode(
            condition,
            self.world_interface,
            self.robot_interface,
        )
        self.roster[json_data["identifier"]] = condition_node
        return condition_node

    ################################* new parse method ################################
    # * json skeleton to bt_stw
    def from_skeleton_to_behavior_tree(
        self, json_data: dict
    ) -> Tuple[Dict[str, Any], py_trees.trees.BehaviourTree, Optional[Dict[int, str]]]:
        """generate a behavior tree from a json tree skeleton.

        Args:
            json_data (dict): the skeleton of the tree

        Returns:
            A tuple of the roster, the behavior tree, and the node skeleton dict.
            roster: a dict of the nodes with their identifiers as keys. (not used yet)
            behavior_tree: the behavior tree.
            node_skeleton_dict: a dict of the skeleton nodes with their ids as keys.
        """
        # ! clear roster here
        self.roster = {}
        self.node_skeleton_dict = {}

        tree_root = self.from_skeleton_to_tree_root(json_data)
        behavior_tree = py_trees.trees.BehaviourTree(tree_root)
        return [self.roster, behavior_tree, self.node_skeleton_dict]

    def from_skeleton_to_tree_root(
        self, skeleton: dict
    ) -> py_trees.behaviour.Behaviour:
        """
        generate a bt based on a skeleton recursively
        """
        if "name" not in skeleton.keys():
            raise ValueError("name is missing in the skeleton.")
        parsed_type = parse_node_type(skeleton["name"])
        if parsed_type == "selector":
            control_flow_node = py_trees.composites.Selector(
                name=fix_node_name(skeleton["name"]), memory=False
            )

            # * skip identifier since it is not a must for a skeleton
            if "identifier" not in skeleton.keys():
                skeleton["identifier"] = next(self.id_generator)

            self.roster[skeleton["identifier"]] = control_flow_node
            # * summary
            self.node_skeleton_dict[control_flow_node.id] = {
                "summary": skeleton["summary"],
                "name": skeleton["name"],
            }

            for child in skeleton["children"]:
                child_node = self.from_skeleton_to_tree_root(child)
                control_flow_node.add_child(child_node)
            return control_flow_node
        elif parsed_type == "sequence":
            control_flow_node = py_trees.composites.Sequence(
                name=fix_node_name(skeleton["name"]), memory=False
            )
            if "identifier" not in skeleton.keys():
                skeleton["identifier"] = next(self.id_generator)
            self.roster[skeleton["identifier"]] = control_flow_node
            # * summary
            self.node_skeleton_dict[control_flow_node.id] = {
                "summary": skeleton["summary"],
                "name": skeleton["name"],
            }
            for child in skeleton["children"]:
                child_node = self.from_skeleton_to_tree_root(child)
                control_flow_node.add_child(child_node)
            return control_flow_node
        elif parsed_type in ["precondition", "condition", "target"]:
            parsed_node = parse_node_name(skeleton["name"])
            return self.from_skeleton_to_condition_node(skeleton, parsed_node)
        elif parsed_type == "action":
            parsed_node = parse_node_name(skeleton["name"])
            return self.from_skeleton_to_action_node(skeleton, parsed_node)
        else:
            raise ValueError(f"unknown type name {parsed_node.typename}")

    def from_skeleton_to_condition_node(
        self, skeleton: dict, parsed_node: ParsedNode
    ) -> ConditionNode:
        """
        from skeleton to condition node
        """
        pprint(skeleton)
        # TODO remove the limitation that the preconditions and targets are only true conditions and negative conditions (predicates with "not") only exist in the effects of actions
        op = ObjectProperty(
            object_name=parsed_node.params[0],
            property_name=parsed_node.itemname,
            property_value=(
                parsed_node.params[1] if len(parsed_node.params) == 2 else None
            ),
            status=True,  # only check true for now.
        )
        if "identifier" not in skeleton.keys():
            skeleton["identifier"] = next(self.id_generator)

        condition = Condition(
            summary=skeleton["summary"],
            identifier=skeleton["identifier"],
            name=skeleton["name"],
            conditions=[op],
        )
        condition_node = ConditionNode(
            condition,
            self.world_interface,
            self.robot_interface,
        )

        self.roster[skeleton["identifier"]] = condition_node

        # * summary
        self.node_skeleton_dict[condition_node.id] = {
            "summary": skeleton["summary"],
            "name": skeleton["name"],
        }

        return condition_node

    def from_skeleton_to_action_node(
        self, skeleton: dict, parsed_node: ParsedNode
    ) -> ActionNode:
        """
        from skeleton to action node
        """
        # ground the action to get the effects
        _, effects = ground_action(parsed_node.itemname, parsed_node.params)

        if "identifier" not in skeleton.keys():
            skeleton["identifier"] = next(self.id_generator)

        action = Action(
            summary=skeleton["summary"],
            identifier=skeleton["identifier"],
            name=skeleton["name"],
            effects=effects,
        )
        action_node = ActionNode(
            action,
            self.world_interface,
            self.robot_interface,
        )
        self.roster[skeleton["identifier"]] = action_node
        # * summary
        self.node_skeleton_dict[action_node.id] = {
            "summary": skeleton["summary"],
            "name": skeleton["name"],
        }

        return action_node
