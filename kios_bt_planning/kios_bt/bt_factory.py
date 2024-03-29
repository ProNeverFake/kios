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
from typing import List, Dict, Any, Tuple
from kios_bt.behavior_nodes import ActionNode, ConditionNode, ActionNodeTest

from kios_bt.pybt_io import BehaviorTreeTemplates

from kios_world.world_interface import WorldInterface
from kios_robot.robot_interface import RobotInterface

from kios_utils.skeleton_parser import (
    parse_node_name,
    ParsedNode,
    ground_action,
    parse_node_type,
)


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

    world_interface: WorldInterface = None
    robot_interface: RobotInterface = None

    id_generator = id_generator()

    def __init__(
        self,
        bt_templates: BehaviorTreeTemplates = None,
        world_interface: WorldInterface = None,
        robot_interface: RobotInterface = None,
        visualization_only: bool = False,
    ):
        """
        initialize the subtree factory with lists of preconditions,
          actions, and effects that are available to the factory
        """
        # if bt_templates is None:
        #     self.bt_templates = BehaviorTreeTemplates()
        #     self.bt_templates.initialize()
        # else:
        #     self.bt_templates = bt_templates

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
    # * visualization only, dirty imp.
    def parse_name(self, name: str) -> str:
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
                raise ValueError("unable to determine the type of the node!")

            json_data["type_name"] = self.parse_name(json_data["name"])

        if json_data["type_name"] == "selector":
            control_flow_node = py_trees.composites.Selector(
                name=json_data["name"], memory=False
            )
            for child in json_data["children"]:
                child_node = self.from_json_to_simple_bt(child)
                control_flow_node.add_child(child_node)

            return control_flow_node

        elif json_data["type_name"] == "sequence":
            control_flow_node = py_trees.composites.Sequence(
                name=json_data["name"], memory=False
            )
            for child in json_data["children"]:
                child_node = self.from_json_to_simple_bt(child)
                control_flow_node.add_child(child_node)

            return control_flow_node

        elif json_data["type_name"] in ["precondition", "condition", "target"]:
            return self.from_json_to_simple_condition_node(json_data)
        elif json_data["type_name"] == "action":
            return self.from_json_to_simple_action_node(json_data)
        else:
            raise ValueError(f"unknown type name {json_data['type_name']}")

    def from_json_to_simple_condition_node(self, json_data: dict) -> ConditionNode:
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

    def from_json_to_simple_action_node(
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

    # def register_node(self, json_data: dict):
    #     """
    #     register a node to the roster
    #     """
    #     if self.visualization_only:
    #         # do nothing if it's visualization only
    #         return
    #     node = self.from_json_to_tree_root(json_data)
    #     self.roster[json_data["identifier"]] = node

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
    # * json to bt_stw
    def from_skeleton_to_behavior_tree(
        self, json_data: dict
    ) -> Tuple[Dict[str, Any], py_trees.trees.BehaviourTree]:

        # ! clear roster here
        self.roster = {}

        tree_root = self.from_skeleton_to_tree_root(json_data)
        behavior_tree = py_trees.trees.BehaviourTree(tree_root)
        return [self.roster, behavior_tree]

    def from_skeleton_to_tree_root(self, skeleton: dict):
        """
        generate a bt based on a skeleton
        """
        if "name" not in skeleton.keys():
            raise ValueError("name is missing in the skeleton.")
        parsed_type = parse_node_type(skeleton["name"])
        if parsed_type == "selector":
            control_flow_node = py_trees.composites.Selector(
                name=skeleton["name"], memory=False
            )

            if "identifier" not in skeleton.keys():
                skeleton["identifier"] = next(self.id_generator)

            self.roster[skeleton["identifier"]] = control_flow_node

            for child in skeleton["children"]:
                child_node = self.from_skeleton_to_tree_root(child)
                control_flow_node.add_child(child_node)
            return control_flow_node
        elif parsed_type == "sequence":
            control_flow_node = py_trees.composites.Sequence(
                name=skeleton["name"], memory=False
            )
            if "identifier" not in skeleton.keys():
                skeleton["identifier"] = next(self.id_generator)
            self.roster[skeleton["identifier"]] = control_flow_node
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
        # ! here we cheat. all the condition check are now only true.
        op = ObjectProperty(
            object_name=parsed_node.params[0],
            property_name=parsed_node.itemname,
            property_value=(
                parsed_node.params[1] if len(parsed_node.params) == 2 else None
            ),
            status=True,
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

        return condition_node

    def from_skeleton_to_action_node(
        self, skeleton: dict, parsed_node: ParsedNode
    ) -> ActionNode:
        """
        from skeleton to action node
        """
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

        return action_node

    ##########################################################
    # # ! FOLLOWING FUNCTIONS ARE NOT IN USE. SHOULD BE MODIFIED LATER FOR TREE MODIFICATION
    # def generate_subtree(
    #     self,
    #     preconditions: List[ConditionNode],
    #     action: ActionNode,
    #     effects: List[ConditionNode],
    # ):
    #     """
    #     generate a subtree from a list of preconditions, an action, and a list of effects
    #     """
    #     subtree = py_trees.composites.Selector(name="subtree", memory=False)
    #     action_sequence = py_trees.composites.Sequence(
    #         name="action_sequence", memory=False
    #     )

    #     action_sequence.add_children(preconditions)
    #     action_sequence.add_child(action)
    #     subtree.add_children(effects)
    #     subtree.add_child(action_sequence)

    #     return subtree

    # def generate_action_node(self, action: Action):
    #     """
    #     generate an action node from an action
    #     """
    #     action_node = ActionNode(action, self.world_interface)
    #     return action_node

    # def generate_fake_action_node(self, action: Action):
    #     """
    #     generate an action node from an action
    #     """
    #     action_node = ActionNodeTest(action, self.world_interface)
    #     return action_node

    # def generate_condition_node(self, condition: Condition):
    #     """
    #     generate a condition node from a condition
    #     """
    #     condition_node = ConditionNode(condition, self.world_interface)
    #     return condition_node

    # def ground_action_instance(self, action: ActionInstance):  # ! discard this function
    #     """
    #     ground an action template with an action instance
    #     """

    #     # check if action exists by searching for its name
    #     action_template = self.bt_templates.get_action(action.name)
    #     if action_template is None:
    #         raise KeyError(
    #             f"action {action.name} does not exist in the behavior tree data!"
    #         )

    #     # transform it into a grounded action
    #     grounded_action = GroundedAction(
    #         tag=copy.deepcopy(action.tag),
    #         name=copy.deepcopy(action.name),
    #         mios_parameters=copy.deepcopy(action_template["mios_parameters"]),
    #         variables=copy.deepcopy(action_template["variables"]),
    #         preconditions=copy.deepcopy(action_template["preconditions"]),
    #         effects=copy.deepcopy(action_template["effects"]),
    #         purposes=copy.deepcopy(action_template["purposes"]),
    #     )

    #     grounded_action.self_ground(action)

    #     return grounded_action

    # def generate_condition_nodes(
    #     self, grounded_action: GroundedAction
    # ):  # ! discard this function
    #     """
    #     generate a list of condition nodes from a list of conditions
    #     """
    #     grounded_preconditions = grounded_action.ground_preconditions()
    #     precondition_nodes = []
    #     for precondition in grounded_preconditions:
    #         node = ConditionNode(precondition, self.world_interface)
    #         precondition_nodes.append(node)

    #     grounded_effects = grounded_action.ground_effects()
    #     effect_nodes = []
    #     for effect in grounded_effects:
    #         node = ConditionNode(effect, self.world_interface)
    #         effect_nodes.append(node)
    #     # TODO: NOT condition
    #     return effect_nodes, precondition_nodes

    # def generate_action_nodes(  # ! discard this function
    #     self, grounded_action: GroundedAction
    # ) -> List[ActionNode]:
    #     # TODO: generate a sequence of actions in the list, not just one
    #     """
    #     generate an action node
    #     """
    #     action_node = ActionNode(grounded_action, self.world_interface)

    #     return [action_node]

    # def generate_standard_subtree(  # ! discard this function
    #     self, action: ActionInstance
    # ) -> py_trees.composites.Selector:
    #     """
    #     generate a standard subtree:
    #     - selector
    #         - effect
    #         - sequence
    #             - precondition
    #             - action
    #     """

    #     grounded_action = self.ground_action_instance(action)

    #     if grounded_action is None:
    #         raise ValueError("action grounding failed!")

    #     # create the subtree
    #     subtree = py_trees.composites.Selector(name=action.tag + "_fb", memory=False)
    #     action_sequence = py_trees.composites.Sequence(
    #         name=action.tag + "_sq", memory=False
    #     )

    #     # create the effect and precondition node
    #     effect_nodes, precondition_nodes = self.generate_condition_nodes(
    #         grounded_action
    #     )

    #     # create the action node
    #     action_nodes = self.generate_action_nodes(grounded_action)

    #     # add the nodes to the tree
    #     action_sequence.add_children(precondition_nodes)
    #     action_sequence.add_children(action_nodes)
    #     subtree.add_children(effect_nodes)
    #     subtree.add_child(action_sequence)

    #     return subtree
