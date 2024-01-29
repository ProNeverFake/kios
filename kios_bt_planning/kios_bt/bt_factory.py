from kios_bt.data_types import (
    ActionInstance,
    GroundedAction,
    Action,
    Condition,
    ObjectProperty,
)
import copy
import py_trees
from typing import List, Dict, Any
from kios_bt.behavior_nodes import ActionNode, ConditionNode, ActionNodeTest

# import kios_bt.behavior_nodes

from kios_bt.pybt_io import BehaviorTreeTemplates

from kios_world.world_interface import WorldInterface
from kios_robot.robot_interface import RobotInterface


class BehaviorTreeFactory:
    roster: Dict[int, Any] = {}

    world_interface: WorldInterface = None
    robot_interface: RobotInterface = None

    def __init__(
        self,
        bt_templates: BehaviorTreeTemplates = None,
        world_interface: WorldInterface = None,
        robot_interface: RobotInterface = None,
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

    def initialize(self):
        pass

    ##########################################################
    # * json to bt
    def from_json_to_bt(self, json_data: dict):
        """
        generate a behavior tree from a json file
        """
        if json_data["type_name"] == "selector":
            control_flow_node = py_trees.composites.Selector(
                name=json_data["name"], memory=False
            )
            for child in json_data["children"]:
                child_node = self.from_json_to_bt(child)
                control_flow_node.add_child(child_node)

            self.roster[json_data["identifier"]] = control_flow_node
            return control_flow_node

        elif json_data["type_name"] == "sequence":
            control_flow_node = py_trees.composites.Sequence(
                name=json_data["name"], memory=False
            )
            for child in json_data["children"]:
                child_node = self.from_json_to_bt(child)
                control_flow_node.add_child(child_node)

            self.roster[json_data["identifier"]] = control_flow_node
            return control_flow_node

        elif json_data["type_name"] == "condition":
            return self.from_json_to_condition_node(json_data)
        elif json_data["type_name"] == "action":
            return self.from_json_to_action_node(json_data)

    def from_json_to_action_node(self, json_data: dict) -> ActionNode:
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
        # ! BBCRITICAL: NOW THE TEST CLASS IS IN USE!!!
        action_node = ActionNodeTest(
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

    ##########################################################
    # * FOLLOWING FUNCTIONS ARE NOT IN USE. SHOULD BE MODIFIED LATER FOR TREE MODIFICATION
    def generate_subtree(
        self,
        preconditions: List[ConditionNode],
        action: ActionNode,
        effects: List[ConditionNode],
    ):
        """
        generate a subtree from a list of preconditions, an action, and a list of effects
        """
        subtree = py_trees.composites.Selector(name="subtree", memory=False)
        action_sequence = py_trees.composites.Sequence(
            name="action_sequence", memory=False
        )

        action_sequence.add_children(preconditions)
        action_sequence.add_child(action)
        subtree.add_children(effects)
        subtree.add_child(action_sequence)

        return subtree

    def generate_action_node(self, action: Action):
        """
        generate an action node from an action
        """
        action_node = ActionNode(action, self.world_interface)
        return action_node

    def generate_fake_action_node(self, action: Action):
        """
        generate an action node from an action
        """
        action_node = ActionNodeTest(action, self.world_interface)
        return action_node

    def generate_condition_node(self, condition: Condition):
        """
        generate a condition node from a condition
        """
        condition_node = ConditionNode(condition, self.world_interface)
        return condition_node

    def ground_action_instance(self, action: ActionInstance):  # ! discard this function
        """
        ground an action template with an action instance
        """

        # check if action exists by searching for its name
        action_template = self.bt_templates.get_action(action.name)
        if action_template is None:
            raise KeyError(
                f"action {action.name} does not exist in the behavior tree data!"
            )

        # transform it into a grounded action
        grounded_action = GroundedAction(
            tag=copy.deepcopy(action.tag),
            name=copy.deepcopy(action.name),
            mios_parameters=copy.deepcopy(action_template["mios_parameters"]),
            variables=copy.deepcopy(action_template["variables"]),
            preconditions=copy.deepcopy(action_template["preconditions"]),
            effects=copy.deepcopy(action_template["effects"]),
            purposes=copy.deepcopy(action_template["purposes"]),
        )

        grounded_action.self_ground(action)

        return grounded_action

    def generate_condition_nodes(
        self, grounded_action: GroundedAction
    ):  # ! discard this function
        """
        generate a list of condition nodes from a list of conditions
        """
        grounded_preconditions = grounded_action.ground_preconditions()
        precondition_nodes = []
        for precondition in grounded_preconditions:
            node = ConditionNode(precondition, self.world_interface)
            precondition_nodes.append(node)

        grounded_effects = grounded_action.ground_effects()
        effect_nodes = []
        for effect in grounded_effects:
            node = ConditionNode(effect, self.world_interface)
            effect_nodes.append(node)
        # TODO: NOT condition
        return effect_nodes, precondition_nodes

    def generate_action_nodes(  # ! discard this function
        self, grounded_action: GroundedAction
    ) -> List[ActionNode]:
        # TODO: generate a sequence of actions in the list, not just one
        """
        generate an action node
        """
        action_node = ActionNode(grounded_action, self.world_interface)

        return [action_node]

    def generate_standard_subtree(  # ! discard this function
        self, action: ActionInstance
    ) -> py_trees.composites.Selector:
        """
        generate a standard subtree:
        - selector
            - effect
            - sequence
                - precondition
                - action
        """

        grounded_action = self.ground_action_instance(action)

        if grounded_action is None:
            raise ValueError("action grounding failed!")

        # create the subtree
        subtree = py_trees.composites.Selector(name=action.tag + "_fb", memory=False)
        action_sequence = py_trees.composites.Sequence(
            name=action.tag + "_sq", memory=False
        )

        # create the effect and precondition node
        effect_nodes, precondition_nodes = self.generate_condition_nodes(
            grounded_action
        )

        # create the action node
        action_nodes = self.generate_action_nodes(grounded_action)

        # add the nodes to the tree
        action_sequence.add_children(precondition_nodes)
        action_sequence.add_children(action_nodes)
        subtree.add_children(effect_nodes)
        subtree.add_child(action_sequence)

        return subtree
