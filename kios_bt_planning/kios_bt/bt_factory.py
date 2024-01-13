from kios_bt.data_types import ActionInstance, GroundedAction
import copy
import py_trees
from typing import List, Dict, Any
from kios_bt.behavior_nodes import ActionNode, ConditionNode

# import kios_bt.action_nodes
# import kios_bt.condition_nodes
import kios_bt.behavior_nodes

from kios_bt.pybt_io import BehaviorTreeTemplates

from kios_world.world_interface import WorldInterface


class BehaviorTreeFactory:
    def __init__(
        self,
        bt_templates: BehaviorTreeTemplates = None,
        world_interface: WorldInterface = None,
    ):
        """
        initialize the subtree factory with lists of preconditions,
          actions, and effects that are available to the factory
        """
        if bt_templates is None:
            self.bt_templates = BehaviorTreeTemplates()
            self.bt_templates.initialize()
        else:
            self.bt_templates = bt_templates

        if world_interface is None:
            self.world_interface = WorldInterface()
            self.world_interface.initialize()

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

    def ground_action_instance(self, action: ActionInstance):
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

    def generate_condition_nodes(self, grounded_action: GroundedAction):
        """
        generate a list of condition nodes from a list of conditions
        """
        grounded_preconditions = grounded_action.ground_preconditions()
        precondition_nodes = []
        for precondition in grounded_preconditions:
            node = kios_bt.behavior_nodes.ConditionNode(
                precondition, self.world_interface
            )
            precondition_nodes.append(node)

        grounded_effects = grounded_action.ground_effects()
        effect_nodes = []
        for effect in grounded_effects:
            node = kios_bt.behavior_nodes.ConditionNode(effect, self.world_interface)
            effect_nodes.append(node)
        # TODO: NOT condition
        return effect_nodes, precondition_nodes

    def generate_action_nodes(
        self, grounded_action: GroundedAction
    ) -> List[ActionNode]:
        # TODO: generate a sequence of actions in the list, not just one
        """
        generate an action node
        """
        action_node = kios_bt.behavior_nodes.ActionNode(
            grounded_action, self.world_interface
        )

        return [action_node]

    def generate_standard_subtree(
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
