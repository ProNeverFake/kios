from kios_bt.data_types import ActionInstance, GroundedAction

import copy

import pytrees

import kios_bt.action_nodes
import kios_bt.condition_nodes

from kios_bt.pybt_io import BehaviorTreeTemplates

# ! remove later
# @dataclass
# class ActionInstance:
#     """
#     an action from the action sequence, with necessary objects provided
#     """

#     tag: str
#     name: str
#     variables: dict


# @dataclass
# class GroundedAction:
#     """
#     an action from the action template, with necessary objects grounded
#     """

#     tag: str
#     name: str
#     variables: Dict[str, Any]
#     preconditions: List[Dict[str, List[Any]]]
#     effects: List[Dict[str, List[Any]]]

#     def self_ground(self, action: ActionInstance):
#         """
#         ground an action template with an action instance
#         """
#         self.tag = action.tag
#         self.name = action.name
#         # ground the extended action
#         for key, value in self.variables.items():
#             if key in action.variables:
#                 self.variables[key] = action.variables[key]
#             else:
#                 raise KeyError(
#                     f"Key {key} not found in action.variables, action cannot be grounded!"
#                 )

#         for key, value in self.variables.items():
#             if value is None:
#                 raise ValueError("Action is not fully grounded. Value cannot be None.")


class SubtreeFactory:
    def __init__(self, bt_templates=None):
        """
        initialize the subtree factory with lists of preconditions,
          actions, and effects that are available to the factory
        """
        if bt_templates is None:
            self.bt_templates = BehaviorTreeTemplates()
            self.bt_templates.initialize()
        else:
            self.bt_templates = bt_templates

    def ground_extended_action(self, action: ActionInstance):
        """
        ground an action template with an action instance
        """

        # check if action exists
        action_template = self.bt_templates.get_action(action.name)

        if action_template is None:
            raise KeyError(
                f"action {action.name} does not exist in the behavior tree data!"
            )

        # transform it into a grounded action
        grounded_action = GroundedAction(
            tag=action.tag,
            name=action.name,
            variables=action_template["variables"],
            preconditions=action_template["preconditions"],
            effects=action_template["effects"],
        )

        grounded_action.self_ground(action)

        # ! below is discarded
        # # ground the preconditions
        # for key, value in grounded_action["preconditions"]["true"].items():
        #     for item in value:
        #         if item in grounded_action["variables"]:
        #             grounded_action["preconditions"]["true"][key] = grounded_action[
        #                 "variables"
        #             ][item]
        #         else:
        #             raise KeyError(
        #                 f"Key {item} not found in action.variables, action cannot be grounded!"
        #             )

        # for item in grounded_action["preconditions"]["false"]:
        #     for key, value in item.items():
        #         if value in grounded_action["variables"]:
        #             item[key] = grounded_action["variables"][value]
        #         else:
        #             raise KeyError(
        #                 f"Key {value} not found in action.variables, action cannot be grounded!"
        #             )

        # # ground the effects
        # for item in grounded_action["effects"]["true"]:
        #     for
        #     if value in grounded_action["variables"]:
        #         grounded_action["effects"]["true"][key] = grounded_action["variables"][
        #             value
        #         ]
        #     else:
        #         raise KeyError(
        #             f"Key {key} not found in action.variables, action cannot be grounded!"
        #         )

        # for key, value in grounded_action["effects"]["false"].items():
        #     if value in grounded_action["variables"]:
        #         grounded_action["effects"]["false"][key] = grounded_action["variables"][
        #             value
        #         ]
        #     else:
        #         raise KeyError(
        #             f"Key {key} not found in action.variables, action cannot be grounded!"
        #         )

        return grounded_action

    def generate_condition_nodes(self, grounded_action: GroundedAction):
        """
        generate a list of condition nodes from a list of conditions
        """
        precondition_nodes = []
        for key, value in grounded_action.preconditions["true"]:
            # key is the thing to check, value is the value to check
            node = key + value.to_string()
            precondition_nodes.append(node)

        effect_nodes = []
        for key, value in grounded_action.effects["true"]:
            # key is the thing to check, value is the value to check
            node = key + value.to_string()
            effect_nodes.append(node)
        # TODO: NOT condition
        return effect_nodes, precondition_nodes

    def generate_action_nodes(self, grounded_action: GroundedAction):
        """
        generate an action node
        """
        action_node = kios_bt.action_nodes.ActionNode()

        action_node = "add action node here"
        return action_node

    def generate_standard_subtree(self, action: ActionInstance):
        """
        generate a standard subtree:
        - selector
            - effect
            - sequence
                - precondition
                - action
        """

        grounded_action = self.ground_extended_action(action)

        if grounded_action is None:
            raise ValueError("action grounding failed!")

        # create the subtree
        subtree = pytrees.composites.Selector(name=action.tag + "_fb", memory=False)
        action_sequence = pytrees.composites.Sequence(
            name=action.tag + "sq", memory=False
        )

        # create the effect and precondition node
        effect_nodes, precondition_nodes = self.generate_condition_nodes(
            grounded_action
        )

        # create the action node
        action_node = self.generate_action_nodes(grounded_action)

        # add the nodes to the tree
        action_sequence.add_children(precondition_nodes)
        action_sequence.add_children(action_node)
        subtree.add_children(effect_nodes)
        subtree.add_children(action_sequence)

        return subtree
