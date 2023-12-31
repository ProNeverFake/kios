import pytrees

import kios_bt.action_nodes
import kios_bt.condition_nodes


class SubtreeFactory:
    def __init__(
        self,
        name=None,
        preconditions: list = None,
        actions: list = None,
        effects: list = None,
    ):
        self.preconditions = preconditions
        self.actions = actions
        self.effects = effects
        self.name = name

    def generate_standard_subtree(self, name, action):
        if name is None:
            name = "standard_subtree"
        else:
            name = name

        # find action nodes in actions

        # extract preconditions from action nodes
        # find condition nodes in preconditions

        # extract effects from action nodes
        # find condition nodes in effects

        return pytrees.Subtree(name, preconditions, effects)
