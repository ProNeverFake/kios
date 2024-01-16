"""
a class that manages the behavior tree, help to localize the behavior nodes in
the behavior tree and help to modify the tree. (interfaces opened for planner/LLM)
"""

from typing import List, Set, Dict, Any, Tuple, Optional

from kios_bt.behavior_nodes import ActionNode, ConditionNode
from kios_bt.bt_factory import BehaviorTreeFactory
from kios_world.world_interface import WorldInterface

import py_trees

import functools


class BehaviorTreeStewardship:
    tree_stewardship: py_trees.trees.BehaviourTree = None
    behaviortree_factory: BehaviorTreeFactory = None
    world_interface: WorldInterface = None
    roster: Dict[int, Any] = {}

    def __init__(
        self,
        behaviortree_factory: BehaviorTreeFactory = None,
        world_interface: WorldInterface = None,
    ) -> None:
        if world_interface is None:
            self.world_interface = WorldInterface()
            self.world_interface.initialize()
        else:
            self.world_interface = world_interface

        if behaviortree_factory is None:
            self.behaviortree_factory = BehaviorTreeFactory(None, self.world_interface)
            self.behaviortree_factory.initialize()
        else:
            self.behaviortree_factory = behaviortree_factory

    def initialize(self):
        # map the roster from the factory
        self.roster = self.behaviortree_factory.roster

    # * tick handlers
    def post_tick_handler(
        snapshot_visitor: py_trees.visitors.SnapshotVisitor,
        behaviour_tree: py_trees.trees.BehaviourTree,
    ) -> None:
        """
        Print data about the part of the tree visited.

        Args:
            snapshot_handler: gather data about the part of the tree visited
            behaviour_tree: tree to gather data from
        """
        print(
            "\n"
            + py_trees.display.unicode_tree(
                root=behaviour_tree.root,
                visited=snapshot_visitor.visited,
                previously_visited=snapshot_visitor.previously_visited,
            )
        )
        print(py_trees.display.unicode_blackboard())

    def pre_tick_handler(behaviour_tree: py_trees.trees.BehaviourTree) -> None:
        """Print a banner with current tick count prior to ticking the tree.

        Args:
        behaviour_tree: the tree to tick (used to fetch the count number)
        """
        print("\n--------- Run %s ---------\n" % behaviour_tree.count)

    def generate_behaviortree_from_json(self, json_data: dict) -> None:
        self.setup_bt_stewardship(self.behaviortree_factory.from_json_to_bt(json_data))

    def setup_bt_stewardship(self, bt: py_trees.behaviour.Behaviour) -> None:
        ####################
        # Tree Stewardship
        ####################
        self.tree_stewardship = py_trees.trees.BehaviourTree(bt)

        self.tree_stewardship.add_pre_tick_handler(self.pre_tick_handler)
        self.tree_stewardship.visitors.append(py_trees.visitors.DebugVisitor())
        snapshot_visitor = py_trees.visitors.SnapshotVisitor()
        self.tree_stewardship.add_post_tick_handler(
            functools.partial(self.post_tick_handler, snapshot_visitor)
        )
        self.tree_stewardship.visitors.append(snapshot_visitor)
        self.tree_stewardship.setup(timeout=15)

    def render_dot_tree(self):
        py_trees.display.render_dot_tree(
            self.tree_stewardship, with_blackboard_variables=False
        )

    def tick_loop_test(self):
        while True:
            try:
                self.tree_stewardship.tick()
                py_trees.console.read_single_keypress()
            except KeyboardInterrupt:
                break

        print("\n")

    def tick_once_test(self):
        self.tree_stewardship.tick()
        py_trees.console.read_single_keypress()

        print("\n")

    def tick_once(self):
        self.tree_stewardship.tick()

    # * methods to modify the tree
    ##########################################################
    def insert_subtree(
        self,
        subtree: py_trees.behaviour.Behaviour,  # * insert this subtree
        parent_identifer: int,  # * insert as the child of the parent node
        index: int = 0,  # * insert as the (first) child in the parent node
    ):
        """
        insert a subtree into the tree as a child of a specific node at a specific index
        """
        parent_id = self.roster[parent_identifer].id
        self.tree_stewardship.insert_subtree(subtree, parent_id, index)

    def replace_subtree(
        self,
        subtree: py_trees.behaviour.Behaviour,
        identifier: int,
    ):
        """
        replace a specific subtree in the tree with another subtree
        """
        to_replace_node_id = self.roster[identifier].id
        self.tree_stewardship.replace_subtree(to_replace_node_id, subtree)

    def prune_subtree(self, identifier: int):
        """
        remove a specific subtree from the tree
        """
        to_remove_node_id = self.roster[identifier].id
        self.tree_stewardship.prune_subtree(to_remove_node_id)

    ##########################################################
