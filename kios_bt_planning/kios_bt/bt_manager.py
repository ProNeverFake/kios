"""
a class that manages the behavior tree, help to localize the behavior nodes in
the behavior tree and help to modify the tree. (interfaces opened for planner/LLM)
"""

from typing import List, Set, Dict, Any, Tuple, Optional

from kios_bt.behavior_nodes import ActionNode, ConditionNode
from kios_bt.bt_factory import BehaviorTreeFactory

import py_trees

import functools


class BehaviorTreeStewardship:
    def __init__(self) -> None:
        pass

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

    def generate_bt_stewardship(self, bt) -> py_trees.trees.BehaviourTree:
        ####################
        # Tree Stewardship
        ####################
        behaviour_tree = py_trees.trees.BehaviourTree(bt)
        behaviour_tree.add_pre_tick_handler(self.pre_tick_handler)
        behaviour_tree.visitors.append(py_trees.visitors.DebugVisitor())
        snapshot_visitor = py_trees.visitors.SnapshotVisitor()
        behaviour_tree.add_post_tick_handler(
            functools.partial(self.post_tick_handler, snapshot_visitor)
        )
        behaviour_tree.visitors.append(snapshot_visitor)
        behaviour_tree.setup(timeout=15)

        return behaviour_tree

    def render_dot_tree(bt: py_trees.trees.BehaviourTree):
        py_trees.display.render_dot_tree(bt.root, with_blackboard_variables=False)

    def tick_loop_test(bt: py_trees.trees.BehaviourTree):
        while True:
            try:
                bt.tick()
                py_trees.console.read_single_keypress()
            except KeyboardInterrupt:
                break

        print("\n")

    def tick_once_test(bt: py_trees.trees.BehaviourTree):
        bt.tick()
        py_trees.console.read_single_keypress()

        print("\n")
