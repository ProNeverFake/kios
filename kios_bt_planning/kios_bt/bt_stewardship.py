"""
a class that manages the behavior tree, help to localize the behavior nodes in
the behavior tree and help to modify the tree. (interfaces opened for planner/LLM)

should provide:

- simulator fake run
- tree mod
- tree visualization
- test run
"""

from typing import List, Set, Dict, Any, Tuple, Optional

from kios_bt.behavior_nodes import (
    ActionNode,
    ConditionNode,
    ActionNodeTest,
    ActionNodeSim,
)
from kios_bt.bt_factory import BehaviorTreeFactory
from kios_world.world_interface import WorldInterface
from kios_robot.robot_interface import RobotInterface
from kios_bt.data_types import TreeResult

import py_trees

import functools
import copy


class BehaviorTreeStewardship:
    tree_stewardship: py_trees.trees.BehaviourTree = None  # for pytree bt functionality
    behaviortree_factory: BehaviorTreeFactory = None  # for the roster
    world_interface: WorldInterface = None  # for world states
    robot_interface: RobotInterface = None  # for robot actions
    roster: Dict[int, Any] = {}  # for tree mod

    def __init__(
        self,
        behaviortree_factory: BehaviorTreeFactory = None,
        world_interface: WorldInterface = None,
        robot_interface: RobotInterface = None,  # ! set up the scene before passing in!
    ) -> None:
        if robot_interface is None:
            raise Exception("robot_interface is not set")
        else:
            self.robot_interface = robot_interface

        if world_interface is None:
            self.world_interface = WorldInterface()
            self.world_interface.initialize()
        else:
            self.world_interface = world_interface

        if behaviortree_factory is None:
            self.behaviortree_factory = BehaviorTreeFactory(
                None,
                world_interface=self.world_interface,
            )
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

    # * you may need to expand the return type if the simulator is imp as a tool
    def tick_until_success(
        self,
        stw: py_trees.trees.BehaviourTree,
        timeout_ms: float = 5000,
    ) -> TreeResult:
        time_ms = 0

        period_ms = 1
        iter_per_round = 10

        while time_ms < timeout_ms:
            stw.tick_tock(period_ms=period_ms, number_of_iterations=iter_per_round)
            time_ms += period_ms * iter_per_round
            if stw.root.status == py_trees.common.Status.SUCCESS:
                return TreeResult(
                    result="success",
                    summary="Behavior tree tick returns success",
                    defect_node=None,
                )
            elif stw.root.status == py_trees.common.Status.FAILURE:
                tip_node = stw.root.tip()
                defect_node = None
                if tip_node is None:
                    defect_node = None
                    return TreeResult(
                        result="failure",
                        summary="Behavior tree tick returns failure with the defect node unknown",
                        defect_node=defect_node,
                    )
                if isinstance(tip_node, ActionNode):
                    defect_node = tip_node.action
                elif isinstance(tip_node, ConditionNode):
                    defect_node = tip_node.condition

                return TreeResult(
                    result="failure",
                    summary="Behavior tree tick returns failure",  # ! the exceptions should be caught and handled!
                    defect_node=defect_node,
                )
            elif stw.root.status == py_trees.common.Status.RUNNING:
                continue
            elif stw.root.status == py_trees.common.Status.INVALID:
                return TreeResult(
                    result="invalid",
                    summary="Behavior tree tick returns invalid status!",
                    defect_node=None,
                )

        # default timeout
        return TreeResult(
            result="timeout",
            summary="Behavior tree tick returns timeout!",
            defect_node=None,
        )

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
    # * fake run methods
    def simulate(self) -> dict:
        # ! maybe it is better to imp a new class for the simulation
        """
        simulate the tree run with action nodes returning only success.

        this should be the following step of the full tree generation.
        """
        sim_stw = copy.deepcopy(self.tree_stewardship)
        root = sim_stw.root

        self.replace_action_node_with_sim(sim_stw, root)

        sim_stw.setup(timeout=10)

        tree_result = self.tick_until_success(sim_stw, timeout_ms=5000)

        return tree_result.to_json()

    def replace_action_node_with_sim(self, stw, root):
        for child in root.children:
            # replace the action node with sim node
            if isinstance(child, ActionNode):
                sim_node = ActionNodeSim.from_action_node(child)
                stw.replace_subtree(child.id, sim_node)
            # recursively replace the children
            self.replace_action_node_with_sim(stw, child)
