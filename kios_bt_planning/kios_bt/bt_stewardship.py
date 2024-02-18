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
from kios_bt.data_types import TreeResult, Action, Condition

from kios_utils.pddl_problem_parser import (
    parse_problem_objects,
    parse_problem_init,
    parse_problem,
)

import py_trees

import functools
import copy
import sched
import time


class BehaviorTreeStewardship:
    behavior_tree: py_trees.trees.BehaviourTree = None  # for pytree bt functionality
    behaviortree_factory: BehaviorTreeFactory = None  # for the roster
    bt_json: dict = None  # for tree regeneration.
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
            # self.world_interface = WorldInterface()
            # self.world_interface.initialize()
            # ! now the world interface is a required parameter
            raise Exception("world_interface is not set")
        else:
            self.world_interface = world_interface

        if behaviortree_factory is None:
            self.behaviortree_factory = BehaviorTreeFactory(
                None,
                world_interface=self.world_interface,
                robot_interface=self.robot_interface,
            )
            self.behaviortree_factory.initialize()
        else:
            self.behaviortree_factory = behaviortree_factory

    def initialize(self):
        py_trees.logging.level = py_trees.logging.Level.DEBUG

        pass

    def load_bt_json(self, bt_json: dict) -> None:
        self.bt_json = bt_json

    def generate_behavior_tree(self) -> None:
        self.roster, self.behavior_tree = (
            self.behaviortree_factory.from_json_to_behavior_tree(self.bt_json)
        )

    def generate_behavior_tree_from_json(self, json_data: dict) -> None:
        self.roster, self.behavior_tree = (
            self.behaviortree_factory.from_json_to_behavior_tree(json_data)
        )

    def load_world_state(self, world_state: dict) -> None:
        self.world_interface.load_world_from_json(world_state)

    def set_world_state(self, world_state: dict) -> None:
        self.world_interface.clear_world()
        self.world_interface.load_world_from_json(world_state)

    # * tick handlers
    def post_tick_handler(
        self,
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

    def pre_tick_handler(self, behaviour_tree: py_trees.trees.BehaviourTree) -> None:
        """Print a banner with current tick count prior to ticking the tree.

        Args:
        behaviour_tree: the tree to tick (used to fetch the count number)
        """
        print("\n--------- Run %s ---------\n" % behaviour_tree.count)

    def setup_behavior_tree(self, behavior_tree=None) -> None:
        """
        setup vor/nachhandlers. setup the tree.
        """
        if behavior_tree == None:
            behavior_tree = self.behavior_tree
        behavior_tree.add_pre_tick_handler(self.pre_tick_handler)
        behavior_tree.visitors.append(py_trees.visitors.DebugVisitor())
        snapshot_visitor = py_trees.visitors.SnapshotVisitor()
        behavior_tree.add_post_tick_handler(
            functools.partial(self.post_tick_handler, snapshot_visitor)
        )
        behavior_tree.visitors.append(snapshot_visitor)
        behavior_tree.setup(timeout=15)

    def render_dot_tree(self):
        py_trees.display.render_dot_tree(
            self.behavior_tree.root, with_blackboard_variables=False
        )

    def tick_loop_test(self):
        while True:
            try:
                self.behavior_tree.tick()
                py_trees.console.read_single_keypress()
            except KeyboardInterrupt:
                break

        print("\n")

    def tick_1000HZ_test(self):
        py_trees.logging.level = py_trees.logging.Level.DEBUG

        tick_count = 0

        def tick():
            nonlocal tick_count
            self.behavior_tree.tick()
            tick_count += 1
            if self.behavior_tree.root.status == py_trees.common.Status.SUCCESS:
                print("\033[94mTree finished with success\033[0m")
            elif self.behavior_tree.root.status == py_trees.common.Status.FAILURE:
                print("\033[91mTree finished with failure\033[0m")
            elif self.behavior_tree.root.status == py_trees.common.Status.RUNNING:
                scheduler.enter(1 / 1000, 1, tick)
            else:
                raise Exception("Unknown status!")

        scheduler = sched.scheduler(time.time, time.sleep)
        scheduler.enter(0, 1, tick)
        scheduler.run()

    def tick_frequency_test(self):
        py_trees.logging.level = py_trees.logging.Level.DEBUG

        tick_count = 0
        start_time = time.time()

        def tick():
            nonlocal tick_count
            self.behavior_tree.tick()
            tick_count += 1
            if self.behavior_tree.root.status == py_trees.common.Status.SUCCESS:
                print("\033[94mTree finished with success\033[0m")
            elif self.behavior_tree.root.status == py_trees.common.Status.FAILURE:
                print("\033[91mTree finished with failure\033[0m")
            elif self.behavior_tree.root.status == py_trees.common.Status.RUNNING:
                scheduler.enter(1 / 1000, 1, tick)
            else:
                raise Exception("Unknown status!")

        scheduler = sched.scheduler(time.time, time.sleep)
        scheduler.enter(0, 1, tick)
        scheduler.run()

        end_time = time.time()
        run_time = end_time - start_time
        frequency = tick_count / run_time

        print(f"Tick count: {tick_count}")
        print(f"Run time: {run_time} seconds")
        print(f"Running frequency: {frequency} Hz")

    def tick_until_success(
        self,
        bt: py_trees.trees.BehaviourTree,
        timeout_ms: float = 5000,
    ) -> TreeResult:
        time_ms = 0

        period_ms = 1
        iter_per_round = 10

        while time_ms < timeout_ms:
            bt.tick_tock(period_ms=period_ms, number_of_iterations=iter_per_round)
            time_ms += period_ms * iter_per_round
            if bt.root.status == py_trees.common.Status.SUCCESS:
                return TreeResult(
                    result="success",
                    summary="Behavior tree tick returns success",
                    defect_node=None,
                )
            elif bt.root.status == py_trees.common.Status.FAILURE:
                tip_node = bt.root.tip()
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
            elif bt.root.status == py_trees.common.Status.RUNNING:
                continue
            elif bt.root.status == py_trees.common.Status.INVALID:
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

    def tick_loop_until_success(
        self,
        bt: py_trees.trees.BehaviourTree,
        loop_max: int = 20,
    ) -> TreeResult:
        i = 0

        while i < loop_max:
            i += 1
            bt.tick()
            if bt.root.status == py_trees.common.Status.SUCCESS:
                return TreeResult(
                    result="success",
                    summary="Behavior tree tick returns success",
                    final_node=None,
                    world_state=self.world_interface.get_world_to_json(),
                )
            elif bt.root.status == py_trees.common.Status.FAILURE:
                tip_node = bt.root.tip()
                if tip_node is None:
                    return TreeResult(
                        result="failure",
                        summary="Behavior tree tick returns failure with the defect node unknown",
                        final_node=None,
                        world_state=self.world_interface.get_world_to_json(),
                    )

                return TreeResult(
                    result="failure",
                    summary="Behavior tree tick returns failure",  # ! the exceptions should be caught and handled!
                    final_node=self.extract_node_metadata(tip_node),
                    world_state=self.world_interface.get_world_to_json(),
                )
            elif bt.root.status == py_trees.common.Status.RUNNING:
                continue
            elif bt.root.status == py_trees.common.Status.INVALID:
                return TreeResult(
                    result="invalid",
                    summary="Behavior tree tick returns invalid status!",
                    final_node=None,
                    world_state=self.world_interface.get_world_to_json(),
                )

        # default timeout
        tip_node = bt.root.tip()
        return TreeResult(
            result="timeout",
            summary="Behavior tree tick returns timeout!",
            final_node=self.extract_node_metadata(tip_node),
            world_state=self.world_interface.get_world_to_json(),
        )

    def extract_node_metadata(
        self, node: py_trees.behaviour.Behaviour
    ) -> Action | Condition | None:
        """
        extract the metadata from the node
        """
        if node is None:
            return None

        if isinstance(node, ActionNode):
            node_meta = node.action
        elif isinstance(node, ConditionNode):
            node_meta = node.condition

        return node_meta

    def tick_once_test(self):
        self.behavior_tree.tick()
        py_trees.console.read_single_keypress()

        print("\n")

    def tick_once(self):
        self.behavior_tree.tick()

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
        self.behavior_tree.insert_subtree(subtree, parent_id, index)

    def replace_subtree(
        self,
        subtree: py_trees.behaviour.Behaviour,
        identifier: int,
    ):
        """
        replace a specific subtree in the tree with another subtree
        """
        to_replace_node_id = self.roster[identifier].id
        self.behavior_tree.replace_subtree(to_replace_node_id, subtree)

    def prune_subtree(self, identifier: int):
        """
        remove a specific subtree from the tree
        """
        to_remove_node_id = self.roster[identifier].id
        self.behavior_tree.prune_subtree(to_remove_node_id)

    ##########################################################
    # * the behavior tree simulator tool
    def simulate_behavior_tree(self) -> TreeResult:
        # ! maybe it is better to imp a new class for the simulation
        """
        simulate the tree run with action nodes returning only success.

        this should be the following step of the full tree generation.
        """
        # * record world check point
        self.world_interface.record_check_point()

        roster, sim_bt = self.behaviortree_factory.from_json_to_behavior_tree(
            self.bt_json
        )
        root = sim_bt.root

        self.replace_action_node_with_sim(sim_bt, root)

        self.setup_behavior_tree(sim_bt)

        tree_result = self.tick_loop_until_success(sim_bt, loop_max=20)

        # * restore world check point
        # ! dirty but works well.
        self.world_interface.restore_check_point()

        return tree_result

    # ! please generate two trees instead.
    def replace_action_node_with_sim(self, stw, root):
        for child in root.children:
            # replace the action node with sim node
            if isinstance(child, ActionNode):
                sim_node = ActionNodeSim.from_action_node(child)
                stw.replace_subtree(child.id, sim_node)
            # recursively replace the children
            self.replace_action_node_with_sim(stw, child)

    def fake_run(self, world_state: dict, bt_json: dict):
        """
        fake run for graph testing
        """
        self.set_world_state(world_state)

        roster, sim_bt = self.behaviortree_factory.from_json_to_behavior_tree(bt_json)
        root = sim_bt.root

        self.replace_action_node_with_sim(sim_bt, root)

        self.setup_behavior_tree(sim_bt)

        tree_result = self.tick_loop_until_success(sim_bt, loop_max=20)

        # # * restore world check point
        # self.world_interface.restore_check_point()

        return tree_result

    ##########################################################

    def query_world_state(self):
        return self.world_interface.get_world_to_json()

    def update_world_state_with_pddl_problem(self, pddl_problem: str):
        """
        bb problem parser
        """

        world_state = parse_problem(pddl_problem)

        self.world_interface.update_world(world_state)
