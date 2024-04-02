"""
a class that manages the behavior tree, help to localize the behavior nodes in
the behavior tree and help to modify the tree. (interfaces opened for planner/LLM)

should provide:

- simulator fake run
- tree mod
- tree visualization
- test run
"""

import logging
import colorlog

from typing import List, Set, Dict, Any, Tuple, Optional

from kios_bt.behavior_nodes import (
    ActionNode,
    ConditionNode,
    ActionNodeTest,
    ActionNodeSim,
    ActionNodeOnlySuccess,
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
from kios_utils.bblab_utils import bb_deprecated

import py_trees

import functools
import copy
import sched
import time

handler = colorlog.StreamHandler()
handler.setFormatter(
    colorlog.ColoredFormatter(
        "%(log_color)s%(levelname)s:%(name)s:%(message)s",
        log_colors={
            "DEBUG": "cyan",
            "INFO": "green",
            "WARNING": "yellow",
            "ERROR": "red",
            "CRITICAL": "red,bg_white",
        },
    )
)

stw_logger = logging.getLogger("behavior_tree_stewardship")
stw_logger.addHandler(handler)
stw_logger.setLevel(logging.INFO)


class BehaviorTreeStewardship:
    behavior_tree: py_trees.trees.BehaviourTree = None  # for pytree bt functionality
    behaviortree_factory: BehaviorTreeFactory = None  # for the roster
    bt_json: dict = None  # for tree regeneration.
    bt_skeleton: dict = None  # for tree regeneration.
    world_interface: WorldInterface = None  # for world states
    robot_interface: RobotInterface = None  # for robot actions
    roster: Dict[int, Any] = {}  # for tree mod
    skeleton_dict = {}  # for tree to dict

    tree_result: TreeResult = None  # to record the result of the tree run

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

    def generate_behavior_tree_from_skeleton(self, skeleton: dict) -> None:
        self.bt_skeleton = skeleton
        self.roster, self.behavior_tree, self.skeleton_dict = (
            self.behaviortree_factory.from_skeleton_to_behavior_tree(skeleton)
        )

    def set_world_state(self, world_state: dict) -> None:
        """
        set the world state to exactly the given state
        """
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
        # behavior_tree.setup(timeout=15) # ! signal bug
        behavior_tree.setup()

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

    def tick_tree(
        self,
        period_msec: int = 1,
        timeout_sec: int = 200,
    ):
        """tick the tree at a specific frequency. the tree will be ticked until it returns success or failure, or timeout is triggered.

        Args:
            period_msec (int, optional): period for tree ticking. Defaults to 1 ms.
            timeout_sec (int, optional): tree execution time limit. Defaults to 200 sec.

        Raises:
            Exception: this should never happen.
        """
        py_trees.logging.level = py_trees.logging.Level.DEBUG
        running_time = 0

        def tick():
            nonlocal running_time
            # * timeout return
            if running_time > timeout_sec:
                stw_logger.error("Tree tick returns timeout!")
                tip_node = self.behavior_tree.root.tip()
                self.tree_result = TreeResult(
                    result="timeout",
                    summary="Behavior tree tick returns timeout! The reason could be: an action node is being ticked repeatedly while the condition for the next step can not be satisfied; two actions are in one sequence and the tree got stuck here. the tree is not well-structured.",
                    final_node=self.extract_node_metadata(tip_node).to_json(),
                    world_state=self.world_interface.get_world_to_json(),
                )

            self.behavior_tree.tick()

            if self.behavior_tree.root.status == py_trees.common.Status.SUCCESS:
                stw_logger.info("Tree finished with success")
                self.tree_result = TreeResult(
                    result="success",
                    summary="Behavior tree tick returns success",
                    final_node=None,
                    world_state=self.world_interface.get_world_to_json(),
                )
            elif self.behavior_tree.root.status == py_trees.common.Status.FAILURE:
                stw_logger.error("Tree finished with failure")
                tip_node = self.behavior_tree.root.tip()
                if tip_node is None:
                    self.tree_result = TreeResult(
                        result="failure",
                        summary="Behavior tree tick returns failure with the defect node unknown",
                        final_node=None,
                        world_state=self.world_interface.get_world_to_json(),
                    )
                else:
                    self.tree_result = TreeResult(
                        result="failure",
                        summary="Behavior tree tick returns failure",
                        final_node=self.extract_node_metadata(tip_node).to_json(),
                        world_state=self.world_interface.get_world_to_json(),
                    )
            elif self.behavior_tree.root.status == py_trees.common.Status.RUNNING:
                scheduler.enter(period_msec / 1000, 1, tick)
                running_time += period_msec / 1000
            else:
                raise Exception("Unknown status!")

        scheduler = sched.scheduler(time.time, time.sleep)
        scheduler.enter(0, 1, tick)
        scheduler.run()

    def tick_1000HZ_test(self):
        py_trees.logging.level = py_trees.logging.Level.DEBUG

        tick_count = 0

        def tick():
            nonlocal tick_count
            self.behavior_tree.tick()
            tick_count += 1
            if self.behavior_tree.root.status == py_trees.common.Status.SUCCESS:
                stw_logger.info("Tree finished with success")
            elif self.behavior_tree.root.status == py_trees.common.Status.FAILURE:
                stw_logger.warn("Tree finished with failure")
            elif self.behavior_tree.root.status == py_trees.common.Status.RUNNING:
                scheduler.enter(1 / 1000, 1, tick)
            else:
                raise Exception("Unknown status!")

        scheduler = sched.scheduler(time.time, time.sleep)
        scheduler.enter(0, 1, tick)
        scheduler.run()

    def tick_frequency_test(self):
        """
        for testing the running frequency of the tree
        """
        py_trees.logging.level = py_trees.logging.Level.DEBUG

        tick_count = 0
        start_time = time.time()

        def tick():
            nonlocal tick_count
            self.behavior_tree.tick()
            tick_count += 1
            if self.behavior_tree.root.status == py_trees.common.Status.SUCCESS:
                stw_logger.info("Tree finished with success")
            elif self.behavior_tree.root.status == py_trees.common.Status.FAILURE:
                stw_logger.warn("Tree finished with failure")
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

    @bb_deprecated("TreeResult has been changed!")
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
                    summary="Behavior tree tick returns failure. The reason could be: A condition failed and an action to satisfy it is not present; An action node failed and no alternative is available; the node is against the definition in domain knowledge; The tree is not well-structured.",
                    final_node=self.extract_node_metadata(tip_node).to_json(),
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
            summary="Behavior tree tick returns timeout! The reason could be: an action node is being ticked repeatedly while the condition for the next step can not be satisfied; two actions are in one sequence and the tree got stuck here. the tree is not well-structured.",
            final_node=self.extract_node_metadata(tip_node).to_json(),
            world_state=self.world_interface.get_world_to_json(),
        )

    def extract_node_metadata(
        self, node: py_trees.behaviour.Behaviour
    ) -> Action | Condition | None:
        """
        extract the metadata from the node: name, id, effect/condition, summary
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

    def setup_simulation(self):
        self.replace_action_node_with_sim(self.behavior_tree, self.behavior_tree.root)

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

    def replace_action_node_with_only_success(self, stw, root):
        for child in root.children:
            # replace the action node with sim node
            if isinstance(child, ActionNode):
                sim_node = ActionNodeOnlySuccess.from_action_node(child)
                stw.replace_subtree(child.id, sim_node)
                # * update the skeleton dict, add the new replacement node
                self.skeleton_dict[sim_node.id] = self.skeleton_dict[child.id]
            # recursively replace the children
            self.replace_action_node_with_only_success(stw, child)

    ######################################################
    # * the behavior tree simulator tools
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

    def sk_fake_run(self, world_state: dict, skeleton_json: dict):
        """
        fake run for testing
        """
        self.set_world_state(world_state)
        self.bt_skeleton = skeleton_json

        roster, bt = self.behaviortree_factory.from_skeleton_to_behavior_tree(
            skeleton_json
        )
        root = bt.root

        self.replace_action_node_with_sim(bt, root)

        self.setup_behavior_tree(bt)

        tree_result = self.tick_loop_until_success(bt, loop_max=100)

        return tree_result, skeleton_json

    def sk_sim_run(self, world_state: dict, skeleton_json: dict):
        """
        simulation run of behavior tree generated from skeleton json.
        try the behavior tree to check if the tree works well.
        restore the world state after the simulation.
        """
        self.set_world_state(world_state)

        self.world_interface.record_check_point()

        _, sim_bt, _ = self.behaviortree_factory.from_skeleton_to_behavior_tree(
            skeleton_json
        )
        root = sim_bt.root

        self.replace_action_node_with_sim(sim_bt, root)

        self.setup_behavior_tree(sim_bt)

        tree_result = self.tick_loop_until_success(sim_bt, loop_max=100)

        # * restore world check point
        self.world_interface.restore_check_point()

        return tree_result, skeleton_json

    def sk_baseline(
        self, world_state: dict, skeleton_json: dict, ut_dict: dict
    ) -> dict:
        """baseline expanding, return a solution bt skeleton to start from this world state to achieve the skeleton json condition node.

        Args:
            world_state (dict): ws to start with
            skeleton_json (dict): the condition sk node to achieve. should be a condition node in sk form with summary and a legal name.

        Raises:
            Exception: see the code

        Returns:
            dict: the solution bt sk json.
        """

        from pprint import pprint

        self.set_world_state(world_state)
        self.world_interface.record_check_point("baseline_start")

        _, sim_bt, self.skeleton_dict = (
            self.behaviortree_factory.from_skeleton_to_behavior_tree(skeleton_json)
        )
        self.replace_action_node_with_only_success(sim_bt, sim_bt.root)
        self.setup_behavior_tree(sim_bt)

        from kios_utils.pybt_test import generate_bt_stewardship, render_dot_tree

        while True:
            sim_bt.tick()
            if sim_bt.root.status == py_trees.common.Status.SUCCESS:
                pprint("Behavior tree tick returns success!")
                break
            elif sim_bt.root.status == py_trees.common.Status.FAILURE:
                tip_node = sim_bt.root.tip()
                if tip_node is None:
                    raise Exception(
                        "Behavior tree tick returns failure with the defect node unknown"
                    )
                assert isinstance(tip_node, ConditionNode)
                new_target = tip_node.condition.to_string()
                new_ut = ut_dict.get(new_target, None)
                if new_ut is None:
                    pprint(f"cannot find a solution ut for the condtiion {new_target}!")
                    pprint("the available targets are:")
                    pprint(ut_dict.keys())
                    raise Exception(
                        f"cannot find a solution ut for the condtiion {new_target}!"
                    )
                    break
                _, new_subtree, new_sk_dict = (
                    self.behaviortree_factory.from_skeleton_to_behavior_tree(new_ut)
                )
                self.skeleton_dict.update(new_sk_dict)
                self.replace_action_node_with_only_success(
                    new_subtree, new_subtree.root
                )
                # render_dot_tree(new_subtree)
                # pause = input("paused here! check the tree.")
                if sim_bt.root.id == tip_node.id:
                    sim_bt = new_subtree
                else:
                    sim_bt.replace_subtree(tip_node.id, new_subtree.root)
                # render_dot_tree(sim_bt)
                # pause = input("paused here! check the tree.")

                sim_bt.setup(timeout=15)

                self.world_interface.restore_check_point("baseline_start")
            elif sim_bt.root.status == py_trees.common.Status.RUNNING:
                print("Behavior tree is running")
            else:
                raise Exception("Unknown status!")

        # * restore world check point
        self.world_interface.restore_check_point("baseline_start")
        render_dot_tree(sim_bt)
        return self.from_tree_root_to_sk_json(sim_bt.root)

    def from_tree_root_to_sk_json(
        self, tree_root: py_trees.behaviour.Behaviour
    ) -> dict:
        """
        convert the tree root to sk_json
        """
        if isinstance(tree_root, py_trees.composites.Selector):
            compound_sk = self.skeleton_dict[tree_root.id]
            compound_sk["children"] = [
                self.from_tree_root_to_sk_json(child) for child in tree_root.children
            ]
            return compound_sk
        elif isinstance(tree_root, py_trees.composites.Sequence):
            compound_sk = self.skeleton_dict[tree_root.id]
            compound_sk["children"] = [
                self.from_tree_root_to_sk_json(child) for child in tree_root.children
            ]
            return compound_sk
        elif isinstance(tree_root, ActionNode):
            return self.skeleton_dict[tree_root.id]
        elif isinstance(tree_root, ConditionNode):
            return self.skeleton_dict[tree_root.id]
        else:
            raise Exception("Unknown node type!")

    ##########################################################

    def query_world_state(self):
        return self.world_interface.get_world_to_json()

    def update_world_state_with_pddl_problem(self, pddl_problem: str):
        """
        bb problem parser
        """

        world_state = parse_problem(pddl_problem)

        self.world_interface.update_world(world_state)
