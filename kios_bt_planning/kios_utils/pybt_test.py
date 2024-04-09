import py_trees
import functools
import os
import time
import sched
from PIL import Image
import re

"""
# ! should delete
"""


def fix_node_name(name: str) -> str:
    """a function to fix the node name for the dot graph.
    # ! BBBUG: the name of the node should not includes colon":".
    # ! this will cause error in rendering the node because
    # ! the colon has special meaning in the dot language.
    """
    if ":" in name:
        name = re.sub(r":", r":\n", name)
    return '"{}"'.format(name)


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


def generate_bt_stewardship(bt) -> py_trees.trees.BehaviourTree:
    ####################
    # Tree Stewardship
    ####################
    behaviour_tree = py_trees.trees.BehaviourTree(bt)
    behaviour_tree.add_pre_tick_handler(pre_tick_handler)
    behaviour_tree.visitors.append(py_trees.visitors.DebugVisitor())
    snapshot_visitor = py_trees.visitors.SnapshotVisitor()
    behaviour_tree.add_post_tick_handler(
        functools.partial(post_tick_handler, snapshot_visitor)
    )
    behaviour_tree.visitors.append(snapshot_visitor)
    behaviour_tree.setup()

    return behaviour_tree


def render_dot_tree(
    bt: py_trees.trees.BehaviourTree,
    name="visualisation",
    dir=None,
):

    py_trees.display.render_dot_tree(
        bt.root,
        # visibility_level=py_trees.common.VisibilityLevel.ALL,
        with_blackboard_variables=False,
        name=name,
        # with_qualified_names=True,
        target_directory=dir if dir is not None else os.path.dirname(__file__),
    )
    Image.open(
        os.path.join(
            dir if dir is not None else os.path.dirname(__file__), name + ".png"
        )
    ).show()
    # Image.open(os.path.join(os.path.dirname(__file__), name + ".png")).show()


def tick_loop_test(bt: py_trees.trees.BehaviourTree):
    py_trees.logging.level = py_trees.logging.Level.DEBUG
    while True:
        try:
            bt.tick()
            py_trees.console.read_single_keypress()
        except KeyboardInterrupt:
            break

    print("\n")


def tick_1000HZ_test(bt: py_trees.trees.BehaviourTree):
    py_trees.logging.level = py_trees.logging.Level.DEBUG

    tick_count = 0

    def tick():
        nonlocal tick_count
        bt.tick()
        tick_count += 1
        if bt.root.status == py_trees.common.Status.SUCCESS:
            print("\033[94mTree finished with success\033[0m")
        elif bt.root.status == py_trees.common.Status.FAILURE:
            print("\033[91mTree finished with failure\033[0m")
        elif bt.root.status == py_trees.common.Status.RUNNING:
            scheduler.enter(1 / 1000, 1, tick)
        else:
            raise Exception("Unknown status!")

    scheduler = sched.scheduler(time.time, time.sleep)
    scheduler.enter(0, 1, tick)
    scheduler.run()


def tick_frequency_test(bt: py_trees.trees.BehaviourTree):
    py_trees.logging.level = py_trees.logging.Level.DEBUG

    tick_count = 0
    start_time = time.time()

    def tick():
        nonlocal tick_count
        bt.tick()
        tick_count += 1
        if bt.root.status == py_trees.common.Status.SUCCESS:
            print("\033[94mTree finished with success\033[0m")
        elif bt.root.status == py_trees.common.Status.FAILURE:
            print("\033[91mTree finished with failure\033[0m")
        elif bt.root.status == py_trees.common.Status.RUNNING:
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


def tick_once_test(bt: py_trees.trees.BehaviourTree):
    py_trees.logging.level = py_trees.logging.Level.DEBUG
    bt.tick()
    py_trees.console.read_single_keypress()

    print("\n")
