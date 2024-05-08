import argparse
import subprocess
import typing

import py_trees
import py_trees.console as console
from kios_bt.behavior_nodes import (
    ConditionNode,
    ActionNode,
    ActionNodeViz,
    ConditionNodeViz,
)


def create_tree(level: str) -> py_trees.behaviour.Behaviour:
    """
    Create the root behaviour and it's subtree.

    Returns:
        the root behaviour
    """
    root = py_trees.composites.Selector(name="Demo Dot Graphs %s" % level, memory=False)
    first_blackbox = py_trees.composites.Sequence(name="BlackBox 1", memory=True)
    first_blackbox.add_child(py_trees.behaviours.Running("Worker"))
    first_blackbox.add_child(py_trees.behaviours.Running("Worker"))
    first_blackbox.add_child(py_trees.behaviours.Running("Worker"))
    # first_blackbox.blackbox_level = py_trees.common.BlackBoxLevel.BIG_PICTURE
    second_blackbox = py_trees.composites.Sequence(name="Blackbox 2", memory=True)
    second_blackbox.add_child(py_trees.behaviours.Running("Worker"))
    second_blackbox.add_child(py_trees.behaviours.Running("Worker"))
    second_blackbox.add_child(py_trees.behaviours.Running("Worker"))
    # second_blackbox.blackbox_level = py_trees.common.BlackBoxLevel.COMPONENT
    third_blackbox = py_trees.composites.Sequence(name="Blackbox 3", memory=True)
    third_blackbox.add_child(py_trees.behaviours.Running("Worker"))
    third_blackbox.add_child(py_trees.behaviours.Running("Worker"))
    third_blackbox.add_child(py_trees.behaviours.Running("Worker"))
    third_blackbox.blackbox_level = py_trees.common.BlackBoxLevel.DETAIL
    root.add_child(first_blackbox)
    root.add_child(second_blackbox)
    first_blackbox.add_child(third_blackbox)
    return root


def create_async_tree(level: str) -> py_trees.behaviour.Behaviour:

    # root = py_trees.composites.Selector(name="Demo Dot Graphs %s" % level, memory=False)
    sel = py_trees.composites.Selector(name="Selector", memory=False)
    sel.add_child(ConditionNodeViz("Target Condition 1"))
    sq = py_trees.composites.Sequence(name="Sequence", memory=False)
    sq.add_child(ConditionNodeViz("Precondition 1"))
    sq.add_child(ConditionNodeViz("Precondition 2"))
    sq.add_child(ActionNodeViz("Action 1"))
    sel.add_child(sq)
    return sel


def create_sync_tree(level: str) -> py_trees.behaviour.Behaviour:

    # root = py_trees.composites.Selector(name="Demo Dot Graphs %s" % level, memory=False)
    sel = py_trees.composites.Selector(name="Selector", memory=False)
    sq1 = py_trees.composites.Sequence(name="Sequence 1", memory=False)
    sq1.add_child(ConditionNodeViz("Precondition 1"))
    sq1.add_child(ActionNodeViz("Action 1"))
    sq2 = py_trees.composites.Sequence(name="Sequence 2", memory=False)
    sq2.add_child(ConditionNodeViz("Precondition 2"))
    sq2.add_child(ActionNodeViz("Action 2"))
    sq2.add_child(ActionNodeViz("Action 3"))
    sel.add_child(sq1)
    sel.add_child(sq2)
    return sel


def create_rec_tree(level: str) -> py_trees.behaviour.Behaviour:

    # root = py_trees.composites.Selector(name="Demo Dot Graphs %s" % level, memory=False)
    sel1 = py_trees.composites.Selector(name="Selector1", memory=False)
    sel1.add_child(ConditionNodeViz("Target Condition 1"))
    sq1 = py_trees.composites.Sequence(name="Sequence 1", memory=False)
    sq1.add_child(ConditionNodeViz("Precondition 1"))

    sel1.add_child(sq1)
    sel2 = py_trees.composites.Selector(name="Selector2", memory=False)
    sel2.add_child(ConditionNodeViz("Target Condition 2"))
    sq2 = py_trees.composites.Sequence(name="Sequence 2", memory=False)
    sq2.add_child(ConditionNodeViz("Precondition 2"))
    sq2.add_child(ActionNodeViz("Action 2"))
    sel2.add_child(sq2)

    sq1.add_child(sel2)
    sq1.add_child(ActionNodeViz("Action 1"))
    return sel1


def create_unit_tree(level: str) -> py_trees.behaviour.Behaviour:

    # root = py_trees.composites.Selector(name="Demo Dot Graphs %s" % level, memory=False)
    sel1 = py_trees.composites.Selector(name="Selector1", memory=False)
    sel1.add_child(ConditionNodeViz("Target Condition 1"))
    sq1 = py_trees.composites.Sequence(name="Sequence 1", memory=False)
    sq1.add_child(ConditionNodeViz("Precondition 1"))

    sel1.add_child(sq1)
    sel2 = py_trees.composites.Selector(name="Selector2", memory=False)
    sel2.add_child(ConditionNodeViz("Target Condition 2"))
    sq2 = py_trees.composites.Sequence(name="Sequence 2", memory=False)
    sq2.add_child(ConditionNodeViz("Precondition 2"))
    sq2.add_child(ActionNodeViz("Action 2"))
    sel2.add_child(sq2)

    sq1.add_child(sel2)
    sq1.add_child(ActionNodeViz("Action 1"))
    return sel1


##############################################################################
# Main
##############################################################################


def main() -> None:
    """Entry point for the demo script."""
    py_trees.logging.level = py_trees.logging.Level.DEBUG

    # root = create_rec_tree("DETAIL")
    root = create_unit_tree("DETAIL")
    py_trees.display.render_dot_tree(root, py_trees.common.VisibilityLevel.DETAIL)

    # if py_trees.utilities.which("xdot"):
    #     try:
    #         subprocess.call(["xdot", "demo_dot_graphs_%s.dot" % "DETAIL"])
    #     except KeyboardInterrupt:
    #         pass
    # else:
    #     print("")
    #     console.logerror(
    #         "No xdot viewer found, skipping display [hint: sudo apt install xdot]"
    #     )
    #     print("")


if __name__ == "__main__":
    main()
