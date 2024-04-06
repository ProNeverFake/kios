import argparse
import subprocess
import typing

import py_trees
import py_trees.console as console


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


def create_v_tree(level: str) -> py_trees.behaviour.Behaviour:

    # root = py_trees.composites.Selector(name="Demo Dot Graphs %s" % level, memory=False)
    name = "selector: abs"
    name = '"{}"'.format(name)
    root = py_trees.composites.Selector(name=name, memory=False)

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


##############################################################################
# Main
##############################################################################


def main() -> None:
    """Entry point for the demo script."""
    py_trees.logging.level = py_trees.logging.Level.DEBUG

    root = create_v_tree("TEST")
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
