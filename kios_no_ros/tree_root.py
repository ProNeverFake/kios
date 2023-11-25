import kios_bt.action_nodes as action_nodes
import py_trees
import functools


def create_root() -> py_trees.behaviour.Behaviour:
    """
    Create the root behaviour and it's subtree.

    Returns:
        the root behaviour
    """
    root = py_trees.composites.Sequence(name="test_sequence", memory=False)

    tool_load = action_nodes.ToolLoad(name="ToolLoad")

    root.add_children([tool_load])

    # condition_one = py_trees.behaviours.StatusQueue(
    #     name="Condition 1",
    #     queue=[
    #         py_trees.common.Status.SUCCESS,
    #         py_trees.common.Status.FAILURE,
    #         py_trees.common.Status.SUCCESS,
    #     ],
    #     eventually=py_trees.common.Status.SUCCESS,
    # )
    # condition_two = py_trees.behaviours.StatusQueue(
    #     name="Condition 2",
    #     queue=[
    #         py_trees.common.Status.SUCCESS,
    #         py_trees.common.Status.SUCCESS,
    #         py_trees.common.Status.FAILURE,
    #     ],
    #     eventually=py_trees.common.Status.SUCCESS,
    # )
    # selector = py_trees.composites.Selector(name="Selector", memory=False)
    # subtask = Action(name="Subtask")

    # selector.add_children([condition_one, subtask])

    # task_sequence = py_trees.composites.Sequence(name="Task Sequence", memory=True)
    # # task_one = py_trees.behaviours.Success(name="Worker 1")
    # # task_two = py_trees.behaviours.Running(name="Worker 2")
    # task_one = Action(name="Worker 1")
    # task_two = Action(name="Worker 2")

    # eternal_guard.add_children([selector, task_sequence])
    # task_sequence.add_children([task_one, task_two])
    return root


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


def main() -> None:
    """Entry point for the demo script."""
    py_trees.logging.level = py_trees.logging.Level.DEBUG
    root = create_root()

    ####################
    # Tree Stewardship
    ####################
    behaviour_tree = py_trees.trees.BehaviourTree(root)
    behaviour_tree.add_pre_tick_handler(pre_tick_handler)
    behaviour_tree.visitors.append(py_trees.visitors.DebugVisitor())
    snapshot_visitor = py_trees.visitors.SnapshotVisitor()
    behaviour_tree.add_post_tick_handler(
        functools.partial(post_tick_handler, snapshot_visitor)
    )
    behaviour_tree.visitors.append(snapshot_visitor)
    behaviour_tree.setup(timeout=15)

    ####################
    # Tick Tock
    ####################

    while True:
        try:
            behaviour_tree.tick()
            py_trees.console.read_single_keypress()
        except KeyboardInterrupt:
            break

    print("\n")


if __name__ == "__main__":
    main()
