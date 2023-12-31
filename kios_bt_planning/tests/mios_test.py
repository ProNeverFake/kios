import kios_bt.action_nodes as action_nodes
import kios_bt.condition_nodes as condition_nodes
import py_trees
import functools


class TreeRoot(py_trees.composites.Sequence):
    "create the tree root."

    def __init__(self, name: str = "root", memory: bool = False):
        super().__init__(name, memory)

    def set_initial_state(self, initial_state: dict):
        try:
            for key, value in initial_state.items():
                py_trees.blackboard.Blackboard().set(variable_name=key, value=value)
        except KeyError:
            print("KeyError: %s" % (key))


def create_condition_test_tree() -> py_trees.behaviour.Behaviour:
    """create condition test tree."""
    selector = py_trees.composites.Selector(name="load the tool", memory=False)

    is_tool_load = condition_nodes.IsToolLoaded(name="is_tool_loaded")

    sub_sequence = py_trees.composites.Sequence(name="Task Sequence", memory=False)

    hand_have = condition_nodes.isInHand(name="does_hand_have")

    load_tool = action_nodes.ToolLoad(name="load_tool")

    sub_sequence.add_children([hand_have, load_tool])

    selector.add_children([is_tool_load, sub_sequence])

    return selector


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


###################################################################################################
def test_blackboard():
    root = TreeRoot()
    initial_state = {
        "inHand": "Tool1",
    }
    root.set_initial_state(initial_state)

    is_in_hand = condition_nodes.isInHand(name="is_in_hand")
    root.add_children([is_in_hand])
    root.setup()
    print("inHand: %s" % (py_trees.blackboard.Blackboard().get(variable_name="inHand")))
    py_trees.blackboard.Blackboard().set(variable_name="inHand", value="Tool2")
    print("inHand: %s" % (py_trees.blackboard.Blackboard().get(variable_name="inHand")))

    root_2 = TreeRoot()
    initial_state_2 = {
        "inHand": "Tool1",
    }
    root_2.set_initial_state(initial_state_2)
    is_in_hand_2 = condition_nodes.isInHand(name="is_in_hand_2")
    root_2.add_children([is_in_hand_2])
    root_2.setup()
    print("inHand: %s" % (py_trees.blackboard.Blackboard().get(variable_name="inHand")))
    py_trees.blackboard.Blackboard().set(variable_name="inHand", value="Tool2")
    print("inHand: %s" % (py_trees.blackboard.Blackboard().get(variable_name="inHand")))
    # * the blackboard is a global variable
    # * so different trees are using the same blackboard


###################################################################################################
def test_tree():
    root = TreeRoot()
    initial_state = {
        "inHand": "nothing",
    }
    root.set_initial_state(initial_state)
    selector = py_trees.composites.Selector(name="load the tool", memory=False)
    is_tool_load = condition_nodes.IsToolLoaded(name="is_tool_loaded")
    sub_sequence = py_trees.composites.Sequence(name="Task Sequence", memory=False)
    in_hand = condition_nodes.isInHand(name="in_hand")
    load_tool = action_nodes.ToolLoadTest(name="load_tool")
    sub_sequence.add_children([in_hand, load_tool])
    selector.add_children([is_tool_load, sub_sequence])
    root.add_children([selector])
    root.setup()

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


def create_load_tool_tree() -> py_trees.composites.Selector:
    selector = py_trees.composites.Selector(name="load the tool", memory=False)

    is_tool_load = condition_nodes.IsToolLoaded(["tool1"])

    sequence = py_trees.composites.Sequence(name="load tool sequence", memory=False)

    hand_free = condition_nodes.isInHand(["nothing"])
    load_tool = action_nodes.ToolLoadTest(["tool1"])
    sequence.add_children([hand_free, load_tool])

    selector.add_children([is_tool_load, sequence])

    return selector


def test_tree():
    root = TreeRoot()
    initial_state = {
        "inHand": "nothing",
        "inTool": "nothing",
        "cube1-at": "cube1",
    }
    root.set_initial_state(initial_state)

    selector = py_trees.composites.Selector(name="pick up cube selector", memory=False)

    is_cube_in_hand = condition_nodes.isInHand(["cube1"])

    sequence = py_trees.composites.Sequence(name="pick up cube sequence", memory=False)
    is_tool_load = create_load_tool_tree()
    tool_free = condition_nodes.isInTool(["nothing"])
    pick_up_cube = action_nodes.ToolPick(["cube1"])
    sequence.add_children([is_tool_load, tool_free, pick_up_cube])

    selector.add_children([is_cube_in_hand, sequence])

    root.add_children([selector])

    root.setup()

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
    # py_trees.display.render_dot_tree(root, with_blackboard_variables=True)

    while True:
        try:
            behaviour_tree.tick()
            py_trees.console.read_single_keypress()
        except KeyboardInterrupt:
            break

    print("\n")


def test_fake_action():
    root = TreeRoot("root", memory=False)
    initial_state = {
        "inHand": "nothing",
        "inTool": "nothing",
        "cube1-at": "cube1",
    }
    root.set_initial_state(initial_state)

    sequence = py_trees.composites.Sequence(name="pick up cube sequence", memory=False)

    load_tool = action_nodes.ToolLoadTest(["tool1"])

    load_tool2 = action_nodes.ToolLoadTest(["tool2"])

    sequence.add_children([load_tool, load_tool2])

    root.add_children([sequence])

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
    # py_trees.display.render_dot_tree(root, with_blackboard_variables=True)

    while True:
        try:
            behaviour_tree.tick()
            py_trees.console.read_single_keypress()
        except KeyboardInterrupt:
            break

    print("\n")


if __name__ == "__main__":
    # test_blackboard()
    py_trees.logging.level = py_trees.logging.Level.DEBUG
    test_fake_action()

    # print("\n")
