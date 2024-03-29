"""
Implements a very simple task planner inspired from 'Towards Blended Reactive Planning and Acting using Behavior Trees'
Generates a behaviors tree to solve task given a set of goals
and behaviors with preconditions and postconditions. Since the
conditions are not always static, it actually runs the tree while evaluating
the conditions.
"""
import py_trees as pt

from behavior_tree_learning.behaviors import RSequence
from behavior_tree_learning.behavior_tree import get_action_list
from behavior_tree_learning.py_trees_interface import PyTree

# ! BBCORE


def handle_precondition(precondition, behaviors, world_interface):
    """
    Handles precondition by creating a subtree whose
    postconditions fulfill the precondition
    """
    print("Condition in: ", precondition)
    condition_parameters = behaviors.get_condition_parameters(precondition)

    for action in get_action_list():
        action_node, _ = behaviors.get_node_from_string(
            action, world_interface, condition_parameters)
        if precondition in action_node.get_postconditions():
            action_preconditions = action_node.get_preconditions()
            # if action precondition is not empty, create a sequence
            if action_preconditions != []:
                bt = RSequence('Sequence')
                # add all preconditions to the sequence
                for action_precondition in action_preconditions:
                    child, _ = behaviors.get_node_from_string(action_precondition,
                                                              world_interface,
                                                              behaviors.get_condition_parameters(action_precondition))
                    bt.add_child(child)
                # add action node to the sequence
                bt.add_child(action_node)
            else:
                # only action node
                bt = action_node

            return bt
    print("ERROR, no matching action found to ensure precondition")
    return None


def extend_leaf_node(leaf_node, behaviors, world_interface):
    """
    If leaf node fails, it should be replaced with a selector that checks leaf node
    and a subtree that fixes the conditon whenever it's not met.
    """
    # create a selector
    bt = pt.composites.Selector(name='Fallback')
    # replace the failed leaf node with the selector
    leaf_node.parent.replace_child(leaf_node, bt)
    # add the failed leaf node to the selector as "target"
    bt.add_child(leaf_node)
    print("What is failing? ", leaf_node.name)

    # call handle_precondition to create a subtree that fixes the condition
    extended = handle_precondition(leaf_node.name, behaviors, world_interface)
    if extended is not None:
        # the extended subtree is the action found to fix the "target"
        bt.add_child(extended)


def expand_tree(node, behaviors, world_interface):
    """
    Expands the part of the tree that fails
    """
    print("TREE COMING IN :", node)

    if node.name == 'Fallback':
        print("Fallback node fails\n")
        for index, child in enumerate(node.children):
            if index >= 1:  # Normally there will only be two children
                expand_tree(child, behaviors, world_interface)
    elif node.name == 'Sequence':
        print("Sequence node fails\n")
        for i in range(len(node.children)):
            if node.children[i].status == pt.common.Status.FAILURE:
                print("Child that fails: ", node.children[i].name)
                expand_tree(node.children[i], behaviors, world_interface)
    elif isinstance(node, pt.behaviour.Behaviour) and node.status == pt.common.Status.FAILURE:
        extend_leaf_node(node, behaviors, world_interface)
    else:
        print("Tree", node.name)


def plan(world_interface, behaviors, goals):
    """
    Main planner function
    Generates a behaviors tree to solve task given a set of goals
    and behaviors with preconditions and postconditions. Since the
    conditions are not always static, it actually runs the tree while evaluating
    the conditions.
    """
    tree = RSequence()

    for goal in goals:
        goal_condition, _ = behaviors.get_node_from_string(
            goal, world_interface, [])
        tree.add_child(goal_condition)
    print(pt.display.unicode_tree(root=tree, show_status=True))

    for i in range(60):
        tree.tick_once()
        print("Tick: ", i)
        print(pt.display.unicode_tree(root=tree, show_status=True))
        if tree.status is pt.common.Status.FAILURE:
            expand_tree(tree, behaviors, world_interface)

            print(pt.display.unicode_tree(root=tree, show_status=True))
        elif tree.status is pt.common.Status.SUCCESS:
            break

    pt.display.render_dot_tree(tree, name='Planned bt', target_directory='')

    print(PyTree('', behaviors, world_interface, tree).get_bt_from_root())
