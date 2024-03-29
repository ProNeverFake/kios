"""
Behaviors for creating figures
"""
import py_trees as pt
from behavior_tree_learning.behaviors import RSequence

def get_node_from_string(string, _world_interface, _verbose=False):
    """
    Returns a py trees behavior or composite given the string
    """
    has_children = False

    if string == 'f(':
        node = pt.composites.Selector('Fallback')
        has_children = True
    elif string == 's(':
        node = RSequence()
        has_children = True
    elif string == 'p(':
        node = pt.composites.Parallel(
            name="Parallel",
            policy=pt.common.ParallelPolicy.SuccessOnAll(synchronise=False))
        has_children = True
    else:
        node = Leaf(string)

    return node, has_children

class Leaf(pt.behaviour.Behaviour):
    """
    Any leaf behavior
    """
    def __init__(self, name):
        super(Leaf, self).__init__(name)
