# pylint: disable=duplicate-code
"""
Implementing various py trees behaviors
For testing
"""
import py_trees as pt
from behavior_tree_learning.behaviors import RSequence

def get_node_from_string(string, world_interface, _verbose=False):
    """
    Returns a py trees behavior or composite given the string
    """
    has_children = False
    behavior = BEHAVIORS_DICT.get(string)
    if behavior is not None:
        node = behavior(string, world_interface)
    elif string == 'f(':
        node = pt.composites.Selector("Fallback")
        has_children = True
    elif string == 's(':
        node = RSequence()
        has_children = True
    elif string == 'p(':
        node = pt.composites.Parallel(
            name="Parallel",
            policy=pt.common.ParallelPolicy.SuccessOnAll(synchronise=False))
        has_children = True
    elif string == 'nonpytreesbehavior':
        return False
    else:
        raise Exception("Unexpected character", string)
    return node, has_children

class ANode(pt.behaviour.Behaviour):
    """
    Simple node for testing
    """
    def __init__(self, act, _):
        self.act = act

        super(ANode, self).__init__(str(self.act))

    def update(self):
        print(self.act)
        return pt.common.Status.SUCCESS

class Toggle1(pt.behaviour.Behaviour):
    """
    Simple node for testing
    """
    def __init__(self, name, world_interface):
        self.world_interface = world_interface
        super(Toggle1, self).__init__(str(name))

    def update(self):
        self.world_interface.toggle1()
        return pt.common.Status.SUCCESS

class Toggle2(pt.behaviour.Behaviour):
    """
    Simple node for testing
    """
    def __init__(self, name, world_interface):
        self.world_interface = world_interface
        super(Toggle2, self).__init__(str(name))

    def update(self):
        self.world_interface.toggle2()
        return pt.common.Status.SUCCESS

class Toggle3(pt.behaviour.Behaviour):
    """
    Simple node for testing
    """
    def __init__(self, name, world_interface):
        self.world_interface = world_interface
        super(Toggle3, self).__init__(str(name))

    def update(self):
        self.world_interface.toggle3()
        return pt.common.Status.SUCCESS

class Toggle4(pt.behaviour.Behaviour):
    """
    Simple node for testing
    """
    def __init__(self, name, world_interface):
        self.world_interface = world_interface
        super(Toggle4, self).__init__(str(name))

    def update(self):
        self.world_interface.toggle4()
        return pt.common.Status.SUCCESS

class Read1(pt.behaviour.Behaviour):
    """
    Simple node for testing
    """
    def __init__(self, name, world_interface):
        self.world_interface = world_interface
        super(Read1, self).__init__(str(name))

    def update(self):
        if self.world_interface.read1():
            return pt.common.Status.SUCCESS
        return pt.common.Status.FAILURE

class Read2(pt.behaviour.Behaviour):
    """
    Simple node for testing
    """
    def __init__(self, name, world_interface):
        self.world_interface = world_interface
        super(Read2, self).__init__(str(name))

    def update(self):
        if self.world_interface.read2():
            return pt.common.Status.SUCCESS
        return pt.common.Status.FAILURE

class Read3(pt.behaviour.Behaviour):
    """
    Simple node for testing
    """
    def __init__(self, name, world_interface):
        self.world_interface = world_interface
        super(Read3, self).__init__(str(name))

    def update(self):
        if self.world_interface.read3():
            return pt.common.Status.SUCCESS
        return pt.common.Status.FAILURE

class Read4(pt.behaviour.Behaviour):
    """
    Simple node for testing
    """
    def __init__(self, name, world_interface):
        self.world_interface = world_interface
        super(Read4, self).__init__(str(name))

    def update(self):
        if self.world_interface.read4():
            return pt.common.Status.SUCCESS
        return pt.common.Status.FAILURE

BEHAVIORS_DICT = {
        'a' : ANode,
        'a0' : ANode,
        'a1' : ANode,
        't1' : Toggle1,
        't2' : Toggle2,
        't3' : Toggle3,
        't4' : Toggle4,
        'r1' : Read1,
        'r2' : Read2,
        'r3' : Read3,
        'r4' : Read4,}
