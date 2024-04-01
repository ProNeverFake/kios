# pylint: disable=duplicate-code
"""
Implementing various py trees behaviors
For duplo brick handling in a state machine env
"""
import re
import py_trees as pt

from behavior_tree_learning.behaviors import RSequence
import duplo_state_machine.state_machine as sm

def get_node_from_string(string, world_interface, verbose=False):
    # pylint: disable=too-many-branches
    """
    Returns a py trees behavior or composite given the string
    """
    has_children = False
    if 'fpick ' in string:
        node = pt.composites.Selector('Fallback')
        node.add_child(Picked(string, world_interface, re.findall(r'\d+', string)))
        node.add_child(Pick(string, world_interface, re.findall(r'\d+', string)))
    elif 'pick ' in string:
        node = Pick(string, world_interface, re.findall(r'\d+', string), verbose)
    elif 'place at' in string:
        node = Place(string, world_interface, position=re.findall(r'-?\d+\.\d+|-?\d+', string), verbose=verbose)
    elif 'place on' in string:
        node = Place(string, world_interface, brick=re.findall(r'\d+', string), verbose=verbose)
    elif 'put' in string:
        node = Put(string, world_interface, re.findall(r'-?\d+\.\d+|-?\d+', string), verbose)
    elif 'apply force' in string:
        node = ApplyForce(string, world_interface, re.findall(r'\d+', string), verbose)

    elif 'hand empty' in string:
        node = HandEmpty(string, world_interface)
    elif 'picked ' in string:
        node = Picked(string, world_interface, re.findall(r'\d+', string))
    elif 'at pos ' in string:
        node = AtPos(string, world_interface, re.findall(r'-?\d+\.\d+|-?\d+', string), verbose)
    elif ' on ' in string:
        node = On(string, world_interface, re.findall(r'\d+', string), verbose)

    elif string == 'f(':
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
    elif string == 'nonpytreesbehavior':
        return False
    else:
        raise Exception("Unexpected character", string)
    return node, has_children

class HandEmpty(pt.behaviour.Behaviour):
    """
    Check if hand is empty
    """
    def __init__(self, name, world_interface):
        self.world_interface = world_interface
        super(HandEmpty, self).__init__(name)

    def update(self):
        if self.world_interface.hand_empty():
            return pt.common.Status.SUCCESS
        return pt.common.Status.FAILURE

class Picked(pt.behaviour.Behaviour):
    """
    Check if brick is picked
    """
    def __init__(self, name, world_interface, brick):
        self.world_interface = world_interface
        self.brick = int(brick[0])
        super(Picked, self).__init__(name)

    def update(self):
        if self.world_interface.get_picked() == self.brick:
            return pt.common.Status.SUCCESS
        return pt.common.Status.FAILURE

class AtPos(pt.behaviour.Behaviour):
    """
    Check if brick is at position
    """
    def __init__(self, name, world_interface, brick_and_pos, verbose=False):
        self.world_interface = world_interface
        self.brick = int(brick_and_pos[0])
        self.pos = sm.Pos(float(brick_and_pos[1]), float(brick_and_pos[2]), float(brick_and_pos[3]))
        self.verbose = verbose
        super(AtPos, self).__init__(name)

    def update(self):
        if self.world_interface.distance(self.brick, self.pos) < self.world_interface.sm_par.pos_margin:
            if self.verbose:
                print(self.name, ": SUCCESS")
            return pt.common.Status.SUCCESS
        if self.verbose:
            print(self.name, ": FAILURE")
        return pt.common.Status.FAILURE

class On(pt.behaviour.Behaviour):
    """
    Check if one brick is on other brick
    """
    def __init__(self, name, world_interface, bricks, verbose=False):
        self.world_interface = world_interface
        self.upper = int(bricks[0])
        self.lower = int(bricks[1])
        self.verbose = verbose
        super(On, self).__init__(name)

    def update(self):
        if self.world_interface.on_top(self.world_interface.state.bricks[self.upper], \
            self.world_interface.state.bricks[self.lower]):
            if self.verbose:
                print(self.name, ": SUCCESS")
            return pt.common.Status.SUCCESS
        if self.verbose:
            print(self.name, ": FAILURE")
        return pt.common.Status.FAILURE

class SmBehavior(pt.behaviour.Behaviour):
    """
    Class template for state machine behaviors
    """
    def __init__(self, name, world_interface, verbose=False):
        self.world_interface = world_interface
        self.state = None
        self.verbose = verbose
        super(SmBehavior, self).__init__(name)

    def update(self):
        if self.verbose and self.state == pt.common.Status.RUNNING:
            print(self.name, ":", self.state)

    def success(self):
        """ Set state success """
        self.state = pt.common.Status.SUCCESS
        if self.verbose:
            print(self.name, ": SUCCESS")

    def failure(self):
        """ Set state failure """
        self.state = pt.common.Status.FAILURE
        if self.verbose:
            print(self.name, ": FAILURE")

class Pick(SmBehavior):
    """
    Pick up a brick
    """
    def __init__(self, name, world_interface, brick, verbose=False):
        self.brick = int(brick[0])
        super(Pick, self).__init__(name, world_interface, verbose)

    def initialise(self):
        if self.world_interface.get_picked() == self.brick:
            self.success()
        elif self.world_interface.get_picked() is not None:
            self.failure()
        else:
            self.state = None

    def update(self):
        super(Pick, self).update()
        if self.state is None:
            self.state = pt.common.Status.RUNNING
        elif self.state is pt.common.Status.RUNNING:
            if self.world_interface.pick(self.brick):
                self.success()
            else:
                self.failure()
            self.world_interface.random_event()
        return self.state

class Place(SmBehavior):
    """
    Place current brick at given position
    """
    def __init__(self, name, world_interface, brick=None, position=None, verbose=False):
        # pylint: disable=too-many-arguments
        if brick is not None:
            self.brick = int(brick[0])
            self.position = None
        elif position is not None:
            self.position = sm.Pos(float(position[0]), float(position[1]), float(position[2]))
            self.brick = None
        super(Place, self).__init__(name, world_interface, verbose)

    def initialise(self):
        if self.world_interface.get_picked() is None:
            self.failure()
        else:
            self.state = None

    def update(self):
        super(Place, self).update()

        if self.state is None:
            self.state = pt.common.Status.RUNNING
        elif self.state is pt.common.Status.RUNNING:
            if self.brick is not None:
                success = self.world_interface.place(brick=self.brick)
            else:
                success = self.world_interface.place(position=self.position)
            if success:
                self.success()
            else:
                self.failure()
            self.world_interface.random_event()
        return self.state

class Put(SmBehavior):
    """
    Picks brick and places it on other brick
    """
    def __init__(self, name, world_interface, brick_and_pos, verbose=False):
        self.brick = int(brick_and_pos[0])
        if len(brick_and_pos) > 2:
            self.position = sm.Pos(float(brick_and_pos[1]), float(brick_and_pos[2]), float(brick_and_pos[3]))
            self.lower = None
        else:
            self.lower = int(brick_and_pos[1])
            self.position = None

        super(Put, self).__init__(name, world_interface, verbose)

    def initialise(self):
        if self.lower is not None:
            if self.world_interface.on_top(self.world_interface.state.bricks[self.brick], \
               self.world_interface.state.bricks[self.lower]):
                self.success()
            else:
                self.state = None
        elif self.world_interface.distance(self.brick, self.position) < self.world_interface.sm_par.pos_margin:
            self.success()
        elif self.world_interface.get_picked() is not None and self.world_interface.get_picked() != self.brick:
            self.failure()
        else:
            self.state = None

    def update(self):
        super(Put, self).update()

        if self.state is None:
            self.state = pt.common.Status.RUNNING
        elif self.state is pt.common.Status.RUNNING:
            success = self.world_interface.pick(self.brick)
            if success:
                if self.lower is not None:
                    success = self.world_interface.place(brick=self.lower)
                else:
                    success = self.world_interface.place(position=self.position)
            if success:
                self.success()
            else:
                self.failure()
            self.world_interface.random_event()
        return self.state

class ApplyForce(SmBehavior):
    """
    Apply force on given brick
    """
    def __init__(self, name, world_interface, brick, verbose=False):
        self.brick = int(brick[0])
        super(ApplyForce, self).__init__(name, world_interface, verbose)

    def initialise(self):
        if self.world_interface.get_picked() is not None:
            self.failure()
        else:
            self.state = None

    def update(self):
        super(ApplyForce, self).update()
        if self.state is None:
            self.state = pt.common.Status.RUNNING
        elif self.state is pt.common.Status.RUNNING:
            if self.world_interface.apply_force(self.brick):
                self.success()
            else:
                self.failure()
            self.world_interface.random_event()
        return self.state
