"""
Behaviors for agx simulation
"""
from copy import copy
from enum import IntEnum
import re

import py_trees as pt

from behavior_tree_learning.behaviors import RSequence
from duplo_simulation.agx_interface import Pos

def get_node_from_string(string, world_interface, verbose=False):
    """
    Returns a py trees behavior or composite given the string
    """
    has_children = False
    if 'pick ' in string:
        node = AgxPick(string, world_interface, re.findall(r'\d+', string), verbose)
    elif 'place at' in string:
        node = AgxPlace(string, world_interface, position=re.findall(r'-?\d+\.\d+|-?\d+', string), verbose=verbose)
    elif 'place on' in string:
        node = AgxPlace(string, world_interface, brick=re.findall(r'\d+', string), verbose=verbose)
    elif 'put' in string:
        node = AgxPut(string, world_interface, re.findall(r'-?\d+\.\d+|-?\d+', string), verbose)
    elif 'apply force' in string:
        node = AgxApplyForce(string, world_interface, re.findall(r'\d+', string), verbose)

    elif 'picked ' in string:
        node = AgxPicked(string, world_interface, re.findall(r'\d+', string))
    elif 'at pos ' in string:
        node = AgxAtPos(string, world_interface, re.findall(r'-?\d+\.\d+|-?\d+', string))
    elif ' on ' in string:
        node = AgxOn(string, world_interface, re.findall(r'\d+', string))

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
    else:
        raise Exception("Unexpected character", string)
    return node, has_children

class AgxPicked(pt.behaviour.Behaviour):
    """
    Check if brick is picked
    """
    def __init__(self, name, world_interface, brick):
        self.world_interface = world_interface
        self.brick = int(brick[0])
        super(AgxPicked, self).__init__(name)

    def update(self):
        if self.world_interface.get_picked() == self.brick:
            return pt.common.Status.SUCCESS
        return pt.common.Status.FAILURE

class AgxAtPos(pt.behaviour.Behaviour):
    """
    Check if brick is at position
    """
    def __init__(self, name, world_interface, brick_and_pos):
        self.world_interface = world_interface
        self.brick = int(brick_and_pos[0])
        self.pos = Pos(float(brick_and_pos[1]), float(brick_and_pos[2]), float(brick_and_pos[3]))
        self.state = pt.common.Status.INVALID
        super(AgxAtPos, self).__init__(name)

    def update(self):
        if self.world_interface.force_applied is not None or self.world_interface.get_picked() is not None:
            #Don't update when currently pressing or gripping a brick or it might end prematurely
            return self.state

        if self.check_position(self.world_interface, self.brick, self.pos):
            self.state = pt.common.Status.SUCCESS
        else:
            self.state = pt.common.Status.FAILURE
        return self.state

    @staticmethod
    def check_position(world_interface, brick, position):
        """ Checks if brick is close to position """
        if world_interface.get_picked() != brick:
            brick_position = world_interface.get_brick_position(brick)
            if position.distance(brick_position) < 1.e-3:
                return True
        return False

class AgxOn(pt.behaviour.Behaviour):
    """
    Check if one brick is on other brick
    """
    def __init__(self, name, world_interface, bricks):
        self.world_interface = world_interface
        self.upper = int(bricks[0])
        self.lower = int(bricks[1])
        super(AgxOn, self).__init__(name)

    def update(self):
        if AgxOn.check_on(self.world_interface, self.upper, self.lower):
            return pt.common.Status.SUCCESS
        return pt.common.Status.FAILURE

    @staticmethod
    def check_on(world_interface, upper, lower):
        """ Checks if upper brick is on top of lower brick """
        if world_interface.get_picked() != upper:
            upper_position = world_interface.get_brick_position(upper)
            lower_position = world_interface.get_brick_position(lower)
            lower_height = world_interface.get_brick_height(lower)
            if abs(upper_position.x - lower_position.x) < 1.e-3 and \
                abs(upper_position.y - lower_position.y) < 1.e-3 and \
                0 < upper_position.z - lower_position.z <= lower_height + 1.e-2:
                return True
        return False

class AgxBehavior(pt.behaviour.Behaviour):
    """
    Class template for agx behaviors
    """
    def __init__(self, name, world_interface, verbose=False, max_ticks=50, max_int_ticks=15):
        # pylint: disable=too-many-arguments
        self.world_interface = world_interface
        self.state = None
        self.int_state = 0
        self.counter = 0
        self.int_counter = 0
        self.max_ticks = max_ticks
        self.max_int_ticks = max_int_ticks
        self.verbose = verbose
        self.target_position = None
        super(AgxBehavior, self).__init__(name)

    def initialise(self):
        self.counter = 0
        self.int_counter = 0

    def update(self):
        self.counter += 1
        self.int_counter += 1
        if self.state is None:
            self.state = pt.common.Status.RUNNING
        elif self.state == pt.common.Status.RUNNING:
            if self.counter > self.max_ticks or self.int_counter > self.max_int_ticks:
                self.failure()
        if self.verbose and self.state == pt.common.Status.RUNNING:
            print(self.name, ":", self.int_state)

    def set_int_state(self, state):
        """ Sets internal state and resets counter """
        self.int_state = state
        self.int_counter = 0

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

    def move_to_target(self):
        """ Move gripper to target position """
        self.world_interface.move_to(self.target_position)

    def is_gripper_in_position(self, atol=1.e-3):
        """ Check if gripper is at target position within atol margin"""
        return self.world_interface.is_gripper_at(self.target_position, atol)

class AgxPick(AgxBehavior):
    """
    Pick up a brick
    """
    class States(IntEnum):
        """ Behavior internal states """
        INITIALIZED = 1
        APPROACHING = 2
        DESCENDING = 3
        GRIPPING = 4
        LIFTING = 5

    def __init__(self, name, world_interface, brick, verbose=False):
        self.brick = int(brick[0])
        self.set_int_state(self.States.INITIALIZED)
        self.approach_margin = 0.04 #Margin when approaching and lifting
        self.pick_margin = 0.002 #Margin from bottom when picking
        super(AgxPick, self).__init__(name, world_interface, verbose)

    def initialise(self):
        super(AgxPick, self).initialise()
        if self.world_interface.get_picked() == self.brick:
            self.success()
        else:
            self.state = pt.common.Status.RUNNING
            self.set_int_state(self.States.INITIALIZED)

    def update(self):
        super(AgxPick, self).update()

        if self.state == pt.common.Status.RUNNING:
            if self.int_state == self.States.INITIALIZED:
                self.set_int_state(self.States.APPROACHING)
                if self.world_interface.get_picked() is None:
                    self.target_position = self.world_interface.get_brick_position(self.brick)
                    self.target_position.z += self.world_interface.get_brick_height(self.brick) + self.approach_margin
                    self.move_to_target()
                    self.world_interface.open_gripper()
                else:
                    self.failure()
            elif self.int_state == self.States.APPROACHING:
                if self.is_gripper_in_position(atol=1.e-2) and self.world_interface.is_gripper_open():
                    self.target_position = self.world_interface.get_brick_position(self.brick)
                    self.target_position.z += self.pick_margin
                    self.move_to_target()
                    self.set_int_state(self.States.DESCENDING)
            elif self.int_state == self.States.DESCENDING:
                if self.is_gripper_in_position(atol=1.e-4):
                    self.world_interface.close_gripper()
                    self.set_int_state(self.States.GRIPPING)
            elif self.int_state == self.States.GRIPPING:
                if self.world_interface.is_gripper_closed():
                    self.target_position.z += self.approach_margin
                    self.move_to_target()
                    self.set_int_state(self.States.LIFTING)
            elif self.int_state == self.States.LIFTING:
                if self.is_gripper_in_position(atol=1.e-2) and self.world_interface.is_gripper_closed():
                    self.world_interface.set_picked(self.brick)
                    self.success()
        return self.state

    def terminate(self, new_status):
        if self.state != pt.common.Status.SUCCESS:
            if self.int_state > self.States.GRIPPING and self.world_interface.is_gripper_closed():
                self.world_interface.set_picked(self.brick)

class AgxPlace(AgxBehavior):
    """
    Place current brick on other given brick or at given position
    """
    class States(IntEnum):
        """ Behavior internal states """
        INITIALIZED = 1
        APPROACHING = 2
        DESCENDING = 3
        RELEASING = 4
        ASCENDING = 5

    def __init__(self, name, world_interface, brick=None, position=None, verbose=False):
        # pylint: disable=too-many-arguments
        if brick is not None:
            self.brick = int(brick[0])
            self.position = None
            self.release_margin = 0.0015 #Margin when releasing
        elif position is not None:
            self.position = Pos(float(position[0]), float(position[1]), float(position[2]))
            self.brick = None
            self.release_margin = 0.0025 #Margin when releasing
        self.set_int_state(self.States.INITIALIZED)
        self.approach_margin = 0.04 #Margin when approaching and ascending
        super(AgxPlace, self).__init__(name, world_interface, verbose)

    def initialise(self):
        super(AgxPlace, self).initialise()
        self.state = pt.common.Status.RUNNING
        self.set_int_state(self.States.INITIALIZED)

    def update(self):
        # pylint: disable=too-many-branches
        super(AgxPlace, self).update()

        if self.state == pt.common.Status.RUNNING:
            if self.int_state == self.States.INITIALIZED:
                self.set_int_state(self.States.APPROACHING)
                if self.world_interface.get_picked() is not None:
                    if self.brick is not None:
                        self.target_position = self.world_interface.get_brick_position(self.brick)
                        self.target_position.z += \
                            self.approach_margin + self.world_interface.get_brick_height(self.brick)
                    else:
                        self.target_position = copy(self.position)
                        self.target_position.z += self.approach_margin
                    self.move_to_target()
                else:
                    self.failure()
            elif self.int_state == self.States.APPROACHING:
                if self.is_gripper_in_position(atol=1.e-3):
                    if self.brick is not None:
                        self.target_position = self.world_interface.get_brick_position(self.brick)
                        self.target_position.z += \
                            self.world_interface.get_brick_height(self.brick) + self.release_margin
                    else:
                        self.target_position = copy(self.position)
                        self.target_position.z += self.release_margin
                    self.move_to_target()
                    self.set_int_state(self.States.DESCENDING)
            elif self.int_state == self.States.DESCENDING:
                if self.is_gripper_in_position(atol=2.e-4):
                    #Make sure that brick is still in place, otherwise something happened
                    if self.brick is not None:
                        old_position = copy(self.target_position)
                        old_position.z -= self.world_interface.get_brick_height(self.brick) + self.release_margin
                        if self.world_interface.distance(self.brick, old_position) > 1.e-3:
                            self.failure()
                            return self.state
                    self.world_interface.set_hold_picked(True)
                    self.world_interface.open_gripper()
                    self.set_int_state(self.States.RELEASING)
            elif self.int_state == self.States.RELEASING:
                if self.world_interface.is_gripper_open():
                    self.target_position.z += self.world_interface.get_brick_height(self.world_interface.get_picked()) \
                                              + self.approach_margin
                    self.move_to_target()
                    self.set_int_state(self.States.ASCENDING)
            elif self.int_state == self.States.ASCENDING:
                if self.is_gripper_in_position(atol=1.e-2):
                    self.world_interface.set_picked(None)
                    self.world_interface.set_hold_picked(False)
                    self.success()
        return self.state

    def terminate(self, new_status):
        if self.state != pt.common.Status.SUCCESS:
            if self.int_state > self.States.DESCENDING and self.world_interface.is_gripper_open():
                self.world_interface.set_picked(None)
                self.world_interface.set_hold_picked(False)

class AgxPut(AgxBehavior):
    """
    Picks brick and places it on other brick
    """
    class States(IntEnum):
        """ Behavior internal states """
        INITIALIZED = 1
        APPROACHING = 2
        DESCENDING = 3
        GRIPPING = 4
        LIFTING = 5
        APPROACHING_PLACE = 6
        DESCENDING_PLACE = 7
        RELEASING = 8
        ASCENDING = 9

    def __init__(self, name, world_interface, brick_and_pos, verbose=False):
        self.brick = int(brick_and_pos[0])
        if len(brick_and_pos) > 2:
            self.position = Pos(float(brick_and_pos[1]), float(brick_and_pos[2]), float(brick_and_pos[3]))
            self.lower = None
            self.release_margin = 0.0025 #Margin when releasing
        else:
            self.lower = int(brick_and_pos[1])
            self.position = None
            self.release_margin = 0.0015 #Margin when releasing
        self.set_int_state(self.States.INITIALIZED)
        self.approach_margin = 0.04 #Margin when approaching, lifting and ascending
        self.pick_margin = 0.002 #Margin from bottom when picking
        super(AgxPut, self).__init__(name, world_interface, verbose)

    def initialise(self):
        super(AgxPut, self).initialise()
        if self.world_interface.get_picked() != self.brick:
            if self.lower is not None:
                if AgxOn.check_on(self.world_interface, self.brick, self.lower):
                    self.success()
                else:
                    self.state = pt.common.Status.RUNNING
                    self.set_int_state(self.States.INITIALIZED)
            elif AgxAtPos.check_position(self.world_interface, self.brick, self.position):
                self.success()
            else:
                self.state = pt.common.Status.RUNNING
                self.set_int_state(self.States.INITIALIZED)

    def update(self):
        # pylint: disable=too-many-branches, too-many-statements
        super(AgxPut, self).update()

        if self.state == pt.common.Status.RUNNING:
            if self.int_state == self.States.INITIALIZED:
                self.set_int_state(self.States.APPROACHING)
                if self.world_interface.get_picked() is None:
                    self.target_position = self.world_interface.get_brick_position(self.brick)
                    self.target_position.z += self.world_interface.get_brick_height(self.brick) + self.approach_margin
                    self.move_to_target()
                    self.world_interface.open_gripper()
                else:
                    self.failure()
            elif self.int_state == self.States.APPROACHING:
                if self.is_gripper_in_position(atol=1.e-2) and self.world_interface.is_gripper_open():
                    self.target_position = self.world_interface.get_brick_position(self.brick)
                    self.target_position.z += self.pick_margin
                    self.move_to_target()
                    self.set_int_state(self.States.DESCENDING)
            elif self.int_state == self.States.DESCENDING:
                if self.is_gripper_in_position(atol=1.e-4):
                    self.world_interface.close_gripper()
                    self.set_int_state(self.States.GRIPPING)
            elif self.int_state == self.States.GRIPPING:
                if self.world_interface.is_gripper_closed():
                    self.world_interface.set_picked(self.brick)
                    self.target_position.z += self.approach_margin
                    self.move_to_target()
                    self.set_int_state(self.States.LIFTING)
            elif self.int_state == self.States.LIFTING:
                if self.is_gripper_in_position(atol=1.e-2) and self.world_interface.is_gripper_closed():
                    if self.lower is not None:
                        self.target_position = self.world_interface.get_brick_position(self.lower)
                        self.target_position.z += \
                            self.approach_margin + self.world_interface.get_brick_height(self.lower)
                    else:
                        self.target_position = copy(self.position)
                        self.target_position.z += self.approach_margin
                    self.move_to_target()
                    self.set_int_state(self.States.APPROACHING_PLACE)
            elif self.int_state == self.States.APPROACHING_PLACE:
                if self.is_gripper_in_position(atol=1.e-3):
                    if self.lower is not None:
                        self.target_position = self.world_interface.get_brick_position(self.lower)
                        self.target_position.z += \
                            self.world_interface.get_brick_height(self.lower) + self.release_margin
                    else:
                        self.target_position = copy(self.position)
                        self.target_position.z += self.release_margin
                    self.move_to_target()
                    self.set_int_state(self.States.DESCENDING_PLACE)
            elif self.int_state == self.States.DESCENDING_PLACE:
                if self.is_gripper_in_position(atol=2.e-4):
                    self.world_interface.set_hold_picked(True)
                    self.world_interface.open_gripper()
                    self.set_int_state(self.States.RELEASING)
            elif self.int_state == self.States.RELEASING:
                if self.world_interface.is_gripper_open():
                    self.target_position.z += self.world_interface.get_brick_height(self.brick) + self.approach_margin
                    self.move_to_target()
                    self.set_int_state(self.States.ASCENDING)
            elif self.int_state == self.States.ASCENDING:
                if self.is_gripper_in_position(atol=1.e-2):
                    self.world_interface.set_picked(None)
                    self.world_interface.set_hold_picked(False)
                    self.success()
        return self.state

    def terminate(self, new_status):
        if self.state != pt.common.Status.SUCCESS:
            if self.int_state > self.States.GRIPPING and self.world_interface.is_gripper_closed():
                self.world_interface.set_picked(self.brick)
            if self.int_state > self.States.DESCENDING_PLACE and self.world_interface.is_gripper_open():
                self.world_interface.set_picked(None)
                self.world_interface.set_hold_picked(False)

class AgxApplyForce(AgxBehavior):
    """
    Apply force on given brick
    """
    class States(IntEnum):
        """ Behavior internal states """
        INITIALIZED = 1
        APPROACHING = 2
        DESCENDING = 3
        PRESSING = 4
        ASCENDING = 5

    def __init__(self, name, world_interface, brick, verbose=False):
        self.brick = int(brick[0])
        self.set_int_state(self.States.INITIALIZED)
        self.approach_margin = 0.04 #Margin when approaching and ascending
        self.press_margin = 0.005 #Margin when starting press
        super(AgxApplyForce, self).__init__(name, world_interface, verbose)

    def initialise(self):
        super(AgxApplyForce, self).initialise()
        self.state = pt.common.Status.RUNNING
        self.set_int_state(self.States.INITIALIZED)

    def update(self):
        # pylint: disable=too-many-branches
        super(AgxApplyForce, self).update()

        if self.state == pt.common.Status.RUNNING:
            if self.int_state == self.States.INITIALIZED:
                self.set_int_state(self.States.APPROACHING)
                if self.world_interface.get_picked() is None:
                    self.target_position = self.world_interface.get_brick_position(self.brick)
                    self.target_position.z += self.approach_margin + self.world_interface.get_brick_height(self.brick)
                    self.move_to_target()
                    self.world_interface.close_gripper()
                else:
                    self.failure()
            elif self.int_state == self.States.APPROACHING:
                if self.is_gripper_in_position(atol=1.e-2) and self.world_interface.is_gripper_closed():
                    self.target_position = self.world_interface.get_brick_position(self.brick)
                    self.target_position.z += self.world_interface.get_brick_height(self.brick) + self.press_margin
                    self.move_to_target()
                    self.set_int_state(self.States.DESCENDING)
            elif self.int_state == self.States.DESCENDING:
                if self.is_gripper_in_position(atol=1.e-3):
                    self.target_position.z -= 0.015
                    self.move_to_target()
                    self.world_interface.apply_force(self.brick)
                    self.set_int_state(self.States.PRESSING)
            elif self.int_state == self.States.PRESSING:
                if self.world_interface.get_force_applied():
                    self.target_position.z += self.approach_margin
                    self.world_interface.release_force()
                    self.move_to_target()
                    self.set_int_state(self.States.ASCENDING)
                else:
                    self.move_to_target()
            elif self.int_state == self.States.ASCENDING:
                if self.is_gripper_in_position(atol=1.e-2):
                    self.world_interface.force_applied = None
                    self.success()
        return self.state

    def terminate(self, new_status):
        if self.state != pt.common.Status.SUCCESS:
            if self.int_state > self.States.DESCENDING:
                self.world_interface.release_force()
                self.world_interface.force_applied = None
