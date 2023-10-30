"""
State Machine Simulator for duplo brick handling
"""
import random
from math import sqrt
from enum import IntEnum
from dataclasses import dataclass
from copy import copy

@dataclass
class Pos:
    """
    Cartesian position
    """
    x: float = 0
    y: float = 0
    z: float = 0
    def __add__(self, other):
        return Pos(self.x + other.x, self.y + other.y, self.z + other.z)

    def __str__(self):
        return '(' + str(self.x) + ', ' + str(self.y) + ', ' + str(self.z) + ')'

@dataclass
class State():
    """
    Definition of substates
    """
    bricks = None
    picked = None

@dataclass
class SMParameters:
    """Data class for parameters for the state machine simulator """
    pick_height: float = 0.04
    brick_height: float = 0.0192
    not_pressed_dist: float = 0.002
    pos_margin: float = 0.001
    ontop_margin: float = 0.003
    random_events: bool = False         #Random events
    verbose: bool = False               #Extra prints

class SMMode(IntEnum):
    """Special state machine modes for testing different situations """
    DEFAULT = 0
    CROISSANT = 1
    BALANCE = 2
    BLOCKING = 3

class StateMachine:
    """
    Class for handling the State Machine Simulator
    """
    def __init__(self, start_positions, random_events=False, parameters=None, mode=0):
        if parameters is None:
            self.sm_par = SMParameters()
        else:
            self.sm_par = parameters
        self.sm_par.random_events = random_events
        self.mode = mode
        self.state = State()
        self.state.bricks = []
        for pos in start_positions:
            self.state.bricks.append(copy(pos))

    def get_feedback(self):
        # pylint: disable=no-self-use
        """ Dummy to fit template """
        return True

    def get_sensor_data(self):
         # pylint: disable=no-self-use
        """ Dummy to fit template """
        return True

    def send_references(self):
         # pylint: disable=no-self-use
        """ Dummy to fit template """
        return

    def random_event(self):
        """
        Has a probability of creating a random event,
        dropping the current picked brick at a random position
        """
        if self.sm_par.random_events:
            number = random.random()
            if number < 0.5:
                if self.state.picked is not None:
                    self.state.bricks[self.state.picked].x += random.gauss(0, 0.05)
                    self.state.bricks[self.state.picked].y += random.gauss(0, 0.05)
                    self.state.bricks[self.state.picked].z = 0
                    self.state.picked = None

    def hand_empty(self):
        """ Checks if any object is picked """
        if self.state.picked is None:
            return True
        return False

    def pick(self, brick):
        """ Picks given brick """
        if self.state.picked is None:
            self.state.picked = brick
            self.state.bricks[brick].z += self.sm_par.pick_height
            return True
        return False

    def place(self, position=None, brick=None):
        # pylint: disable=too-many-branches
        """ Place current picked object on given position or given brick """
        if self.state.picked is not None and \
            (position is not None or brick is not None):
            if self.mode == SMMode.CROISSANT:
                if self.state.picked == 2 or self.state.picked == 3:
                    if abs(self.state.bricks[1].y) < 0.01:
                        return False
            if position is not None:
                new_brick_position = position
            elif brick is not None:
                new_brick_position = copy(self.state.bricks[brick])
                new_brick_position.z += self.sm_par.brick_height + self.sm_par.not_pressed_dist

            if self.mode == SMMode.BALANCE:
                if self.state.picked == 0 and brick == 1:
                    new_brick_position.y += 0.01
                elif self.state.picked == 2 and brick is None and self.state.bricks[1].y == 0.0:
                    new_brick_position.z += 0.0192
            elif self.mode == SMMode.BLOCKING:
                if position is not None:
                    if self.state.picked == 2:
                        for i in range(len(self.state.bricks)):
                            if self.state.picked != i and \
                               new_brick_position.x == self.state.bricks[i].x and \
                               abs(new_brick_position.y - self.state.bricks[i].y) < 0.1:
                                return False
                    for i in range(len(self.state.bricks)):
                        if self.state.picked != i and self.distance(i, new_brick_position) < 0.001:
                            return False
                elif brick is not None:
                    if brick == 2:
                        new_brick_position.z += 0.05

            move_brick_to(self.state.bricks[self.state.picked], new_brick_position)
            self.state.picked = None
            return True
        return False

    def apply_force(self, brick):
        """ Applies force on brick """
        if self.state.picked is None:
            upper_brick = self.state.bricks[brick]
            for lower_brick in self.state.bricks:
                if lower_brick is not upper_brick:
                    if self.on_top(upper_brick, lower_brick):
                        upper_brick.z = lower_brick.z + self.sm_par.brick_height
                        break
            return True
        return False

    def get_picked(self):
        """ return picked state """
        return self.state.picked

    def distance(self, brick, position):
        """ Returns distance between given brick and given position """
        return sqrt((self.state.bricks[brick].x - position.x)**2 + \
            (self.state.bricks[brick].y - position.y)**2 + \
            (self.state.bricks[brick].z - position.z)**2)

    def on_top(self, upper_brick, lower_brick):
        """ Checks if upper brick is on top of lower brick with margins """
        if abs(upper_brick.x - lower_brick.x) < self.sm_par.ontop_margin and \
            abs(upper_brick.y - lower_brick.y) < self.sm_par.ontop_margin and \
            0 < upper_brick.z - lower_brick.z <= self.sm_par.brick_height + self.sm_par.ontop_margin:
            return True
        return False

def move_brick_to(brick, position):
    """ Move brick to given position """
    brick.x = position.x
    brick.y = position.y
    brick.z = position.z
