"""
State Machine Simulator
"""
from enum import IntEnum
from dataclasses import dataclass


class State(IntEnum):
    """
    Definition of substates
    """
    Boolstate1 = 0
    Boolstate2 = 1
    Boolstate3 = 2
    Boolstate4 = 3

@dataclass
class SMParameters:
    """Data class for parameters for the state machine simulator """
    verbose: bool = False                                  #Extra prints

class StateMachine:
    """
    Class for handling the State Machine Simulator
    """

    def __init__(self):
        self.sm_par = SMParameters()
        self.state = [False]*(len(State))

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

    def toggle1(self):
        """ Just a simple toggle """
        self.state[State.Boolstate1] = True

    def toggle2(self):
        """ Just a simple toggle """
        self.state[State.Boolstate2] = True

    def toggle3(self):
        """ Just a simple toggle """
        self.state[State.Boolstate3] = True

    def toggle4(self):
        """ Just a simple toggle """
        self.state[State.Boolstate4] = True

    def read1(self):
        """ Just a simple state reader """
        return self.state[State.Boolstate1]

    def read2(self):
        """ Just a simple state reader """
        return self.state[State.Boolstate2]

    def read3(self):
        """ Just a simple state reader """
        return self.state[State.Boolstate3]

    def read4(self):
        """ Just a simple state reader """
        return self.state[State.Boolstate4]
