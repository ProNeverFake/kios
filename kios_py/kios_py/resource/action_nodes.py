#!/usr/bin/env python

##############################################################################
# Imports
##############################################################################


import atexit
import multiprocessing
import multiprocessing.connection
import time

import py_trees.common
import py_trees.console as console

from kios_utils import *

##############################################################################
# Classes
##############################################################################


def planning(pipe_connection: multiprocessing.connection.Connection) -> None:
    """Emulate a (potentially) long running external process.

    Args:
        pipe_connection: connection to the planning process
    """
    idle = True
    percentage_complete = 0
    try:
        while True:
            if pipe_connection.poll():
                pipe_connection.recv()
                percentage_complete = 0
                idle = False
            if not idle:
                percentage_complete += 10
                pipe_connection.send([percentage_complete])
                if percentage_complete == 100:
                    idle = True
            time.sleep(0.5)
    except KeyboardInterrupt:
        pass


class Action(py_trees.behaviour.Behaviour):
    """Demonstrates the at-a-distance style action behaviour.

    This behaviour connects to a separately running process
    (initiated in setup()) and proceeeds to work with that subprocess to
    initiate a task and monitor the progress of that task at each tick
    until completed. While the task is running the behaviour returns
    :data:`~py_trees.common.Status.RUNNING`.

    On completion, the the behaviour returns with success or failure
    (depending on success or failure of the task itself).

    Key point - this behaviour itself should not be doing any work!
    """

    def __init__(self, name: str):
        """Configure the name of the behaviour."""
        super(Action, self).__init__(name)
        self.logger.debug("%s.__init__()" % (self.__class__.__name__))

    def setup(self, **kwargs: int) -> None:
        """Kickstart the separate process this behaviour will work with.

        Ordinarily this process will be already running. In this case,
        setup is usually just responsible for verifying it exists.
        """
        self.logger.debug(
            "%s.setup()->connections to an external process" % (self.__class__.__name__)
        )
        self.parent_connection, self.child_connection = multiprocessing.Pipe()
        self.planning = multiprocessing.Process(
            target=planning, args=(self.child_connection,)
        )
        atexit.register(self.planning.terminate)
        self.planning.start()

    def initialise(self) -> None:
        """Reset a counter variable."""
        self.logger.debug(
            "%s.initialise()->sending new goal" % (self.__class__.__name__)
        )
        self.parent_connection.send(["new goal"])
        self.percentage_completion = 0

    def update(self) -> py_trees.common.Status:
        """Increment the counter, monitor and decide on a new status."""
        new_status = py_trees.common.Status.RUNNING
        if self.parent_connection.poll():
            self.percentage_completion = self.parent_connection.recv().pop()
            if self.percentage_completion == 100:
                new_status = py_trees.common.Status.SUCCESS
        if new_status == py_trees.common.Status.SUCCESS:
            self.feedback_message = "Processing finished"
            self.logger.debug(
                "%s.update()[%s->%s][%s]"
                % (
                    self.__class__.__name__,
                    self.status,
                    new_status,
                    self.feedback_message,
                )
            )
        else:
            self.feedback_message = "{0}%".format(self.percentage_completion)
            self.logger.debug(
                "%s.update()[%s][%s]"
                % (self.__class__.__name__, self.status, self.feedback_message)
            )
        return new_status

    def terminate(self, new_status: py_trees.common.Status) -> None:
        """Nothing to clean up in this example."""
        self.logger.debug(
            "%s.terminate()[%s->%s]"
            % (self.__class__.__name__, self.status, new_status)
        )


class ToolPick(py_trees.behaviour.Behaviour):
    """BBToolPick behaviour."""

    def __init__(self, name: str):
        """Configure the name of the behaviour."""
        super(Action, self).__init__(name)
        self.logger.debug("%s.__init__()" % (self.__class__.__name__))

    def setup(self, **kwargs: int) -> None:
        # get the parameters from the parameter server
        self.skill_type = ActionPhase.TOOL_PICK

        self.skill_parameters = {
            "tool_pick": {
                "skill": {
                    "objects": {"Pick": "tool_pick"},
                    "time_max": 30,
                    "action_context": {
                        "action_name": "BBPick",
                        "action_phase": "ActionPhase::TOOL_PICK",
                    },
                    "MoveAbove": {
                        "dX_d": [0.2, 0.2],
                        "ddX_d": [0.2, 0.2],
                        "DeltaX": [0, 0, 0, 0, 0, 0],
                        "K_x": [1500, 1500, 1500, 600, 600, 600],
                    },
                    "MoveIn": {
                        "dX_d": [0.2, 0.2],
                        "ddX_d": [0.1, 0.1],
                        "DeltaX": [0, 0, 0, 0, 0, 0],
                        "K_x": [1500, 1500, 1500, 600, 600, 600],
                    },
                    "GripperForce": {
                        "width": 0.016,
                        "speed": 1,
                        "force": 120,
                        "K_x": [1500, 1500, 1500, 100, 100, 100],
                        "eps_in": 0,
                        "eps_out": 0.022,
                    },
                    "Retreat": {
                        "dX_d": [0.2, 0.2],
                        "ddX_d": [0.1, 0.1],
                        "DeltaX": [0, 0, 0, 0, 0, 0],
                        "K_x": [1500, 1500, 1500, 600, 600, 600],
                    },
                },
                "control": {"control_mode": 0},
                "user": {
                    "env_X": [0.01, 0.01, 0.002, 0.05, 0.05, 0.05],
                    "env_dX": [0.001, 0.001, 0.001, 0.005, 0.005, 0.005],
                    "F_ext_contact": [3.0, 2.0],
                },
            }
        }

        self.logger.debug(
            "%s.setup()->connections to an external process" % (self.__class__.__name__)
        )


    def initialise(self) -> None:
        """Reset a counter variable."""
        self.logger.debug(
            "%s.initialise()->sending new goal" % (self.__class__.__name__)
        )

        # start ros2 action here

        # here get the handle for further status check


    def update(self) -> py_trees.common.Status:
        """Increment the counter, monitor and decide on a new status."""
        new_status = py_trees.common.Status.RUNNING

        # here check the status of the future

        if finished:
            # here check if success or failure
            if success:
                new_status = py_trees.common.Status.SUCCESS
            else:
                new_status = py_trees.common.Status.FAILURE

        # if ros2 action finished
        if self.parent_connection.poll():
            self.percentage_completion = self.parent_connection.recv().pop()
            if self.percentage_completion == 100:
                new_status = py_trees.common.Status.SUCCESS
        # if success
        if new_status == py_trees.common.Status.SUCCESS:
            self.feedback_message = "Processing finished"
            self.logger.debug(
                "%s.update()[%s->%s][%s]"
                % (
                    self.__class__.__name__,
                    self.status,
                    new_status,
                    self.feedback_message,
                )
            )
        else:
            self.feedback_message = "{0}%".format(self.percentage_completion)
            self.logger.debug(
                "%s.update()[%s][%s]"
                % (self.__class__.__name__, self.status, self.feedback_message)
            )
        return new_status

    def terminate(self, new_status: py_trees.common.Status) -> None:
        """Nothing to clean up in this example."""
        self.logger.debug(
            "%s.terminate()[%s->%s]"
            % (self.__class__.__name__, self.status, new_status)
        )


##############################################################################
# Main
##############################################################################


def main() -> None:
    """Entry point for the demo script."""

    py_trees.logging.level = py_trees.logging.Level.DEBUG

    action = Action(name="Action")
    action.setup()
    try:
        for _unused_i in range(0, 12):
            action.tick_once()
            time.sleep(0.5)
        print("\n")
    except KeyboardInterrupt:
        pass
