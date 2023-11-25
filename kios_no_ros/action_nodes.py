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

from kios_utils import ActionPhase
from task import *

# from abc import ABC, abstractmethod


##############################################################################
# Classes
##############################################################################

MIOS = "127.0.0.1"


def mios_monitor(
    task: Task, pipe_connection: multiprocessing.connection.Connection
) -> None:
    """Emulate a (potentially) long running external process.

    Args:
        pipe_connection: connection to the mios_monitor process
    """
    try:
        task.start()
        # hanlde startup failure
        # ! check the response here
        print(str(task.task_start_response))

        if bool(task.task_start_response["result"]["result"]) == False:
            pipe_connection.send([False])
            return

        _ = task.wait()

        if bool(task.task_wait_response["result"]["result"]) == True:
            pipe_connection.send([True])
        else:
            pipe_connection.send([False])
    except KeyboardInterrupt:
        pass


class ActionNode(py_trees.behaviour.Behaviour):
    """Demonstrates the at-a-distance style action behaviour."""

    def __init__(self, name: str):
        """Configure the name of the behaviour."""
        super(ActionNode, self).__init__(name)
        self.logger.debug("%s.__init__()" % (self.__class__.__name__))
        self.mios_monitor = None

    def setup(self, **kwargs: int) -> None:
        # setup the parameters here
        self.logger.debug(
            "%s.setup()->connections to an external process" % (self.__class__.__name__)
        )

    def initialise(self) -> None:
        # start the subprocess
        self.parent_connection, self.child_connection = multiprocessing.Pipe()
        self.mios_monitor = multiprocessing.Process(
            target=mios_monitor, args=(self.child_connection,)
        )
        atexit.register(self.mios_monitor.terminate)
        self.mios_monitor.start()

    def update(self) -> py_trees.common.Status:
        """Increment the counter, monitor and decide on a new status."""
        self.logger.debug("%s.update()" % (self.__class__.__name__))
        new_status = py_trees.common.Status.RUNNING
        if self.parent_connection.poll():
            self.result = self.parent_connection.recv().pop()  # ! here only bool
            if self.result == True:
                self.logger.debug("Task finished successfully")
                new_status = py_trees.common.Status.SUCCESS
            else:
                self.logger.debug("Task finished with error")
                new_status = py_trees.common.Status.FAILURE
        return new_status

    def terminate(self, new_status: py_trees.common.Status) -> None:
        """called after execution or when interrupted."""
        self.mios_monitor.terminate()

        self.logger.debug(
            "%s.terminate()[%s->%s]"
            % (self.__class__.__name__, self.status, new_status)
        )


class ToolPick(ActionNode):
    """BBToolPick behaviour."""

    def __init__(self, name: str):
        """Configure the name of the behaviour."""
        super(ToolPick, self).__init__(name)
        self.logger.debug("%s.__init__()" % (self.__class__.__name__))

    # ! you must override this
    def setup(self, **kwargs: int) -> None:
        # get the parameters from the parameter server
        # ! test, parameters given
        self.skill_type = "BBToolLoad"
        self.skill_parameters = {
            "skill": {
                "objects": {"ToolLoad": "tool_load"},
                "time_max": 30,
                "action_context": {
                    "action_name": "BBToolLoad",
                    "action_phase": "ActionPhase::TOOL_LOAD",
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
                "GripperMove": {
                    "width": 0.03907,  # this is for load the tool box
                    "speed": 1,
                    "K_x": [1500, 1500, 1500, 100, 100, 100],
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

        # setup the task
        self.task = Task(MIOS)
        self.task.add_skill("bbskill", self.skill_type, self.skill_parameters)

        self.logger.debug(
            "%s.setup()->connections to an external process" % (self.__class__.__name__)
        )

    # modify parameters, start up the external process
    def initialise(self) -> None:

        # start the subprocess
        self.parent_connection, self.child_connection = multiprocessing.Pipe()
        self.mios_monitor = multiprocessing.Process(
            target=mios_monitor,
            args=(
                self.task,
                self.child_connection,
            ),
        )
        atexit.register(self.mios_monitor.terminate)
        self.mios_monitor.start()

    # ! you don't need to override this:
    # def update(self) -> py_trees.common.Status:

    # ! you don't need to override this:
    # def terminate(self, new_status: py_trees.common.Status) -> None:


##############################################################################
# Main
##############################################################################


def main() -> None:
    """Entry point for the demo script."""

    py_trees.logging.level = py_trees.logging.Level.DEBUG

    action = ActionNode(name="Action")
    action.setup()
    try:
        for _unused_i in range(0, 12):
            action.tick_once()
            time.sleep(0.5)
        print("\n")
    except KeyboardInterrupt:
        pass
