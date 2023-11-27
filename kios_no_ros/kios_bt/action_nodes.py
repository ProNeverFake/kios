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

from kios_utils.kios_utils import ActionPhase
from kios_utils.task import *

from abc import ABC, abstractmethod


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
        task.interrupt()
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


class ActionNode(py_trees.behaviour.Behaviour, ABC):
    """Demonstrates the at-a-distance style action behaviour."""

    def __init__(self):
        """Configure the name of the behaviour."""
        super(ActionNode, self).__init__(self.node_name + "-" + self.target_name)
        self.logger.debug("%s.__init__()" % (self.__class__.__name__))
        self.mios_monitor = None
        # the blackboard client
        self.blackboard = py_trees.blackboard.Client(name=self.__class__.__name__)
        # the effects to exert
        self.effects = None
        self.set_effects()
        self.register_predicates()

    @abstractmethod
    def set_effects(self) -> None:
        """Register the effects"""
        pass

    def register_predicates(self) -> None:
        """Register the predicates on the blackboard, access = write."""
        if self.effects is not None:
            for key, _ in self.effects.items():
                self.blackboard.register_key(
                    key=key, access=py_trees.common.Access.WRITE
                )

    def take_effect(self):
        "change the predicates on the blackboard according to the set effects"
        if self.effects is not None:
            try:
                for key, value in self.effects.items():
                    self.blackboard.set(name=key, value=value)
            except KeyError:
                print("KeyError: %s" % (key))

    @abstractmethod
    def setup(self, **kwargs: int) -> None:
        # setup the parameters here
        self.logger.debug(
            "%s.setup()->connections to an external process" % (self.__class__.__name__)
        )

    # nicht unbedingt notwendig zu überschreiben
    def initialise(self) -> None:
        self.logger.debug("%s.initialise()" % (self.__class__.__name__))
        # * reset the task
        self.task.initialize()
        # * launch the subprocess, start the mios skill execution
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

        # * check the result of the startup of the task
        if self.task.task_start_response is not None:
            if bool(self.task.task_start_response["result"]["result"]) == False:
                self.logger.debug("Task startup failed")
                new_status = py_trees.common.Status.FAILURE
                return new_status
        else:
            # ! this should never happen
            self.logger.debug("Task startup in progress")
            self.logger.debug("ERRRRRRRRRRRRRRRRRRRRRRRORRR")
            new_status = py_trees.common.Status.RUNNING
            return new_status

        # * check if the task is finished
        if self.parent_connection.poll():
            self.result = self.parent_connection.recv().pop()  # ! here only bool
            if self.result == True:
                self.logger.debug("Task finished successfully")
                new_status = py_trees.common.Status.SUCCESS
                # * exert the effects
                self.take_effect()
            else:
                self.logger.debug("Task finished with error")
                new_status = py_trees.common.Status.FAILURE
        return new_status

    def terminate(self, new_status: py_trees.common.Status) -> None:
        """called after execution or when interrupted."""
        # * stop the mios_monitor process, regardless of the result
        self.mios_monitor.terminate()

        self.logger.debug(
            "%s.terminate()[%s->%s]"
            % (self.__class__.__name__, self.status, new_status)
        )


class ToolPick(ActionNode):
    """BBToolPick behaviour."""

    def __init__(self, objects_: list):
        """Configure the name of the behaviour."""

        self.objects_ = {
            "inTool": objects_[0],
        }

        self.node_name = "ToolPick"
        self.target_name = objects_[0]
        super(ToolPick, self).__init__()

        self.logger.debug("%s.__init__()" % (self.__class__.__name__))

    # ! you must override this
    def set_effects(self) -> None:
        self.effects = {
            "InTool": self.objects_["inTool"],
        }

    # ! you must override this
    def setup(self, **kwargs: int) -> None:
        # get the parameters from the parameter server
        # ! test, parameters given
        self.skill_type = "BBGripperForce"
        self.skill_parameters = {
            "skill": {
                "objects": {"Pick": self.objects_["inTool"]},
                "time_max": 30,
                # "action_context": {
                #     "action_name": "BBPick",
                #     "action_phase": "TOOL_PICK",  # Adjusted for Python enum style
                # },
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
                    "eps_in": 0,  # 0.016
                    "eps_out": 0.022,  # 0.038
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

        # * setup the task
        self.task = Task(MIOS)
        self.task.add_skill("bbskill", self.skill_type, self.skill_parameters)

        self.logger.debug("%s.setup()" % (self.__class__.__name__))

    # # modify parameters, start up the external process
    # def initialise(self) -> None:
    #     self.logger.debug("%s.initialise()" % (self.__class__.__name__))
    #     self.task.initialize()
    #     # start the subprocess
    #     self.parent_connection, self.child_connection = multiprocessing.Pipe()
    #     self.mios_monitor = multiprocessing.Process(
    #         target=mios_monitor,
    #         args=(
    #             self.task,
    #             self.child_connection,
    #         ),
    #     )
    #     atexit.register(self.mios_monitor.terminate)
    #     self.mios_monitor.start()

    # ! you don't need to override this:
    # def update(self) -> py_trees.common.Status:

    # ! you don't need to override this:
    # def terminate(self, new_status: py_trees.common.Status) -> None:


class ToolLoad(ActionNode):
    """BBToolPick behaviour."""

    def __init__(self, object_: list):
        """Configure the name of the behaviour."""
        self.objects_ = {
            "inHand": object_[0],
        }
        self.node_name = "ToolLoad"
        self.target_name = object_[0]
        super(ToolLoad, self).__init__()
        self.logger.debug("%s.__init__()" % (self.__class__.__name__))

    # ! you must override this
    def set_effects(self) -> None:
        self.effects = {
            "inHand": self.objects_["inHand"],
        }

    # ! you must override this
    def setup(self, **kwargs: int) -> None:
        # get the parameters from the parameter server
        # ! test, parameters given
        self.skill_type = "BBToolLoad"
        self.skill_parameters = {
            "skill": {
                "objects": {"ToolLoad": self.objects_["inHand"]},
                "time_max": 30,
                # "action_context": {
                #     "action_name": "BBToolLoad",
                #     "action_phase": "ActionPhase::TOOL_LOAD",
                # },
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

        # * setup the task
        self.task = Task(MIOS)
        self.task.add_skill("bbskill", self.skill_type, self.skill_parameters)

        self.logger.debug("%s.setup()" % (self.__class__.__name__))

    # # modify parameters, start up the external process
    # def initialise(self) -> None:
    #     self.logger.debug("%s.initialise()" % (self.__class__.__name__))
    #     self.task.initialize()
    #     # start the subprocess
    #     self.parent_connection, self.child_connection = multiprocessing.Pipe()
    #     self.mios_monitor = multiprocessing.Process(
    #         target=mios_monitor,
    #         args=(
    #             self.task,
    #             self.child_connection,
    #         ),
    #     )
    #     atexit.register(self.mios_monitor.terminate)
    #     self.mios_monitor.start()

    # ! you don't need to override this:
    # def update(self) -> py_trees.common.Status:

    # ! you don't need to override this:
    # def terminate(self, new_status: py_trees.common.Status) -> None:


class ToolLoadTest(ToolLoad):
    """a dummy behavior for testing."""

    def __init__(self, objects: str):
        """Configure the name of the behaviour."""
        self.objects_ = {
            "tool": objects[0],
        }
        super(ToolLoadTest, self).__init__(["tool1"])
        self.logger.debug("%s.__init__()" % (self.__class__.__name__))

    def setup(self, **kwargs: int) -> None:
        pass

    def set_effects(self) -> None:
        super().set_effects()

    def initialise(self) -> None:
        """do nothing."""
        self.logger.debug("%s.initialise()" % (self.__class__.__name__))
        pass

    def update(self) -> py_trees.common.Status:
        """only take effect."""
        self.logger.debug("%s.update()" % (self.__class__.__name__))
        new_status = py_trees.common.Status.SUCCESS
        self.take_effect()

        return new_status

    def terminate(self, new_status: py_trees.common.Status) -> None:
        """do nothing."""
        self.logger.debug(
            "%s.terminate()[%s->%s]"
            % (self.__class__.__name__, self.status, new_status)
        )
        pass


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
