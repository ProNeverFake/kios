"""
action nodes and condition nodes that used for generating the nodes in subtrees
"""

##############################################################################
# Imports
##############################################################################

# for multiprocessing
import atexit
import multiprocessing
import multiprocessing.connection
from multiprocessing import Manager

# for abstract class
from abc import ABC, abstractmethod
from typing import Any

# for testing
import time

# pytrees
import py_trees.common
import py_trees.console as console

# kios
from backups.kios_utils import ActionPhase
from kios_utils.task import *
from kios_bt.data_types import (
    ActionInstance,
    GroundedAction,
    GroundedCondition,
    Action,
    Condition,
)
from kios_world.world_interface import WorldInterface
from kios_robot.robot_interface import RobotInterface
from kios_robot.robot_command import RobotCommand
from kios_robot.mios_async import fake_robot_command_monitor, robot_command_monitor

# from kios_bt.mios_async import mios_monitor, fake_monitor


##############################################################################
# Classes
##############################################################################

# * mios server address: localhost
MIOS = "127.0.0.1"


class BehaviorNode(py_trees.behaviour.Behaviour, ABC):
    """kios_bt template node."""

    def __init__(
        self,
        world_interface: WorldInterface,
        robot_interface: RobotInterface,
    ):
        """Configure the name of the behaviour."""
        super(BehaviorNode, self).__init__(self.behavior_name)
        self.monitor = None
        self.world_interface = world_interface
        self.robot_interface = robot_interface

    def terminate(self, new_status: py_trees.common.Status) -> None:
        """called after execution or when interrupted."""
        # * stop the monitor process, regardless of the result
        if self.monitor is None:
            pass
            # print(self.__class__.__name__ + ": monitor is None")
        else:
            self.monitor.terminate()

        self.logger.debug(
            "%s.terminate()[%s->%s]"
            % (self.__class__.__name__, self.status, new_status)
        )


class ActionNode(BehaviorNode):
    """Demonstrates the at-a-distance style action behaviour."""

    action: Action

    world_interface: WorldInterface
    robot_interface: RobotInterface

    monitor: ...
    shared_data: Any
    robot_command: RobotCommand
    multiprocessing_manager: Manager
    parent_connection: multiprocessing.connection.Connection
    child_connection: multiprocessing.connection.Connection

    def __init__(
        self,
        action: Action,
        world_interface: WorldInterface,
        robot_interface: RobotInterface,
    ):
        self.action = action
        """Configure the name of the behaviour."""
        self.identifier = action.identifier
        self.behavior_name = self.action.name
        super().__init__(world_interface, robot_interface)

        self.monitor = None

        # * setup the task

        self.multiprocessing_manager = Manager()
        self.shared_data = self.multiprocessing_manager.dict()
        self.robot_command = self.robot_interface.generate_robot_command(
            action, self.shared_data
        )
        self.robot_command.initialize()

        self.logger.debug("%s.__init__()" % (self.__class__.__name__))

    def take_effect(self):
        """
        interact with the world interface to exert the effects
        """
        self.world_interface.take_effect(self.action)

    def setup(self, **kwargs: int) -> None:
        # setup the task
        # skipped. the task is setup in the __init__ function
        self.logger.debug("%s.setup()" % (self.__class__.__name__))

    def initialise(self) -> None:
        # else, reset the task and start the external process
        self.logger.debug("%s.initialise()" % (self.__class__.__name__))
        self.robot_command.interrupt()
        # * launch the subprocess, start the mios skill execution
        self.parent_connection, self.child_connection = multiprocessing.Pipe()
        self.monitor = multiprocessing.Process(
            target=robot_command_monitor,
            args=(
                self.robot_command,
                self.child_connection,
            ),
        )
        atexit.register(self.monitor.terminate)
        self.monitor.start()

    def update(self) -> py_trees.common.Status:
        """Increment the counter, monitor and decide on a new status."""
        self.logger.debug("%s.update()" % (self.__class__.__name__))
        new_status = py_trees.common.Status.RUNNING

        # ! BBREMOVE this part. now startup won't be checked anymore.
        # # * check the result of the startup of the task
        # task_start_response = self.task.shared_data["task_start_response"]
        # print(task_start_response)

        # if task_start_response is not None:
        #     if bool(task_start_response["result"]["result"]) == False:
        #         self.logger.debug("Task startup failed")
        #         new_status = py_trees.common.Status.FAILURE
        #         return new_status

        #     if bool(task_start_response["result"]["result"]) == True:
        #         print("Task startup succeeded")

        # else:
        #     # ! this should never happen
        #     self.logger.debug("Task startup in progress")
        #     self.logger.debug("ERRRRRRRRRRRRRRRRRRRRRRRORRR")
        #     new_status = py_trees.common.Status.RUNNING
        #     return new_status

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


class ConditionNode(BehaviorNode):
    """abstract condition node."""

    def __init__(
        self,
        condition: Condition,
        world_interface: WorldInterface,
        robot_interface: RobotInterface,
    ):
        self.condition = condition
        """Configure the name of the behaviour."""
        self.identifier = condition.identifier
        self.behavior_name = condition.name
        super().__init__(world_interface, robot_interface)

        self.logger.debug("%s.__init__()" % (self.__class__.__name__))

    def register_predicates(self) -> None:
        # don't need this for now
        pass

    def setup(self, **kwargs: int) -> None:
        # register the predicates on the blackboard here
        # self.register_predicates()
        # self.logger.debug(
        #     "%s.setup()->register the predicates" % (self.__class__.__name__)
        # )
        pass

    def initialise(self) -> None:
        self.logger.debug("%s.initialise()" % (self.__class__.__name__))
        # may implement some observing actions here

    def update(self) -> py_trees.common.Status:
        """Increment the counter, monitor and decide on a new status."""
        self.logger.debug("%s.update()" % (self.__class__.__name__))
        new_status = py_trees.common.Status.FAILURE

        result = self.world_interface.check_condition(self.condition)

        if result == True:
            new_status = py_trees.common.Status.SUCCESS
        else:
            new_status = py_trees.common.Status.FAILURE

        return new_status


class ActionNodeTest(ActionNode):
    def __init__(
        self,
        action: Action,
        world_interface: WorldInterface,
        robot_interface: RobotInterface,
    ):
        super().__init__(action, world_interface, robot_interface)

    def register_predicates(self) -> None:
        pass

    def setup(self, **kwargs: int) -> None:
        self.logger.debug("%s.setup()" % (self.__class__.__name__))

    def initialise(self) -> None:
        # else, reset the task and start the external process
        self.logger.debug("%s.initialise()" % (self.__class__.__name__))
        self.robot_command.interrupt()
        # * launch the subprocess, start the mios skill execution
        self.parent_connection, self.child_connection = multiprocessing.Pipe()
        self.monitor = multiprocessing.Process(
            target=fake_robot_command_monitor,
            args=(
                self.robot_command,
                self.child_connection,
            ),
        )
        atexit.register(self.monitor.terminate)
        self.monitor.start()

    def update(self) -> py_trees.common.Status:
        """Increment the counter, monitor and decide on a new status."""
        self.logger.debug("%s.update()" % (self.__class__.__name__))
        new_status = py_trees.common.Status.RUNNING

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


##############################################################################
# Main
##############################################################################


# def main() -> None:
#     """Entry point for the demo script."""

#     py_trees.logging.level = py_trees.logging.Level.DEBUG

#     action = ActionNode(name="Action")
#     action.setup()
#     try:
#         for _unused_i in range(0, 12):
#             action.tick_once()
#             time.sleep(0.5)
#         print("\n")
#     except KeyboardInterrupt:
#         pass
