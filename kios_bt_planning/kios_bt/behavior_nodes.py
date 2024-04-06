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
from kios_utils.task import *
from kios_utils.pybt_test import fix_node_name
from kios_bt.data_types import (
    Action,
    Condition,
)
from kios_world.world_interface import WorldInterface
from kios_robot.robot_interface import RobotInterface
from kios_robot.robot_command import RobotCommand
from kios_robot.mios_async import fake_robot_command_monitor, robot_command_monitor

##############################################################################
# Classes
##############################################################################


class BehaviorNode(
    py_trees.behaviour.Behaviour,
    # ABC,
):
    """kios_bt template node."""

    def __init__(
        self,
        behavior_name: str,
        world_interface: WorldInterface,
        robot_interface: RobotInterface,
    ):
        """Configure the name of the behaviour."""
        # ! BBFIX 05042024
        self.behavior_name = fix_node_name(behavior_name)
        super(BehaviorNode, self).__init__(self.behavior_name)
        self.monitor = None
        self.world_interface = world_interface
        self.robot_interface = robot_interface

    def terminate(self, new_status: py_trees.common.Status) -> None:
        """called after execution or when interrupted."""
        # * stop the monitor process, regardless of the result
        if self.monitor is None:
            pass
            # self.debug(f'Node "{self.behavior_name}" has no monitor to terminate')
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
    multiprocessing_manager: Any
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
        super().__init__(self.behavior_name, world_interface, robot_interface)

        self.monitor = None

        # * setup the task

        self.multiprocessing_manager = Manager()

        # ! as a hint, multi-threading is better than multi-processing if you need to share memory.

        self.shared_data = self.multiprocessing_manager.dict()
        # ! BBBUG: use shared_data for robot command later!
        self.robot_command = self.robot_interface.generate_robot_command(
            action, self.shared_data
        )
        self.robot_command.initialize()
        self.logger.debug("%s.__init__()" % (self.__class__.__name__))

    def take_effect(self):
        """
        interact with the world interface to exert the effects
        """
        self.logger.info(f"Try to exert the effects of action {self.behavior_name}.")
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
        self.logger.info(f"Action node {self.behavior_name} started.")

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
                self.logger.info(f'Action "{self.behavior_name}" finished successfully')
                # new_status = py_trees.common.Status.SUCCESS
                # * exert the effects
                self.take_effect()
                # ! I think here the action node should not return a success. it should always return running.
                # ! and it should "task effect".
                # ! the target condition node in the selector will be fulfilled by the "task effect"
                # ! the action node will be interrupted by the selector, and the tick goes on...
            else:
                self.logger.info(f'Action "{self.behavior_name}" failed with error')
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
        super().__init__(self.behavior_name, world_interface, robot_interface)

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
            self.logger.info(f'Condition "{self.behavior_name}" is satisfied')
            new_status = py_trees.common.Status.SUCCESS

        else:
            self.logger.info(f'Condition "{self.behavior_name}" is not satisfied')
            new_status = py_trees.common.Status.FAILURE

        return new_status


class ActionNodeTest(ActionNode):
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
        super(ActionNode, self).__init__(
            self.behavior_name, world_interface, robot_interface
        )

        # * setup the task
        self.multiprocessing_manager = Manager()
        self.shared_data = self.multiprocessing_manager.dict()
        self.robot_command = None
        self.logger.debug("%s.__init__()" % (self.__class__.__name__))

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
        self.logger.info(f"Action node {self.behavior_name} started.")

    def update(self) -> py_trees.common.Status:
        """Increment the counter, monitor and decide on a new status."""
        self.logger.debug("%s.update()" % (self.__class__.__name__))
        new_status = py_trees.common.Status.RUNNING

        # * check if the task is finished
        if self.parent_connection.poll():
            self.result = self.parent_connection.recv().pop()  # ! here only bool
            if self.result == True:
                self.logger.info(f'Action "{self.behavior_name}" finished successfully')
                new_status = py_trees.common.Status.SUCCESS
                # * exert the effects
                self.take_effect()
            else:
                self.logger.info(f'Action "{self.behavior_name}" failed with error')
                new_status = py_trees.common.Status.FAILURE

        return new_status


class ActionNodeSim(ActionNode):
    """Node for simulation. This node will succeed after a certain number of ticks.

    Args:
        ActionNode (_type_): _description_

    Returns:
        _type_: _description_
    """

    success_flag: bool

    @staticmethod
    def from_action_node(
        action_node: ActionNode, willFail: bool = False
    ) -> "ActionNodeSim":
        return ActionNodeSim(
            action_node.action,
            action_node.world_interface,
            willFail=willFail,
        )

    def __init__(
        self,
        action: Action,
        world_interface: WorldInterface,
        robot_interface: RobotInterface = None,  #  not needed for simulation
        willFail: bool = False,
    ):
        self.willFail: bool = willFail
        self.success_flag = False
        self.tick_times = 0
        self.tick_times_target = 3
        self.hasTakenEffect = False
        self.action = action
        """Configure the name of the behaviour."""
        self.identifier = action.identifier
        self.behavior_name = self.action.name
        super(ActionNode, self).__init__(
            self.behavior_name, world_interface, robot_interface
        )
        self.logger.debug("%s.__init__()" % (self.__class__.__name__))

    def setup(self, **kwargs: int) -> None:
        self.logger.debug("%s.setup()" % (self.__class__.__name__))

    def initialise(self) -> None:
        self.logger.debug("%s.initialise()" % (self.__class__.__name__))
        self.tick_times = 0
        self.hasTakenEffect = False
        self.logger.info(f"Sim action node {self.behavior_name} started.")

    def take_effect(self):
        """
        interact with the world interface to exert the effects
        """
        if self.hasTakenEffect:
            return
        self.logger.info(f"Try to exert the effects of action {self.behavior_name}.")
        self.world_interface.take_effect(self.action)
        self.hasTakenEffect = True  # ! warning

    def update(self) -> py_trees.common.Status:
        """
        running ---> success
        """
        self.logger.debug("%s.update()" % (self.__class__.__name__))

        if self.tick_times >= self.tick_times_target:
            self.logger.info(f'Action "{self.behavior_name}" finished successfully')
            if self.willFail:
                return py_trees.common.Status.FAILURE
            # new_status = py_trees.common.Status.SUCCESS
            new_status = py_trees.common.Status.RUNNING
            # ! As I said, the action node should always return running.
            # ! this will help to catch the mistakes in action effects.
            self.take_effect()

        else:
            self.tick_times += 1
            new_status = py_trees.common.Status.RUNNING

        return new_status


class ActionNodeOnlySuccess(ActionNode):

    @staticmethod
    def from_action_node(action_node: ActionNode) -> "ActionNodeSim":
        return ActionNodeSim(action_node.action, action_node.world_interface)

    def __init__(
        self,
        action: Action,
        world_interface: WorldInterface,
        robot_interface: RobotInterface = None,  #  not needed for simulation
    ):
        self.success_flag = False
        self.tick_times = 0
        self.tick_times_target = 3
        self.hasTakenEffect = False
        self.action = action
        """Configure the name of the behaviour."""
        self.identifier = action.identifier
        self.behavior_name = self.action.name
        super(ActionNode, self).__init__(
            self.behavior_name, world_interface, robot_interface
        )
        self.logger.debug("%s.__init__()" % (self.__class__.__name__))

    def setup(self, **kwargs: int) -> None:
        self.logger.debug("%s.setup()" % (self.__class__.__name__))

    def initialise(self) -> None:
        self.logger.debug("%s.initialise()" % (self.__class__.__name__))
        self.tick_times = 0
        self.hasTakenEffect = False
        self.logger.info(f"Sim action node {self.behavior_name} started.")

    def take_effect(self):
        """
        interact with the world interface to exert the effects
        """
        if self.hasTakenEffect:
            return
        self.logger.info(f"Try to exert the effects of action {self.behavior_name}.")
        self.world_interface.take_effect(self.action)
        self.hasTakenEffect = True

    def update(self) -> py_trees.common.Status:
        new_status = py_trees.common.Status.SUCCESS
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
