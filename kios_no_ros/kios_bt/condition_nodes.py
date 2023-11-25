import atexit
import multiprocessing
import multiprocessing.connection
import time

import py_trees.common
import py_trees.console as console

from kios_utils.kios_utils import ActionPhase
from kios_utils.task import *

from abc import ABC, abstractmethod


class ConditionNode(py_trees.behaviour.Behaviour, ABC):
    """abstract condition node."""

    def __init__(self, name: str = "Condition"):
        """Configure the name of the behaviour."""
        super(ConditionNode, self).__init__(name)
        self.logger.debug("%s.__init__()" % (self.__class__.__name__))
        self.check_predicates = None

    @abstractmethod
    def setup(self, **kwargs: int) -> None:
        # register the predicates on the blackboard here
        # register what is "success" for the predicates here
        self.logger.debug(
            "%s.setup()->register the predicates" % (self.__class__.__name__)
        )

    def initialise(self) -> None:
        self.logger.debug("%s.initialise()" % (self.__class__.__name__))
        # * may implement some observing actions here
        # nothing to do here

    def update(self) -> py_trees.common.Status:
        """Increment the counter, monitor and decide on a new status."""
        self.logger.debug("%s.update()" % (self.__class__.__name__))
        new_status = py_trees.common.Status.SUCCESS

        # * check the predicates
        if self.check_predicates is not None:
            for key, value in self.check_predicates.items():
                # find the predicate on the blackboard
                # ! what if the predicate is not on the blackboard?
                current_value = py_trees.blackboard.Blackboard.get(key)
                # compare the value
                if current_value != value:
                    new_status = py_trees.common.Status.FAILURE
                    return new_status
                else:
                    continue

        return new_status

    def terminate(self, new_status: py_trees.common.Status) -> None:
        """called after execution or when interrupted."""
        # nothing to do here

        self.logger.debug(
            "%s.terminate()[%s->%s]"
            % (self.__class__.__name__, self.status, new_status)
        )


class IsToolLoaded(ConditionNode):
    """BBToolPick behaviour."""

    def __init__(self, name: str):
        """Configure the name of the behaviour."""
        super(ToolLoaded, self).__init__(name)
        self.logger.debug("%s.__init__()" % (self.__class__.__name__))

    # ! you must override this
    def setup(self, **kwargs: int) -> None:
        # register the predicates on the blackboard here

        self.logger.debug(
            "%s.setup()->register the predicates" % (self.__class__.__name__)
        )
        self.check_predicates = {
            "isToolLoaded": True,
        }
        # register what is "success" for the predicates here
