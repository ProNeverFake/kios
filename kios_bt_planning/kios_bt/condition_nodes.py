"""
old version for testing
"""
# ! discarded

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

    def __init__(self):
        """Configure the name of the behaviour."""
        super(ConditionNode, self).__init__(self.node_name + "-" + self.target_name)
        self.logger.debug("%s.__init__()" % (self.__class__.__name__))
        # the blackboard client
        self.blackboard = py_trees.blackboard.Client(name=self.__class__.__name__)
        # the conditions to check
        self.conditions = None

    def register_predicates(self) -> None:
        """Register the predicates on the blackboard."""
        # register the readonly keys
        # use self.conditions
        if self.conditions is not None:
            for key, _ in self.conditions.items():
                self.blackboard.register_key(
                    key=key, access=py_trees.common.Access.READ
                )
                # ? if the key is not on the blackboard, set it to default value

    @abstractmethod
    def set_conditions(self) -> None:
        """Register the condition"""
        pass

    def setup(self, **kwargs: int) -> None:
        # register what is "success" for the predicates here
        self.set_conditions()
        # register the predicates on the blackboard here
        self.register_predicates()
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
        if self.conditions is not None:
            for key, value in self.conditions.items():
                # find the predicate on the blackboard
                # ! what if the predicate is not on the blackboard?
                current_value = self.blackboard.get(key)
                # compare the value
                if current_value == value:
                    continue
                else:
                    new_status = py_trees.common.Status.FAILURE
                    return new_status

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

    def __init__(self, objects_: list):
        """Configure the name of the behaviour."""
        self.objects_ = {
            "tool": objects_[0],
        }
        self.node_name = "IsToolLoaded"
        self.target_name = objects_[0]
        super(IsToolLoaded, self).__init__()
        # self.logger.debug("%s.__init__()" % (self.__class__.__name__))

    # ! you must override this
    def set_conditions(self) -> None:
        self.conditions = {
            "inHand": self.objects_["tool"],
        }


class isInHand(ConditionNode):
    """"""

    def __init__(self, objects_: list):
        """Configure the name of the behaviour."""
        self.objects_ = {
            "tool": objects_[0],
        }
        self.node_name = "isInHand"
        self.target_name = objects_[0]
        super(isInHand, self).__init__()
        # self.logger.debug("%s.__init__()" % (self.__class__.__name__))

    # ! you must override this
    def set_conditions(self) -> None:
        self.conditions = {
            "inHand": self.objects_["tool"],
        }


class isInTool(ConditionNode):
    """"""

    def __init__(self, objects_: list):
        """Configure the name of the behaviour."""
        self.objects_ = {
            "object": objects_[0],
        }
        self.node_name = "isInTool"
        self.target_name = objects_[0]
        super(isInTool, self).__init__()
        # self.logger.debug("%s.__init__()" % (self.__class__.__name__))

    # ! you must override this
    def set_conditions(self) -> None:
        self.conditions = {
            "inTool": self.objects_["object"],
        }


class isObjectAt(ConditionNode):
    """"""

    def __init__(self, objects_: list):
        """Configure the name of the behaviour."""
        self.objects_ = {
            "object": objects_[0],
            "location": objects_[1],
        }
        self.node_name = "isObjectAt"
        self.target_name = objects_[0] + "-" + objects_[1]
        super(isInHand, self).__init__()
        # self.logger.debug("%s.__init__()" % (self.__class__.__name__))

    # ! you must override this
    def set_conditions(self) -> None:
        self.conditions = {
            self.objects_["object"] + "-at": self.objects_["location"],
        }
