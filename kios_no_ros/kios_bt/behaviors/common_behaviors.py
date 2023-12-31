"""Implementing various common py trees behaviors.

- behavior node Behavior, abstract class for all behaviors: ActionBehavior, ComparisonCondition
- control node Reactive Sequence
- control node Random Selector

"""

# Copyright (c) 2022, ABB
# All rights reserved.
#
# Redistribution and use in source and binary forms, with
# or without modification, are permitted provided that
# the following conditions are met:
#
#   * Redistributions of source code must retain the
#     above copyright notice, this list of conditions
#     and the following disclaimer.
#   * Redistributions in binary form must reproduce the
#     above copyright notice, this list of conditions
#     and the following disclaimer in the documentation
#     and/or other materials provided with the
#     distribution.
#   * Neither the name of ABB nor the names of its
#     contributors may be used to endorse or promote
#     products derived from this software without
#     specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
# THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

import random
from abc import ABC, abstractmethod
from typing import Any, List, Tuple

from behaviors.behavior_lists import ParameterizedNode
import py_trees as pt
from py_trees.composites import Parallel, Selector, Sequence


def get_node(
    node_descriptor: str or ParameterizedNode,
    world_interface: Any = None,
    verbose: bool = False,
) -> Tuple[Selector or Sequence or Parallel or "RandomSelector" or "RSequence", bool]:
    """Return a py_trees behavior or composite given the descriptor."""
    has_children = False

    if isinstance(node_descriptor, ParameterizedNode):
        node = node_descriptor.behavior(
            str(node_descriptor),
            node_descriptor.get_parameters(),
            world_interface,
            verbose,
        )
    else:
        if node_descriptor == "f(":
            node = pt.composites.Selector("Fallback", memory=False)
            has_children = True
        elif node_descriptor == "fm(":
            node = pt.composites.Selector("Fallback", memory=True)
            has_children = True
        elif node_descriptor == "fr(":
            node = RandomSelector("RandomSelector")
            has_children = True
        elif node_descriptor == "s(":
            # node = pt.composites.Sequence('Sequence', memory=False)
            node = RSequence("Sequence")
            has_children = True
        elif node_descriptor == "sm(":
            node = pt.composites.Sequence("Sequence", memory=True)
            has_children = True
        elif node_descriptor == "p(":
            node = pt.composites.Parallel(
                name="Parallel",
                policy=pt.common.ParallelPolicy.SuccessOnAll(synchronise=False),
            )
            has_children = True
        else:
            raise Exception("Unrecognized node", node_descriptor)

    return node, has_children


class Behavior(pt.behaviour.Behaviour):
    """The general behavior implementation."""

    def __init__(self, name: str, world_interface: Any, verbose: bool = False):
        self.world_interface = world_interface
        self.state = None
        self.verbose = verbose
        super().__init__(name)

    def initialise(self) -> None:
        self.state = pt.common.Status.RUNNING

    def update(self) -> None:
        if self.verbose and self.state == pt.common.Status.RUNNING:
            print(self.name, ":", self.state)

    def success(self) -> None:
        """Set state success."""
        self.state = pt.common.Status.SUCCESS
        if self.verbose:
            print(self.name, ": SUCCESS")

    def failure(self) -> None:
        """Set state failure."""
        self.state = pt.common.Status.FAILURE
        if self.verbose:
            print(self.name, ": FAILURE")


class ActionBehavior(Behavior, ABC):
    """Represents an action node with pre- and postconditions."""

    @abstractmethod
    def get_preconditions(self) -> List[str]:
        """Return a list of precondition strings."""

    @abstractmethod
    def get_postconditions(self) -> List[str]:
        """Return a list of postcondition strings."""

    @abstractmethod
    def cost(self) -> int:
        """Return the cost of executing this action."""


class ComparisonCondition(pt.behaviour.Behaviour):
    """Class template for conditions comparing against constants.
    BB: condition node for comparing.
    """

    def __init__(
        self, name: str, parameters: list, world_interface: Any, _verbose: bool = False
    ):
        self.world_interface = world_interface
        self.larger_than = parameters[1]
        self.value = int(parameters[0])
        super().__init__(name)

    def compare(self, variable: Any) -> pt.common.Status:
        """Compare input variable to stored value."""
        if (self.larger_than and variable > self.value) or (
            not self.larger_than and variable < self.value
        ):
            return pt.common.Status.SUCCESS
        return pt.common.Status.FAILURE


class RSequence(pt.composites.Selector):
    """
    Rsequence for py_trees.

    Reactive sequence overriding sequence with memory, py_trees' only available sequence.
    Author: Christopher Iliffe Sprague, sprague@kth.se
    """

    def __init__(self, name: str = "Sequence", children: List[Any] = None):
        super().__init__(name=name, children=children)

    def tick(self):
        """
        Run the tick behaviour for this selector.

        Note that the status of the tick is always determined by its children,
        not by the user customized update function.

        Yields
        ------
            class:`~py_trees.behaviour.Behaviour`: a reference to itself or one of its children.

        """
        self.logger.debug(
            "%s.tick()" % self.__class__.__name__
        )  # pylint: disable=consider-using-f-string
        # Required behaviour for *all* behaviours and composites is
        # for tick() to check if it isn't running and initialise
        if self.status != pt.common.Status.RUNNING:
            # selectors dont do anything specific on initialization
            #   - the current child is managed by the update, never needs to be 'initialized'
            # run subclass (user) handles
            self.initialise()
        # run any work designated by a customized instance of this class
        self.update()
        previous = self.current_child
        for child in self.children:
            for node in child.tick():
                yield node
                if node is child and (
                    node.status == pt.common.Status.RUNNING
                    or node.status == pt.common.Status.FAILURE
                ):
                    self.current_child = child
                    self.status = node.status
                    if previous is None or previous != self.current_child:
                        # we interrupted, invalidate everything at a lower priority
                        passed = False
                        for sibling in self.children:
                            if passed and sibling.status != pt.common.Status.INVALID:
                                sibling.stop(pt.common.Status.INVALID)
                            if sibling == self.current_child:
                                passed = True
                    yield self
                    return
        # all children succeeded,
        # set succeed ourselves and current child to the last bugger who failed us
        self.status = pt.common.Status.SUCCESS
        try:
            self.current_child = self.children[-1]
        except IndexError:
            self.current_child = None
        yield self


class RandomSelector(pt.composites.Selector):
    """
    Random selector node for py_trees
    """

    def __init__(self, name="RandomSelector", children=None):
        super().__init__(name=name, children=children)

    def tick(self):
        """
        Run the tick behaviour for this selector.

        Note that the status of the tick is always determined by its children,
        not by the user customized update function.

        Yields
        ------
            class:`~py_trees.behaviour.Behaviour`: a reference to itself or one of its children.

        """
        self.logger.debug(
            "%s.tick()" % self.__class__.__name__
        )  # pylint: disable=consider-using-f-string
        # initialise
        if (
            self.status == pt.common.Status.FAILURE
            or self.status == pt.common.Status.INVALID
        ):
            # selector specific initialization - leave initialise() free for users to
            # re-implement without having to make calls to super()
            self.logger.debug(
                "%s.tick() [!RUNNING->reset current_child]" % self.__class__.__name__
            )  # pylint: disable=consider-using-f-string
            if len(self.children) > 1:
                # Select one child at random except the child we last tried executing.
                # If self.current_child is None we will choose a child entirely at random.
                self.current_child = random.choice(
                    [
                        child
                        for child in self.children
                        if child is not self.current_child
                    ]
                )
            elif len(self.children) == 1:
                # If there is only one child we should always execute it
                self.current_child = self.children[0]
            else:
                self.current_child = None

            # reset the children - don't need to worry since they will be handled
            # a) prior to a remembered starting point, or
            # b) invalidated by a higher level priority

            # user specific initialization
            self.initialise()

        for child in self.children:
            if child is not self.current_child:
                child.stop()

        # customized work
        self.update()

        # nothing to do
        if not self.children:
            self.current_child = None
            self.stop(pt.common.Status.FAILURE)
            yield self
            return

        # actual work
        previous_children = []
        while len(previous_children) < len(self.children):
            for node in self.current_child.tick():
                yield node
                if node is self.current_child:
                    if (
                        node.status == pt.common.Status.RUNNING
                        or node.status == pt.common.Status.SUCCESS
                    ):
                        self.status = node.status
                        yield self
                        return
            previous_children.append(self.current_child)
            children_left = [
                child for child in self.children if child not in previous_children
            ]
            if len(children_left) > 0:
                # Don't set current_child in last loop so we remember the last
                # child that failed
                self.current_child = random.choice(children_left)
        # all children failed,
        # set failure ourselves and current child to the last bugger who failed us
        self.status = pt.common.Status.FAILURE
        yield self
