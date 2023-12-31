"""List of behaviors to be used for the genetic programming.

- class of behavior list
- class of parameterized node
- choose random node
- distinguish between different types of nodes

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

from copy import deepcopy
from dataclasses import dataclass
from dataclasses import field
from enum import IntEnum
import random
import string
from typing import Any, List


class ParameterTypes(IntEnum):
    """Define the parameter types."""

    INDEX = 0  # Index of a list, typically in a list of objects
    POSITION = 1  # A position consists of three values, x y and z
    INTEGER = 2
    FLOAT = 3
    STRING = 4  # Used to describe object targets semantically
    TUPLE = 5  # Used for storing other parameters that don't fall in previous cases


def get_parameter_type_from_value(value: Any) -> ParameterTypes:
    """
    Return the ParameterType associated to the input data.

    Note: the data type INDEX is not automatically detected.
    """
    data_type = None
    if (
        isinstance(value, tuple)
        and len(value) == 3
        and (isinstance(sum(list(value)), float) or isinstance(sum(list(value)), int))
    ):
        data_type = ParameterTypes.POSITION
    elif isinstance(value, int):
        data_type = ParameterTypes.INTEGER
    elif isinstance(value, float):
        data_type = ParameterTypes.FLOAT
    elif isinstance(value, str):
        data_type = ParameterTypes.STRING
    elif isinstance(value, tuple):
        data_type = ParameterTypes.TUPLE
    else:
        raise TypeError(
            f"Data Type {type(value)} not supported by class ParameterTypes!"
        )

    return data_type


def random_range(min_value: int, max_value: int, step: int) -> int:
    """
    Return a value from random range with some checks for step.

    Also includes max in the range unlike randrange.
    """
    if step == 0:
        return min_value
    return random.randrange(min_value, max_value + step, step)


@dataclass
class NodeParameter:
    """Define the settings for the parametrized node."""

    list_of_values: list = field(default_factory=list)
    min: Any = None
    max: Any = None
    step: Any = None
    placement: int = -1  # Placement within string, just for readability
    data_type: Any = ParameterTypes.INTEGER
    value: Any = None  # Current value of this parameter

    def set_random_value(self) -> int or float:
        """Give the parameter a random value within the constraints."""
        if self.list_of_values:
            self.value = random.choice(self.list_of_values)
        elif self.data_type in (ParameterTypes.INDEX, ParameterTypes.INTEGER):
            self.value = random_range(self.min, self.max, self.step)
        elif self.data_type == ParameterTypes.POSITION:
            if self.step:  # Integer values
                self.value = (
                    random_range(self.min[0], self.max[0], self.step[0]),
                    random_range(self.min[1], self.max[1], self.step[1]),
                    random_range(self.min[2], self.max[2], self.step[2]),
                )
            else:
                self.value = (
                    random.uniform(self.min[0], self.max[0]),
                    random.uniform(self.min[1], self.max[1]),
                    random.uniform(self.min[2], self.max[2]),
                )
        elif self.data_type == ParameterTypes.FLOAT:
            self.value = random.uniform(self.min, self.max)
        elif self.data_type == ParameterTypes.STRING:
            self.value = "".join(
                random.choices(string.ascii_lowercase + string.digits, k=5)
            )
        else:
            raise Exception("Unknown data_type: ", self.data_type)


class ParameterizedNode:
    """Define the parametrized node."""

    def __init__(
        self,
        name: str,
        behavior: Any = None,
        parameters: list = None,
        condition: bool = True,
        comparing: bool = False,
        larger_than: bool = True,
    ):
        # pylint: disable=too-many-arguments
        self.name = name
        self.behavior = behavior
        self.parameters = deepcopy(parameters) if parameters else None
        self.condition = condition
        self.comparing = comparing
        self.larger_than = larger_than

    def __eq__(self, other: Any) -> bool:
        if not isinstance(other, ParameterizedNode):
            # don't attempt to compare against unrelated types
            return False

        return (
            self.name == other.name
            and self.parameters == other.parameters
            and self.condition == other.condition
        )

    def __repr__(self) -> str:
        """Return the string version of the node."""
        return self.to_string()

    def get_parameters(self, with_type: bool = False):
        """Return parameter values."""
        if not self.parameters:
            return None
        parameters = [
            x.value for x in self.parameters
        ]  # pylint: disable=not-an-iterable
        if with_type:
            parameters = []
            for param in self.parameters:  # pylint: disable=not-an-iterable
                if (
                    param.data_type == ParameterTypes.INDEX
                    or param.data_type == ParameterTypes.INTEGER
                ):
                    parameters.append(int(param.value))
                elif param.data_type == ParameterTypes.FLOAT:
                    parameters.append(float(param.value))
                elif param.data_type == ParameterTypes.POSITION:
                    parameters.append(
                        [
                            float(param.value[0]),
                            float(param.value[1]),
                            float(param.value[2]),
                        ]
                    )
                else:
                    parameters.append(str(param.value))

        if self.comparing:
            parameters.append(self.larger_than)
        return parameters

    def add_random_parameters(self):
        """Add random parameters to node."""
        if self.parameters:
            for parameter in self.parameters:  # pylint: disable=not-an-iterable
                parameter.set_random_value()
        if self.comparing:
            self.larger_than = random.choice([True, False])

    def to_string(self) -> str:
        """Return string representation of node for printing/logging/hashing."""
        string_node = self.name
        if self.parameters:
            for parameter in self.parameters:  # pylint: disable=not-an-iterable
                parameter_string = ""
                if self.comparing:
                    if self.larger_than:
                        parameter_string += "> "
                    else:
                        parameter_string += "< "

                parameter_string += str(parameter.value)

                if parameter.placement == 0:
                    string_node = "".join((parameter_string, " ", string_node))
                elif parameter.placement == -1:
                    string_node = "".join((string_node, " ", parameter_string))
                else:
                    string_node = (
                        string_node[: parameter.placement]
                        + " "
                        + parameter_string
                        + " "
                        + string_node[parameter.placement :]
                    )

        if self.condition:
            string_node += "?"
        else:
            string_node += "!"
        return string_node

    def set_behavior(self, behavior: Any):
        """Set the associated behavior."""
        self.behavior = behavior


# ! core class
class BehaviorLists:
    """A list of all the available nodes."""

    def __init__(
        self,
        fallback_nodes: List[str] = None,
        atomic_fallback_nodes: List[str] = None,
        sequence_nodes: List[str] = None,
        atomic_sequence_nodes: List[str] = None,
        condition_nodes: List[str] = None,
        action_nodes: List[str] = None,
    ):
        # A list of all types of fallback nodes used, typically just one
        if fallback_nodes is not None:
            self.fallback_nodes = fallback_nodes
        else:
            self.fallback_nodes = ["f("]

        # A list of all types of sequence nodes used, typically just one
        if sequence_nodes is not None:
            self.sequence_nodes = sequence_nodes
        else:
            self.sequence_nodes = ["s("]

        # Control nodes are nodes that may have one or more children/subtrees.
        # Subsequent nodes will be children/subtrees until the related up character is reached.
        # List will contain fallback_nodes, sequence_nodes and any other control nodes.
        self.control_nodes = self.fallback_nodes + self.sequence_nodes

        # Conditions nodes are childless leaf nodes that never return RUNNING state.
        self.condition_nodes = []
        if condition_nodes is not None:
            self.condition_nodes = condition_nodes

        # Action nodes are also childless leaf nodes but may return RUNNING state.
        self.action_nodes = []
        if action_nodes is not None:
            self.action_nodes = action_nodes

        # Atomic fallback nodes are fallback nodes that have a predetermined set of
        # children/subtrees that cannot be changed.
        # They behave mostly like action nodes except that they may not be
        # the children of fallback nodes. Length is counted as one.
        self.atomic_fallback_nodes = []
        if atomic_fallback_nodes is not None:
            self.atomic_fallback_nodes = atomic_fallback_nodes

        # Atomic sequence nodes are sequence nodes that have a predetermined set of
        # children/subtrees that cannot be changed.
        # They behave mostly like action nodes except that they may not be
        # the children of sequence nodes. Length is counted as one.
        self.atomic_sequence_nodes = []
        if atomic_sequence_nodes is not None:
            self.atomic_sequence_nodes = atomic_sequence_nodes

        # The up node is not a node but a character that marks the end of a control nodes
        # set of children and subtrees.
        self.up_node = [")"]

        # behavior_nodes: primitive (atomic) action nodes.
        self.behavior_nodes = (
            self.action_nodes + self.atomic_fallback_nodes + self.atomic_sequence_nodes
        )
        self.leaf_nodes = self.condition_nodes + self.behavior_nodes
        self.nonleaf_nodes = self.control_nodes + self.up_node

    def merge_behaviors(self, other_bl: "BehaviorLists") -> None:
        """Merge this behaviors with those of another BehaviorList.
        used to import behaviors from another BehaviorList
        """
        self.action_nodes += [
            node for node in other_bl.action_nodes if node not in self.action_nodes
        ]
        self.condition_nodes += [
            node
            for node in other_bl.condition_nodes
            if node not in self.condition_nodes
        ]
        self.atomic_fallback_nodes += [
            node
            for node in other_bl.atomic_fallback_nodes
            if node not in self.atomic_fallback_nodes
        ]
        self.atomic_sequence_nodes += [
            node
            for node in other_bl.atomic_sequence_nodes
            if node not in self.atomic_sequence_nodes
        ]
        self.fallback_nodes += [
            node for node in other_bl.fallback_nodes if node not in self.fallback_nodes
        ]
        self.sequence_nodes += [
            node for node in other_bl.sequence_nodes if node not in self.sequence_nodes
        ]

        self.behavior_nodes = (
            self.action_nodes + self.atomic_fallback_nodes + self.atomic_sequence_nodes
        )
        self.leaf_nodes = self.condition_nodes + self.behavior_nodes
        self.nonleaf_nodes = self.control_nodes + self.up_node

    def is_fallback_node(self, node: str) -> bool:
        """Is node a fallback node."""
        if node in self.fallback_nodes:
            return True
        return False

    def is_sequence_node(self, node: str) -> bool:
        """Is node a sequence node."""
        if node in self.sequence_nodes:
            return True
        return False

    def is_control_node(self, node: str) -> bool:
        """Is node a control node."""
        if node in self.control_nodes:
            return True
        return False

    def get_random_control_node(self) -> ParameterizedNode:
        """Return a random control node."""
        node = random.choice(self.control_nodes)

        return node

    def is_condition_node(self, node: ParameterizedNode) -> bool:
        # pylint: disable=no-self-use
        """Is node a condition node."""
        if isinstance(node, ParameterizedNode):
            return node.condition
        elif node in self.condition_nodes:
            return True
        return False

    def get_random_condition_node(self) -> ParameterizedNode:
        """Return a random condition node."""
        node = deepcopy(random.choice(self.condition_nodes))
        if isinstance(node, ParameterizedNode):
            node.add_random_parameters()
        return node

    def is_action_node(self, node: ParameterizedNode) -> bool:
        # pylint: disable=no-self-use
        """Is node an action node."""
        if isinstance(node, ParameterizedNode):
            return node.condition
        elif node in self.action_nodes:
            return True
        return False

    def get_random_action_node(self) -> ParameterizedNode:
        """Return a random condition node."""
        node = deepcopy(random.choice(self.action_nodes))
        if isinstance(node, ParameterizedNode):
            node.add_random_parameters()
        return node

    def is_behavior_node(self, node: ParameterizedNode) -> bool:
        # pylint: disable=no-self-use
        """Is node a behavior node."""
        if isinstance(node, ParameterizedNode):
            return not node.condition
        elif node in self.behavior_nodes:
            return True
        return False

    def get_random_behavior_node(self) -> ParameterizedNode:
        """Return a random behavior node."""
        node = deepcopy(random.choice(self.behavior_nodes))
        if isinstance(node, ParameterizedNode):
            node.add_random_parameters()
        return node

    def is_leaf_node(self, node: ParameterizedNode) -> bool:
        """Is node a leaf node."""
        if node in self.leaf_nodes:
            return True
        return isinstance(node, ParameterizedNode)

    def get_random_leaf_node(self) -> ParameterizedNode:
        """Return a random leaf node."""
        node = deepcopy(random.choice(self.leaf_nodes))
        if isinstance(node, ParameterizedNode):
            node.add_random_parameters()
        return node

    def is_up_node(self, node: str) -> bool:
        """Is node an up node."""
        return node in self.up_node

    def get_up_node(self) -> str:
        """Return up node."""
        return self.up_node[0]

    def is_valid_node(self, node: ParameterizedNode) -> bool:
        """Return True if node is valid node, False otherwise."""
        return self.is_leaf_node(node) or node in self.nonleaf_nodes
