"""Extension of the Behaviors class with elements required for the LfD framework."""

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

from abc import ABC, abstractmethod
import logging
from typing import Any, List, Tuple

from behaviors.behavior_lists import BehaviorLists, ParameterizedNode
import behaviors.common_behaviors as cb
from bt_learning.learning_from_demo.demonstration import Demonstration
import yaml


logger = logging.getLogger("lfd_behaviors")


class Behaviors(ABC):
    """Represent classes that parse behavior strings and return behavior tree representations."""

    def __init__(self, directory_path: str):
        """Directory_path is the path to the directory where planner settings are saved."""
        self.directory_path = directory_path

    def get_behavior_list(self) -> BehaviorLists:
        """Return a Behavior List from parsing a yaml file."""
        # initialize the dictionary an populate it while parsing the yaml
        condition_nodes = []
        action_nodes = []

        file = self.directory_path + "/BT_SETTINGS.yaml"
        if file is not None:
            try:
                with open(file) as f:
                    bt_settings = yaml.load(f, Loader=yaml.FullLoader)
                # print('Loading BT settings.')
            except FileNotFoundError:
                print("BT settings not ready.")
                print("File " + file + " not found.")
                return None
            try:
                node_name = bt_settings["condition_nodes"]
                for node in node_name:
                    condition_nodes.append(node)
            except KeyError:
                pass
            try:
                node_name = bt_settings["action_nodes"]
                for node in node_name:
                    action_nodes.append(node)
            except KeyError:
                pass

        behavior_list = BehaviorLists(
            condition_nodes=condition_nodes, action_nodes=action_nodes
        )

        return behavior_list

    @abstractmethod
    def get_node(
        self,
        node: str or ParameterizedNode,
        world_interface: Any,
        condition_parameters: Any,
    ) -> Tuple[Any, bool]:
        """
        Get a node from the behavior tree from its string representation.

        Args
        ----
            node: ParametrizedNode or string representation of the node.
            world_itnterface: interface to the robot.
            condition_parameters: list of the condition for the node.

        Returns
        -------
            node: the node of the Behavior Tree.
            has_children: if the node has children then it is a control node.

        """
        has_children = False
        node, has_children = cb.get_node(node)

        return node, has_children

    def get_condition_parameters(self, condition: str) -> List[Any]:
        """Return a list of parameters associated with condition."""
        pass

    @abstractmethod
    def get_actions(self, demonstrations: List[Demonstration]) -> List[Any]:
        """
        Return a list of available actions not necessarily included in the demonstration.

        Args:
        ----
            demonstrations: is a list of lists of EquivalentAction.

        """
        pass

    @abstractmethod
    def get_conditions(self, demonstrations: List[Demonstration]) -> List[Any]:
        """
        Return a list of available conditions not necessarily included in the demonstration.

        Args:
        ----
            demonstrations: is a list of lists of EquivalentAction.

        """
        pass

    @abstractmethod
    def compatible(self, condition1: str, condition2: str) -> bool:
        """
        Determine if two conditions are compatible.

        It must satisfy compatible(condition1, condition2) == compatible(condition2, condition1).

        Args
        ----
            condition1: string representing the first condition.
            condition2: string representing the second condition.

        Returns
        -------
            True if condition1 and condition2 are compatible, i.e. can be True at the same time.

        """
        pass
