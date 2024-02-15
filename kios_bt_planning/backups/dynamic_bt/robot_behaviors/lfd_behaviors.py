"""
Definition of Mobile YuMi Behaviors.

Combines YuMi behaviors with MobileYuMi behaviors.
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

from typing import Any, List, Tuple

from behaviors.behavior_lists import BehaviorLists
from behaviors.common_behaviors import Behaviour, RandomSelector, RSequence
from bt_learning.learning_from_demo.demonstration import Demonstration
from py_trees.composites import Selector, Sequence
from robot_behaviors.hri_behaviors.hri_behaviors import HRIBehaviors
from robot_behaviors.mobile_yumi_behaviors.lfd_behaviors import MobileYuMiBehaviors
from robot_interface.interface import HRIInterface, MobileBaseInterface, YuMiInterface
import yaml


class LfDBehaviors(HRIBehaviors, MobileYuMiBehaviors):
    """Combine YuMi behaviors with the Mobile base ones."""

    def __init__(self, directory_path: str):
        """Directory_path is the path to the directory where planner settings are saved."""
        super().__init__(directory_path)
        self.behavior_list = self.get_behavior_list()

    def get_behavior_list(self) -> BehaviorLists:
        """
        Return the behavior list.

        In order to avoid superclass method to load wrong behaviors for the class,
        it just overrides the method (it is a copy of the same method).
        """
        behavior_list = super().get_behavior_list()
        return behavior_list

    def get_node(
        self,
        node: str,
        world_interface: HRIInterface or MobileBaseInterface or YuMiInterface,
        condition_parameters: Any
    ) -> Tuple[Behaviour or RSequence or RandomSelector or Selector or Sequence, bool]:
        """
        Link a string representation of the skill to the behavior.

        Args
        ----
            string: name of the robot skill as string.
            world_interface: interface to the robot hardware.
            condition_parameters: pre- and post-conditions of the skill.

        Returns
        -------
            node: behavior tree node, eventually inherits from py_trees
            has_children: bool to determine if the node is a control node or a behavior.

        """
        has_children = False

        try:
            # get nodes from HRI superclass
            node, has_children = HRIBehaviors.get_node(
                self, node, world_interface, condition_parameters)
        except Exception:
            # if it doesn't work, try with the robot instead
            node, has_children = MobileYuMiBehaviors.get_node(
                self, node, world_interface, condition_parameters)

        return node, has_children

    def get_actions(self, demonstrations: Demonstration) -> List[str]:
        """
        Get the combined actions for YuMi and the MobileBase from a demonstration.

        Args
        ----
            demonstration: the demonstration to parse.

        Returns
        -------
            actions: list of the actions in the demonstration.

        """
        actions = HRIBehaviors.get_actions(self, demonstrations) +\
            MobileYuMiBehaviors.get_actions(self, demonstrations)
        print(actions)

        return actions

    def get_conditions(self, demonstrations: Demonstration) -> List[str]:
        """
        Get the combined conditions for YuMi and the MobileBase from a demonstration.

        Args
        ----
            demonstration: the demonstration to parse.

        Returns
        -------
            conditions: list of the conditions in the demonstration.

        """
        conditions = HRIBehaviors.get_conditions(self, demonstrations) +\
            MobileYuMiBehaviors.get_conditions(self, demonstrations)

        return conditions
