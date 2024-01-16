"""Module containing methods that allow to identify task goals."""

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

from typing import Any, List

from behaviors.common_behaviors import RSequence
from bt_learning.planner.constraints_identification import contains_conflicting
from bt_learning.learning_from_demo.demonstration import Demonstration
from py_trees.trees import BehaviourTree


def goal_conditions_for_demo(
    demo: Demonstration,
    behaviors: Any
) -> List[str]:
    """
    Infer the goal conditions of a single demonstration.

    Args
    ----
        demo: the demonstration to infer the goal of.
        behavior: check the behavior to remove conflicting conditions.

    Returns
    -------
        goals: list of the goals inferred in the demonstration.

    """
    goals = []
    for i in range(len(demo)-1, -1, -1):
        for condition in demo[i].postconditions():
            if condition not in goals and not contains_conflicting(behaviors, goals, condition):
                goals.append(condition)

    goals.reverse()
    return goals


def goal_tree(
    goals: List[str],
    behaviors: Any,
    world_interface: Any
) -> BehaviourTree:
    """
    Construct a Behavior Tree strarting from the goals.

    Args
    ----
        goals: list of all goals inferred from the demonstration.
        behaviors: behavior in the demontration, as defined in robot_behaviors package.
        world_interface: interface to the robot.

    Returns
    -------
        tree: a Behavior Tree of goal conditions.

    """
    tree = RSequence()
    for goal in goals:
        node, _ = behaviors.get_node(goal, world_interface, None)
        tree.add_child(node)

    return tree
