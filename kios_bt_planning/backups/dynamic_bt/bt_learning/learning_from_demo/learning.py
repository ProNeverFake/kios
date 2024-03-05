"""Methods that allow to learn and build a BT from demonstration."""

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

import logging
import os
from typing import Any, List

from bt_learning.planner.constraints_identification import infer_preconditions
from bt_learning.learning_from_demo.demonstration import Demonstration
import bt_learning.learning_from_demo.lfd_behaviors as lfd_bt
from bt_learning.learning_from_demo.goal_identification import goal_conditions_for_demo, goal_tree
import bt_learning.planner.lfd_planner as planner
import py_trees as pt
from py_trees.trees import BehaviourTree
from simulation.py_trees_interface import PyTree, PyTreeParameters
import yaml


logger = logging.getLogger('learning')


def prepare_bt_settings(
    directory_path: str,
    demonstrations: List[Demonstration],
    behaviors: lfd_bt.Behaviors
):
    """
    Create the directory and populate it with settings and pickle files for the planner.

    Args:
    ----
        directory_path: path to the directory to create.
        demonstrations: is a list of demonstrations.
        behaviors: is a Behaviors object.

    """
    if not os.path.exists(directory_path):
        os.mkdir(directory_path)

    actions = behaviors.get_actions(demonstrations)
    conditions = behaviors.get_conditions(demonstrations)

    for demo in demonstrations:
        for action in demo:
            if action.action_string() not in actions:
                action.save_actions(directory_path)
                actions.append(action.action_string())

            for condition in action.preconditions_with_additional() + action.postconditions():

                if condition not in conditions:
                    conditions.append(condition)

    settings = {
        'fallback_nodes': ['f('],
        'sequence_nodes': ['s('],
        'condition_nodes': conditions,
        'action_nodes': actions,
        'up_node': [')']
    }

    with open(directory_path + '/BT_SETTINGS.yaml', 'w') as f:
        yaml.dump(settings, f)


def __plan(
    behaviors: lfd_bt.Behaviors,
    world_interface: Any,
    goals: List[str],
    iterations: int
) -> BehaviourTree:
    """
    Plan a behavior tree.

    Args
    ----
        behaviors: type of behaviors in the demonstration.
        world_interface: is an object that is passed to each behavior during execution.
        goals: is a list of goal conditions (strings) or None.
               If it is None, the function tries to guess the goal conditions.
        iterations: if iterations > 1, world_interface must define a reset() function.

    Returns
    -------
        tree: the Behavior Tree.

    """
    tree = goal_tree(goals, behaviors, world_interface)

    for i in range(iterations):
        world_interface.reset()
        tree = planner.plan(world_interface, behaviors, tree)

    return tree


def learn_tree(
    directory_path: str,
    demonstrations: List[Demonstration],
    behaviors: lfd_bt.Behaviors,
    world_interface: Any,
    iterations: int = 20,
    callback: Any = None
) -> PyTree:
    """
    Learns a behavior tree from demonstrations.

    Args:
    ----
        directory_path: is the path to a directory where settings and resulting tree are saved.
        demonstrations: is a list of demonstrations.
        behaviors: type of behaviors in the demonstration.
        world_interface: is an object passed to each behavior during execution for planning.
        iterations: is the number of times that the planner is run for each subtask
                    If iterations > 1, world_interface must define a reset() function.
        callback: optional callback function counting the planning steps.

                If callback is given, it is called with two arguments after each planning step:
                    - planning step number.
                    - max planning step number.

    """
    # Find goal of each demonstration
    goals = list(map(
        lambda demo: (demo, goal_conditions_for_demo(demo, behaviors)), demonstrations))

    # Group demonstration by goal condition (subtask)
    subtasks = []
    subtask_goals = []
    while len(goals) > 0:
        demo, goal_conditions = goals.pop()
        subtask = [demo]
        subtask_goals.append(goal_conditions)
        for i in range(len(goals)-1, -1, -1):
            if set(goal_conditions) == set(goals[i][1]):
                other_demo, _ = goals.pop(i)
                subtask.append(other_demo)

        subtasks.append(subtask)

    total_iterations = len(subtask_goals)
    iteration = 0
    # Find constraints for each subtask and plan
    subtask_trees = []
    for subtask, goals in zip(subtasks, subtask_goals):
        infer_preconditions(subtask, behaviors)

        prepare_bt_settings(directory_path, demonstrations, behaviors)

        tree = __plan(behaviors, world_interface, goals, iterations)
        subtask_trees.append(tree)

        iteration += 1

        if callback is not None:
            callback(iteration, total_iterations-1)

        # Reset inferred preconditions
        for demo in subtask:
            for action in demo:
                action.additional_preconditions.clear()

    # Build a tree of all subtasks
    # Memory fallback so that if one solution doesn't work, another is attempted.

    root = pt.composites.Selector('Fallback', memory=True)
    for subtask in subtask_trees:
        root.add_child(subtask)

    # Resolve conflicts resulting from pasting the trees together
    planner.handle_priority(root, behaviors)

    if len(root.children) == 1:
        # Only one child so root is unnecessary. Replace with child.
        root = root.children[0]
        root.parent = None

    parameters = PyTreeParameters()
    parameters.behavior_lists = behaviors.get_behavior_list()
    parameters.behaviors = behaviors

    tree = PyTree('', parameters, world_interface=world_interface, root=root)

    return tree
