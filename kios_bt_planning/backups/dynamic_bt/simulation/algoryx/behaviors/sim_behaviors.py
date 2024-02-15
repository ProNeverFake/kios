"""Definition of Robot Behaviors."""

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

import re
import os
from typing import Any, List, Tuple

from behaviors.behavior_lists import BehaviorLists
from behaviors.common_behaviors import Behavior, RandomSelector, RSequence
from bt_learning.learning_from_demo.demonstration import Demonstration
import bt_learning.learning_from_demo.lfd_behaviors as lfd_bt
from py_trees.composites import Selector, Sequence
from simulation.algoryx.behaviors.agx_interface import AGXInterface
import simulation.algoryx.behaviors.sim_skills as skills
from simulation.algoryx.lfd.planning_itnerface import PlanningInterface
import yaml

NUMBER_REGEX = r'[-+]?(?:(?:\d*\.\d+)|(?:\d+\.?))(?:[Ee][+-]?\d+)?'

"""
The string representations of the behaviors are:
    - pick name object
    - place name object x y z tolerance frame
    - open_gripper
    - close_gripper
    - move name x y z tolerance frame
    - gripper_state open/closed
    - empty_gripper
    - in_gripper object
    - object_at object x y z tolerance frame
    - object_roughly_at object x y z tolerance frame
    - robot_at x y z tolerance frame

where name is the name of the pickle file where demonstrated actions are stored.
"""

# TODO: add support for both Right and Left gripper


class RobotBehaviors(lfd_bt.Behaviors):
    """Defines all executable actions and conditions of simulated Robot."""

    def __init__(self, directory_path: str, object_data: List[str] = None):
        """Directory_path is the path to the directory where planner settings are saved."""
        super().__init__(directory_path)
        self.behavior_list = self.get_behavior_list()
        self.object_data = object_data

    def get_behavior_list(self) -> BehaviorLists:
        """Parse the yaml file and returns the behavior list."""
        # initialize the dictionary an populate it while parsing the yaml
        behavior_list = super().get_behavior_list()
        return behavior_list

    def unstack_bt(self, name: str, world_interface: Any, condition_parameters: Any) -> RSequence:
        """Build a subtree that unstacks the items."""
        root = Selector('Fallback')
        with open(os.path.join(self.directory_path, f'{name}.yaml'), 'r') as f:
            bt = yaml.safe_load(f)

        subtree = RSequence('Sequence')
        children = [self.get_node(x, world_interface, condition_parameters)[0] for x in bt[1:-1]]
        subtree.add_children(children)

        root.add_children([
            skills.Unstack('unstacked', world_interface, self.object_data),
            subtree
        ])
        return root

    def pick_and_place_bt(
        self,
        node: str,
        world_interface: Any,
        name: str,
        target: str,
        pose: List[float],
        reference: str,
        tolerance: float,
        drop: bool
    ) -> Selector:
        """Build a pick and place subtree."""
        gripper_fb = Selector('Fallback', memory=False)
        gripper_fb.add_children([
            skills.GripperState('gripper_state open', world_interface, 'open'),
            skills.SetGripper('open_gripper', world_interface, 'open')
        ])
        pick_seq = RSequence('Sequence')
        pick_seq.add_children([
            gripper_fb,
            skills.PickBehavior(
                f'pick{name[-1]} {target}',
                self.directory_path,
                f'pick{name[-1]}',
                world_interface,
                target
            )
        ])
        pick_fb = Selector('Fallback', memory=False)
        pick_fb.add_children([
            skills.InGripper(f'in_gripper {target}', world_interface, target),
            pick_seq
        ])
        place_seq = RSequence('Sequence')
        if drop:
            place_action = skills.DropBehavior(
                f'drop{name[-1]} {target} {pose} {tolerance} {reference}',
                self.directory_path,
                f'drop{name[-1]}',
                world_interface,
                target,
                pose,
                reference,
                tolerance
            )
            place_condition = skills.ObjectRoughlyAt(
                f'object_roughly_at {target} {pose} {tolerance} {reference}',
                world_interface,
                target,
                pose,
                reference
            )
        else:
            place_action = skills.PlaceBehavior(
                f'place{name[-1]} {target} {pose} {tolerance} {reference}',
                self.directory_path,
                f'place{name[-1]}',
                world_interface,
                target,
                pose,
                reference,
                tolerance
            )
            place_condition = skills.ObjectAt(
                f'object_at {target} {pose} {tolerance} {reference}',
                world_interface,
                target,
                pose,
                reference
            )
        place_seq.add_children([pick_fb, place_action])

        root = Selector('Fallback', memory=False)
        root.add_children([
            place_condition,
            place_seq
        ])

        return root

    def get_node(
        self,
        node: str,
        world_interface: PlanningInterface or AGXInterface,
        condition_parameters: Any
    ) -> Tuple[Behavior or RSequence or RandomSelector or Selector or Sequence, bool]:
        """
        Return the Behavior Tree node given its string representation.

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

        # This parser depends on what it is written during the demo --> defined in the Action GUI

        # Actions
        if node.startswith('pick'):
            # Pick is parametrized as 'pickN obj' where N is the unique number of
            # the pick action and obj is the object.
            match = re.match('^(pick\\d+) (.+)$', node)
            node = skills.PickBehavior(
                node,
                self.directory_path,
                match[1],
                world_interface,
                match[2]
            )
        elif node.startswith('place'):
            # Place is parameterized as 'placeN object x y z'
            match_str = f'^(place\\d+) (.+) ({NUMBER_REGEX}) ({NUMBER_REGEX})' +\
                f' ({NUMBER_REGEX}) ({NUMBER_REGEX}) (.+)$'
            match = re.match(match_str, node)
            target_list = [float(i) for i in match.group(3, 4, 5)]
            node = skills.PlaceBehavior(
                node,
                self.directory_path,
                match[1],
                world_interface,
                match[2],
                [round(num, 3) for num in target_list],
                match[7],
                match[6]
            )
        elif node.startswith('drop'):
            match_str = f'^(drop\\d+) (.+) ({NUMBER_REGEX}) ({NUMBER_REGEX})' +\
                f' ({NUMBER_REGEX}) ({NUMBER_REGEX}) (.+)$'
            match = re.match(match_str, node)
            target_list = [float(i) for i in match.group(3, 4, 5)]
            node = skills.DropBehavior(
                node,
                self.directory_path,
                match[1],
                world_interface,
                match[2],
                [round(num, 3) for num in target_list],
                match[7],
                match[6]
            )
        elif node.startswith('close_gripper'):
            node = skills.SetGripper(node, world_interface, 'closed')
        elif node.startswith('open_gripper'):
            node = skills.SetGripper(node, world_interface, 'open')
        elif node.startswith('navigate'):
            match_str = f'^(navigate\\d+) ({NUMBER_REGEX}) ({NUMBER_REGEX}) ({NUMBER_REGEX})' +\
                f' ({NUMBER_REGEX}) (.+)$'
            match = re.match(match_str, node)
            node = skills.MoveTo(
                node,
                self.directory_path,
                match[1],
                world_interface,
                match.group(3, 4, 5),
                match[7]
            )
        elif node.startswith('approach'):
            parts = node.split()
            if len(parts) == 2:
                # Apporach object
                node = skills.Approach(node, world_interface, target_object=parts[1])
            else:
                # Approach position
                node = skills.Approach(
                    node,
                    world_interface,
                    [float(parts[1]), float(parts[2]), float(parts[3])],
                    parts[4]
                )
        elif node.startswith('move'):
            drop = False
            if node.startswith('move_roughly'):
                match_str = f'^(move_roughly\\d+) (.+) ({NUMBER_REGEX}) ({NUMBER_REGEX})' +\
                    f' ({NUMBER_REGEX}) ({NUMBER_REGEX}) (.+)$'
                drop = True
            else:
                match_str = f'^(move\\d+) (.+) ({NUMBER_REGEX}) ({NUMBER_REGEX})' +\
                    f' ({NUMBER_REGEX}) ({NUMBER_REGEX}) (.+)$'
            match = re.match(match_str, node)
            target_list = [float(i) for i in match.group(3, 4, 5)]
            node = self.pick_and_place_bt(
                node,
                world_interface,
                name=match[1],
                target=match[2],
                pose=[round(num, 3) for num in target_list],
                reference=match[7],
                tolerance=match[6],
                drop=drop
            )
        elif node.startswith('unstack_boxes'):
            node = self.unstack_bt(node, world_interface, condition_parameters)

        # Conditions
        elif node.startswith('unstacked'):
            node = skills.Unstack(node, world_interface, self.object_data)
        elif node.startswith('in_gripper'):
            node = skills.InGripper(node, world_interface, node[11:])
        elif node.startswith('gripper_state'):
            node = skills.GripperState(node, world_interface, node[14:])
        elif node.startswith('object_at'):
            match_str = f'^object_at (.+) ({NUMBER_REGEX}) ({NUMBER_REGEX})' +\
                f' ({NUMBER_REGEX}) ({NUMBER_REGEX}) (.+)$'
            match = re.match(match_str, node)
            target_list = [float(i) for i in match.group(2, 3, 4)]
            node = skills.ObjectAt(
                node,
                world_interface,
                match[1],
                [round(num, 3) for num in target_list],
                match[6]
            )
        elif node.startswith('object_roughly_at'):
            match_str = f'^object_roughly_at (.+) ({NUMBER_REGEX}) ({NUMBER_REGEX})' +\
                f' ({NUMBER_REGEX}) ({NUMBER_REGEX}) (.+)$'
            match = re.match(match_str, node)
            target_list = [float(i) for i in match.group(2, 3, 4)]
            node = skills.ObjectRoughlyAt(
                node,
                world_interface,
                match[1],
                [round(num, 3) for num in target_list],
                match[6]
            )
        elif node.startswith('robot_at'):
            match_str = f'^robot_at ({NUMBER_REGEX}) ({NUMBER_REGEX}) ({NUMBER_REGEX})' +\
                f' ({NUMBER_REGEX}) (.+)$'
            match = re.match(match_str, node)
            target_list = [float(i) for i in match.group(2, 3, 4)]
            node = skills.RobotAt(
                node,
                world_interface,
                [round(num, 3) for num in target_list],
                match[6]
            )
        elif node.startswith('reachable'):
            parts = node.split()
            if len(parts) == 2:
                node = skills.Reachable(node, world_interface, target_object=parts[1])
            else:
                node = skills.Reachable(
                    node,
                    world_interface,
                    [float(parts[1]), float(parts[2]), float(parts[3])],
                    parts[4]
                )
        else:
            # get control node from the super class
            node, has_children = super().get_node(
                node, world_interface, condition_parameters)

        return node, has_children

    def get_actions(self, demonstrations: Demonstration) -> List[str]:
        """
        Get the combined actions for the robot from a demonstration.

        Args
        ----
            demonstration: the demonstration to parse.

        Returns
        -------
            actions: list of the actions in the demonstration.

        """
        actions = ['open_gripper', 'close_gripper']

        # Add approach actions dynamically to match demonstrated actions
        for demo in demonstrations:
            for action in demo:
                name = action.action_string()
                action_string = None
                if name.startswith('pick'):
                    # Approach object
                    target_object = action.actions[0].parameters[0]
                    action_string = f'approach {target_object}'
                elif name.startswith('place'):
                    # Approach position
                    action_string = f'approach {action.targets[0,0]} {action.targets[0,1]}' +\
                        f' {action.targets[0,2]} {action.frames[0]}'

                if action_string is not None and action_string not in actions:
                    actions.append(action_string)

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
        conditions = ['gripper_state open', 'gripper_state closed', 'in_gripper none']

        # Add reachable conditions dynamically to match demonstrated conditions
        for demo in demonstrations:
            for action in demo:
                name = action.action_string()
                condition_string = None
                if name.startswith('pick'):
                    # Reach object
                    target_object = action.actions[0].parameters[0]
                    condition_string = f'reachable {target_object}'
                elif name.startswith('place'):
                    # Approach position
                    condition_string = f'reachable {action.targets[0, 0]}' +\
                        f' {action.targets[0, 1]} {action.targets[0, 2]} {action.frames[0]}'

                if condition_string is not None and condition_string not in conditions:
                    conditions.append(condition_string)

        return conditions

    def compatible(
        self,
        condition1: str,
        condition2: str
    ) -> bool:
        """Return True if the conditions are compatible, False otherwise."""
        parts1 = condition1.split()
        parts2 = condition2.split()
        # The condition type is the first "word"
        type1 = parts1[0]
        type2 = parts2[0]

        # Incompatible conditions of the same type
        if type1 == type2:
            if type1 == 'in_gripper' and parts1[1:] != parts2[1:]:
                return False
            elif type1 == 'gripper_state' and parts1[1:] != parts2[1:]:
                return False
            elif type1 == 'object_at' and parts1[1] == parts2[1] and parts1[2:] != parts2[2:]:
                return False
            elif type1 == 'object_roughly_at' and \
                    parts1[1] == parts2[1] and parts1[2:] != parts2[2:]:
                return False
            elif type1 == 'robot_at' and condition1 != condition2:
                return False

        # object_at and roughly_at are incompatible
        if (type1 == 'object_roughly_at' and type2 == 'object_at' or
           type1 == 'object_at' and type2 == 'object_roughly_at') and \
           parts1[1] == parts2[1] and parts1[2:] != parts2[2:]:
            return False

        # We cannot hold something and have the gripper open at the same time
        if type1 == 'in_gripper' and \
           condition1 != 'in_gripper none' and condition2 == 'gripper_state open' or \
           type2 == 'in_gripper' and\
           condition2 != 'in_gripper none' and condition1 == 'gripper_state open':
            return False

        return True
