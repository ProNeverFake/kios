"""Definition of YuMi Actions with heuristics and pre- and post-conditions."""

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
from typing import Dict, List

from bt_learning.learning_from_demo.demonstration import EquivalentAction
import numpy as np
from robot_interface.demonstration import RobotAction


# Demonstrated actions

class PickAction(RobotAction):
    """Provides heuristics for the picking action."""

    def __init__(
        self,
        data: Dict,
        all_frames: List[str],
        default_frame: str,
        exclude_frames: List[str],
        last_action: str
    ):
        """
        Initialize the action as a Robot Action.

        Args:
        ----
            - data: dictionary specifying an action.
            - all_frames: list of all reference frames.
            - default_frame: default frame this action takes place in.
            - last_action: action performed before (to use with pre- and post-conditions).

        """
        super().__init__(data, all_frames, default_frame, last_action)
        self.exclude_frames = exclude_frames
        self.eps = 0.2

    def heuristic(self) -> None:
        """Determine what object is being picked and sets reference frame."""
        # Iterate over the frames and find the one where the target is closest to the origin.
        # This is reasonably the object we are picking.
        least_distance = np.inf
        for f in self.all_frames:
            # Ignore static frames
            if f in self.exclude_frames:
                continue

            target = self.target_position(f)
            distance = np.linalg.norm(target)
            if distance < least_distance:
                least_distance = distance
                self.frame = [f]

        # assign additional parameter to the action: what object to pick
        self.parameters = deepcopy(self.frame)

    def get_excluded_frames(self) -> List[str]:
        """Return the frames to exclude during clustering."""
        return self.exclude_frames


class PlaceAction(RobotAction):
    """
    Provides heuristic for determining what object is being placed.

    Does not determine reference frame.
    """

    def __init__(
        self,
        data: dict,
        all_frames: List[str],
        default_frame: str,
        exclude_frames: List[str],
        last_action: str
    ):
        """
        Initialize the action as a Robot Action.

        Args:
        ----
            - data: dictionary specifying an action.
            - all_frames: list of all reference frames.
            - default_frame: default frame this action takes place in.
            - last_action: action performed before (to use with pre- and post-conditions).

        """
        super().__init__(data, all_frames, default_frame, last_action)
        self.exclude_frames = exclude_frames
        self.eps = 0.2

        # Check if it is an actual place or drop
        if self.type == 'drop':
            self.place_type = 'drop'
        else:
            self.place_type = 'place'

    def heuristic(self) -> None:
        """Determine what object is being placed."""
        if self.last_action is not None and len(self.last_action.parameters) > 0:
            # assign additional parameter to the action: what object to place (the last object picked)
            self.parameters = deepcopy(self.last_action.parameters)

    def get_excluded_frames(self) -> List[str]:
        """Return the frames to exclude during clustering."""
        return self.parameters + self.exclude_frames


# Equivalent actions
class EquivalentPick(EquivalentAction):

    def preconditions(self) -> List[str]:
        """Return action's pre-conditions."""
        return ['gripper_state open', f'reachable {self.actions[0].parameters[0]}']

    def postconditions(self) -> List[str]:
        """Return action's post-conditions."""
        return ['gripper_state closed', f'in_gripper {self.actions[0].parameters[0]}']

    def action_string(self) -> str:
        """Return action's name."""
        return f'{self.name} {self.actions[0].parameters[0]}'


class EquivalentPlace(EquivalentAction):

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)

        self.object = self.actions[0].parameters[0]
        self.target = self.targets[0, :].reshape((3,))

        # If this cluster contains at least one drop, everything is drop
        self.is_drop = False
        for action in self.actions:
            if action.place_type == 'drop':
                self.is_drop = True
                break

    def preconditions(self) -> List[str]:
        """Return action's pre-conditions."""
        return [f'in_gripper {self.object}']

    def postconditions(self) -> List[str]:
        """Return action's post-conditions."""
        conditions = ['gripper_state open', 'in_gripper none']
        if self.is_drop:
            conditions.append(
                f'object_roughly_at {self.object} {self.target[0]} {self.target[1]}' +
                f' {self.target[2]} {self.max_distance[0]} {self.actions[0].frame[0]}')
        else:
            conditions.append(
                f'object_roughly_at {self.object} {self.target[0]} {self.target[1]}' +
                f' {self.target[2]} {self.max_distance[0]} {self.actions[0].frame[0]}')

        return conditions

    def action_string(self) -> str:
        """Return action's name."""
        return (f'{self.name} {self.object} {self.target[0]} {self.target[1]} {self.target[2]}' +
                f' {self.max_distance[0]} {self.actions[0].frame[0]}')
