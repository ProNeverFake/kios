"""Definition of Mobile Base Actions with heuristics and pre- and post-conditions."""

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
from typing import List

# TODO: make this a mobileyumi actions library and import from yumi
from ...bt_learning.learning_from_demo.demonstration import EquivalentAction
from ...robot_interface.demonstration import RobotAction


# Demonstrated Actions


class MoveAction(RobotAction):
    """Communicates that the base frame is to be ignored for move actions."""

    def __init__(
        self,
        data: dict,
        all_frames: List[str],
        default_frame: str,
        exclude_frames: List[str],
        last_action: str,
    ):
        super().__init__(data, all_frames, default_frame, last_action)
        self.exclude_frames = exclude_frames
        self.eps = 2.0

    def heuristic(self) -> None:
        """Determine what object is being carried."""
        if self.last_action is not None and len(self.last_action.parameters) > 0:
            # Assign additional parameter to the action: the carried obj (the last picked obj)
            self.parameters = deepcopy(self.last_action.parameters)

    def get_excluded_frames(self) -> List[str]:
        """Return the frames to exclude during clustering."""
        excluded = self.exclude_frames
        for param in self.parameters:
            excluded.append(param) if param not in excluded else excluded
        return excluded


# Equivalent actions


class EquivalentMove(EquivalentAction):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)

        self.target = self.targets[0, :].reshape((3,))

    def preconditions(self) -> List[str]:
        """Return action's pre-conditions."""
        return []

    def postconditions(self) -> List[str]:
        """Return action's post-conditions."""
        return [
            str(
                f"robot_at {self.target[0]} {self.target[1]} {self.target[2]}"
                + f" {self.max_distance[0]} {self.actions[0].frame[0]}"
            )
        ]

    def action_string(self) -> str:
        """Return action's name."""
        return (
            f"{self.name} {self.target[0]} {self.target[1]} {self.target[2]}"
            + f" {self.max_distance[0]} {self.actions[0].frame[0]}"
        )
