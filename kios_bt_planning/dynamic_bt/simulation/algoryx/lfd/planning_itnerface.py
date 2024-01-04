"""
Offline Interface for execution of behaviors.

Used by the planner in LfD framework to expand the BT.
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

import math
import random
from typing import Dict, List, Tuple

import numpy as np


class PlanningInterface():
    """An interface for the planner to execute robot behaviors."""

    # Limits for position initialization
    WORLD_LIMITS_XY = (-5, 5)
    WORLD_LIMITS_Z = (0, 1)
    # Probability of a random event
    RONDOM_EVENT_PROB = 0.3

    def __init__(
        self,
        available_objects: List[str],
        frames: List[str],
        default_frame: str,
        random_events: bool = True,
        init_state: Dict = {},
        robot_frame: str = 'base'
    ):
        """
        Instantiate a new PlanningInterface with random initial conditions.

        Args:
        ----
            - available_objects: list of the objects that can be manipulated.
            - frames: list of reference frames.
            - default_frame: used frame if nothing else is specified.
            - random_events: if True, random disturbances might occur during execution.
            - init_state: state of the robot.
            - robot_frame: name of the end effector frame of the robot.

        """
        self.default_frame = default_frame
        self.all_frames = frames
        if robot_frame not in self.all_frames:
            self.all_frames.append(robot_frame)
        self.objects = available_objects
        self.random_events = random_events
        self.init_state = init_state
        self.robot_frame = robot_frame

        # Dictionary of frames and their position in default_frame
        self.frames = {}

        # Set initial conditions
        self.reset()

    def pick(self, item: str):
        """Simulate a picking task."""
        self.gripper = 'closed'
        self.holding = item
        self.__random_event()

    def place(
        self,
        target: str,
        reference: str,
        position: np.ndarray
    ):
        """
        Simulate a picking task.

        Args
        ----
            - target: target object.
            - reference: reference frame for the object.
            - pose: position of the target object.

        """
        if reference == self.default_frame:
            self.frames[self.holding] = position
        else:
            # Transform frame
            self.frames[self.holding] = position + self.frames[reference]

        self.gripper = 'open'
        self.holding = ''

        self.__random_event()

    def grasped(self, held_object: str) -> bool:
        """Return true if the object is held by the robot."""
        return self.holding == held_object

    def is_gripper_state(self, state: str) -> bool:
        """Return true if gripper is open."""
        return False

    def empty_gripper(self) -> bool:
        """Return true if gripper is open."""
        return self.holding == ''

    def get_item_in_frame(
        self,
        target_object: str,
        frame: str
    ) -> Tuple[np.ndarray, np.ndarray]:
        """Return the pose of object in reference frame."""
        orientation = np.array([0, 0, 0, 1])
        if frame == self.default_frame:
            position = self.frames[target_object]
        else:
            position = self.frames[target_object] - self.frames[frame]
        return position, orientation

    def at_pose(
        self,
        target: str,
        reference: str,
        pose: np.ndarray,
        tolerance: float,
        rough: bool = False
    ) -> Tuple[np.ndarray, np.ndarray]:
        """Return the pose of object in reference frame."""
        position, _ = self.get_item_in_frame(target, reference)

        diff = np.abs(position[:2] - pose[:2]) if rough else np.abs(position - pose)
        distance = math.sqrt(sum([x*x for x in diff]))

        return float(distance) < float(tolerance)

    def set_gripper(self, state: str):
        """Set the gripper state."""
        self.gripper = state
        if state == 'open' and self.holding != '':
            # The robot is holding something that will be dropped at
            # a random location
            self.frames[self.holding] = self.__random_location()
            self.holding = ''
        self.__random_event()

    def navigate(self, goal: np.ndarray, ref_frame: str):
        """Simulate a move action for the mobile manipulator."""
        # TODO: make it not dependent on Action
        if ref_frame == self.default_frame:
            self.frames[self.robot_frame] = goal
        else:
            # Transform frame
            self.frames[self.robot_frame] = goal + self.frames[ref_frame]

        self.__random_event()

    def robot_position(self, frame: str) -> np.ndarray:
        """Return the position of the robot End Effector."""
        if frame == self.default_frame:
            return self.frames[self.robot_frame]
        else:
            return self.frames[self.robot_frame] - self.frames[frame]

    def approach(
        self,
        target: str,
        frame: str
    ):
        """Move closer to target so it is reachable with the gripper."""
        target = target.copy()
        if frame != self.default_frame:
            target += self.frames[frame]

        # Sample a point uniformely from the 1.5m radius disk centered at target
        r = np.random.uniform(low=0, high=1.5, size=(1,)).item()
        theta = np.random.uniform(low=0, high=np.pi, size=(1,)).item()
        self.frames[self.robot_frame] = np.array(
            [r*np.cos(theta) + target[0], r*np.sin(theta) + target[1], 0.1]
        )

        self.__random_event()

    def reachable(
        self,
        target: str,
        frame: str = None
    ) -> bool:
        """Target is reachable if it is within 1.5m of the robot. Orientation is not considered."""
        if isinstance(target, str):
            target = self.frames[target]
        else:
            target = target.copy()
            if frame != self.default_frame:
                target += self.frames[frame]

        return np.linalg.norm(target[0:2] - self.frames[self.robot_frame][0:2]) <= 1.5

    def reset(self):
        """Reset the internal state to a random initial state."""
        for f in self.all_frames:
            if f == self.default_frame:
                self.frames[f] = np.zeros((3,))
            elif f == self.robot_frame:
                # EE frame cannot have arbitrary z component
                position = self.__random_location()
                position[2] = 0.1
                self.frames[f] = position
            else:
                self.frames[f] = self.__random_location()

        for f in self.init_state:
            self.frames[f] = self.init_state[f]

        if 'holding' in self.init_state:
            self.holding = self.init_state['holding']
        else:
            self.holding = random.choice(self.objects + [''])

        if self.holding != '':
            self.gripper = 'closed'
        else:
            self.gripper = random.choice(['open', 'closed'])

    def __random_location(self) -> np.ndarray:
        """Generate a random position."""
        return np.random.uniform(
            (self.WORLD_LIMITS_XY[0], self.WORLD_LIMITS_XY[0], self.WORLD_LIMITS_Z[0]),
            (self.WORLD_LIMITS_XY[1], self.WORLD_LIMITS_XY[1], self.WORLD_LIMITS_Z[1]),
            (3,)
        )

    def __random_event(self):
        """
        Produce a random event with probability RANDOM_EVENT_PROB.

        An example is dropping what is being held.
        """
        if not self.random_events:
            return

        if np.random.rand(1).item() > self.RONDOM_EVENT_PROB:
            # No random event
            return

        # Possible random events are change gripper state and randomly move an object
        events = ['gripper', 'displace']
        if self.holding != '':
            # If the robot is holding something it can also drop it
            events.append('drop')

        event = random.choice(events)

        if event == 'gripper':
            self.gripper = 'closed' if self.gripper == 'open' else 'open'
            # If we were holding something when the gripper opened we drop it at a random location
            if self.gripper == 'open' and self.holding != '':
                self.frames[self.holding] = self.__random_location()
                self.holding = ''
        elif event == 'displace':
            target_object = random.choice(self.objects)
            if target_object == self.holding:
                # If the robot is holding the robot it means it has been dropped
                # without opening the gripper
                self.holding == ''
            self.frames[target_object] = self.__random_location()
        elif event == 'drop':
            # Drop without opening gripper
            self.frames[self.holding] = self.__random_location()
            self.holding = ''
