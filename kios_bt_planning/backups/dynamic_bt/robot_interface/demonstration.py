"""Realization of Demonstration and Action from Robot perspective."""

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
import glob
import itertools
import os
from typing import List

from ..bt_learning.learning_from_demo.demonstration import Action, Demonstration
import numpy as np
import yaml


class RobotDemonstration(Demonstration):
    """This class represents a collection of demonstrations as recorded from the robot."""

    def __init__(self, demo: str, custom_actions: dict = {}, exclude_frames: dict = {}):
        """
        Construct a set of demonstrations from the directory pointed to by demo.

        Args:
        ----
            demo: path to the folder where the demonstration is stored.
            custom_actions: dictionary with string-class pairs indicating cusom classes,
                that can be used for different action types.
            exclude_frames: fixed frames to be excluded.

        Note: All classes must be a subclass of RobotAction.
                frames: list of all reference frames.
                default_frame: frame that all actions are assigned to by default.

        """
        super().__init__(RobotAction)

        for action_type, cls in custom_actions.items():
            if not issubclass(cls, RobotAction):
                raise ValueError(
                    f'Class "{cls}" of action "{action_type}" should be a subclass of RobotAction.'
                )
            self.register_action(action_type, cls)

        # Folder with demonstrations must exist
        if not os.path.isdir(demo):
            raise FileNotFoundError('Folder "' + demo + '" does not exist.')
        # Demonstrations folder has to contain information file
        if not os.path.isfile(demo + '/info.yaml'):
            raise FileNotFoundError('Folder "' + demo + '/info.yaml" does not exist.')
        # There has to be at least one demonstration sequence
        if not os.path.isdir(demo + '/demo1'):
            raise FileNotFoundError(
                'Folder "' + demo + '" must contain at least one demonstration.')

        # Read information about the demonstratoins
        with open(demo + '/info.yaml') as f:
            info = yaml.safe_load(f)
            self.frames = info['frames']
            self.default_frame = info['default_frame']

        # self.__demonstrations contains a list of actions for each demonstration
        self.__demonstrations = []
        for folder in glob.glob(demo + '/demo[0-9]*/'):
            # Each demonstration has to contain at least one action
            if not os.path.isfile(folder + '/data_1.yaml'):
                raise FileNotFoundError(
                    'Demonstration "' + folder + '/" must contain at least one action.')
            actions = []
            # Read actions in numerical order
            n_actions = len(glob.glob(folder + '/*.yaml'))
            for action_number in range(1, n_actions + 1):
                file = f'{folder}/data_{action_number}.yaml'
                with open(file) as f:
                    data = yaml.safe_load(f)
                    if len(actions) == 0:
                        last_action = None
                    else:
                        last_action = actions[-1]
                    action = self.make_action(
                        data['type'],
                        data,
                        info['frames'],
                        self.default_frame,
                        exclude_frames[data['type']] if exclude_frames != {} else [],
                        last_action
                    )
                    action.heuristic()
                    actions.append(action)
            self.__demonstrations.append(actions)

        with open(demo + '/exclude_frames.yaml', 'w') as f:
            yaml.dump(exclude_frames, f)

    def demonstrations(self) -> List[str]:
        """Get the list of the actions performed in the demosntration."""
        return self.__demonstrations


class RobotAction(Action):
    """Class that represents a single action as demonstrated."""

    def __init__(
        self,
        data: dict,
        all_frames: List[str],
        default_frame: str,
        last_action: str = None,
        n_targets: int = 1
    ):
        """
        Initialize a robot Action.

        Action data is specified according to the documentation.

        Args:
        ----
            - data: dictionary specifying an action.
            - all_frames: list of all reference frames.
            - default_frame: default frame this action takes place in.
            - last_action: action performed before (to use with pre- and post-conditions).
            - n_targets: number of targets this action has.

        """
        super().__init__(
            data['type'],
            list(itertools.repeat(default_frame, n_targets))
        )

        self.all_frames = all_frames
        self.position = {}
        self.orientation = {}
        self.last_action = last_action
        for frame in data['vec_pos']:
            self.position[frame] = np.array(data['vec_pos'][frame]).reshape((-1, 3))
        for frame in data['vec_quat']:
            self.orientation[frame] = np.array(data['vec_quat'][frame]).reshape((-1, 4))

    def target_position(self, frame: str, i: int = 0) -> np.ndarray:
        """
        Return the target position i of this action.

        Raises a KeyError if no position exists in frame.
        This class assumes there is only one target (i=0 is the first target).
        If an action has multiple targets, a subclass must be written.

        Args
        ----
            frame: frame with respect to the position is defined.
            i: index of the target.

        Returns
        -------
            NumPy array with the position values [x, y, z].

        """
        if i != 0:
            raise ValueError(f'Action "{self.type}" has no target "{i}."')

        return self.position[frame][-1, :].reshape((3,))

    def target_orientation(self, frame: str, i: int = 0) -> np.ndarray:
        """
        Return the target orientation of this action.

        Raises a KeyError if no orientation exists in frame.
        This class assumes there is only one target.
        If an action has multiple targets, a subclass must be written.

        Args
        ----
            frame: frame with respect to the orientation is defined.
            i: index of the target.

        Returns
        -------
            NumPy array with the orientation values [x, y, z, w]

        """
        if i != 0:
            raise ValueError(f'Action "{self.type}" has no target "{i}."')
        return self.orientation[frame][-1]
