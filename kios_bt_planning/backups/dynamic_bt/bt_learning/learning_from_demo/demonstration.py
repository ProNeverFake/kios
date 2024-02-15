"""
Abstract class definig important aspects of the demonstration.

- the action description
- the equivalent actions handling actions pre- and post- conditions.
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

from abc import ABC, abstractmethod
from dataclasses import dataclass
import pickle
from typing import Any, List

import numpy as np


class Demonstration(ABC):
    """
    The Demonstration class represents demonstrations on an abstract level.

    A subclass has to be made that can read demonstrations.
    """

    def __init__(self, default_action_type: 'Action'):
        """
        Initialize a demonstration object.

        Args:
        ----
            default_action_type: class of actions that do not have a specific registered type.

        """
        if not issubclass(default_action_type, Action):
            raise ValueError('default_action_type has to be a subclass of Action')
        self.__default_action_type = default_action_type

        # Dictionary of action types with a custom action class
        self.__registered_classes = {}

    def register_action(self, action_type: Any, action_class: Any):
        """Register actions of type "action_type" to be instances of action_class."""
        self.__registered_classes[action_type] = action_class

    def make_action(self, action_type: str, *args, **kwargs) -> 'Action':
        """
        Create and return an instance of the class corresponding to action type.

        The constructor takes *args and **kwargs.
        """
        action_class = self.__registered_classes.get(action_type, self.__default_action_type)
        return action_class(*args, **kwargs)

    @abstractmethod
    def demonstrations(self):
        """Return a list of demos where each demo is in turn a list of actions."""
        pass

    def n_demonstrations(self) -> int:
        """Return the number of demonstrations."""
        return len(self.demonstrations())

    def all_actions(self) -> List[str]:
        """Return a list of all actions without reference to the demonstration they belong to."""
        actions = []
        for demo in self.demonstrations():
            actions += demo

        return actions


class Action(ABC):
    """
    Represent a single action as demonstrated.

    The Action class does not represent a collection of equivalent actions.
    """

    def __init__(self, action_type: str, default_frame: List[str]):
        """
        Instantiate a new Action.

        This constructor has to be called byt the subclass constructor.

        Args:
        ----
            - action_type: action type.
            - default_frame: is a list of default frames (strings, one for each target).

        Important attributes to override:
        --------------------------------
            - _parameters: list of action-specific parameters. Defaults to [].
            - _exclude_frames: list of frames to exclude when inferring similar actions.
                               Defaults to [].
            - _n_targets: the number of targets. Defaults to len(default_frame).
            - _has_heuristic: whether this action can determine reference frames with a heuristic.
                              Defaults to False.

        Other attributes:
        ----------------
            - type: string with the action type.
            - frame: list of frames (strings). One for each target.

        """
        self.type = action_type
        self.frame = default_frame
        if not hasattr(self, 'parameters'):
            self.parameters = []
        if not hasattr(self, 'exclude_frames'):
            self.exclude_frames = []
        if not hasattr(self, 'n_targets'):
            self.n_targets = len(default_frame)
        if not hasattr(self, 'has_heuristic'):
            self.has_heuristic = False

    @abstractmethod
    def target_position(self, frame: str, i: int) -> np.ndarray:
        """Return the target position in the given frame."""
        pass

    @abstractmethod
    def target_orientation(self, frame: str, i: int) -> np.ndarray:
        """Return the target orientation in the given frame."""
        pass

    def heuristic(self) -> None:
        """
        Override this in a subclass to infer properties about the action using heuristics.

        This function is typically where paramters are set.
        If has_heuristic is True, it is assumed that this function infers reference frame
        and all actions of this type are assumed equivalent if the parameters and frame are equal.
        Even though has_heuristic is False this function can be overriden to set parameters.
        """
        pass

    def get_excluded_frames(self) -> List[str]:
        """Override this in a subclass to personalize the frames to exclude during clustering."""
        pass

    def __eq__(self, other) -> bool:
        """Return whether this Action is equal to another (gvein as input parameter)."""
        try:
            np.testing.assert_equal(self.type, other.type)
            np.testing.assert_equal(self.parameters, other.parameters)
            np.testing.assert_equal(self.frame, other.frame)
            for target, frame in enumerate(self.frame):
                np.testing.assert_equal(
                    self.target_position(frame, target),
                    other.target_position(frame, target)
                )
                np.testing.assert_equal(
                    self.target_orientation(frame, target),
                    other.target_orientation(frame, target)
                )
            return True
        except AssertionError:
            return False

    def __neq__(self, other) -> bool:
        return not self.__eq__(other)


class EquivalentAction(ABC):
    """
    Represent an equivalence class of actions as determined by the clustering.

    Attributes
    ----------
        - targets: [n_targets x 3] array with mean targets.
                   It is possible to have multiple targets (NOT USED AT THE MOMENT)
        - target_orientations: [n_targets] quaternion array of mean target orientations.
        - max_distance: [n_targets] array with the maximal euclidean distance to targets observed.
        - frames: list of reference frames for each target.
        - actions: list of Action that are considered equivalent.
        - additional_preconditions: list of Condition that represent additional preconditions.

    """

    # List of used action names to give each action a unique name
    __names = {}

    def __init__(self, actions: List[Action]):
        """
        Construct a new instance of EquivalentAction.

        Args:
        ----
            - actions: list of Action containing equivalent actions.

        """
        self.actions = actions
        self.additional_preconditions = []

        if actions[0].type in self.__names:
            EquivalentAction.__names[actions[0].type] += 1
        else:
            EquivalentAction.__names[actions[0].type] = 0
        self.name = actions[0].type + str(EquivalentAction.__names[actions[0].type])

        # Don't do any more calculations if there are no targets
        if actions[0].n_targets == 0:
            self.targets = None
            self.target_orientations = None
            self.max_distance = None
            self.frames = None
            return

        self.frames = actions[0].frame

        # Combine all targets into a n_actions x n_targets x 3 (or 2) array of target positions
        # and a n_actions x n_targets array of target orientations
        all_targets = np.zeros((len(actions), actions[0].n_targets, 3))
        all_orientations = np.zeros((len(actions), actions[0].n_targets), dtype=np.ndarray)
        for action_idx, action in enumerate(actions):
            for target in range(actions[0].n_targets):
                all_targets[action_idx, target, :] =\
                    action.target_position(self.frames[target], target)
                all_orientations[action_idx, target] =\
                    action.target_orientation(self.frames[target], target)

        # Compute mean position
        # Shape: n_targets x 3
        self.targets = np.mean(all_targets, axis=0)
        self.targets = np.round(self.targets, 3)
        # Shape: n_actions x n_targets
        distances = np.linalg.norm(all_targets - self.targets, axis=2)
        # max distance of a demonstrated action to the mean
        # TODO: (use to compute action tolerance) --> NOT USED (this was called spread!)
        # Shape: n_targets
        self.max_distance = np.max(distances, axis=0)
        self.max_distance = np.round(self.max_distance, 3)

        # Compute mean orientation
        # TODO: now orientation is not used to determine the target
        self.target_orientations = all_orientations[0]

    @abstractmethod
    def preconditions(self) -> List[str]:
        """Return a list of strings corresponding to pre-conditions."""
        pass

    def preconditions_with_additional(self) -> List[str]:
        """
        Return a list of strings with this action's pre-conditions.

        It includes those specified in additional_preconditions
        which are present in the beginning of the returned list.
        """
        return self.preconditions() + self.additional_preconditions

    @abstractmethod
    def postconditions(self) -> List[str]:
        """Return a list of strings corresponding to post-conditions."""
        pass

    @abstractmethod
    def action_string(self) -> str:
        """Return a string representing this action for use with the planner."""
        pass

    def save_actions(self, directory: str):
        """Save the list of demonstrated actions as a pickle file in the given directory."""
        with open(directory + '/' + self.name + '.pkl', 'wb') as f:
            pickle.dump(
                ActionInfo(
                    actions=self.actions,
                    additional_preconditions=self.additional_preconditions,
                    equivalent_action=self),
                f
            )

    def can_form_constraint(self) -> bool:
        """Return True if this action can be a part of a constraint."""
        return True

    def __repr__(self) -> str:
        return self.action_string()


@dataclass
class ActionInfo:
    """
    Information about an EquivalentAction.

    Attributes
    ----------
        - actions: is the list of lfd.demonstration.Action that makes up the EquivalentAction.
        - additional_preconditions: is a list of additional preconditions.
        - equivalent_action: is the EquivalentAction.

    """

    actions: List[Action]
    additional_preconditions: list
    equivalent_action: EquivalentAction
