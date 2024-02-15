"""This module contains functions for clustering actions into sets of equivalent actions."""

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

import itertools
import logging
from typing import Any, Dict, List, Tuple, Type

from bt_learning.learning_from_demo.demonstration import Action, Demonstration, EquivalentAction
import numpy as np
from sklearn.cluster import DBSCAN


logger = logging.getLogger('clustering')


class __Cluster:
    """Represent a cluster with its score and the actions that belong to it."""

    def __init__(
        self,
        actions: List['__ClusterElement'],
        target_index: int,
        frame: str
    ):
        """
        Initialize the cluster object.

        Args:
        ----
            - actions: current executed action.
            - target_index: index of the current action target.
            - frame: frame representing the pose of the target.

        """
        self.actions = actions
        self.target_index = target_index
        self.frame = frame
        self.score = cluster_score(actions, target_index, frame)

    def __repr__(self) -> str:
        return f'Cluster of {self.actions[0].action.type}({self.actions[0].action.parameters})' +\
            f' with score {self.score} in frame {self.frame} (size: {len(self.actions)}).'


class __ClusterElement:
    """Represents an element in a cluster."""

    def __init__(self, action: Action):
        self.__belonging_clusters = []
        self.action = action

    def add_cluster(self, cluster: Type['__Cluster']):
        added = False
        for i, c in enumerate(self.__belonging_clusters):
            if cluster.score > c.score:
                self.__belonging_clusters.insert(i, cluster)
                added = True
                break
        if not added:
            self.__belonging_clusters.append(cluster)

    @property
    def clusters(self) -> Type['__Cluster']:
        return self.__belonging_clusters


def belongs_to_initial_group(
    action: Action,
    action_type: str,
    parameters: List[Any],
    n_targets: int,
    frame: str
) -> bool:
    """Return True if the properties of action match action_type, parameters, and n_targets."""
    return action.type == action_type and action.parameters == parameters and\
        action.n_targets == n_targets and action.frame == frame


def initial_partition(actions: List[Action]) -> List[List[Action]]:
    """
    Compute an initial partitioning of the demonstrated actions.

    It groups actions with equal type, parameter, and number of targets.
    Returns a list of lists of Actions.
    """
    actions = actions
    partitioning = []

    while len(actions) > 0:
        # At this stage all actions are initialized with a default frame except
        # those that have a heuristic. Therefore we can also group actions by
        # frame to separate actions with heuristic into groups with equal frame.
        action_type = actions[0].type
        parameters = actions[0].parameters
        n_targets = actions[0].n_targets
        frame = actions[0].frame

        group = list(filter(
            lambda x: belongs_to_initial_group(x, action_type, parameters, n_targets, frame),
            actions
        ))
        partitioning.append(group)

        # Remove these elements
        actions = list(filter(
            lambda x: not belongs_to_initial_group(x, action_type, parameters, n_targets, frame),
            actions
        ))

    return partitioning


def get_targets(
    actions: List[__ClusterElement],
    target_index: int,
    frame: str
) -> np.ndarray:
    """
    Get the targets of the given actions.

    Args
    ----
        - actions: list of the actions.
        - target_index: index of the target of the actions.
        - frame: frame in which the target is defined.

    Returns
    -------
        - Nx3 numpy array of targets in actions where N = len(actions).

    """
    targets = np.zeros((len(actions), 3))
    for i, action in enumerate(actions):
        targets[i, :] = action.action.target_position(frame, target_index)

    return targets


def group_of(
    action: Action,
    partitioning: List[List[Action]]
) -> int:
    """Get the index of the group the action belongs to in partitioning."""
    for i, group in enumerate(partitioning):
        if action in group:
            return i

    return None


def intersection(*lists) -> Tuple[Any]:
    """Compute the intersection of n lists."""
    if len(lists) == 1:
        # The intersection of a single list is the list itself
        return lists[0]
    else:
        # Compute the intersection recursively
        return [x for x in lists[0] if x in intersection(*lists[1:])]


def cluster_score(
    cluster: Type['__Cluster'],
    target_idx: int,
    frame: str
) -> float:
    """
    Compute cluster score as size of cluster / the distance to the mean.

    Args:
    ----
        - cluster: the cluster object to compute the score of.
        - target_idx: the target to consider to compute the score.
        - frame: reference frame representing the pose of the target.

    """
    if len(cluster) == 1:
        return -np.inf

    targets = get_targets(cluster, target_idx, frame)
    center = np.mean(targets, axis=0)
    distances = np.linalg.norm(targets - center, axis=1)
    max_dist = np.max(distances)
    max_dist = 0.001 if max_dist == 0.0 else max_dist
    return len(cluster)/max_dist


def cluster(
    actions_list: List[Action],
    frames: List[str],
    target_index: int,
    n_demos: int
) -> List[EquivalentAction]:
    """
    Find equivalent actions among actions_list and assigns a reference frame from frames.

    All actions in the list actions_list whould have equal type, parameters, and n_targets.

    Args:
    ----
        - actions_list: the list of actions to consider in the cluster.
        - frames: all possible frames in which the action is executed.
        - target_index: index of the target of the action.
        - n_demos: number of demonstrations performed.

    """
    actions = [__ClusterElement(action) for action in actions_list]
    if actions[0].action.has_heuristic:
        # Reference frame already inferred. No need to do clustering.
        # All actions are equivalent.
        return [actions_list]

    clusters = []
    # Assign all actions to their own cluster initially
    for action in actions:
        new_cluster = __Cluster([action], target_index, action.action.frame[target_index])
        action.add_cluster(new_cluster)
        clusters.append(new_cluster)

    for frame in frames:
        # logger.debug(f'Running for frame: {frame}')
        to_exclude = actions[0].action.get_excluded_frames()
        # logger.debug(f'Excluding: {to_exclude}')
        allowed_actions = list(filter(lambda x: frame not in to_exclude, actions))
        # logger.debug(f'Allowed: {[x.action.type for x in allowed_actions]}')
        if len(allowed_actions) == 0:
            continue
        targets = get_targets(allowed_actions, target_index, frame)
        # set cluster in samples to be the majority consensus if we have more than 3 demos
        cluster_samples = 3 if n_demos <= 3 else (n_demos//2 + 1)
        eps_ = actions_list[0].eps
        labels = DBSCAN(eps=eps_, min_samples=cluster_samples).fit_predict(targets)

        for cluster in np.unique(labels):
            if cluster == -1:
                # Noise cluster
                continue

            actions_in_cluster = []
            for i in np.where(cluster == labels)[0]:
                actions_in_cluster.append(allowed_actions[i])
            clusters.append(__Cluster(actions_in_cluster, target_index, frame))
            logger.debug('Found: %s', clusters[-1])
            for a in actions_in_cluster:
                a.add_cluster(clusters[-1])

    equivalent = []
    for cluster in clusters:
        # All actions that have this cluster as their preferred cluster
        actions_in_cluster = []
        for i in range(len(actions)-1, -1, -1):
            if actions[i].clusters[0] is cluster:
                actions_in_cluster.append(actions.pop(i))
                actions_in_cluster[-1].action.frame[target_index] = cluster.frame
        if len(actions_in_cluster) == 0:
            continue

        # List of actions that don't have an alternative cluster and are
        # thus equivalent
        base_eq = []
        for i in range(len(actions_in_cluster)-1, -1, -1):
            if len(actions_in_cluster[i].clusters) == 1 or\
                    actions_in_cluster[i].clusters[1].score < 20.0*n_demos:
                base_eq.append(actions_in_cluster.pop(i).action)
        eq_actions = []
        if len(base_eq) > 0:
            eq_actions.append(base_eq)

        # Group rest of actions according to second best cluster
        msg = str(cluster) + ' was divided into:'
        while len(actions_in_cluster) > 0:
            cluster_2 = actions_in_cluster[0].clusters[1]
            msg += '\n\t' + str(cluster_2)
            eq_2 = []
            for i in range(len(actions_in_cluster)-1, -1, -1):
                if actions_in_cluster[i].clusters[1] is cluster_2:
                    eq_2.append(actions_in_cluster.pop(i).action)
            eq_actions.append(eq_2)

        if len(eq_actions) > 1:
            logger.debug(msg)
        elif len(eq_actions) == 1:
            logger.debug('Keeping %s', cluster)

        if len(eq_actions) > 0:
            equivalent += eq_actions

    return equivalent


def partition(
    actions: List[Action],
    frames: List[str],
    n_demos: int
) -> List[List[Action]]:
    """
    Partition the list actions into a list of lists of Actions.

    All actions in the same group are equivalent.
    The reference frame is inferred and set for all actions.

    Args:
    ----
        - actions: all actions performed in the demonstration.
        - frames: all possible frames in which the action is executed.
        - n_demos: number of demonstrations performed.

    """
    # Validate actions
    for action in actions:
        if type(action.frame) != list:
            raise ValueError(
                f'action.frame of action "{action.type}" is of type "{type(action.frame)}"\
                and not list.')
        if len(action.frame) != action.n_targets:
            raise ValueError(
                f'Length of action.frame ({len(action.frame)}) does not match action.n_targets\
                ({action.n_targets}).')

    initial = initial_partition(actions)
    partitioning = []

    for group in initial:
        equivalent = []
        for i in range(group[0].n_targets):
            equivalent.append(cluster(group, frames, i, n_demos))

        for factor in itertools.product(*equivalent):
            partition = intersection(*factor)
            if len(partition) > 0:
                partitioning.append(partition)

    return partitioning


def find_equivalent_actions(
    demonstration: Type[Demonstration],
    action_defs: Dict
) -> List[Demonstration]:
    """
    Find and group equivalent actions across demonstrations in demonstration.

    All classes must be subclasses of EquivalentAction.
    All occurances of an action are the same EquivalentAction instance
    and can be compared with 'is'.

    Args
    ----
        - demonstration: is an instance of Demonstration.
        - action_defs: is a dictionary of string-class pairs indicating
                       what classes to use to represent each action.

    Returns
    -------
        List of abstracted demos where each abstracted demo is a list of EquivalentAction.

    """
    partitioned = partition(
        demonstration.all_actions(), demonstration.frames, len(demonstration.demonstrations()))
    actions = []
    for group in partitioned:
        if group[0].type not in action_defs:
            raise ValueError(
                f'Action type "{group[0].type}" is not present in action_defs.')

        action_cls = action_defs[group[0].type]
        if not issubclass(action_cls, EquivalentAction):
            raise ValueError(
                f'Action class for "{group[0].type}" is not a subclass of EquivalentAction.')

        actions.append(action_cls(group))

    demos = []
    for demo in demonstration.demonstrations():
        current_demo = []
        for action in demo:
            # Find the index of the group of equivalent actions that this action belongs to
            try:
                group_idx = next(i for i, group in enumerate(partitioned) if action in group)
            except StopIteration:
                raise RuntimeError(
                    f'Action "{action}" was not found in any of the demonstrations')

            current_demo.append(actions[group_idx])

        demos.append(current_demo)

    return demos
