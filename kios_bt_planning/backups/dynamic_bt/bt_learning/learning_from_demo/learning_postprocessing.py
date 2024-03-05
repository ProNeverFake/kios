"""Methods that allow to modify a learnt tree."""

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

from copy import copy, deepcopy
import logging
import math
import os
from typing import Any, List, Tuple

from behaviors.behavior_lists import BehaviorLists
from bt_learning.learning_from_demo.demonstration import Demonstration
import numpy.random as prob
import random
import yaml

# Notation [(Xmin, Xmax), (Ymin, Ymax), (Zmin, Zmax)]
WORKSPACE = [(0.1, 0.5), (-0.5, 0.5), (0.002, 0.4)]


logger = logging.getLogger('learning_postprocessing')


def update_bt_settings(
    file_path: str,
    demonstrations: Demonstration,
    behaviors: BehaviorLists
) -> None:
    """Update the BT settings with the added behaviors."""
    file_name = file_path + '/BT_SETTINGS.yaml'
    with open(file_name) as f:
        data = yaml.safe_load(f)

    data_actions = data['action_nodes']
    data_conditions = data['condition_nodes']

    actions = behaviors.get_actions(demonstrations)
    conditions = behaviors.get_conditions(demonstrations)

    for action in actions:
        if action not in data_actions:
            data_actions.append(action)
    for condition in conditions:
        if condition not in data_conditions:
            data_conditions.append(condition)

    settings = {
        'fallback_nodes': ['f('],
        'sequence_nodes': ['s('],
        'condition_nodes': data_conditions,
        'action_nodes': data_actions,
        'up_node': [')']
    }

    with open(file_name, 'w') as f:
        yaml.dump(settings, f)


def add_synthetic_demos(
    file_path: str,
    action_description: dict = {'grasp': [], 'place': [], 'move': []},
    target_demo: int = 1,
    samples: int = 2,
    dialogue_menu: Any = None,
    workspace: List[Tuple[float, float]] = WORKSPACE
) -> None:
    """Load an existing demonstration to generate 2 synthetic demonstrations."""
    demo_to_replicate = os.path.join(file_path, 'demo%d' % (target_demo))

    synthetic_data = []
    # this frame is demonstration specific: reset only at the beginning of generation method
    pick_target_frame = None

    # ------ Load frames to exclude ------
    file_name = os.path.join(file_path, 'exclude_frames.yaml')
    with open(file_name) as f:
        excluded_frames = yaml.safe_load(f)

    # ------ Load demo ------
    # Count how many data files are there --> number of actions performed in a single demo
    n_files = len([name for name in os.listdir(demo_to_replicate)])
    for i in range(n_files):
        file_name = os.path.join(demo_to_replicate, 'data_%d.yaml' % (i + 1))
        with open(file_name) as f:
            data = yaml.safe_load(f)

        # ----------- Inspect data ------------
        # Compute the distance of the recorded positions to find the most meaningful ref. frame
        target_ref_frame = None
        move_frames = []
        min_distance = float('inf')
        for ref_frame in data['vec_pos']:
            # Skip the frame that is excluded from the distance computation
            if ref_frame in excluded_frames[data['type']]:
                continue
            position = data['vec_pos'][ref_frame]
            # distance to the end effector of the robot
            distance3 = math.sqrt(sum(x*x for x in position))
            distance2 = math.sqrt(sum(x*x for x in position[0:2]))
            if distance3 < min_distance:
                # In case of pick we consider the shortest distance
                if data['type'] in action_description['grasp']:
                    min_distance = distance3
                    target_ref_frame = deepcopy(ref_frame)
                    pick_target_frame = deepcopy(ref_frame)
                # In case of place we disregard the ref_frame of the object held
                elif data['type'] in action_description['place'] and\
                        ref_frame != pick_target_frame:
                    min_distance = distance3
                    target_ref_frame = deepcopy(ref_frame)
            if distance2 < 1.5 and data['type'] in action_description['move']:
                move_frames.append(ref_frame)

        # disambiguate move action
        if len(move_frames) == 1:
            target_ref_frame = deepcopy(move_frames[0])
        if target_ref_frame is None:
            # this means that we inspected a move action
            if dialogue_menu is not None:
                target_ref_frame = disambiguate_navigation(
                    move_frames, deepcopy(dialogue_menu), i+1)
            else:
                target_ref_frame = random.choice(move_frames)

        # ------ Generate synthetic data ------
        for j in range(samples):
            new_data = copy(data)
            for ref_frame in data['vec_pos']:
                nav = True if data['type'] in action_description['move'] else False
                # Sample uniformly in a given interval.
                sample = sampling_heuristic(
                    new_data, ref_frame, target_ref_frame, pick_target_frame, workspace, nav)
                new_data['vec_pos'][ref_frame] = sample
                # DO NOT CHANGE ORIENTATION

            # print(f'New Data:\n {new_data}')
            synthetic_data.append(deepcopy(new_data))

    # ------ Save demo ------
    folder_nr = target_demo
    for i in range(samples):
        folder_exists = True
        while folder_exists:
            folder_nr += 1
            new_demo_folder = os.path.join(file_path, 'demo%d' % (folder_nr))
            folder_exists = os.path.isdir(new_demo_folder)

        os.mkdir(new_demo_folder)

        for j in range(n_files):
            new_demo_file = os.path.join(new_demo_folder, 'data_%d.yaml' % (j + 1))
            # print(f'Dumping:\n {synthetic_data[i*samples + j]}')
            with open(new_demo_file, 'w') as f:
                yaml.dump(synthetic_data[i + j*samples], f, default_flow_style=None)


def disambiguate_navigation(move_frames: List[str], dialogue_menu: Any, idx: int = 0) -> str:
    """Disambiguate the navigation action using human input."""
    # check properties of the menu
    dialogue_menu.set_title(
        f'Move action {idx} ambiguous! Select the desired target reference frame.')
    dialogue_menu.set_options(move_frames)
    # run menu
    key_ = dialogue_menu.initialize_layout()
    window = dialogue_menu.get_window()
    event, values = window.read()
    disambiguated_frame = values[key_]
    window.close()

    return disambiguated_frame


def sampling_heuristic(
    data: dict,
    input_frame: str,
    action_target_frame: str,
    pick_object: str,
    workspace: List[Tuple[float, float]],
    navigation: bool = False
) -> List[float]:
    """Heuristic for determining the variation to add to the data in case of manipulation."""
    # The default frame is always the first of the list
    sample = [None, None, None]
    data_position = data['vec_pos'][input_frame]
    low_ = 0.01 if navigation else 0.001
    high_ = 0.05 if navigation else 0.01

    if input_frame == action_target_frame or input_frame == pick_object:
        sample[0] = prob.uniform(low=low_, high=high_)*prob.choice((-1, +1)) + data_position[0]
        sample[1] = prob.uniform(low=low_, high=high_)*prob.choice((-1, +1)) + data_position[1]
    else:
        sample[0] = prob.uniform(low=workspace[0][0], high=workspace[0][1])
        sample[1] = prob.uniform(low=workspace[1][0], high=workspace[1][1])

    # the height is not very important so we do not variate much
    sample[2] = prob.uniform(low=0.0, high=0.005) + data_position[2]
    sample = [float(round(num, 3)) for num in sample]

    return sample
