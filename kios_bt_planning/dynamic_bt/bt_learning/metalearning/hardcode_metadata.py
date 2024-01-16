"""Extract metadata from a Behavior Tree built with backchaining."""

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

from datetime import datetime
import os
import re
from typing import List

from bt_library.module_path import get_btlibrary_path
import yaml
from .yaml_custom_dumper import add_custom_dumper


def get_timestamp() -> str:
    """Return a string indicating current time."""
    dateTimeObj = datetime.now()
    current = str(dateTimeObj.year) + str(dateTimeObj.month) + \
        str(dateTimeObj.day) + '_' + str(dateTimeObj.hour) + str(dateTimeObj.minute)

    return current


def get_indexed_name(file_name: str, file_path: str) -> str:
    """Return a string indicating how many instances of the file are there."""
    indexed_name = file_name
    count = len([
        name for name in os.listdir(file_path)
        if os.path.isfile(os.path.join(file_path, name)) and name.startswith(file_name)
    ])

    indexed_name = indexed_name + '_' + str(count + 1) + '.yaml'
    return indexed_name


def find_task_trees(bt: str) -> List[str]:
    """
    Find the subtrees of a Behavior Tree solving a task.

    Note
    ----
        The BT needs to be deisgned according to the backchaining principle for this to work.
        According to this principle, a subtask is represented with a Fallback as a root.
        If multiple tasks are solved by the BT, then the BT is a squence of those tasks.

    """
    # If the root is sequence, then remove it and also the 'up' node closing it.
    if bt[0] == 's(':
        bt = bt[1:-1]

    open_count = 0
    lv = 0
    subtrees = []
    subtree = []
    for node in bt:
        if open_count == 0:
            # Skip node until next fallback
            if node == 'f(':
                open_count += 1
                lv += 1
                subtree.append(node)
            else:
                continue
        else:
            if node == 's(' or node == 'f(':
                lv += 1
            elif node == ')':
                lv -= 1

            subtree.append(node)

        # Reset
        if lv == 0:
            open_count = 0
            if len(subtree) > 0:
                subtrees.append(subtree)
            subtree = []

    return subtrees


def build_metadata(bt: List[str]) -> dict:
    """Build the metadata from the goals of the Beahvior Tree"""
    subtrees = find_task_trees(bt)
    goals = [tree[1] for tree in subtrees]

    metadata = {
        'label': '',
        'description': '',
        'target_object': '',
        'position_predicate': '',
        'referring_object': '',
        'hri': False,
        'tree': bt
    }

    for goal in goals:
        if goal == 'clear':
            metadata['hri'] = True
            continue
        elif goal.startswith('object_at'):
            match_str = f'^object_at (.+) (.+) (.+)$'
            match = re.match(match_str, goal)
            action = 'place'

        elif goal.startswith('object_roughly_at'):
            match_str = f'^object_roughly_at (.+) (.+) (.+)$'
            match = re.match(match_str, goal)
            action = 'drop'

        metadata['target_object'] = str(match[1])
        metadata['position_predicate'] = str(match[2])
        metadata['referring_object'] = str(match[3])
        metadata['label'] = action + '_' + str(match[2])
        metadata['description'] = action + ' ' + \
            str(match[1]) + ' ' + str(match[2]) + ' ' + str(match[3])

    return metadata


def add_bt_to_library(behavior_tree: List[str], library_dir: str = None) -> str:
    """Add a Yaml file to the input dir with a BT and its metadata."""
    metadata = build_metadata(behavior_tree)

    add_custom_dumper()

    if library_dir is None:
        library_dir = get_btlibrary_path()

    name_with_stamp = metadata['label'] + '_' + get_timestamp()
    indexed_name = get_indexed_name(name_with_stamp, library_dir)
    file_path = os.path.join(library_dir, indexed_name)

    if os.path.isfile(file_path):
        os.remove(file_path)
    with open(file_path, 'w') as f:
        yaml.dump(metadata, f, sort_keys=False)

    return file_path
