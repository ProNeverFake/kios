"""Load a previously learnt BT with custom parameters."""

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
import re
from typing import List


# TODO: handle case in which also the Load Battery behavior is considered


def handle_hri(bt: List[str], use_hri: bool) -> List[str]:
    """Check if disambiguation subtree is in the BT and then add/remove it."""
    hri_subtree = ['f(', 'clear', 'disambiguate', ')']

    # If HRI subtree is there but not required, remove it
    if set(hri_subtree).issubset(set(bt)) and not use_hri:
        hri_root_idx = bt.index('clear') - 1
        handled_bt = bt[0:hri_root_idx] + bt[hri_root_idx + len(hri_subtree):]
    # If HRI subtree is not there but required, add it
    elif not set(hri_subtree).issubset(set(bt)) and use_hri:
        if bt[0] == 's(':
            handled_bt = bt[0] + hri_subtree + bt[1:]
        else:
            handled_bt = ['s('] + hri_subtree + bt + [')']
    # HRI subtree not there and not required, or HRI already there and required
    else:
        handled_bt = bt

    return handled_bt


def load_bt(
    bt: List[str],
    metadata: dict,
    target_obj: str = None,
    predicate: str = None,
    referring_obj: str = None,
    use_hri: bool = False
) -> List[str]:
    """Load a Behavior Tree from the BT library, overwriting parameters if necessary."""
    modified_bt = deepcopy(bt)
    new_target = target_obj if target_obj is not None else metadata['target_object']
    new_predicate = predicate if predicate is not None else metadata['position_predicate']
    new_reference = referring_obj if referring_obj is not None else metadata['referring_object']

    for i, node in enumerate(bt):
        # Actions
        if node.startswith('pick'):
            match = re.match('(.+) (.+)$', node)
            modified_bt[i] = str(match[1]) + ' ' + new_target
        elif node.startswith('place') or node.startswith('drop'):
            match = re.match('(.+) (.+) (.+) (.+)$', node)
            modified_bt[i] = str(match[1]) + ' ' + new_target + ' ' + \
                new_predicate + ' ' + new_reference
        # Conditions
        elif node.startswith('in_gripper'):
            match = re.match('(.+) (.+)$', node)
            modified_bt[i] = str(match[1]) + ' ' + new_target
        elif node.startswith('object_'):
            match = re.match('(.+) (.+) (.+) (.+)$', node)
            modified_bt[i] = str(match[1]) + ' ' + new_target + ' ' + \
                new_predicate + ' ' + new_reference
        else:
            continue

    modified_bt = handle_hri(modified_bt, use_hri)

    return modified_bt
