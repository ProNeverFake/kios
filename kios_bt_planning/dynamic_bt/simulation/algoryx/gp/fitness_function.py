"""Task dependent cost function."""

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

from dataclasses import dataclass
from typing import Any, Dict

from simulation.py_trees_interface import PyTree
from simulation.algoryx.gp.gp_interface import GPInterface


@dataclass
class Coefficients:
    """Coefficients for tuning the cost function."""
    task_completion: float = 1000.0
    task_unstacked = 0.0
    pos_acc: float = 0.0
    depth: float = 0.0
    length: float = 10.0
    ticks: float = 0.0
    failed: float = 50.0
    timeout: float = 10.0
    hand_not_empty: float = 0.0


def compute_fitness(
    world_interface: GPInterface,
    behavior_tree: PyTree,
    ticks: int,
    targets: Dict,
    coeff: Coefficients = None,
    verbose: bool = False
):
    # pylint: disable=too-many-arguments
    """Retrieve values and compute cost."""
    if coeff is None:
        coeff = Coefficients()

    depth = behavior_tree.bt.depth()
    length = behavior_tree.bt.length()

    cost = coeff.length * length + \
        coeff.depth * depth + \
        coeff.ticks * ticks
    if verbose:
        print('Cost from length:', cost)
    for i, target in enumerate(targets.keys()):
        target_pose = world_interface.as_vec3(targets[target]['pose'])
        position, _ = world_interface.get_item_in_frame(target, targets[target]['reference'])
        pose = world_interface.as_vec3(position)
        cost += coeff.task_completion * \
            max(0, world_interface._distance(target_pose, pose) - coeff.pos_acc)
        if verbose:
            print('Cost:', cost)
            print(i, ': ', position)
            print('Distance', world_interface._distance(target_pose, pose))

    if not world_interface.unstacked:
        cost += coeff.task_unstacked
        if verbose:
            print('Failed: ', cost)

    if behavior_tree.failed:
        cost += coeff.failed
        if verbose:
            print('Failed: ', cost)
    if behavior_tree.timeout:
        cost += coeff.timeout
        if verbose:
            print('Timed out: ', cost)
    if world_interface.holding is not None and world_interface.holding != '':
        cost += coeff.hand_not_empty
        if verbose:
            print('Hand not empty: ', cost)
    fitness = -cost
    return fitness
