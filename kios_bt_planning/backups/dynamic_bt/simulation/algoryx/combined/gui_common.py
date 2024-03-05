"""Functionalities for the GP+LfD framework."""

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

import os
import re
import subprocess
from typing import Any, Dict, List

from behaviors.behavior_lists import BehaviorLists
import bt_learning.gp.genetic_programming as gp
from bt_learning.metalearning.hardcode_metadata import find_task_trees
from simulation.algoryx.gp.fitness_function import Coefficients
from simulation.py_trees_interface import PyTreeParameters
import yaml


def init_gp_params(data: Dict, log_path: str) -> gp.GpParameters:
    gp_par = gp.GpParameters()
    gp_par.save_interval = data['genetic']['save_interval']
    gp_par.ind_start_length = data['genetic']['ind_length']
    gp_par.n_population = data['genetic']['n_population']
    gp_par.n_generations = data['genetic']['n_generations']
    gp_par.rerun_fitness = 0
    gp_par.f_crossover = data['genetic']['p_crossover']
    gp_par.n_offspring_crossover = 2
    gp_par.replace_crossover = False
    gp_par.f_mutation = data['genetic']['p_mutation']
    gp_par.n_offspring_mutation = 2
    gp_par.parent_selection = get_selection_from_data(
        data['genetic']['selection'])
    gp_par.survivor_selection = get_selection_from_data(
        data['genetic']['selection'])
    gp_par.f_elites = data['genetic']['p_elite']
    gp_par.f_parents = gp_par.f_elites
    gp_par.max_multi_mutation = data['genetic']['mutations_per_individual']
    gp_par.mutate_co_offspring = False
    gp_par.mutate_co_parents = True
    gp_par.mutation_p_add = data['genetic']['mutation_add']
    gp_par.mutation_p_delete = data['genetic']['mutation_delete']
    gp_par.mutation_p_variable = 0.0
    gp_par.mutation_p_replace = 0.0
    gp_par.mutation_p_swap = 0.0
    gp_par.allow_identical = False
    gp_par.plot = True
    gp_par.verbose = True
    gp_par.fig_last_gen = False
    gp_par.log_name = get_log_number(log_path)

    return gp_par


def get_fitness_coefficients() -> Coefficients:
    coeff = Coefficients()
    coeff.task_completion = 1000.0
    coeff.task_unstacked = 500.0
    coeff.pos_acc = 0.0
    coeff.depth = 0.0
    coeff.length = 10.0
    coeff.ticks = 0.0
    coeff.failed = 50.0
    coeff.timeout = 30.0
    coeff.hand_not_empty = 0.0

    return coeff


def get_pytree_params(behavior_lists: Any, behaviors: Any) -> PyTreeParameters:
    params = PyTreeParameters()
    params.behavior_lists = behavior_lists
    params.behaviors = behaviors
    params.max_ticks = 15
    params.sim_tick_time = 2.5
    params.max_time = 50
    params.max_fails = 1
    params.successes_required = 2
    params.show_world = False
    params.verbose = False

    return params


def get_selection_from_data(data: str) -> gp.SelectionMethods:
    """Read the string and return correct type."""
    if data == 'tournament':
        return gp.SelectionMethods.TOURNAMENT
    elif data == 'rank':
        return gp.SelectionMethods.RANK
    elif data == 'elitism':
        return gp.SelectionMethods.ELITISM
    elif data == 'random':
        return gp.SelectionMethods.RANDOM


def get_log_number(log_path: str) -> str:
    """Return a name for the log folder that is not currently used."""
    number = 1
    while os.path.isdir(os.path.join(log_path, f'logs/log_{number}')):
        number += 1

    return str(number)


def display(path_to_img: str) -> None:
    """Open the image specified by the path."""
    if os.name == 'nt':  # Windows
        os.startfile(path_to_img)
    else:
        opener = 'xdg-open'
        subprocess.call([opener, path_to_img])


def process_lfd_bt(
        bt: List[str],
        bt_list: BehaviorLists = None,
        shrink: bool = False,
        name: str = 'shrunken_bt',
        location: str = os.path.dirname((os.path.abspath(__file__)))
) -> List[str]:
    """Extract the subtree moving the object and replace it with a behavior as in the GP pool."""
    subtrees = find_task_trees(bt)
    goals = [tree[1] for tree in subtrees]

    move_bts = []
    idx = 1

    for i, goal in enumerate(goals):
        if goal.startswith('object_at'):
            match_str = f'^object_at (.+) (.+) (.+)$'
            match = re.match(match_str, goal)
            action = 'place'

            move_bts += ['move0' + ' ' + str(match[1]) + ' ' + str(match[2]) + ' ' + str(match[3])]
            idx += len(subtrees[i])
            new_btlist = BehaviorLists(atomic_fallback_nodes=move_bts)

        elif goal.startswith('object_roughly_at'):
            match_str = f'^object_roughly_at (.+) (.+) (.+)$'
            match = re.match(match_str, goal)
            action = 'drop'

            move_bts += ['move0' + ' ' + str(match[1]) + ' ' + str(match[2]) + ' ' + str(match[3])]
            idx += len(subtrees[i])
            new_btlist = BehaviorLists(atomic_fallback_nodes=move_bts)

    # recompose the LfD tree with the changed behavior
    gp_bt = ['s(']
    gp_bt += move_bts
    # close
    gp_bt.append(')')

    if shrink:
        with open(os.path.join(location, f'{name}.yaml'), 'w+') as f:
            yaml.dump(gp_bt, f)
        gp_bt = ['s(', f'{name}', ')']
        new_btlist = BehaviorLists(atomic_fallback_nodes=[f'{name}'])

    if bt_list is not None:
        bt_list.merge_behaviors(new_btlist)

    return gp_bt
