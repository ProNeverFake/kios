"""Provide an interface between a GP algorithm and behavior tree functions."""

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

import random
from typing import List, Tuple

from behaviors import behavior_tree
from behaviors.behavior_lists import BehaviorLists, ParameterizedNode


def random_genome(
    length: int,
    p_leaf: float,
    behavior_lists: BehaviorLists
) -> List[ParameterizedNode]:
    """Return a random genome."""
    bt = behavior_tree.BT([], behavior_lists)
    return bt.random(length, p_leaf)


def mutate_gene(
    genome: List[ParameterizedNode],
    p_add: float,
    p_delete: float,
    p_variable: float,
    p_replace: float,
    p_swap: float,
    p_leaf: float,
    behavior_lists: BehaviorLists
):
    """ Mutate only a single gene."""
    if p_add < 0 or p_delete < 0 or p_variable < 0 or p_replace < 0 or p_swap < 0:
        raise Exception('Mutation parameters must not be negative.')

    if p_add + p_delete + p_variable + p_replace + p_swap > 1:
        raise Exception('Sum of the mutation probabilities must be less than 1.')

    mutated_individual = behavior_tree.BT([], behavior_lists)
    max_attempts = 100
    attempts = 0
    while (not mutated_individual.is_valid() or mutated_individual.bt == genome) and\
            attempts < max_attempts:
        mutated_individual.set(genome)
        index = random.randint(0, len(genome) - 1)
        mutation = random.random()

        if mutation < p_delete:
            mutated_individual.delete_node(index)
        elif mutation < p_delete + p_add:
            mutated_individual.add_node(index, p_leaf)
        elif mutation < p_delete + p_add + p_variable:
            mutated_individual.change_variable(index)
        elif mutation < p_delete + p_add + p_variable + p_replace:
            mutated_individual.replace_parent_with_subtree(index)
        elif mutation < p_delete + p_add + p_variable + p_replace + p_swap:
            mutated_individual.swap_siblings(index)
        else:
            mutated_individual.change_node(index, p_leaf)

        # Close and trim bt accordingly to the change
        mutated_individual.close()
        mutated_individual.trim()
        attempts += 1

    if attempts >= max_attempts and\
       (not mutated_individual.is_valid() or mutated_individual.bt == genome):
        mutated_individual = behavior_tree.BT([], behavior_lists)

    return mutated_individual.bt


def crossover_genome(
    baseline: List[ParameterizedNode],
    baseline_index: int,
    genome1: List[ParameterizedNode],
    genome2: List[ParameterizedNode],
    behavior_lists: BehaviorLists,
    replace: bool = True
) -> Tuple[List[ParameterizedNode], List[ParameterizedNode]]:
    # pylint: disable=too-many-branches, too-many-locals
    """Do crossover between genomes at random points."""
    bt1 = behavior_tree.BT(genome1, behavior_lists)
    bt2 = behavior_tree.BT(genome2, behavior_lists)
    offspring1 = behavior_tree.BT([], behavior_lists)
    offspring2 = behavior_tree.BT([], behavior_lists)

    if bt1.is_valid() and bt2.is_valid():
        max_attempts = 100
        attempts = 0
        found = False
        while not found and attempts < max_attempts:
            offspring1.set(bt1.bt)
            offspring2.set(bt2.bt)
            cop1 = -1
            cop2 = -1
            if len(genome1) == 1:
                cop1 = 0  # Change whole tree
            else:
                while not offspring1.is_subtree(cop1):
                    cop1 = random.randint(1, len(genome1) - 1)
            if len(genome2) == 1:
                cop2 = 0  # Change whole tree
            else:
                while not offspring2.is_subtree(cop2):
                    cop2 = random.randint(1, len(genome2) - 1)

            if replace:
                offspring1.swap_subtrees(offspring2, cop1, cop2)
            else:
                subtree1 = offspring1.get_subtree(cop1)
                subtree2 = offspring2.get_subtree(cop2)
                if len(genome1) == 1:
                    index1 = random.randint(0, 1)
                else:
                    index1 = random.randint(1, len(genome1) - 1)
                if len(genome2) == 1:
                    index2 = random.randint(0, 1)
                else:
                    index2 = random.randint(1, len(genome2) - 1)

                if genome1 == baseline and baseline_index is not None:
                    index2 = baseline_index
                    index1 = baseline_index + 1
                elif genome2 == baseline and baseline_index is not None:
                    index1 = baseline_index
                    index2 = baseline_index + 1
                offspring1.insert_subtree(subtree2, index1)
                offspring2.insert_subtree(subtree1, index2)

            attempts += 1
            if offspring1.is_valid() and offspring2.is_valid():
                found = True
        if not found:
            offspring1.set([])
            offspring2.set([])

    return offspring1.bt, offspring2.bt
