"""Unit test for gp_bt_interface.py module."""

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
import pytest
from behaviors import behavior_list_test_settings, behavior_tree
from behaviors import behavior_lists as bl
from bt_learning.gp import gp_bt_interface


behavior_lists = bl.BehaviorLists(
    condition_nodes=behavior_list_test_settings.get_condition_nodes(),
    action_nodes=behavior_list_test_settings.get_action_nodes())


def test_mutate_gene():
    """Test mutate_gene function."""
    ab = bl.ParameterizedNode('ab', [], False)
    ac = bl.ParameterizedNode('ac', [], False)
    genome = ['s(', ab, ac, ')']

    with pytest.raises(Exception):
        gp_bt_interface.mutate_gene(
            genome, -1, 1, 0, 0, 0, 0.0, behavior_lists)

    with pytest.raises(Exception):
        gp_bt_interface.mutate_gene(
            genome, 1, 1, 0, 0, 0, 0.0, behavior_lists)

    for _ in range(10):
        # Loop many times to catch random errors
        mutated_genome = gp_bt_interface.mutate_gene(
            genome, 1, 0, 0, 0, 0, 0.5, behavior_lists)
        assert len(mutated_genome) >= len(genome)

        mutated_genome = gp_bt_interface.mutate_gene(
            genome, 0, 1, 0, 0, 0, 0.0, behavior_lists)
        assert len(mutated_genome) <= len(genome)

        mutated_genome = gp_bt_interface.mutate_gene(
            genome, 0, 0, 0, 0, 0, 0.5, behavior_lists)
        bt = behavior_tree.BT(mutated_genome, behavior_lists)
        assert mutated_genome != genome
        assert bt.is_valid()

        mutated_genome = gp_bt_interface.mutate_gene(
            genome, 0.3, 0.3, 0, 0, 0, 0.5, behavior_lists)
        bt.set(mutated_genome)
        assert mutated_genome != genome
        assert bt.is_valid()

    genome = ['dummy1', 'dummy2', 'dummy3']
    mutated_genome = gp_bt_interface.mutate_gene(
        genome, 1, 0, 0, 0, 0, 0.0, behavior_lists)
    assert mutated_genome == []


def test_crossover_genome():
    """Test crossover_genome function."""
    b = bl.ParameterizedNode('b', [], True)
    c = bl.ParameterizedNode('c', [], True)
    ab = bl.ParameterizedNode('ab', [], False)
    ac = bl.ParameterizedNode('ac', [], False)

    genome1 = ['s(', b, 'f(', b, ab, ')', ab, ')']
    genome2 = ['f(', c, 's(', c, ac, ')', ac, ')']

    offspring1, offspring2 = gp_bt_interface.crossover_genome(genome1, genome2, behavior_lists)

    assert offspring1 != []
    assert offspring2 != []
    assert offspring1 != genome1
    assert offspring1 != genome2
    assert offspring2 != genome1
    assert offspring2 != genome2

    bt1 = behavior_tree.BT(offspring1, behavior_lists)
    assert bt1.is_valid()
    bt1 = bt1.set(offspring2)
    assert bt1.is_valid()

    genome1 = [ab]
    genome2 = [ac]
    offspring1, offspring2 = gp_bt_interface.crossover_genome(genome1, genome2, behavior_lists)
    assert offspring1 == genome2
    assert offspring2 == genome1

    genome1 = []
    offspring1, offspring2 = gp_bt_interface.crossover_genome(genome1, genome2, behavior_lists)
    assert offspring1 == []
    assert offspring2 == []

    for i in range(10):
        random.seed(i)
        offspring1, offspring2 = gp_bt_interface.crossover_genome(
            gp_bt_interface.random_genome(10, 0.5, behavior_lists),
            gp_bt_interface.random_genome(10, 0.5, behavior_lists), behavior_lists)
        bt1 = bt1.set(offspring1)
        assert bt1.is_valid()
        bt1 = bt1.set(offspring2)
        assert bt1.is_valid()

    genome1 = ['s(', 'f(', b, ab, ')', ab, ')']
    genome2 = ['f(', 's(', c, ac, ')', ac, ')']
    offspring1, offspring2 = gp_bt_interface.crossover_genome(
        genome1, genome2, behavior_lists, replace=False)
    assert offspring1 != genome1
    assert offspring2 != genome2
    for gene in genome1:
        assert gene in offspring1
    for gene in genome2:
        assert gene in offspring2
