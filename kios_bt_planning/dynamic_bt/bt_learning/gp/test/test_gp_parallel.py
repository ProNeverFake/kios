"""Unit test for gp_parallel.py module."""

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

from behaviors import behavior_list_test_settings
from behaviors import behavior_lists as bl
import bt_learning.gp.genetic_programming as gp
import bt_learning.gp.gp_parallel as gpp
from bt_learning.gp.hash_table import HashTable

from . import environment_strings as environment


behavior_lists = bl.BehaviorLists(
    condition_nodes=behavior_list_test_settings.get_condition_nodes(),
    action_nodes=behavior_list_test_settings.get_action_nodes())


class TestEnvironment:
    """
    Environment for the test routines.

    The current version of get fitness doesn't allow pass module as param
    because of multi process limitation.
    """

    def get_fitness(self, individual, _seed=None):
        """ Wrap get_fitness function"""
        return environment.get_fitness(individual, _seed)


def test_get_fitness():
    """Test get_fitness function."""
    gp_par = gp.GpParameters()
    t_environment = TestEnvironment()
    gp_par.behavior_lists = behavior_lists
    hash_table = HashTable()
    fitness = gp.get_fitness([], hash_table, t_environment)
    assert fitness == 0

    fitness = gp.get_fitness(['b?'], hash_table, t_environment)
    assert fitness == 0.9

    def randomized_testing(num_individual):
        individual_list = []
        target_fitness = []
        for _ in range(num_individual):
            if random.randint(0, 1) == 0:
                individual_list.append([])
                target_fitness.append(0)
            else:
                individual_list.append(['b?'])
                target_fitness.append(0.9)

        tester_hash_table = HashTable()
        mp_fitness = gpp.evaluate(individual_list, tester_hash_table, t_environment, 0, 5, 2)
        assert mp_fitness == target_fitness
    randomized_testing(1)
    randomized_testing(10)
    randomized_testing(50)
    randomized_testing(100)
    randomized_testing(500)
    randomized_testing(1000)
