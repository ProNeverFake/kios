"""Unit test for genetic_programming.py module."""

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
from statistics import mean
import pytest

from behaviors import behavior_list_test_settings
from behaviors import behavior_lists as bl
import bt_learning.gp.genetic_programming as gp
from bt_learning.gp.hash_table import HashTable


from . import environment_strings as environment


behavior_lists = bl.BehaviorLists(
    condition_nodes=behavior_list_test_settings.get_condition_nodes(),
    action_nodes=behavior_list_test_settings.get_action_nodes())


def test_create_population():
    """Test create_population function."""
    pop_size = 5
    genome_length = 5
    for _ in range(5):  # Test a number of random possibilities
        population = gp.create_population(pop_size, genome_length, 0.5, behavior_lists)
        for i in range(pop_size):
            assert len(population[i]) >= genome_length
            for j in range(genome_length):
                assert population[i][j] != []


def test_mutation():
    """Test mutation function."""
    gp_par = gp.GpParameters()
    gp_par.behavior_lists = behavior_lists
    gp_par.n_population = 5
    gp_par.n_offspring_mutation = 1
    gp_par.mutation_p_add = 0.3
    gp_par.mutation_p_delete = 0.3
    gp_par.mutation_p_variable = 0.0
    gp_par.mutation_p_replace = 0.0
    gp_par.mutation_p_swap = 0.0
    gp_par.allow_identical = False
    gp_par.max_multi_mutation = 3
    for _ in range(5):  # Test a number of random possibilities
        population = gp.create_population(gp_par.n_population, 5, 0.5, behavior_lists)
        mutated_population = gp.mutation(population, range(len(population)), gp_par)
        assert len(mutated_population) == gp_par.n_offspring_mutation * gp_par.n_population
        for i in range(len(mutated_population)):
            assert mutated_population[i] not in population
            assert mutated_population.count(mutated_population[i]) == 1

    gp_par.max_multi_mutation = 1
    for _ in range(5):  # Test a number of random possibilities
        population = gp.create_population(gp_par.n_population, 5, 0.5, behavior_lists)
        mutated_population = gp.mutation(population, range(len(population)), gp_par)
        assert len(mutated_population) == gp_par.n_offspring_mutation * gp_par.n_population
        for i in range(len(mutated_population)):
            assert mutated_population[i] not in population
            for j in range(len(mutated_population)):
                if i != j:
                    assert mutated_population[i] != mutated_population[j]

    gp_par.n_offspring_mutation = 5 * gp_par.n_population + 1
    gp_par.allow_identical = True
    for _ in range(5):  # Test a number of random possibilities
        population = gp.create_population(gp_par.n_population, 5, 0.5, behavior_lists)
        mutated_population = gp.mutation(population, range(len(population)), gp_par)
        assert len(mutated_population) == gp_par.n_offspring_mutation * gp_par.n_population

    # Test with too limited mutation possibilities
    gp_par.n_population = 5
    gp_par.n_offspring_mutation = 1
    gp_par.mutation_p_add = 0.0
    gp_par.mutation_p_delete = 1.0
    gp_par.mutation_p_variable = 0.0
    gp_par.mutation_p_replace = 0.0
    gp_par.mutation_p_swap = 0.0
    gp_par.allow_identical = False
    parents = gp.create_population(gp_par.n_population, 2, 0.5, behavior_lists)
    blockers = gp.create_population(7, 1, 0.5, behavior_lists)
    population = parents + blockers
    mutated_population = gp.mutation(population, range(len(parents)), gp_par)
    assert len(mutated_population) != gp_par.n_offspring_mutation * gp_par.n_population
    for i in range(len(mutated_population)):
        assert mutated_population[i] not in population
        for j in range(len(mutated_population)):
            if i != j:
                assert mutated_population[i] != mutated_population[j]


def test_crossover():
    """Test crossover function."""
    pop_size = 10
    gp_par = gp.GpParameters()
    gp_par.behavior_lists = behavior_lists
    gp_par.n_population = pop_size
    gp_par.f_crossover = 0.5
    gp_par.n_offspring_crossover = 1
    gp_par.allow_identical = False
    for _ in range(5):  # Test a number of random possibilities
        population = gp.create_population(pop_size, 5, 0.5, behavior_lists)
        crossover_population = gp.crossover(population, range(len(population)), gp_par)
        for i in range(len(crossover_population)):
            assert crossover_population[i] not in population
            for j in range(len(crossover_population)):
                if i != j:
                    assert crossover_population[i] != crossover_population[j]

    gp_par.n_offspring_crossover = 3 * pop_size
    gp_par.allow_identical = True
    for _ in range(5):  # Test a number of random possibilities
        population = gp.create_population(pop_size, 5, 0.5, behavior_lists)
        crossover_population = gp.crossover(population, range(len(population)), gp_par)
        assert len(crossover_population) == gp_par.n_offspring_crossover * gp_par.n_population

    # Test what happens if no valid crossovers can be found
    ab = bl.ParameterizedNode('ab')
    ac = bl.ParameterizedNode('ac')
    parents = [['s(', ab, ')'], ['s(', ac, ')']]
    gp_par.n_offspring_crossover = 1
    gp_par.n_offspring_mutation = 1
    gp_par.allow_identical = False
    gp_par.n_population = 2
    blockers = [['s(', ac, ab, ')'], ['s(', ab, ac, ')']]
    population = parents + blockers
    crossover_population = gp.crossover(population, range(len(parents)), gp_par)
    assert len(crossover_population) == gp_par.n_offspring_crossover * gp_par.n_population
    for i in range(len(crossover_population)):
        assert crossover_population[i] not in population
        for j in range(len(crossover_population)):
            if i != j:
                assert crossover_population[i] != crossover_population[j]

    # Number of parents not factor of two
    with pytest.raises(ValueError):
        crossover_population = gp.crossover(population, range(3), gp_par)


def test_rerun_probability():
    """Test rerun_probability function."""
    assert gp.rerun_probability(0) == 1

    assert gp.rerun_probability(1) == 1

    assert gp.rerun_probability(2) == 0.5

    assert gp.rerun_probability(100) == 0.01


def test_get_fitness():
    """Test get_fitness function."""
    gp_par = gp.GpParameters()
    gp_par.behavior_lists = behavior_lists
    hash_table = HashTable()
    fitness = gp.get_fitness([], hash_table, environment)
    assert fitness == 0

    fitness = gp.get_fitness(['b?'], hash_table, environment)
    assert fitness == 0.9

    gp_par.rerun_fitness = 2
    gp_par.min_episodes = 3
    individual = ['r0']
    fitness = gp.get_fitness(
        individual, hash_table, environment, gp_par.rerun_fitness, gp_par.min_episodes)
    values = hash_table.find(individual)
    assert len(values) == gp_par.min_episodes
    assert mean(values) == fitness

    fitness = gp.get_fitness(
        individual, hash_table, environment, gp_par.rerun_fitness, gp_par.min_episodes)
    values = hash_table.find(individual)
    assert len(values) == gp_par.min_episodes + 1
    assert mean(values) == fitness

    gp_par.rerun_fitness = 1
    hash_table = HashTable()
    fitness = gp.get_fitness(
        individual, hash_table, environment, gp_par.rerun_fitness, gp_par.min_episodes)
    values = hash_table.find(individual)
    assert len(values) == gp_par.min_episodes
    assert mean(values) == fitness


def test_crossover_parent_selection():
    """Test crossover_parent_selection function."""
    gp_par = gp.GpParameters()
    gp_par.behavior_lists = behavior_lists
    gp_par.n_population = 10
    gp_par.f_crossover = 0.4
    gp_par.parent_selection = gp.SelectionMethods.ELITISM
    population = gp.create_population(gp_par.n_population, 5, 0.5, behavior_lists)
    fitness = [0, 1, 2, 1, 2, 1, 1, 2, 2, 0]
    parents = gp.crossover_parent_selection(population, fitness, gp_par)
    assert parents == [2, 4, 7, 8]

    gp_par.n_population = 4
    gp_par.f_crossover = 0.1
    fitness = [0, 1, 2, 1]
    parents = gp.crossover_parent_selection(population, fitness, gp_par)
    assert parents == []


def test_mutation_parent_selection():
    """Test mutation_parent_selection function."""
    gp_par = gp.GpParameters()
    gp_par.behavior_lists = behavior_lists
    gp_par.n_population = 8
    gp_par.f_mutation = 0.5
    gp_par.parent_selection = gp.SelectionMethods.ELITISM
    gp_par.mutate_co_parents = True
    gp_par.mutate_co_offspring = True
    population = gp.create_population(gp_par.n_population, 5, 0.5, behavior_lists)
    crossover_parents = [5, 6, 7]
    crossover_offspring = gp.create_population(2, 5, 0.5, behavior_lists)
    fitness = [0, 1, 2, 1, 2, 1, 1, 2, 2, 0]
    parents = gp.mutation_parent_selection(
        population, fitness, crossover_parents, crossover_offspring, gp_par)
    assert parents == [2, 4, 7, 8]

    gp_par.mutate_co_parents = False
    gp_par.mutate_co_offspring = False
    parents = gp.mutation_parent_selection(
        population, fitness, crossover_parents, crossover_offspring, gp_par)
    assert parents == [2, 4, 1, 3]

    gp_par.n_population = 4
    gp_par.f_mutation = 0.1
    fitness = [0, 1, 2, 1]
    parents = gp.mutation_parent_selection(population, fitness, [], [], gp_par)
    assert parents == []


def test_survivor_selection():
    """Test survivor_selection function."""
    gp_par = gp.GpParameters()
    gp_par.behavior_lists = behavior_lists
    gp_par.f_parents = 1/3
    gp_par.f_elites = 0
    gp_par.n_population = 6
    gp_par.survivor_selection = gp.SelectionMethods.ELITISM
    population = list(range(gp_par.n_population))
    crossover_offspring = list(range(gp_par.n_population, gp_par.n_population + 2))
    mutated_offspring = list(range(gp_par.n_population + 2, gp_par.n_population + 4))
    fitness = [0, 1, 2, 1, 2, 1, 1, 2, 2, 0]
    survivors, survivor_fitness = gp.survivor_selection(
        population,
        fitness,
        crossover_offspring,
        mutated_offspring,
        gp_par
    )
    assert len(survivors) == len(survivor_fitness)
    assert survivors == [2, 4, 7, 8, 6, 9]
    assert survivor_fitness == [2, 2, 2, 2, 1, 0]

    gp_par.f_parents = 0
    gp_par.f_elites = 2/3
    gp_par.n_population = 3
    gp_par.survivor_selection = gp.SelectionMethods.RANDOM
    survivors, survivor_fitness = gp.survivor_selection(
        population,
        fitness,
        crossover_offspring,
        mutated_offspring,
        gp_par
    )
    assert len(survivors) == len(survivor_fitness)
    assert survivors[0] == 8
    assert survivors[1] == 7
    assert survivors[2] == 9 or survivors[2] == 6


def test_selection():
    """Test selection function."""
    population = list(range(6))
    fitness = [0, 1, 2, 1, 3, 1]

    selected = gp.selection(population, fitness, 2, gp.SelectionMethods.ELITISM)
    assert selected == [4, 2]

    selected = gp.selection(population, fitness, 3, gp.SelectionMethods.TOURNAMENT)
    assert 4 in selected
    assert 0 not in selected

    selected = []
    for _ in range(10):
        selected += gp.selection(population, fitness, 2, gp.SelectionMethods.RANK)
    assert 4 in selected

    random.seed(0)
    selected1 = gp.selection(population, fitness, 3, gp.SelectionMethods.RANDOM)
    selected2 = gp.selection(population, fitness, 3, gp.SelectionMethods.RANDOM)
    assert selected1 != selected2

    selected = gp.selection(population, fitness, 2, gp.SelectionMethods.ALL)
    assert selected == population

    with pytest.raises(Exception):
        selected = gp.selection(population, fitness, 2, -1)


def test_elite_selection():
    """Test elite_selection function."""
    population = list(range(8))
    fitness = [0, 6, 2, 8, 3, 2, 1, 1]

    selected = gp.elite_selection(population, fitness, 2)
    assert selected == [3, 1]

    fitness = [0, 6, 2, 6, 3, 2, 7, 1]
    selected = gp.elite_selection(population, fitness, 2)
    assert selected == [6, 1]


def test_tournament_selection():
    """Test tournament_selection function."""
    population = list(range(10))
    fitness = [2, 1, 2, 1, 3, 1, 4, 5, 3, 0]

    selected = gp.tournament_selection(population, fitness, 5)
    assert 7 in selected
    assert 9 not in selected

    selected = gp.tournament_selection(population, fitness, 4)
    assert 7 in selected
    assert 9 not in selected

    selected = gp.tournament_selection(population, fitness, 3)
    assert 9 not in selected


def test_rank_selection():
    """Test tournament_selection function."""
    population = list(range(10))
    fitness = [2, 1, 1, 1, 3, 1, 4, 5, 3, 0]

    n_times_selected = 0
    n_runs = 100
    for seed in range(0, n_runs):
        gp.set_seeds(seed)
        selected = gp.rank_selection(population, fitness, 2)
        if 0 in selected:
            n_times_selected += 1

    # 0 is the 5th in rank so the probability of getting selected
    # should be (10 - 4) / sum(1 to 10) = 6 / 55
    # Probability of getting selected when picking 2 out of 10 is then
    # 6 / 55 + (55 - 6) / 55 * 6 / 55 = 6 / 55 * (2 - 6 / 55)
    # Check is with some margin
    assert 6/55 * (2 - 6 / 55) - 0.05 < n_times_selected / n_runs < 6/55 * (2 - 6 / 55) + 0.05


def test_adjustable_rank_selection():
    """Test adjustable ranked selection function."""
    # rank selection is a special case of adjustable ranked selection
    # therefore, we can repeat the rank selection testing method
    # by using the corresponding pressure factor
    population = list(range(10))
    fitness = [2, 1, 1, 1, 3, 1, 4, 5, 3, 0]

    n_times_selected = 0
    n_runs = 100
    size_population = len(fitness)
    for seed in range(0, n_runs):
        gp.set_seeds(seed)
        selected = gp.adjustable_rank_selection(
            population, fitness, 2, pressure_factor=(size_population-1)/(size_population+1))
        if 0 in selected:
            n_times_selected += 1
    assert 6/55 * (2 - 6 / 55) - 0.05 < n_times_selected / n_runs < 6/55 * (2 - 6 / 55) + 0.05

    # pressure_factor = 1, that is the worst one will never be selected
    n_times_selected = 0
    for seed in range(0, n_runs):
        selected = gp.adjustable_rank_selection(population, fitness, 2, pressure_factor=1)
        if 9 in selected:
            n_times_selected += 1
    assert n_times_selected == 0

    # pressure_factor = 0, that is even distribution
    n_times_selected = 0
    n_runs = 100000
    selected_history = [0] * 10
    for _ in range(0, n_runs):
        selected = gp.adjustable_rank_selection(population, fitness, 2, pressure_factor=0)
        for i in selected:
            selected_history[i] += 1
    assert max(selected_history) / min(selected_history) < 1.1


def test_print_population():
    """Test print_population function."""
    population = list(range(8))
    fitness = [0, 6, 2, 8, 3, 2, 1, 1]
    gp.print_population(population, fitness, 5)


def test_run():
    """Test run function."""
    gp_par = gp.GpParameters()
    gp_par.behavior_lists = behavior_lists
    gp_par.ind_start_length = 3
    gp_par.n_population = 20
    gp_par.f_crossover = 0.5
    gp_par.n_offspring_crossover = 10
    gp_par.f_mutation = 0.25
    gp_par.n_offspring_mutation = 10
    gp_par.f_elites = 0.1
    gp_par.f_parents = 1
    gp_par.plot = False
    gp_par.n_generations = 20
    gp_par.verbose = True
    gp_par.fig_best = False

    gp.set_seeds(1337)
    population, fitness, _, _ = gp.run(environment, gp_par)
    assert (max(fitness)) == 3.4

    gp.set_seeds(1337)
    population2, fitness2, _, _ = gp.run(environment, gp_par)
    assert population == population2
    assert fitness == fitness2

    gp_par.parent_selection = gp.SelectionMethods.RANK
    gp_par.survivor_selection = gp.SelectionMethods.RANK
    gp.set_seeds(1337)
    population, fitness, _, _ = gp.run(environment, gp_par)
    assert (max(fitness)) >= 3.2

    gp.set_seeds(1337)
    population2, fitness2, _, _ = gp.run(environment, gp_par)
    assert population == population2
    assert fitness == fitness2


def test_hotstart():
    """Test with hotstart."""
    gp_par = gp.GpParameters()
    gp_par.behavior_lists = behavior_lists
    gp_par.ind_start_length = 3
    gp_par.n_population = 8
    gp_par.f_crossover = 0.5
    gp_par.n_offspring_crossover = 2
    gp_par.f_mutation = 0.5
    gp_par.n_offspring_mutation = 2
    gp_par.f_elites = 1/8
    gp_par.f_parents = gp_par.f_elites
    gp_par.plot = False
    gp_par.verbose = False
    gp_par.fig_best = False
    gp_par.fig_last_gen = False

    gp.set_seeds(0)
    gp_par.n_generations = 6
    gp_par.save_interval = 5
    gp.run(environment, gp_par, hotstart=False)

    gp_par.n_generations = 7
    population, fitness, best_fitness, best_individual = gp.run(
        environment, gp_par, hotstart=True)

    gp_par.log_name = '2'
    gp.set_seeds(0)
    population2, fitness2, best_fitness2, best_individual2 = gp.run(
        environment, gp_par, hotstart=False)

    assert population == population2
    assert fitness == fitness2
    assert best_fitness == best_fitness2
    assert best_individual == best_individual2

    gp.set_seeds(0)
    gp_par.n_generations = 5
    population, fitness, best_fitness, best_individual = gp.run(
        environment, gp_par, hotstart=False)

    gp_par.n_generations = 5
    population2, fitness2, best_fitness2, best_individual2 = gp.run(
        environment, gp_par, hotstart=True)

    assert population == population2
    assert fitness == fitness2
    assert best_fitness == best_fitness2
    assert best_individual == best_individual2

    gp.set_seeds(0)
    gp_par.n_generations = 5
    gp_par.rerun_fitness = 1
    population, fitness, best_fitness, best_individual = gp.run(
        environment, gp_par, hotstart=False)

    gp_par.n_generations = 5
    population2, fitness2, best_fitness2, best_individual2 = gp.run(
        environment, gp_par, hotstart=True)

    assert population == population2
    assert fitness == fitness2
    assert best_fitness == best_fitness2
    assert best_individual == best_individual2


def test_baseline():
    """Test with baseline."""
    gp_par = gp.GpParameters()
    gp_par.behavior_lists = behavior_lists
    gp_par.ind_start_length = 3
    gp_par.n_population = 4
    gp_par.f_crossover = 0.5
    gp_par.n_offspring_crossover = 2
    gp_par.f_mutation = 0.5
    gp_par.n_offspring_mutation = 2
    gp_par.f_elites = 1/4
    gp_par.f_parents = gp_par.f_elites
    gp_par.selection_method = gp.SelectionMethods.ELITISM
    gp_par.boost_baseline = True
    gp_par.plot = False
    gp_par.verbose = False
    gp_par.fig_best = False
    gp_par.fig_last_gen = False

    gp.set_seeds(1)
    gp_par.n_generations = 10
    baseline = [bl.ParameterizedNode('b')]
    population, _, best_fitness, _ = gp.run(environment, gp_par, hotstart=False, baseline=baseline)
    assert best_fitness[-1] >= environment.get_fitness(baseline)

    gp.set_seeds(0)
    gp_par.n_generations = 5
    gp_par.boost_baseline = False
    gp_par.f_crossover = 0.0
    gp_par.n_offspring_mutation = 4
    gp_par.save_interval = 5
    baseline = [bl.ParameterizedNode('ab', condition=False)]
    gp.run(environment, gp_par, hotstart=False, baseline=baseline)

    gp_par.n_generations = 6
    gp_par.f_crossover = 0.0
    gp_par.f_mutation = 0.0
    gp_par.f_parents = 1.25
    gp_par.survivor_selection = gp.SelectionMethods.ALL
    population, _, best_fitness, _ = gp.run(environment, gp_par, hotstart=True, baseline=baseline)
    assert best_fitness[-1] > environment.get_fitness(baseline)
    assert baseline in population

    gp_par.n_generations = 7
    gp_par.f_crossover = 0.5
    gp_par.f_mutation = 0.5
    gp_par.f_parents = 1/4
    gp_par.keep_baseline = False
    gp_par.survivor_selection = gp.SelectionMethods.RANK
    population, _, best_fitness, _ = gp.run(environment, gp_par, hotstart=True, baseline=baseline)
    assert best_fitness[-1] > environment.get_fitness(baseline)
    assert baseline not in population


def test_plots():
    """Test plot functionality."""
    gp_par = gp.GpParameters()
    gp_par.behavior_lists = behavior_lists
    gp_par.ind_start_length = 3
    gp_par.n_population = 20
    gp_par.f_crossover = 0.5
    gp_par.n_offspring_crossover = 10
    gp_par.f_mutation = 0.25
    gp_par.n_offspring_mutation = 10
    gp_par.f_elites = 0.1
    gp_par.f_parents = 1
    gp_par.plot = True
    gp_par.verbose = False
    gp_par.fig_best = True
    gp_par.fig_last_gen = True
    gp_par.n_generations = 1

    gp.set_seeds(1337)
    gp.run(environment, gp_par)
