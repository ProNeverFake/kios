"""Parallelize the Genetic Programming algorithm computation."""

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

import functools
import multiprocessing as mp
import random
from statistics import mean
from typing import Any, List, Tuple

import bt_learning.gp.genetic_programming as gp
from bt_learning.gp.hash_table import HashTable


def get_fitness_simple(
    individual: Any,
    environment: Any,
    min_episodes: int = 1
) -> float:
    """
    Return the fitness value.

    This version excludes rerun and hash table for easier parallelism.
    """
    values = []
    for i in range(min_episodes):
        fitness = environment.get_fitness(individual, i)
        values.append(fitness)
    return mean(values)


def get_fitness_min_population(
    min_population: list,
    population: list,
    environment: Any,
    min_episodes: int
) -> List[float]:
    """Get the fitness of a reduced population."""
    result = []
    for individual in min_population:
        result.append(get_fitness_simple(population[individual], environment, min_episodes))
    return result


def find_individuals_to_eval(
    population: Any,
    hash_table: HashTable,
    environment: Any,
    rerun_fitness: int
) -> Tuple[List[float], List[int]]:
    """Get the indexes of the individual to re-evaluate."""
    fitness_list = []
    individual_to_eval = []
    # find the individuals to be evaluated
    for idx, individual in enumerate(population):
        values = hash_table.find(individual)
        if values is None:
            fitness_list.append(None)
            individual_to_eval.append(idx)
        elif rerun_fitness == 2 or\
                (rerun_fitness == 1 and random.random() < gp.rerun_probability(len(values))):
            fitness = environment.get_fitness(individual, len(values))
            hash_table.insert(individual, fitness)
            fitness_list.append(mean(values))
        else:
            fitness_list.append(mean(values))
    return fitness_list, individual_to_eval


def parallel_evaluate(
    eval_func: Any,
    fitness_list: List[float],
    eval_list: list,
    batch_size: int,
    core_limit: int
) -> List[float]:
    """Spread the computation in parallel processes."""
    # find an appropriate number of process
    # the number of processes k should be 2<= k <= (core_limit * 2)
    # k too high inefficient, k too small multiprocessing is not necessary
    num_process = max(min(len(eval_list) // batch_size + 1, core_limit*2), 2)
    part_size = len(eval_list) // num_process
    if len(eval_list) % num_process != 0:
        part_size += 1
    mp_input = []
    # prepare input for each process
    for core_id in range(num_process):
        mp_input.append(eval_list[core_id * part_size:(core_id + 1)*part_size])

    with mp.Pool(processes=num_process) as pool:
        temp_list = pool.map(eval_func, mp_input)
    # retrieve fitness scores from the processes
    write_back_count = 0
    for core_id in range(num_process):
        for p_idx in range(len(temp_list[core_id])):
            fitness_list[eval_list[write_back_count]] = temp_list[core_id][p_idx]
            write_back_count += 1
    assert write_back_count == len(eval_list)
    return fitness_list


def evaluate(
    population: list,
    hash_table: HashTable,
    environment: Any,
    rerun_fitness: int,
    min_episodes: int,
    num_core: int,
    batch_size: int = 100
) -> List[float]:
    """Evaluate the population using the environment."""
    # the evaluation workload increases according to min_episodes
    batch_size = batch_size // min_episodes
    fitness_list, to_eval = find_individuals_to_eval(
        population, hash_table, environment, rerun_fitness)
    # decide whether to parallel compute or not
    if len(to_eval) < batch_size:
        for idx in to_eval:
            fitness_list[idx] = get_fitness_simple(population[idx], environment, min_episodes)
    else:
        eval_func = functools.partial(
            get_fitness_min_population,
            population=population,
            environment=environment,
            min_episodes=min_episodes
        )
        fitness_list = parallel_evaluate(eval_func, fitness_list, to_eval, batch_size, num_core)
    for eval_idx in to_eval:
        if rerun_fitness == 0 and hash_table.find(population[eval_idx]):
            continue
        hash_table.insert(population[eval_idx], fitness_list[eval_idx])
    return fitness_list
