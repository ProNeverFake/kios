"""
An object-oriented implementation of genetic programming.

Designed for distributed island model and general genetic programming.
"""

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
from typing import Any, List, Tuple
import numpy as np


from behaviors.behavior_tree import BT
from bt_learning.gp import logplot
import bt_learning.gp.genetic_programming as gp
import bt_learning.gp.gp_parallel as gpp
from bt_learning.gp.hash_table import HashTable


class GPInstance:
    """ Main GP object """
    def __init__(
        self,
        instance_name: str,
        environment: Any,
        num_core: int,
        batch_size: int,
        gp_par: gp.GpParameters,
        hotstart: bool = False
    ):
        self.environment = environment
        self.params = gp_par
        self.batch_size = batch_size
        self.num_core = num_core  # the max number of core the instance can use
        self.num_gen = 0   # the number of generation passed
        # use a separated folder for each instance
        self.my_path = gp_par.log_name + '_instance_' + instance_name
        self.hash_table = HashTable(gp_par.hash_table_size, self.my_path)
        if hotstart:
            self.population = logplot.get_last_population(self.my_path)
            self.fitness = self.evaluate_population(self.population)
            self.best_fitness = logplot.get_best_fitness(self.my_path)
            self.n_episodes = logplot.get_n_episodes(self.my_path)
            randomstate, np_randomstate, self.num_gen = logplot.get_state(self.my_path)
            random.setstate(randomstate)
            np.random.set_state(np_randomstate)
            logplot.clear_after_generation(self.my_path, self.num_gen)
            self.hash_table.load()
        else:
            self.population = gp.create_population(
                gp_par.n_population,
                gp_par.ind_start_length,
                gp_par.mutation_p_leaf,
                gp_par.behavior_lists
            )
            logplot.clear_logs(self.my_path)
            self.fitness = self.evaluate_population(self.population)
            self.best_fitness = [max(self.fitness)]
            self.best_individual = [self.fitness.index(self.best_fitness[0])]
            logplot.log_fitness(self.my_path, self.fitness)
            logplot.log_population(self.my_path, self.population)
            self.n_episodes = []
            self.n_episodes.append(self.hash_table.n_values)

        if gp_par.verbose:
            gp.print_population(self.population, self.fitness, self.num_gen)
            print('Generation: ', self.num_gen, ' Best fitness: ', self.best_fitness[-1])
        # use these counters to track where
        # the best individuals come from.
        self.use_tracker = gp_par.use_tracker
        self.exchange_count = 0
        self.best_from_exchange_count = 0
        self.best_from_mutation_count = 0
        self.best_from_crossover_count = 0
        self.best_from_replication_count = 0

        self.diversity_types = ['ed1', 'ed2', 'isomorphs', 'entropy']
        self.max_diversity = 0
        self.min_diversity = 99999
        self.selection_pressure = 0.9

        self.individual_received = []
        self.fitness_received = []

    def is_runnable(self) -> bool:
        """Check if the GP can be run."""
        return self.num_gen < self.params.n_generations

    def get_current_population(self) -> Tuple[list, List[float]]:
        """Return the current population with the fitness values."""
        return self.population, self.fitness

    def get_best_individual(self) -> Tuple[Any, float]:
        """Return the best individual with its score."""
        best_fitness_score = max(self.fitness)
        best_idx = self.fitness.index(best_fitness_score)
        return self.population[best_idx], best_fitness_score

    def get_episodes(self) -> int:
        """Return the number of episodes the GP has run."""
        return self.hash_table.n_values

    def __get_best_k_from_population(self, k: int) -> Tuple[list, List[float]]:
        """
        Select the best k individuals from the populations.

        A private function for get_migrants() and merge_migrants.
        """
        selected = gp.selection(
            range(len(self.population)), self.fitness, k, gp.SelectionMethods.ELITISM)
        best_k = []
        best_k_fitness = []
        for i in selected:
            best_k.append(self.population[i])
            best_k_fitness.append(self.fitness[i])
        return best_k, best_k_fitness

    def get_migrants(self, num_migrants: int) -> Tuple[list, List[float]]:
        """
        Return a list of migrants and the corresponding fitness score.

        The best ones are selected as migrants.
        """
        return self.__get_best_k_from_population(num_migrants)

    def receive_migrants(self, migrants: list, migrant_fitness: List[float]) -> None:
        """Save the migrants received."""
        self.individual_received += migrants
        self.fitness_received += migrant_fitness

    def merge_migrants(self) -> None:
        """
        Merge the received migrants to the population.

        The worst ones in the population are replaced with the migrants.
        """
        num_from_population = self.params.n_population - len(self.individual_received)
        # keep a record of the received ones in the hash table
        for idx in range(len(self.individual_received)):
            self.hash_table.insert(self.individual_received[idx], self.fitness_received[idx])
        self.population, self.fitness = self.__get_best_k_from_population(num_from_population)

        # check if the new best one is from exchanging
        if self.use_tracker:
            self.exchange_count += 1
            if max(self.fitness) < max(self.fitness_received):
                self.best_from_exchange_count += 1

        self.population += self.individual_received
        self.fitness += self.fitness_received
        self.individual_received = []
        self.fitness_received = []
        self.log_diversity(True)

    def evaluate_population(self, population: list) -> List[float]:
        """Simplify the parameters for gpp.evaluate()."""
        return gpp.evaluate(
            population,
            self.hash_table,
            self.environment,
            self.params.rerun_fitness,
            self.params.min_episodes,
            self.num_core,
            self.batch_size
        )

    def step_gp(self) -> None:
        """
        Step one generation of the main procedure of GP.

        That is finish the evaluation -> crossover -> mutation -> survivors.
        """
        if not self.is_runnable():
            return
        if self.params.rerun_fitness != 0 and self.num_gen > 1:
            self.fitness = self.evaluate_population(self.population)
        # crossover steps
        co_parents = gp.crossover_parent_selection(
            self.population, self.fitness, self.params, self.selection_pressure)
        co_offspring = gp.crossover(self.population, co_parents, self.params)
        self.fitness += self.evaluate_population(co_offspring)
        # mutation steps
        mutation_parents = gp.mutation_parent_selection(
            self.population,
            self.fitness,
            co_parents,
            co_offspring,
            self.params,
            self.selection_pressure
        )
        mutated_offspring = gp.mutation(
            self.population + co_offspring, mutation_parents, self.params)
        self.fitness += self.evaluate_population(mutated_offspring)

        # find where is the best individual from
        if self.use_tracker:
            best_fitness_idx = self.fitness.index(max(self.fitness))
            if best_fitness_idx <= len(self.population):
                self.best_from_replication_count += 1
            elif best_fitness_idx <= len(self.population) + len(co_offspring):
                self.best_from_crossover_count += 1
            else:
                self.best_from_mutation_count += 1

        # select survivors
        self.population, self.fitness = gp.survivor_selection(
            self.population,
            self.fitness,
            co_offspring,
            mutated_offspring,
            self.params,
            self.selection_pressure
        )

        self.best_fitness.append(max(self.fitness))
        self.n_episodes.append(self.hash_table.n_values)

        self.log_diversity(False)
        logplot.log_fitness(self.my_path, self.fitness)
        logplot.log_population(self.my_path, self.population)

        if self.params.verbose:
            print(
                'Generation: ', self.num_gen,
                'Fitness: ', self.fitness,
                'Best fitness: ', self.best_fitness[self.num_gen]
            )

        if (self.num_gen + 1) % self.params.save_interval == 0 and\
           self.num_gen < self.params.n_generations - 1:
            self.save_state(None)
        self.num_gen += 1

    def get_final_info(self) -> tuple:
        """Return the information about the generated population and their fitness score."""
        gp.print_population(self.population, self.fitness, self.num_gen)

        best_individual = gp.selection(
            self.population, self.fitness, 1, gp.SelectionMethods.ELITISM)[0]

        self.save_state(None)

        if self.use_tracker:
            logplot.log_tracking(
                self.my_path,
                self.exchange_count,
                self.best_from_exchange_count,
                self.best_from_replication_count,
                self.best_from_crossover_count,
                self.best_from_mutation_count
            )
        logplot.log_best_fitness_episodes(self.my_path, self.best_fitness, self.n_episodes)
        if self.params.plot:
            logplot.plot_fitness(self.my_path, self.best_fitness, self.n_episodes)
        if self.params.fig_best:
            self.environment.plot_individual(
                logplot.get_log_folder(self.my_path),
                'best individual',
                best_individual
            )
        if self.params.fig_last_gen:
            for i in range(self.params.n_population):
                self.environment.plot_individual(
                    logplot.get_log_folder(self.my_path),
                    'individual_' + str(i),
                    self.population[i]
                )

        return self.population, self.fitness, self.best_fitness, best_individual

    def get_genotype_diversity(self) -> Tuple[float, float, int]:
        """
        Return three type of diversity that are related to genotype.

        1. normalized edit distance type 1
        2. edit distance type 2
        3. number of unique pseudo_isomorphs 3-tuple

        The diversity measurement based on edit distance is the mean distance
        between the individual with best fitness score and the population.
        """
        best_individual = gp.selection(
            self.population, self.fitness, 1, gp.SelectionMethods.ELITISM)[0]
        best_individual_len = len(best_individual)
        best_individual = BT(best_individual, self.params.behavior_lists)
        ed1_list = []
        ed2_list = []
        pseudo_isomorphs_list = []
        for individual in self.population:
            p = BT(individual, self.params.behavior_lists)
            ed1_list.append(
                best_individual.get_edit_distance(p, 0, 0, k=1)/min(
                    best_individual_len, len(individual)))
            ed2_list.append(
                best_individual.get_edit_distance(p, 0, 0, k=0.5)
            )
            pseudo_isomorphs_list.append(p.get_pseudo_isomorphs_tuple())
        return np.mean(ed1_list), np.mean(ed2_list), len(set(pseudo_isomorphs_list))

    def get_entropy_diversity(self) -> float:
        """
        Compute the probabilities of each fitness score within the population.

        Then computes the entropy based on those probabilities.
        Diversity based on entropy, measures the phenotype diversity.
        """
        _, counts = np.unique(self.fitness, return_counts=True)
        prob = counts / len(self.fitness)
        return - np.sum(prob * np.log(prob))

    def update_selection_pressure(self, diversity: List[str]) -> None:
        """ Updates the selection pressure based on diversity """
        chosen_diversity = self.diversity_types.index(self.params.selection_pressure_diversity)
        diversity_val = diversity[chosen_diversity]
        self.max_diversity = max(self.max_diversity, diversity_val)
        self.min_diversity = min(self.min_diversity, diversity_val)
        self.selection_pressure = \
            (diversity_val-self.min_diversity) / (self.max_diversity - self.min_diversity + 1e-6)

    def log_diversity(self, exchanged: bool) -> None:
        """ Logs the diversity """
        diversity = list(self.get_genotype_diversity())
        diversity.append(self.get_entropy_diversity())
        logplot.log_diversity(self.my_path, diversity, self.num_gen, exchanged)

        # choose one diversity to update selection pressure
        if self.params.parent_selection == gp.SelectionMethods.AD_RANK or \
                self.params.survivor_selection == gp.SelectionMethods.AD_RANK:
            self.update_selection_pressure(diversity)

    def save_state(self, best_individual: Any) -> None:
        """ Save current state """
        logplot.log_last_population(self.my_path, self.population)
        if best_individual:
            logplot.log_best_individual(self.my_path, best_individual)
        logplot.log_best_fitness(self.my_path, self.best_fitness)
        logplot.log_n_episodes(self.my_path, self.n_episodes)
        logplot.log_settings(self.my_path, self.params, None)
        logplot.log_state(self.my_path, random.getstate(), np.random.get_state(), self.num_gen)
        self.hash_table.write_table()
