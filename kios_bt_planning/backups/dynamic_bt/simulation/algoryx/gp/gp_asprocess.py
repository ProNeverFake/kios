"""A genetic programming algorithm with many possible settings."""

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

import multiprocessing as mp
import os
from typing import Any, List

from bt_learning.gp import logplot
import bt_learning.gp.genetic_programming as gp
from bt_learning.gp.hash_table import HashTable
from simulation.algoryx.agx_application import CloudpickleWrapper
import yaml


class GPProcess(mp.Process):

    def __init__(
        self,
        config_folder: str,
        environment: Any,
        gp_par: gp.GpParameters,
        hotstart: bool = False,
        baseline: Any = None,
        baseline_index: int = None,
        visual: bool = False,
        args: Any = None
    ) -> None:

        with open(os.path.join(config_folder, 'sim_data.yaml')) as f:
            sim_data = yaml.safe_load(f)
            self.ref_objects = sim_data['demonstration']['reference_frames']
        with open(os.path.join(config_folder, 'sim_objects.yaml')) as f:
            self.obj_data = yaml.safe_load(f)

        self.gp_par = gp_par
        self.hotstart = hotstart
        self.baseline = baseline
        self.baseline_index = baseline_index
        self.visual = visual

        self.event = mp.Event()
        self.data = mp.Value('i', 0)

        self.envinronment = environment

        super().__init__(target=self.worker, args=args)

    def get_current(self):
        return self.data.value

    def stop(self):
        self.event.set()
        self.join()

    def worker(self, env_fn_wrapper: CloudpickleWrapper, args: List[str]) -> None:
        # pylint: disable=too-many-statements, too-many-locals, too-many-branches
        """Run the genetic programming algorithm."""
        app = env_fn_wrapper.var(args)
        app.bringup(self.obj_data, visual=self.visual)
        self.envinronment.set_application(app)

        hash_table = HashTable(self.gp_par.hash_table_size, self.gp_par.log_name)

        if self.hotstart:
            best_fitness, n_episodes, last_generation, population =\
                gp.load_state(self.gp_par.log_name, hash_table)
        else:
            population = gp.create_population(
                self.gp_par.n_population,
                self.gp_par.ind_start_length,
                self.gp_par.mutation_p_leaf,
                self.gp_par.behavior_lists
            )
            # logplot.clear_logs(self.gp_par.log_name)
            best_fitness = []
            n_episodes = []
            n_episodes.append(hash_table.n_values)
            last_generation = 0

            if self.baseline is not None:
                population[0] = self.baseline
                baseline_index = 0

        fitness = []
        for individual in population:
            if self.gp_par.verbose:
                print(f'Individual: {individual}')
            fitness.append(gp.get_fitness(
                individual,
                hash_table,
                self.envinronment,
                rerun_fitness=0,
                min_episodes=self.gp_par.min_episodes
            ))
            if self.gp_par.verbose:
                print(f'Fitness: {fitness[-1]}')

        if not self.hotstart:
            best_fitness.append(max(fitness))

            if self.gp_par.verbose:
                gp.print_population(population, fitness, last_generation)
                print('\n')
                print('Generation: ', last_generation, ' Best fitness: ', best_fitness[-1])
                print('\n')

            logplot.log_fitness(self.gp_par.log_name, fitness)
            logplot.log_population(self.gp_par.log_name, population)

        generation = self.gp_par.n_generations - 1  # In case loop is skipped due to hotstart
        for generation in range(last_generation + 1, self.gp_par.n_generations):
            self.data.value = generation
            if self.gp_par.keep_baseline:
                if self.baseline is not None and self.baseline not in population:
                    # Make sure we are able to source from baseline
                    population.append(self.baseline)

            if generation > 1:
                fitness = []
                for index, individual in enumerate(population):
                    if self.gp_par.verbose:
                        print(f'Individual: {individual}')
                    fitness.append(gp.get_fitness(
                        individual,
                        hash_table,
                        self.envinronment,
                        self.gp_par.rerun_fitness,
                        self.gp_par.min_episodes
                    ))
                    if self.gp_par.verbose:
                        print(f'Fitness: {fitness[-1]}')
                    if self.baseline is not None and individual == self.baseline:
                        baseline_index = index
            if self.gp_par.keep_baseline and self.gp_par.boost_baseline and self.baseline is not None:
                baseline_fitness = fitness[baseline_index]
                fitness[baseline_index] = max(fitness)

            co_parents = gp.crossover_parent_selection(population, fitness, self.gp_par)
            co_offspring = gp.crossover(
                population, co_parents, self.gp_par, self.baseline, self.baseline_index)
            for offspring in co_offspring:
                if self.gp_par.verbose:
                    print(f'Individual: {offspring}')
                fitness.append(gp.get_fitness(
                    offspring,
                    hash_table,
                    self.envinronment,
                    self.gp_par.rerun_fitness,
                    self.gp_par.min_episodes
                ))
                if self.gp_par.verbose:
                    print(f'Fitness: {fitness[-1]}')

            if self.gp_par.boost_baseline and self.gp_par.boost_baseline_only_co and \
                    self.baseline is not None:
                # Restore original fitness for survivor selection
                fitness[baseline_index] = baseline_fitness

            mutation_parents = gp.mutation_parent_selection(
                population, fitness, co_parents, co_offspring, self.gp_par)
            mutated_offspring = gp.mutation(
                population + co_offspring, mutation_parents, self.gp_par)
            for offspring in mutated_offspring:
                if self.gp_par.verbose:
                    print(f'Individual: {offspring}')
                fitness.append(gp.get_fitness(
                    offspring,
                    hash_table,
                    self.envinronment,
                    self.gp_par.rerun_fitness,
                    self.gp_par.min_episodes
                ))
                if self.gp_par.verbose:
                    print(f'Fitness: {fitness[-1]}')

            if self.gp_par.boost_baseline and self.baseline is not None:
                # Restore original fitness for survivor selection
                fitness[baseline_index] = baseline_fitness

            population, fitness = gp.survivor_selection(
                population, fitness, co_offspring, mutated_offspring, self.gp_par)

            best_fitness.append(max(fitness))
            n_episodes.append(hash_table.n_values)

            logplot.log_fitness(self.gp_par.log_name, fitness)
            logplot.log_population(self.gp_par.log_name, population)

            if self.gp_par.verbose:
                print('\n')
                print(
                    'Generation: ', generation,
                    ' Fitness: ', fitness,
                    ' Best fitness: ', best_fitness[generation]
                )
                print('\n')

            if self.event.is_set():
                print('\n')
                print(
                    'Generation: ', generation,
                    ' Fitness: ', fitness,
                    ' Best fitness: ', best_fitness[generation]
                )
                print('\n')
                gp.save_state(
                    self.gp_par,
                    population,
                    None,
                    best_fitness,
                    n_episodes,
                    self.baseline,
                    generation,
                    hash_table
                )
                best_individual = gp.selection(
                    population, fitness, 1, gp.SelectionMethods.ELITISM)[0]
                self.__make_plots(n_episodes, population, best_individual, best_fitness)
                app.shutdown()
                break

            if (generation + 1) % self.gp_par.save_interval == 0 and\
                    generation < self.gp_par.n_generations - 1:  # Last is saved later
                gp.save_state(
                    self.gp_par,
                    population,
                    None,
                    best_fitness,
                    n_episodes,
                    self.baseline,
                    generation,
                    hash_table
                )

        print('\nFINAL POPULATION: ')
        gp.print_population(population, fitness, generation)

        best_individual = gp.selection(population, fitness, 1, gp.SelectionMethods.ELITISM)[0]

        gp.save_state(
            self.gp_par,
            population,
            best_individual,
            best_fitness,
            n_episodes,
            self.baseline,
            generation,
            hash_table
        )

        self.__make_plots(n_episodes, population, best_individual, best_fitness)

    def __make_plots(self, n_episodes, population, best_individual, best_fitness):
        if self.gp_par.plot:
            logplot.plot_fitness(self.gp_par.log_name, best_fitness, n_episodes)
        if self.gp_par.fig_best:
            self.envinronment.plot_individual(
                logplot.get_log_folder(self.gp_par.log_name),
                'best individual',
                best_individual
            )
        if self.gp_par.fig_last_gen:
            for i in range(self.gp_par.n_population):
                self.envinronment.plot_individual(
                    logplot.get_log_folder(self.gp_par.log_name),
                    'individual_' + str(i),
                    population[i]
                )
