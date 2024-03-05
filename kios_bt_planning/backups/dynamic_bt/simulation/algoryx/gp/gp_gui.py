"""
Launch the GUI to simulate the GP framework with the AGX Simulator.

Use with: python gp/gp_gui.py --model models/BTDemo.yml:CoarseGripperInLabDR --decorate --timeStep 0.05
from the /algoryx folder
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

import logging
import multiprocessing as mp
import os
import subprocess
import time
from typing import Any, List

from agxBrick.brickLoaderUtils import createArgumentParser
from behaviors.behavior_lists import BehaviorLists
from bt_learning.gp import logplot
import bt_learning.gp.genetic_programming as gp
import simulation.algoryx.agx_application as app
from simulation.algoryx.behaviors.sim_behaviors import RobotBehaviors
from simulation.algoryx.gp import gp_asprocess, gp_environment, gp_interface
from simulation.py_trees_interface import PyTree, PyTreeParameters
from user_interface.gp_gui import GPGUILayout
import yaml


class GP_GUI():

    def __init__(self, args: Any) -> None:
        self._logger = logging.getLogger(__file__)

        self.repo_path = os.path.dirname(os.path.dirname((os.path.abspath(__file__))))
        with open(os.path.join(self.repo_path, 'config/sim_data.yaml')) as f:
            self.sim_data = yaml.safe_load(f)
        with open(os.path.join(self.repo_path, 'config/sim_objects.yaml')) as f:
            self.obj_data = yaml.safe_load(f)
        with open(os.path.join(self.repo_path, 'config/gp_targets.yaml')) as f:
            targets = yaml.safe_load(f)

        self.task_objects = list(self.obj_data.keys())
        self.ref_objects = self.sim_data['demonstration']['reference_frames']
        app.GRAPHICS = False
        self.interface = gp_interface.GPInterface(self.task_objects, self.ref_objects)

        self.agx_GUI = GPGUILayout('Algoryx Simulation for Genetic Programming')

        # bt flags
        self.tree_dir = None
        self.running = False
        self.bt = None
        self.viz = None
        self.bt_tick_freq = self.sim_data['behavior_tree']['tick_freq']

        # GP parameters
        self.gp_par = gp.GpParameters()
        self.gp_par.save_interval = self.sim_data['genetic']['save_interval']
        self.gp_par.ind_start_length = self.sim_data['genetic']['ind_length']
        self.gp_par.n_population = self.sim_data['genetic']['n_population']
        self.gp_par.n_generations = self.sim_data['genetic']['n_generations']
        self.gp_par.f_crossover = self.sim_data['genetic']['p_crossover']
        self.gp_par.n_offspring_crossover = 2
        self.gp_par.replace_crossover = False
        self.gp_par.f_mutation = self.sim_data['genetic']['p_mutation']
        self.gp_par.n_offspring_mutation = 2
        self.gp_par.parent_selection = self.__get_selection_from_data(
            self.sim_data['genetic']['selection'])
        self.gp_par.survivor_selection = self.__get_selection_from_data(
            self.sim_data['genetic']['selection'])
        self.gp_par.f_elites = self.sim_data['genetic']['p_elite']
        self.gp_par.f_parents = self.gp_par.f_elites
        self.gp_par.max_multi_mutation = self.sim_data['genetic']['mutations_per_individual']
        self.gp_par.mutate_co_offspring = False
        self.gp_par.mutate_co_parents = True
        self.gp_par.mutation_p_add = self.sim_data['genetic']['mutation_add']
        self.gp_par.mutation_p_delete = self.sim_data['genetic']['mutation_delete']
        self.gp_par.mutation_p_variable = 0.0
        self.gp_par.mutation_p_replace = 0.0
        self.gp_par.mutation_p_swap = 0.0
        self.gp_par.rerun_fitness = 0
        self.gp_par.allow_identical = False
        self.gp_par.plot = True
        self.gp_par.verbose = True
        self.gp_par.fig_last_gen = False
        self.gp_par.log_name = self.__get_log_number()

        self.args = (app.CloudpickleWrapper(app.Application), args)

        self.gp_path = os.path.join(self.repo_path, 'gp')
        # Create behaviors
        self.gp_par.behavior_lists = self.generate_behaviors()

        self.behaviors = RobotBehaviors(self.gp_path)

        self.gp_env = gp_environment.GPEnvironment(
            self.interface, self.behaviors, targets)

        self.gp_process = None
        self.current_gen = 0

        self.visual = self.sim_data['genetic']['graphics']

    def show(self) -> None:
        """Define the GUI functionalities."""
        self.agx_GUI.initialize_layout()

        window = self.agx_GUI.get_window()

        # Interaction with the GUI Main menu
        ret = 0
        while ret is not None:
            ret, values = window.read()

            if ret == '__folder_display__' and os.path.isdir(values['__folder_display__']):
                try:
                    id = int(values['__folder_display__'][-2:])
                    self.gp_par.log_name = str(id)
                except ValueError:
                    self.gp_par.log_name = values['__folder_display__'][-1]
                self.update_gp_values(window)
                window.find_element('__run__').update(disabled=False)
                window.find_element('__show_tree__').update(disabled=False)

            elif ret == '__start__':
                if not self.running:
                    # disable all elements
                    window.find_element('__start__').update('Stop GP')
                    window.find_element('__run__').update(disabled=True)
                    window.find_element('__show_tree__').update(disabled=True)

                    self.running = True
                    hotstart = True if self.current_gen > 0 else False
                    app.GRAPHICS = self.visual
                    self.gp_process = gp_asprocess.GPProcess(
                        os.path.join(self.repo_path, 'config'),
                        self.gp_env,
                        self.gp_par,
                        hotstart=hotstart,
                        baseline=self.get_baseline(),
                        visual=self.visual,
                        args=self.args
                    )
                    self.gp_process.start()

                else:
                    self.running = False
                    window.find_element('__start__').update('Start GP')
                    window.find_element('__run__').update(disabled=False)
                    window.find_element('__show_tree__').update(disabled=False)

                    self.gp_process.stop()
                    self.gp_process.join()
                    print(f'Stopped at: {self.current_gen}.')
                    self.update_gp_values(window)

            elif ret == '__run__':
                if self.running is False:
                    # disable all elements
                    window.find_element('__start__').update(disabled=True)
                    window.find_element('__show_tree__').update(disabled=True)

                    window.find_element('__run__').update('Stop')
                    print('Launching tree...')
                    self.running = True

                    app.GRAPHICS = True
                    self.bt_process = app.RunBT(
                        self.interface,
                        self.obj_data,
                        self.behaviors,
                        os.path.join(self.repo_path, f'logs/log_{self.gp_par.log_name}'),
                        self.bt_tick_freq,
                        self.args
                    )
                    self.bt_process.start()

                else:
                    self.running = False
                    self.bt_process.stop()
                    self.bt_process.join()
                    window.find_element('__run__').update('Run')
                    window.find_element('__start__').update(disabled=False)
                    window.find_element('__show_tree__').update(disabled=False)

            elif ret == '__show_tree__':
                path_to_best = os.path.join(
                    self.repo_path, f'logs/log_{self.gp_par.log_name}/best_individual.svg')
                if os.name == 'nt':  # Windows
                    os.startfile(path_to_best)
                else:
                    opener = 'xdg-open'
                    subprocess.call([opener, path_to_best])

    def update_gp_values(self, window) -> None:
        try:
            self.current_gen = logplot.get_state(self.gp_par.log_name)[2]
            self.best_fitness = max(logplot.get_best_fitness(self.gp_par.log_name))
            self.best_individual = logplot.get_best_individual(self.gp_par.log_name)

            window.find_element('__gen__').update(
                'Current generation: %d' % self.current_gen)
            window.find_element('__fitness__').update(
                'Best fitness score: %d' % self.best_fitness)

            proc = mp.Process(target=self._bt_worker, args=self.args)
            proc.start()
            time.sleep(5)
            print('Behavior Tree generated!')
        except AttributeError:
            window.find_element('__gen__').update(
                'Current generation: %d' % self.current_gen)
            window.find_element('__fitness__').update(
                'Best fitness score: -inf')

    def get_baseline(self) -> List[str] or None:
        """Get the baseline from the given folder."""
        try:
            best_individual = logplot.get_best_individual(self.gp_par.log_name)
            return best_individual
        except (FileNotFoundError, UnicodeDecodeError):
            print('Error in loading the baseline.')
            return None

    def generate_behaviors(self) -> BehaviorLists or None:
        """Generate behaviors for the GP parameters."""
        proc = mp.Process(target=self._btlist_worker, args=self.args)
        proc.start()
        time.sleep(5)
        print('Behaviors generated!')
        try:
            with open(os.path.join(self.gp_path, 'BT_SETTINGS.yaml')) as f:
                settings = yaml.safe_load(f)
                behavior_list = BehaviorLists(
                    settings['fallback_nodes'],
                    settings['atomic_fallback_nodes'],
                    settings['sequence_nodes'],
                    settings['atomic_sequence_nodes'],
                    settings['condition_nodes'],
                    settings['action_nodes']
                )
            return behavior_list
        except FileNotFoundError:
            print('Error in loading the behavior lists.')
            return None

    def _bt_worker(self, env_fn_wrapper: app.CloudpickleWrapper, args: List[str]) -> None:
        app = env_fn_wrapper.var(args)
        app.bringup(self.obj_data, visual=False)
        self.interface.initialize()
        parameters = PyTreeParameters()
        parameters.behavior_lists = self.behaviors.get_behavior_list()
        parameters.behaviors = self.behaviors
        bt = PyTree(
            self.best_individual, parameters, world_interface=self.interface)
        with open(
                os.path.join(
                    self.repo_path, f'logs/log_{self.gp_par.log_name}/tree.yaml'), 'w') as f:
            yaml.dump(bt.bt.bt, f)
        app.shutdown()

    def _btlist_worker(self, env_fn_wrapper: app.CloudpickleWrapper, args: List[str]) -> None:
        app = env_fn_wrapper.var(args)
        app.bringup(self.obj_data, visual=False)
        actions = self.sim_data['demonstration']['grasping_actions'] + \
            self.sim_data['demonstration']['placing_actions']
        # _ = self.interface.generate_behavior_list(self.gp_path, actions)
        _ = self.interface.generate_pickplace_list(self.gp_path)
        app.shutdown()

    def __get_selection_from_data(self, data: str) -> gp.SelectionMethods:
        """Read the string and return correct type."""
        if data == 'tournament':
            return gp.SelectionMethods.TOURNAMENT
        elif data == 'rank':
            return gp.SelectionMethods.RANK
        elif data == 'elitism':
            return gp.SelectionMethods.ELITISM
        elif data == 'random':
            return gp.SelectionMethods.RANDOM

    def __get_log_number(self) -> str:
        """Return a name for the log folder that is not currently used."""
        number = 1
        while os.path.isdir(os.path.join(self.repo_path, f'logs/log_{number}')):
            number += 1

        return str(number)


def main() -> None:
    parser, args, leftover_args = createArgumentParser()

    gui = GP_GUI(args)
    gui.show()


if __name__ == '__main__':
    main()
