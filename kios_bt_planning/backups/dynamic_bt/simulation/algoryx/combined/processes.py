"""Script to collect process classes and workers for the GP+LfD framework."""

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
from typing import Any, Dict, List

import bt_learning.learning_from_demo.render_tree as tree_render
from py_trees.display import render_dot_tree
from simulation.algoryx.agx_application import CloudpickleWrapper
from simulation.algoryx.gp.gp_environment import GPEnvironment
from simulation.py_trees_interface import PyTree, PyTreeParameters
import yaml


def btlist_worker(
    env_fn_wrapper: CloudpickleWrapper,
    agx_args: List[str],
    obj_data: Dict,
    sim_data: Dict,
    target_dir: str,
    interface: Any
) -> None:
    app = env_fn_wrapper.var(agx_args)
    app.bringup(obj_data, visual=False)
    actions = sim_data['demonstration']['grasping_actions'] +\
        sim_data['demonstration']['placing_actions']
    random_bringup = sim_data['algoryx']['random_bringup']
    # _ = interface.generate_behavior_list(target_dir, actions)
    _ = interface.generate_pickplace_list(target_dir, random_bringup)
    app.shutdown()


def bt_worker(
    env_fn_wrapper: CloudpickleWrapper,
    agx_args: List[str],
    target_directory: str,
    string_tree: str,
    data: Dict,
    behaviors: Any,
    interface: Any
) -> None:
    app = env_fn_wrapper.var(agx_args)
    app.bringup(data, visual=False)
    interface.initialize()
    parameters = PyTreeParameters()
    parameters.behavior_lists = behaviors.get_behavior_list()
    parameters.behaviors = behaviors
    bt = PyTree(
        string_tree, parameters, world_interface=interface)

    with open(
            os.path.join(target_directory, 'tree.yaml'), 'w') as f:
        yaml.dump(bt.bt.bt, f)
    app.shutdown()


def bt_saver(
    conn: mp.connection.Connection,
    env_fn_wrapper: CloudpickleWrapper,
    agx_args: List[str],
    target_directory: str,
    string_tree: str,
    data: Dict,
    behaviors: Any,
    interface: Any
) -> None:
    app = env_fn_wrapper.var(agx_args)
    app.bringup(data, visual=False)
    interface.initialize()
    parameters = PyTreeParameters()
    parameters.behavior_lists = behaviors.get_behavior_list()
    parameters.behaviors = behaviors
    bt = PyTree(
        string_tree, parameters, world_interface=interface)

    render_dot_tree(
        bt.root,
        name='debug_pytree',
        target_directory=target_directory,
        with_blackboard_variables=True
    )

    if conn.poll():
        data = conn.recv()
        conn.send(bt.bt.length())

    tree_render.dot_graph(bt).write_svg(
        os.path.join(target_directory, 'full.svg'), encoding='utf-8')
    positions = tree_render.write_tikz_tree(
        bt, os.path.join(target_directory, 'full.tex'))
    tree_render.py_trees_dot(bt).write_svg(
        os.path.join(target_directory, 'full_pytrees.svg'), encoding='utf-8')

    with open(os.path.join(target_directory, 'positions.yaml'), 'w') as f:
        yaml.dump(positions, f)

    print('BT data saved!')
    app.shutdown()


class SimProcess(mp.Process):
    def __init__(self, args: Any = None) -> None:
        self.event = mp.Event()
        super().__init__(target=self.worker, args=args)

    def stop(self) -> None:
        self.event.set()
        self.join()

    def worker(
        self,
        env_fn_wrapper: CloudpickleWrapper,
        agx_args: List[str],
        data: Dict,
        target: Dict = None
    ) -> None:
        """Show the simulation environment."""
        app = env_fn_wrapper.var(agx_args)
        app.bringup(data, target, True)
        while True:
            if self.event.is_set():
                app.shutdown()
                break
            continue


class GetFitness(mp.Process):
    def __init__(
        self,
        environment: GPEnvironment,
        args: Any = None
    ) -> None:
        self.data = mp.Value('d', float('-inf'))
        self.env = environment
        super().__init__(target=self.worker, args=args)

    def get_value(self) -> float:
        return self.data.value

    def worker(
        self,
        env_fn_wrapper: CloudpickleWrapper,
        agx_args: List[str],
        data: Dict,
        individual: List[str]
    ) -> None:
        """Show the simulation environment."""
        app = env_fn_wrapper.var(agx_args)
        app.bringup(data, visual=False)
        self.env.set_application(app)

        self.data.value = self.env.get_fitness(individual)
