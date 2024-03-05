"""Define the objects in the AGX simulaiton."""

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

from typing import Any, Dict, List, Tuple

from simulation.py_trees_interface import PyTree, PyTreeParameters
from simulation.algoryx.agx_application import Application
from simulation.algoryx.behaviors.agx_interface import AGXInterface
from simulation.algoryx.gp import fitness_function as fit
import time


class GPEnvironment():

    def __init__(
        self,
        world_interface: AGXInterface,
        behaviors: Any,
        initial: Dict,
        targets: Dict,
        fitness_coeff: fit.Coefficients = None,
        pytrees_param: PyTreeParameters = None,
        verbose: bool = False
    ) -> None:
        """
        Interface to the simulation environment for the GP algorithm.

        Args:
        ----
            - world_interface: interface to the simulation environment where skills are defined
            - behaviors: behaviors that allow the robot to act on the environment
            - initial: initial configuration of the objects in the environment
            - targets: target configuration of the objects in the environment
            - fitness_coeff: coefficient to compute the fitness function
            - pytrees_param: parameters to compute BT-related weights in the fitness function
            - verbose: toggle printouts
        """

        self.world_interface = world_interface
        self.behaviors = behaviors
        self.initial = initial
        self.targets = targets
        self.verbose = verbose
        self.fitness_coeff = fitness_coeff
        self.pytrees_param = pytrees_param
        if self.pytrees_param is not None:
            self.pytrees_param.verbose = verbose

        self.app = None

    def set_application(self, application: Application) -> None:
        """Set the AGX application to allow control of the simulator."""
        self.app = application

    def get_fitness(self, individual: List[str], seed: int = None) -> float:
        """
        Run the simulation and return the fitness.

        In case of error, restarts world_interface and tries again.
        """
        self.world_interface.initialize()
        if self.pytrees_param is None:
            self.pytrees_param = PyTreeParameters()
            self.pytrees_param.behavior_lists = self.behaviors.get_behavior_list()
            self.pytrees_param.behaviors = self.behaviors
            self.pytrees_param.max_ticks = 15
            self.pytrees_param.max_time = 30
        pytree = PyTree(
            individual[:], parameters=self.pytrees_param, world_interface=self.world_interface)

        status_ok = True
        fitness = None

        self.app.reset(self.initial)
        while fitness is None:
            if self.verbose:
                self.app.start_video()

            ticks, _ = pytree.run_bt(simulator=self.app)

            if self.verbose:
                time.sleep(1)
                self.app.stop_video()

            fitness = fit.compute_fitness(
                self.world_interface,
                pytree,
                ticks,
                self.targets,
                self.fitness_coeff,
                self.verbose
            )

        return fitness

    def plot_individual(self, path: str, plot_name: str, individual: List[str]) -> None:
        """Saves a graphical representation of the individual"""
        if self.pytrees_param is None:
            self.pytrees_param = PyTreeParameters()
            self.pytrees_param.behavior_lists = self.behaviors.get_behavior_list()
            self.pytrees_param.behaviors = self.behaviors
        pytree = PyTree(
            individual[:], parameters=self.pytrees_param, world_interface=self.world_interface)
        pytree.save_fig(path, name=plot_name)
