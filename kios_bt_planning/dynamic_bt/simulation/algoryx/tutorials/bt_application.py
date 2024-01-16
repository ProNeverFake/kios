"""Behavior Tree Application to run the AGX Simulation."""

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
from typing import Dict, List

import agx
from agxClick import AgxApplication
from simulation.algoryx import agx_environment
import py_trees as pt


class BTApplication(AgxApplication):

    def __init__(self, args: List[str]) -> None:
        super().__init__(args)
        self._logger = logging.getLogger(__file__)
        self._env = None

        self.buffer = []
        self.last_status = pt.common.Status.INVALID

    def init_agx_environment(self, args: List[str], data: Dict) -> None:
        """Initialize AGX Environment."""
        env = agx_environment.AGXEnvironment(args, objects=data)

        brickSimulation = env.build_scene()
        env.spawn(brickSimulation)
        env.create_visual()
        self._env = env

    def run_bt(self, behavior_tree: pt.trees.BehaviourTree, freq_ratio: int = 50) -> None:
        """
        Run episode by ticking the Behavior Tree.

        Args
        ----
        behavior_tree: the BT to run.
        freq_ratio: how many simulation steps per BT tick.

        """
        done = False
        freq_ratio = 50 if freq_ratio < 50 else freq_ratio
        ticks = 0
        while not done:
            ticks += 1
            print(f'Current tick: {ticks}')
            # print(behavior_tree.root.status)
            behavior_tree.root.tick_once()
            # Add some little delay
            for _ in range(freq_ratio):
                self.stepApplication()

            if self.bt_done(behavior_tree, 4):
                done = True
                print(f'Ticks: {ticks}')

    def bt_done(self, behavior_tree: pt.trees.BehaviourTree, consecutive: int = 2) -> bool:
        """Return True if the BT returns N consecutive SUCCESS or FAILURE."""
        is_failing = True if behavior_tree.root.status is pt.common.Status.FAILURE else False
        is_successful = True if behavior_tree.root.status is pt.common.Status.SUCCESS else False
        done = False
        if behavior_tree.root.status == self.last_status and is_successful:
            self.buffer.append(behavior_tree.root.status)
            if len(self.buffer) >= consecutive:
                done = all(self.buffer)
        else:
            self.buffer = []

        self.last_status = behavior_tree.root.status
        return done

    def reset(self) -> None:
        self._env.reset_simulation()
