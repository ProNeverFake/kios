"""Application wrapper to handle interaction with AGX."""

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

import os
import time
import cloudpickle
import logging
import multiprocessing as mp
from typing import Any, Dict, List

from agxClick import AgxApplication
import yaml
from bt_learning.learning_from_demo.debug import BTVisualizer
from simulation.algoryx import agx_environment
from simulation.py_trees_interface import PyTree, PyTreeParameters

GRAPHICS = True
DUMP = False


class CloudpickleWrapper:
    """
    Uses cloudpickle to serialize contents (otherwise multiprocessing tries to use pickle)

    :param var: the variable you wish to wrap for pickling with cloudpickle
    """

    def __init__(self, var: Any):
        self.var = var

    def __getstate__(self) -> Any:
        return cloudpickle.dumps(self.var)

    def __setstate__(self, var: Any) -> None:
        self.var = cloudpickle.loads(var)


class Application(AgxApplication):
    def __init__(self, args: Any) -> None:
        if GRAPHICS:
            super().__init__(args)
        else:
            super().__init__(['-a'])
        self._logger = logging.getLogger(__file__)
        self._env = None
        self.args = args

    def bringup(
        self,
        data: Dict,
        target: Dict = None,
        visual: bool = False,
        debug: bool = False
    ) -> None:
        """Initialize the Environment."""
        self.__init_agx_environment(data, target, visual, debug)
        if DUMP:
            print('Dumping AGX state')
            self._env.sim.write('bringup.agx')
        self.__step_ahead(1)

    def reset(self, target: Dict = None) -> None:
        """Reset the simulation."""
        self._env.reset_simulation()
        if target is not None:
            self._env.apply_configuration(target)
        self.__step_ahead(1)

    def shutdown(self) -> None:
        """Shut down the Environment."""
        self._env.shutdown()
        del self._env
        self._env = None
        self.stop()
        print('Environment DOWN!')

    def step(self, time: float):
        self.__step_ahead(time)

    def start_video(self):
        pass

    def stop_video(self):
        pass

    def __init_agx_environment(
        self,
        data: Dict,
        target: Dict = None,
        visual: bool = False,
        debug: bool = False
    ) -> None:
        """Initialize AGX Environment."""
        env = agx_environment.AGXEnvironment(self.args, objects=data, debug=debug)

        brickSimulation = env.build_scene()
        env.spawn(brickSimulation)
        if target is not None:
            env.apply_configuration(target)
        if visual:
            env.create_visual()
        self._env = env

    def __step_ahead(self, time: float):
        """Step forward in time."""
        t1 = self._env.sim.getTimeStamp()
        while self._env.sim.getTimeStamp() - t1 < time:
            self.stepApplication()


class RunBT(mp.Process):
    def __init__(
        self,
        interface: Any,
        data: Dict,
        behaviors: Any,
        bt: str or List[str],
        tick_freq: int,
        args
    ):

        self.interface = interface
        self.data = data
        self.behaviors = behaviors
        self.bt = bt
        self.bt_tick_freq = tick_freq

        self.event = mp.Event()

        super().__init__(target=self.worker, args=args)

    def stop(self):
        self.event.set()
        self.join()

    def worker(self, env_fn_wrapper: CloudpickleWrapper, args: List[str]):
        app = env_fn_wrapper.var(args)
        app.bringup(self.data, visual=True)
        self.interface.initialize()

        if type(self.bt) is str:
            with open(os.path.join(self.bt, 'tree.yaml')) as f:
                string_tree = yaml.safe_load(f)
        else:
            string_tree = self.bt

        parameters = PyTreeParameters()
        parameters.behavior_lists = self.behaviors.get_behavior_list()
        parameters.behaviors = self.behaviors
        parameters.max_ticks = 15
        parameters.max_time = 30
        bt = PyTree(
            string_tree, parameters, world_interface=self.interface)

        viz = BTVisualizer(bt)

        self.interface.remove_locks()
        self.interface.preempt_skill()
        self.interface.add_merger()

        self.interface.set_gripper('open')

        print('Launching tree...')
        for _ in range(25):
            if self.event.is_set():
                app.shutdown()
                break
            viz.tick()
            current_time = app.app.getTimeStamp()
            while app.app.getTimeStamp() < 2.5 + current_time:
                app.stepApplication()
