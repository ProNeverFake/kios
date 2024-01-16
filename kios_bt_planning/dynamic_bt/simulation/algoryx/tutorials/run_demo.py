"""
This script is to demo the Brick models in the file `models/BTDemo.yml`

To test the automatic spawning of items run:
`python tutorials/run_demo.py --model models/BTDemo.yml:CoarseGripperInLabDR --decorate --timeStep 0.05`
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
from typing import Dict, List

import agx
from agxBrick.brickLoaderUtils import createArgumentParser
from agxClick import AgxApplication
from agxPythonModules.utils.environment import init_app, simulation
from agxPythonModules.utils.callbacks import StepEventCallback as sec
import numpy as np
from simulation.algoryx import agx_environment
from simulation.algoryx.behaviors import agx_interface

import os
import yaml


class RunDemo(AgxApplication):

    def __init__(self, args: List[str]) -> None:
        super().__init__(args)
        self._logger = logging.getLogger(__file__)
        self._env = None

    def init_agx_environment(self, args: List[str], data: Dict) -> None:
        """Initialize AGX Environment."""
        env = agx_environment.AGXEnvironment(args, objects=data)

        brickSimulation = env.build_scene()
        env.spawn(brickSimulation)
        env.create_visual()
        self._env = env

    def reset(self) -> None:
        self._env.reset_simulation()

    def run_episode(self, behaviors) -> None:
        behaviors.pick('YellowBox')
        self.__step_ahead(3.0)
        behaviors.place('YellowBox', 'KittingBox', agx.Vec3(0, 0, 0.12))
        self.__step_ahead(3.0)
        behaviors.pick('GreenBox')
        self.__step_ahead(3.0)
        behaviors.place('GreenBox', 'YellowBox', agx.Vec3(0, 0, 0.18))
        self.__step_ahead(3.0)
        behaviors.pick('BlueBox')
        self.__step_ahead(3.0)
        behaviors.place('BlueBox', 'GreenBox', agx.Vec3(0, 0, 0.18))
        self.__step_ahead(3.0)

    def __step_ahead(self, time: float):
        """Step forward in time."""
        t1 = self._env.sim.getTimeStamp()
        while self._env.sim.getTimeStamp() - t1 < time:
            self.stepApplication()


def print_poses(interface: agx_interface.AGXInterface):
    y_pos, _ = interface.get_item_in_frame('YellowBox', 'KittingBox')
    g_pos, _ = interface.get_item_in_frame('GreenBox', 'YellowBox')
    b_pos, _ = interface.get_item_in_frame('BlueBox', 'GreenBox')
    print('Yellow:', y_pos)
    print('Green:', g_pos)
    print('Blue:', b_pos)


def main() -> None:
    parser, args, leftover_args = createArgumentParser()
    repo_path = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    with open(os.path.join(repo_path, 'config/sim_objects.yaml')) as f:
        data = yaml.safe_load(f)

    demo = RunDemo(args)
    demo.init_agx_environment(args, data)

    print(demo._env.get_bodies())
    w_T_yb = demo._env.sim.getRigidBody('YellowBox').getTransform()
    pos_yb = w_T_yb.getTranslate()
    w_T_ee = demo._env.sim.getRigidBody('GripperControlBody').getTransform()
    pos_ee = w_T_ee.getTranslate()

    # method1
    print(pos_ee - pos_yb)
    # method2
    yb_T_ee = w_T_yb.inverse()*w_T_ee
    print(yb_T_ee.getTranslate())

    # KitBox geometry
    bottom_plate = demo._env.sim.getRigidBody('KittingBox').getGeometries()[0]
    print(bottom_plate.getShape().asBox().getHalfExtents())

    behaviors = agx_interface.AGXInterface(list(data.keys()))

    for _ in range(2):
        demo.run_episode(behaviors)
        print_poses(behaviors)
        demo.reset()

    # del demo
    # demo = None


if __name__ == "__main__":
    main()
