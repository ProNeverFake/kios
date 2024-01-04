"""
This script is to demo the Brick models in the file `models/BTDemo.yml`

To test the automatic spawning of items run:
`python tutorials/run_interaction.py --model models/BTDemo.yml:CoarseGripperInLabDR --decorate --timeStep 0.05`
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

import agx
from agxBrick.brickLoaderUtils import createArgumentParser
from agxPythonModules.utils.environment import init_app, simulation
from agxPythonModules.utils.callbacks import StepEventCallback as sec

from simulation.algoryx import agx_environment
from simulation.algoryx.behaviors import agx_interface
from simulation.algoryx.lfd import user_interaction


import os
import yaml


def buildScene() -> None:
    repo_path = os.path.dirname(os.path.dirname((os.path.abspath(__file__))))
    with open(os.path.join(repo_path, 'config/sim_objects.yaml')) as f:
        data = yaml.safe_load(f)

    env = agx_environment.AGXEnvironment(args, objects=data)

    brickSimulation = env.build_scene()
    env.spawn(brickSimulation)
    env.create_visual()

    behaviors = agx_interface.AGXInterface(list(data.keys()))

    user_input = user_interaction.UserControls(behaviors)
    user_input.setup_keyboard_listener()

    brickSimulation.ConnectToROS()


try:
    args
except:
    args = None

if args is None:
    parser, args, leftover_args = createArgumentParser()


init = init_app(
    name=__name__,
    scenes=[('buildScene', '1')],
    autoStepping=True
)
