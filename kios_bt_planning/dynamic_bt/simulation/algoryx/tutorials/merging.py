"""
This tutorial merges two bodies in the simulation for faster manipulation.

Use with: agxViewer tutorials/merging.py
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
import agxOSG
import agxCollide

from agxPythonModules.utils.environment import init_app, simulation, application, root
from agxPythonModules.utils.callbacks import KeyboardCallback as kc


def buildScene():
    """Merge with right arrow key and split with left arrow key."""
    ground = agxCollide.Geometry(agxCollide.Box(10, 10, 1))
    ground.setPosition(0, 0, -1)
    simulation().add(ground)

    box1 = agx.RigidBody(agxCollide.Geometry(agxCollide.Box(1, 1, 1)))
    box1.setPosition(-1.4, 0, 1)
    simulation().add(box1)

    box2 = agx.RigidBody(agxCollide.Geometry(agxCollide.Box(1, 1, 1)))
    box2.setPosition(1.4, 0, 1)
    simulation().add(box2)

    mb = agx.MergedBody()
    simulation().add(mb)

    def merge(t):
        mb.add(agx.MergedBodyEmptyEdgeInteraction(box1, box2))

    def split(t):
        mb.remove(box1)

    kc.bind(
        name='merge',
        key=kc.KEY_Right,
        mode=kc.Mode.NATIVE,
        callback=merge
    )
    kc.bind(
        name='split',
        key=kc.KEY_Left,
        mode=kc.Mode.NATIVE,
        callback=split
    )

    application().setEnableDebugRenderer(True)


init_app(
    name=__name__,
    scenes=['buildScene', '1']
)
