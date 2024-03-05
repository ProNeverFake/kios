"""
This script is to demo the Brick models in the file `models/BTDemo.yml`

To test the automatic spawning of items run:
`python tutorials/run_bt_demo.py --model models/BTDemo.yml:CoarseGripperInLabDR --decorate --timeStep 0.05`
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

import functools
import os
from typing import Any
import yaml

import agx
from agxBrick.brickLoaderUtils import createArgumentParser
from simulation.algoryx.behaviors.agx_interface import AGXInterface
from behaviors.common_behaviors import RSequence
from simulation.algoryx.tutorials.bt_application import BTApplication
from simulation.algoryx.tutorials import bt_wrapper as bt
import numpy as np
import py_trees


def post_tick_handler(snapshot_visitor, behavior_tree) -> None:
    """Prints an ascii tree with the current snapshot status."""
    print(
        '\n'
        + py_trees.display.unicode_tree(
            root=behavior_tree.root,
            visited=snapshot_visitor.visited,
            previously_visited=snapshot_visitor.visited,
        )
    )
    name_ = 'root' + str(behavior_tree.count)


def pick_and_place_BT(
    target: str,
    reference: str,
    pose: agx.Vec3 or np.ndarray,
    world_interface: Any
) -> py_trees.composites.Selector or py_trees.composites.Sequence:
    """Behavior Tree template for pick and place."""
    pick_box = py_trees.composites.Selector(name='Fallback')
    pick_box.add_children(
        [
            bt.InGripper(
                name=f'{target} Grasped?',
                world_interface=world_interface,
                held_object=target,
            ),
            bt.PickBehavior(
                name=f'Pick {target}!',
                world_interface=world_interface,
                target_object=target,
            ),
        ]
    )

    place_box = RSequence(name='Sequence')
    place_box.add_children(
        [
            pick_box,
            bt.PlaceBehavior(
                name=f'Place {target}!',
                world_interface=world_interface,
                target_object=target,
                reference_object=reference,
                pose=pose,
            ),
        ]
    )

    task = py_trees.composites.Selector(name='Fallback')
    task.add_children(
        [
            bt.AtPose(
                name=f'{target} on {reference}?',
                world_interface=world_interface,
                target=target,
                reference=reference,
                pose=pose,
            ),
            place_box
        ]
    )

    return task


class BTDemo:
    def __init__(self, world_interface: Any) -> None:

        self.world_interface = world_interface
        self.root = None

    def build_tree(self) -> None:
        """Hand code the BT."""

        box1 = pick_and_place_BT(
            'YellowBox', 'KittingBox', agx.Vec3(0, 0, 0.12), self.world_interface)
        box2 = pick_and_place_BT(
            'GreenBox', 'YellowBox', agx.Vec3(0, 0, 0.18), self.world_interface)
        box3 = pick_and_place_BT(
            'BlueBox', 'GreenBox', agx.Vec3(0, 0, 0.18), self.world_interface)

        self.root = RSequence(name='Sequence')
        self.root.add_children([box1, box2, box3])
        self.tree = py_trees.trees.BehaviourTree(self.root)

    def get_root(self) -> py_trees.composites.Selector:
        return self.root

    def visualize(self, bt_name: str) -> None:
        """Compute the number of nodes and transition in a BT and save it as figure."""
        py_trees.display.render_dot_tree(
            self.tree.root,
            name=bt_name,
            target_directory=os.path.dirname(os.path.abspath(__file__))
        )

    def run(self, simulation_app: BTApplication) -> None:
        """The BT execution is visualized in the terminal."""
        self.tree.visitors.append(py_trees.visitors.DebugVisitor())
        snapshot_visitor = py_trees.visitors.SnapshotVisitor()
        self.tree.add_post_tick_handler(
            functools.partial(post_tick_handler, snapshot_visitor)
        )
        self.tree.visitors.append(snapshot_visitor)
        self.tree.setup(timeout=15)

        self.world_interface.remove_locks()
        self.world_interface.preempt_skill()
        self.world_interface.add_merger()

        simulation_app.run_bt(self.tree)

        self.tree.root.terminate(py_trees.common.Status.INVALID)
        self.tree.shutdown()
        self.tree.root = None


def main() -> None:
    parser, args, leftover_args = createArgumentParser()
    repo_path = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    with open(os.path.join(repo_path, 'config/sim_objects.yaml')) as f:
        data = yaml.safe_load(f)

    bt_app = BTApplication(args)
    bt_app.init_agx_environment(args, data)

    agx_interface = AGXInterface(list(data.keys()))

    behavior_tree = BTDemo(agx_interface)
    behavior_tree.build_tree()
    behavior_tree.visualize('Task_BT')

    for _ in range(2):
        behavior_tree.build_tree()
        behavior_tree.run(bt_app)
        bt_app.reset()

    del bt_app
    bt_app = None


if __name__ == "__main__":
    main()
