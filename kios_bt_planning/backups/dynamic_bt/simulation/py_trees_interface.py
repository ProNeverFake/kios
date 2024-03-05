"""Interfaces to py_trees from behavior tree strings."""

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

from dataclasses import dataclass
import time
from typing import Any, List, Tuple

from behaviors.behavior_lists import ParameterizedNode
from behaviors.behavior_tree import BT
from behaviors import common_behaviors
import py_trees as pt


@dataclass
class PyTreeParameters:
    """Data class for parameters for the PyTree run."""

    behavior_lists: Any = None   # Lists of the types of behaviors
    behaviors: Any = None        # Module containing actual behaviors
    max_ticks: int = 200         # Maximum number of ticks to run
    sim_tick_time: int = 2.5     # If the simulator is given, how much time in the simulator every tick takes
    max_time: float = 10000.0    # Maximum time in s to run
    max_fails: int = 1           # Maximum number of failure states before breaking
    successes_required: int = 2  # Number of success states required before breaking
    show_world: bool = False     # Animate the run
    verbose: bool = False        # Extra prints


class PyTree(pt.trees.BehaviourTree):
    """A class containing a behavior tree. Inherits from the py tree BehaviorTree class."""

    def __init__(
        self,
        node_list: List[ParameterizedNode],
        parameters: PyTreeParameters = None,
        world_interface: Any = None,
        root: Any = None
    ):
        if parameters is not None:
            self.par = parameters
        else:
            self.par = PyTreeParameters()
        if root is not None:
            self.root = root
            node_list = self.get_bt_from_root()

        self.bt = BT(node_list, self.par.behavior_lists)
        self.behaviors = self.par.behaviors
        self.world_interface = world_interface
        self.failed = False
        self.timeout = False

        if root is None:
            self.root, has_children = common_behaviors.get_node(
                node_list[0], world_interface, self.par.verbose)
            node_list.pop(0)
        else:
            has_children = False

        super().__init__(root=self.root)
        if has_children:
            self.create_from_list(node_list, self.root)

    def get_bt_from_root(self) -> List[str]:
        """
        Return bt string from py tree root by cleaning the ascii tree from py trees.

        Not complete or beautiful by any means but works for many trees.
        """
        string = pt.display.ascii_tree(self.root)
        string = string.replace('[o] ', '')
        string = string.replace('[-] ', '')
        string = string.replace('\t', '')
        string = string.replace('-->', '')
        string = string.replace('Fallback', 'f(')
        string = string.replace('Sequence', 's(')
        bt = string.split('\n')
        bt = bt[:-1]  # Remove empty element because of final newline

        prev_leading_spaces = 999999
        for i in range(len(bt) - 1, -1, -1):
            leading_spaces = len(bt[i]) - len(bt[i].lstrip(' '))
            bt[i] = bt[i].lstrip(' ')
            if leading_spaces > prev_leading_spaces:
                for _ in range(round((leading_spaces - prev_leading_spaces) / 4)):
                    bt.insert(i + 1, ')')
            prev_leading_spaces = leading_spaces

        bt_obj = BT(bt, self.par.behavior_lists)
        bt_obj.close()

        return bt_obj.bt

    def create_from_list(
        self,
        node_list: List[str],
        node: pt.composites.Composite
    ) -> pt.composites.Composite:
        """Recursive function to generate the tree from a list."""
        while len(node_list) > 0:
            if node_list[0] == ')':
                node_list.pop(0)
                return node

            newnode, has_children = self.behaviors.get_node(
                node_list[0], self.world_interface, self.par.verbose)
            node_list.pop(0)
            if has_children:
                # Node is a control node or decorator with children.
                # Add subtree via string and then add to parent
                newnode = self.create_from_list(node_list, newnode)
                node.add_child(newnode)
            else:
                # Node is a leaf/action node - add to parent, then keep looking for siblings
                node.add_child(newnode)

        # This return is only reached if there are too few up nodes
        return node

    def run_bt(self, simulator=None) -> Tuple[int, bool]:
        """Run the behavior tree."""
        ticks = 0
        straight_fails = 0
        successes = 0
        status_ok = True

        start = time.time()
        if simulator is not None:
            self.world_interface.remove_locks()
            self.world_interface.preempt_skill()
            self.world_interface.add_merger()

        while (self.root.status is not pt.common.Status.FAILURE or
                straight_fails < self.par.max_fails) and\
              (self.root.status is not pt.common.Status.SUCCESS or
                successes < self.par.successes_required) and\
                ticks < self.par.max_ticks and status_ok:

            status_ok = self.world_interface.get_feedback()  # Wait for connection

            if status_ok:
                if self.par.verbose:
                    print('Tick', ticks)
                    print(self.root.status)
                self.root.tick_once()
                self.world_interface.send_references()
                # HACK!!
                if simulator is not None:
                    current_time = simulator.app.getTimeStamp()
                    while simulator.app.getTimeStamp() < self.par.sim_tick_time + current_time:
                        simulator.stepApplication()

                if self.par.show_world:
                    world.animate_state(self.world_interface.state)

                ticks += 1
                if self.root.status is pt.common.Status.SUCCESS:
                    successes += 1
                else:
                    successes = 0

                if self.root.status is pt.common.Status.FAILURE:
                    straight_fails += 1
                else:
                    straight_fails = 0

                if time.time() - start > self.par.max_time:
                    status_ok = False
                    self.timeout = True
                    print('Max time expired')

        if self.par.verbose:
            print('Total episode ticks: ', ticks)
            print('Total episode time: ', time.time() - start)

        if self.par.show_world:
            world.animate()
            world.save_world('testworld')

        if ticks >= self.par.max_ticks:
            self.timeout = True
        if straight_fails >= self.par.max_fails:
            self.failed = True
        return ticks, status_ok

    def step_bt(self, simulator=None) -> bool:
        """Step the BT one step."""
        status_ok = True

        status_ok = self.world_interface.get_feedback()  # Wait for connection

        if status_ok:
            self.root.tick_once()
            self.world_interface.send_references()
            # HACK!!
            if simulator is not None:
                simulator.__step_ahead(3)

        return status_ok

    def save_fig(
        self,
        path: str,
        name: str = 'Behavior tree',
        static: bool = True,
        blackboard: bool = False
    ) -> None:
        """Save the tree as a figure."""
        pt.display.render_dot_tree(
            self.root,
            name=name,
            target_directory=path,
            static=static,
            with_blackboard_variables=blackboard
        )
