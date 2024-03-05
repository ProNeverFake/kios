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

from copy import copy
from typing import Any

import agx
import numpy as np
import py_trees as pt

VERBOSE = True


class PickBehavior(pt.behaviour.Behaviour):
    def __init__(self, name: str, world_interface: Any, target_object: str) -> None:
        """Initialize the pick task."""
        super().__init__(name)

        self.world_interface = world_interface
        self.target_obj = target_object

        self.max_ticks = 10
        self.counter = 0

    def initialise(self) -> None:
        self.picking_task = self.world_interface.pick(self.target_obj)
        self.counter = 0
        if VERBOSE:
            print(self.counter)
        return super().initialise()

    def update(self) -> pt.common.Status:
        """Return the status of the behavior."""
        self.counter += 1
        if VERBOSE:
            print(self.counter)
        if self.world_interface.grasped(self.target_obj):
            return pt.common.Status.SUCCESS
        elif not self.world_interface.reachable(self.target_obj) or self.counter > self.max_ticks:
            return pt.common.Status.FAILURE
        else:
            return pt.common.Status.RUNNING

    def terminate(self, new_status) -> None:
        # self.world_interface.preempt_skill()
        return super().terminate(new_status)


class PlaceBehavior(pt.behaviour.Behaviour):
    def __init__(
        self,
        name: str,
        world_interface: Any,
        target_object: str,
        reference_object: str,
        pose: agx.Vec3 or np.ndarray
    ) -> None:
        """Initialize the place task."""
        super().__init__(name)

        self.world_interface = world_interface
        self.target_obj = target_object
        self.holding = None
        self.ref_obj = reference_object
        self.pose = pose

        self.max_ticks = 10
        self.counter = 0

    def initialise(self) -> None:
        self.placing_task = self.world_interface.place(
            self.target_obj, self.ref_obj, self.pose)
        self.counter = 0
        if VERBOSE:
            print(self.counter)
        self.holding = self.world_interface.holding
        return super().initialise()

    def update(self) -> pt.common.Status:
        """Return the status of the behavior."""
        self.counter += 1
        if VERBOSE:
            print(self.counter)
        if self.world_interface.at_pose(self.target_obj, self.ref_obj, self.pose, 0.5):
            return pt.common.Status.SUCCESS
        elif self.holding != self.target_obj or self.counter > self.max_ticks:
            return pt.common.Status.FAILURE
        else:
            return pt.common.Status.RUNNING

    def terminate(self, new_status) -> None:
        # self.world_interface.preempt_skill()
        return super().terminate(new_status)


class InGripper(pt.behaviour.Behaviour):
    """Returns SUCCESS if the gripper is holding object."""

    def __init__(
            self, name: str, world_interface: Any, held_object: str) -> None:
        """Initialize the condition."""
        super().__init__(name)

        self.world_interface = world_interface
        self.object = held_object

    def update(self) -> pt.common.Status:
        """Return the status of the behavior."""
        holding = self.world_interface.grasped(self.object)

        if holding:
            return pt.common.Status.SUCCESS
        else:
            return pt.common.Status.FAILURE


class AtPose(pt.behaviour.Behaviour):
    """Returns SUCCESS if the target object is at given pose."""

    def __init__(
        self,
        name: str,
        world_interface: Any,
        target: str,
        reference: str,
        pose: agx.Vec3 or np.ndarray
    ) -> None:
        """Initialize the condition."""
        super().__init__(name)

        self.world_interface = world_interface
        self.target = target
        self.reference = reference
        self.pose = pose

    def update(self) -> pt.common.Status:
        """Return the status of the behavior."""
        pose_ok = self.world_interface.at_pose(self.target, self.reference, self.pose, 0.5)

        if pose_ok:
            return pt.common.Status.SUCCESS
        else:
            return pt.common.Status.FAILURE
