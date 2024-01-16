"""Interface to generate poses in AGX for the GP."""

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

from typing import List

import agx
from behaviors.behavior_lists import BehaviorLists
import numpy as np
from simulation.algoryx.behaviors.agx_interface import AGXInterface
import yaml


"""
The string representations of the behaviors are:
    - pick object
    - place object x y z tolerance frame
    - open_gripper
    - close_gripper
    - move x y z tolerance frame
    - gripper_state open/closed
    - empty_gripper
    - in_gripper object
    - object_at object x y z tolerance frame
    - object_roughly_at object x y z tolerance frame
    - robot_at x y z tolerance frame

"""


class GPInterface(AGXInterface):
    """An interface for generation of skills in AGX."""

    def __init__(self, task_objects: List[str], ref_objects: List[str]) -> None:
        """Instantiate a new AGX Interface."""
        self.movable = task_objects
        self.fixed = ref_objects

    def initialize(self):
        super().__init__(self.movable)

    def generate_pickplace_list(
        self,
        file_path: str,
        random: bool = True
    ) -> BehaviorLists:
        """Generate a behavior list where every element moves a block."""
        super().__init__(self.movable)
        new_list = BehaviorLists()
        pickplace_list = self.pickplace_list()
        new_list.merge_behaviors(pickplace_list)

        if not random:
            unstack_list = BehaviorLists(
                condition_nodes=['unstacked']
            )
            new_list.merge_behaviors(unstack_list)

        settings = {
            'fallback_nodes': new_list.fallback_nodes,
            'atomic_fallback_nodes': new_list.atomic_fallback_nodes,
            'sequence_nodes': new_list.sequence_nodes,
            'atomic_sequence_nodes': new_list.atomic_sequence_nodes,
            'condition_nodes': new_list.condition_nodes,
            'action_nodes': new_list.action_nodes,
            'up_node': new_list.up_node
        }
        # Use another destination than the one where the LfD stuff is stored
        file_name = file_path + '/BT_SETTINGS.yaml'
        with open(file_name, 'w+') as f:
            yaml.dump(settings, f)

        return new_list

    def generate_behavior_list(
        self,
        file_path: str,
        actions: List[str],
        old_behavior_list: BehaviorLists = None
    ) -> BehaviorLists:
        """Extend the behavior list by generating poses for each type of behavior."""
        super().__init__(self.movable)
        new_list = BehaviorLists()
        # Note: merge behaviors removes duplicates
        if 'pick' in actions:
            pick_list = self.pick_list()
            new_list.merge_behaviors(pick_list)
        if 'place' in actions:
            place_list = self.place_list()
            new_list.merge_behaviors(place_list)
        if 'drop' in actions:
            drop_list = self.place_list(rough=True)
            new_list.merge_behaviors(drop_list)
        if 'move' in actions:
            move_list = self.move_list()
            new_list.merge_behaviors(move_list)

        if old_behavior_list is not None:
            new_list.merge_behaviors(old_behavior_list)

        settings = {
            'fallback_nodes': new_list.fallback_nodes,
            'sequence_nodes': new_list.sequence_nodes,
            'condition_nodes': new_list.condition_nodes,
            'action_nodes': new_list.action_nodes,
            'up_node': new_list.up_node
        }
        # Use another destination than the one where the LfD stuff is stored
        file_name = file_path + '/BT_SETTINGS.yaml'
        with open(file_name, 'w+') as f:
            yaml.dump(settings, f)

        return new_list

    def pick_list(self) -> BehaviorLists:
        """Create behaviors for picking in the LfD fashion."""
        action_list = []
        condition_list = []
        for name in self.movable:
            action_list.append(str('pick0 ' + name))
            condition_list.append(str('in_gripper ' + name))

        action_list.append('open_gripper')
        condition_list.append('gripper_state closed')

        return BehaviorLists(action_nodes=action_list, condition_nodes=condition_list)

    def place_list(self, rough: bool = False) -> BehaviorLists:
        """
        Create behaviors for placing in the LfD fashion.

        If rough, behaviors for dropping are created instead.
        """
        action_list = []
        condition_list = []
        for target in self.movable:
            for reference in self.movable:
                if target == reference:
                    continue
                poses = self._sample_cube(reference, rough)
                for pose in poses:
                    if rough:
                        action_list.append(str('drop0 ' + target + pose + reference))
                        condition_list.append(
                            str('object_roughly_at ' + target + pose + reference))
                    else:
                        action_list.append(str('place0 ' + target + pose + reference))
                        condition_list.append(str('object_at ' + target + pose + reference))

        for target in self.movable:
            for reference in self.fixed:
                poses = self._sample_box(target, reference, rough)
                for pose in poses:
                    if rough:
                        action_list.append(str('drop0 ' + target + pose + reference))
                        condition_list.append(
                            str('object_roughly_at ' + target + pose + reference))
                    else:
                        action_list.append(str('place0 ' + target + pose + reference))
                        condition_list.append(str('object_at ' + target + pose + reference))

        action_list.append('close_gripper')
        condition_list.append('in_gripper none')
        condition_list.append('gripper_state open')

        return BehaviorLists(action_nodes=action_list, condition_nodes=condition_list)

    def pickplace_list(self, rough: bool = False) -> BehaviorLists:
        """
        Create behaviors for placing in the LfD fashion.

        If rough, behaviors for dropping are created instead.
        """
        atomic_fallback_nodes = []
        for target in self.movable:
            for reference in self.movable:
                if target == reference:
                    continue
                poses = self._sample_cube(reference, rough)
                for pose in poses:
                    if rough:
                        atomic_fallback_nodes.append(
                            str('move_roughly0 ' + target + pose + reference))
                    else:
                        atomic_fallback_nodes.append(str('move0 ' + target + pose + reference))

        for target in self.movable:
            for reference in self.fixed:
                poses = self._sample_box(target, reference, rough)
                for pose in poses:
                    if rough:
                        atomic_fallback_nodes.append(
                            str('move_roughly0 ' + target + pose + reference))
                    else:
                        atomic_fallback_nodes.append(str('move0 ' + target + pose + reference))

        action_list = ['close_gripper', 'open_gripper']
        condition_list = ['gripper_state closed', 'in_gripper none', 'gripper_state open']

        return BehaviorLists(
            atomic_fallback_nodes=atomic_fallback_nodes,
            condition_nodes=condition_list,
            action_nodes=action_list)

    def move_list(self) -> BehaviorLists:
        pass

    def _sample_cube(self, reference: str, rough: bool = False) -> List[str]:
        """
        Sample 5 poses adjacent to the reference object.

        Above, Left, Below, Right, On Top.
        """
        obj_ = self.sim.getRigidBody(reference)
        # origin = self.__round(obj_.getPosition())
        size = obj_.getGeometries()[0].getShape().asBox().getHalfExtents()[0]*2
        tolerance = 0.02
        if rough:
            tolerance = 0.05

        side_offset = 0.02

        pose_list = []
        for x_step in [-1, 1]:
            pose = self.__round(
                agx.Vec3(x_step*(size + side_offset), 0, self.gripper_offset))
            pose_list.append(' ' + ' '.join(str(x) for x in pose) + ' ' + str(tolerance) + ' ')
        for y_step in [-1, 1]:
            pose = self.__round(
                agx.Vec3(0, y_step*(size + side_offset), self.gripper_offset))
            pose_list.append(' ' + ' '.join(str(x) for x in pose) + ' ' + str(tolerance) + ' ')

        plus_z = self.__round(agx.Vec3(0, 0, size + self.gripper_offset))
        pose_list.append(' ' + ' '.join(str(x) for x in plus_z) + ' ' + str(tolerance) + ' ')

        return pose_list

    def _sample_box(self, target: str, reference: str, rough: bool = False) -> List[str]:
        """
        Sample 5 poses adjacent to the reference kit box.

        Above, Left, Below, Right, On Top.
        """
        tg_obj_ = self.sim.getRigidBody(target)
        size = tg_obj_.getGeometries()[0].getShape().asBox().getHalfExtents()[0]*2
        ref_obj_ = self.sim.getRigidBody(reference)
        bottom_plate_geom = ref_obj_.getGeometries()[0].getShape().asBox().getHalfExtents()
        length = bottom_plate_geom[0]*2
        depth = bottom_plate_geom[1]*2
        height = bottom_plate_geom[2]*2
        dx = round(length/3, 3)
        dy = round(depth/3, 3)
        dz = round(height + size/2, 3)
        tolerance = 0.02
        if rough:
            tolerance = 0.05

        pose_list = []
        for x_step in [-1, 0, 1]:
            for y_step in [-1, 0, 1]:
                pose = self.__round(agx.Vec3(x_step*dx, y_step*dy, self.gripper_offset + dz))
                pose_list.append(' ' + ' '.join(str(x) for x in pose) + ' ' + str(tolerance) + ' ')

        return pose_list

    def __get_fixed_objs(self) -> List[str]:
        """Return the names of the RigidBodies in the scene."""
        # This is the same function in the Environment class
        bodies = self.sim.getRigidBodies()
        names = []
        for body in bodies:
            if body not in self.movable:
                names.append(str(body.getName()))
        return names

    def __round(self, vec3: agx.Vec3) -> agx.Vec3:
        """Round function for Vec3."""
        return agx.Vec3(round(vec3.x(), 3), round(vec3.y(), 3), round(vec3.z(), 3))
