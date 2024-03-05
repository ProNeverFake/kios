"""Definition of the low level behaviors for the AGX simulation."""

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

# TODO:
# search:
#   - get the tables position from the sim and move the gripper closed to the tables
#   - if there is a box in the neighborhood of the gripper then update position of the box in the robot's knowledge
# move:
#   - move the gripper to the input pose/obj
# pick:
#   - combine pre-grasp, grasp, and post-grasp motion into a single action that takes as input a pose/obj
# place:
#   - combine pre-place, place, and post-place motion into a single action that takes as input a pose/obj
# recharge:
#   - move the robot to a fix pose where it recharges the batteries
# localize:
#   - make the robot bleief of its pose equal to the pose of the model
# tuck:
#   - move the robot arm to a fix position in the robot ref frame

# these skills should have a definition compatible with the lfd_behaviors in abb_robot_robot_behaviors !!
# so AGXInterface should have methods and attributes as the OnlineYuMiInterface and the OfflineInterface

from copy import copy, deepcopy
import math
from typing import Any, List, Tuple

import agx
from agxPythonModules.utils.callbacks import StepEventCallback as sec
from agxPythonModules.utils.environment import simulation
import numpy as np


class AGXInterface():
    """An interface for execution of YuMi behaviors in the AGX simulator."""

    def __init__(self, task_objects: List[str]) -> None:
        """Instantiate a new AGX Interface."""
        self.sim = simulation()
        self.task_objects = task_objects

        # Get things from the simulation that is setup in the Brick model.
        # End Effecor
        self.gripper_ctrl = self.sim.getRigidBody('GripperControlBody')
        self.gripper_obj = self.sim.getRigidBody('gripper')
        self.gripper_offset = 0.12
        # Robot
        self.robot = self.sim.getRigidBody('RobotControlBody')
        # EE-Robot lock for control
        self.gripper_lock = self.sim.getConstraint('controlGripperLock').asLockJoint()

        # Object to merge and split bodies:
        self.with_merger = True
        self.merger_name = 'Merger'

        if not self.with_merger:
            self.item_locks = {}
            self.__create_constraints(task_objects)

        # Gripper states
        self.holding = ''
        self._gripper_state = 'closed'

        # Flags
        self.unstacked = False

        # Reachability Condition
        self.workspace = self.workspace_limit()
        self.max_z = deepcopy(round(self.gripper_ctrl.getPosition().z(), 3))

    def preempt_skill(self) -> None:
        """Preempt all planned sequences."""
        sec.instance()._clear()

    def add_merger(self) -> None:
        merger = agx.MergedBody()
        merger.setName(self.merger_name)
        self.sim.add(merger)

    def remove_locks(self) -> None:
        """Remove locks and merged items."""
        self.holding = ''
        for obj in self.task_objects:
            self._release(obj)
        merger = self.sim.getMergedBody(self.merger_name)
        if merger is not None:
            success = self.sim.remove(merger)

    def get_feedback(self) -> bool:
        """Dummy function to fit world interface template."""
        return (self.gripper_ctrl is not None) or (self.robot is not None)

    def send_references(self) -> None:
        """Dummy function to fit world interface template."""
        pass

    def workspace_limit(self) -> Tuple[float, float]:
        """Define the limit of the workspace given the two external tables."""
        table_1 = self.sim.getRigidBody('Table1')
        table_2 = self.sim.getRigidBody('Table2')
        table_size = round(
            table_1.getGeometries()[0].getShape().asBox().getHalfExtents()[0], 3)*2

        delta_x = round(table_2.getPosition().x() - table_1.getPosition().x(), 3) + table_size
        delta_y = table_size
        return delta_x, delta_y

    # SKILLS
    def pick(self, target: str) -> None:
        """Pick the target item."""
        target_obj = self.sim.getRigidBody(target)
        if target_obj is None:
            raise ValueError(f'Target item {target} does not exist!')
        # Time Stamps
        t0 = self.sim.getTimeStamp() + 1.0
        t1 = t0 + 0.5
        t2 = t1 + 0.1
        t3 = t2 + 0.5
        # Move gripper to pre-grasp position
        size = round(
            target_obj.getGeometries()[0].getShape().asBox().getHalfExtents()[0], 3)*2
        sec.callAt(t0, lambda: self._move_gripper(
            target_obj.getPosition() + agx.Vec3(0, 0, 2*size)))
        sec.callAt(t1, lambda: self._move_gripper(
            target_obj.getPosition() + agx.Vec3(0, 0, 0.7*size)))
        # Grasp
        sec.callAt(t2, lambda: self._grasp(target))
        # Move gripper to post-grasp position
        sec.callAt(t3, lambda: self._move_gripper(
            self.gripper_ctrl.getPosition() + agx.Vec3(0, 0, 1.5*size)))

    def place(self, target: str, reference: str, pose: agx.Vec3 or np.ndarray) -> None:
        """Place the target item in a posed espressed in the desired reference frame."""
        # Preparations
        target_obj = self.sim.getRigidBody(target)
        if target_obj is None:
            raise ValueError(f'Target item {target} does not exist!')
        target_size = round(
            target_obj.getGeometries()[0].getShape().asBox().getHalfExtents()[0], 3)*2
        ref_obj = self.sim.getRigidBody(reference)
        if ref_obj is None:
            raise ValueError(f'Reference item {reference} does not exist!')
        ref_size = round(
            target_obj.getGeometries()[0].getShape().asBox().getHalfExtents()[0], 3)*2

        pose = agx.Vec3(pose[0], pose[1], pose[2]) if type(pose) is np.ndarray else pose

        # Time Stamps
        t0 = self.sim.getTimeStamp() + 1.0
        t1 = t0 + 0.5
        t2 = t1 + 0.1
        t3 = t2 + 0.5
        # Move gripper to pre-place position
        sec.callAt(t0, lambda: self._move_gripper(
            self.__round(ref_obj.getPosition() + pose + agx.Vec3(0, 0, 2*ref_size))))
        sec.callAt(t1, lambda: self._move_gripper(self.__round(ref_obj.getPosition() + pose)))
        # Release
        sec.callAt(t2, lambda: self._release(target))
        # Move gripper to post-release position
        sec.callAt(t3, lambda: self._move_gripper(
            self.__round(target_obj.getPosition() + agx.Vec3(0, 0, 3*target_size))))

    def set_gripper(self, state: str) -> None:
        """Open and closed the gripper."""
        if state == 'open':
            self._gripper_state = state
            self.holding = ''
            for obj in self.task_objects:
                self._release(obj)
        elif state == 'closed':
            self._gripper_state = state
        else:
            raise ValueError('Gripper state not valid!')

        if state == 'open' and self.holding != '':
            # The robot is holding something that will be dropped
            self._release(self.holding)

    def lift_gripper(self) -> None:
        """Move the gripper along the Z azis."""
        t0 = self.sim.getTimeStamp() + 1.0
        sec.callAt(t0, lambda: self._move_gripper(
            self.gripper_ctrl.getPosition() + agx.Vec3(0.0, 0.0, 0.3)))

    def tuck_gripper(self) -> None:
        """Tuck the robot arm."""
        if self.robot is None:
            raise ValueError('Robot not found!')

        tuck_position = self.robot.getFrame().transformPointToLocal(self.gripper_ctrl.getPosition())
        self.gripper_ctrl.setPosition(self.robot.getFrame().transformPointToWorld(tuck_position))

    def move_robot(self, pos: agx.Vec3 or np.ndarray or str) -> None:
        """Move the robot at the desired position."""
        if self.robot is None:
            raise ValueError('Robot not found!')

        self.gripper_lock.setEnable(False)
        position = agx.Vec3(pos[0], pos[1], pos[2]) if type(pos) is np.ndarray else pos
        self.robot.setPosition(position)

        # TODO: implement it similarly to place, i.e. with a reference object and a pose

    def approach(self) -> None:
        pass

    def navigate(self) -> None:
        pass

    # CHECKS
    def grasped(self, target: str) -> bool:
        """Return if the target item is grasped or not."""
        if target is None or target == 'none':
            return self.holding == ''
        elif self.with_merger:
            return False if agx.MergedBody.get(self.sim.getRigidBody(target)) is None else True
        else:
            return self.item_locks[target].getEnable()

    def empty_gripper(self) -> bool:
        """Return true if the gripper is open."""
        return self.holding == ''

    def is_gripper_state(self, state: str) -> bool:
        """Return true gripper is open."""
        if state == 'open' or state == 'closed':
            return self._gripper_state == state
        else:
            raise ValueError('Gripper state not valid!')

    def get_item_in_frame(self, target: str, reference: str) -> Tuple[np.ndarray, np.ndarray]:
        """Return if the target item is at the desired pose in the frame of the reference item."""
        w_T_t = self.sim.getRigidBody(target).getTransform()
        w_T_r = self.sim.getRigidBody(reference).getTransform()
        r_T_w = w_T_r.inverse()
        r_T_t = r_T_w*w_T_t
        position = r_T_t.getTranslate()
        np_position = self.__as_array(position)
        rotation = r_T_t.getRotate()
        np_rotation = self.__as_array(rotation)

        return np_position.round(3), np_rotation.round(3)

    def items_unstacked(self, items: List[str]) -> bool:
        """Check that the items are placed on the table and not over some other object."""
        table = self.sim.getRigidBody('Table1')
        table_height = round(
            table.getGeometries()[0].getShape().asBox().getHalfExtents()[0], 3)*2

        unstacked = True
        for item in items:
            item_obj = self.sim.getRigidBody(item)
            item_size = round(
                item_obj.getGeometries()[0].getShape().asBox().getHalfExtents()[0], 3)*2
            item_height = round(item_obj.getPosition().z(), 3)
            if item_height > table_height + item_size:
                unstacked = False

        return unstacked

    def at_pose(
        self,
        target: str,
        reference: str,
        pose: agx.Vec3 or np.ndarray,
        tolerance: float,
        rough: bool = False
    ) -> bool:
        """Return if the target item is at the desired pose in the frame of the reference item."""
        position, _ = self.get_item_in_frame(target, reference)
        # print(f'{target} at {position} in {reference} frame.')
        real = self.as_vec3(position)
        target = pose if type(pose) is agx.Vec3 else self.as_vec3(pose)
        corrected = self.__round(agx.Vec3(target.x(), target.y(), target.z()-self.gripper_offset))
        # print(f'Checking for position {corrected}.')
        distance = self._distance(real, corrected, 2) if rough else self._distance(real, corrected)
        # print(f'At pose? Distance: {distance}, tolerance: {tolerance}.')

        return distance < float(tolerance)

    def robot_position(self):
        pass

    def reachable(self, target: str) -> bool:
        """Return True if the target is reachable."""
        max_distance = math.sqrt(self.workspace[0]**2 + self.workspace[1]**2 + self.max_z**2)
        gripper_pose, _ = self.get_item_in_frame('GripperControlBody', target)
        current_distance = math.sqrt(gripper_pose[0]**2 + gripper_pose[1]**2 + gripper_pose[2]**2)
        if current_distance < max_distance:
            return True
        return False

    # AUXILIARY

    def as_vec3(self,  array: np.ndarray or List[float]) -> agx.Vec3:
        """Return the Numpy Array object as AGX Vec3."""
        return agx.Vec3(array[0], array[1], array[2])

    def _distance(self, v1: agx.Vec3 or np.ndarray, v2: agx.Vec3 or np.ndarray, d: int = 3) -> float:
        """Return the distance between two vectors in 2D or 3D."""
        v1_ = copy(v1) if type(v1) is agx.Vec3 else self.as_vec3(v1)
        v2_ = copy(v2) if type(v2) is agx.Vec3 else self.as_vec3(v2)

        if d == 2:
            v1_ = agx.Vec3(v1_.x(), v1_.y(), 0.0)
            v2_ = agx.Vec3(v2_.x(), v2_.y(), 0.0)

        return round(float(agx.Vec3.distance(v1, v2)), 3)

    def _move_gripper(self, pos: agx.Vec3) -> None:
        """Move the gripper to the desired position."""
        # print('Moving to: ', self.__round(pos))
        self.gripper_lock.setEnable(True)
        self.gripper_ctrl.setPosition(pos)

    def _grasp(self, item: str) -> None:
        """Grasp an intem by attaching it to the gripper."""
        item_body = self.sim.getRigidBody(item)
        if self.with_merger:
            self.__split(item_body)
        else:
            self.item_locks[item].setEnable(False)

        d = self._distance(self.gripper_ctrl.getPosition(), item_body.getPosition())

        if d < 0.08:
            # print(f'Grasping {item}.')
            if self.with_merger:
                self.__merge(item_body, self.gripper_obj)
            else:
                self.item_locks[item].rebind()
                self.item_locks[item].setEnable(True)
            self.holding = item
            self._gripper_state = 'closed'
        else:
            if self.with_merger:
                self.__split(item_body)
            else:
                self.item_locks[item].setEnable(False)
            self.holding = ''
            self._gripper_state = 'open'

    def _release(self, item: str) -> bool:
        """Release the held object in the gripper."""
        # print(f'Releasing {item}.')
        if self.with_merger:
            self.__split(self.sim.getRigidBody(item))
            if agx.MergedBody.get(self.sim.getRigidBody(item)) is None:
                self.holding = ''
                self._gripper_state = 'open'
                return True
            else:
                return False
        else:
            if not self.item_locks[item].getEnable():
                return False
            else:
                self.item_locks[item].setEnable(False)
                self.holding = ''
                self._gripper_state = 'open'
                return True

    def __merge(self, body1: agx.RigidBody, body2: agx.RigidBody) -> None:
        """Merge two bodies for faster simulation."""
        merger = self.sim.getMergedBody(self.merger_name)
        merger.add(agx.MergedBodyEmptyEdgeInteraction(body1, body2))

    def __split(self, body1: agx.RigidBody) -> None:
        """Merge two bodies for faster simulation."""
        merger = self.sim.getMergedBody(self.merger_name)
        if merger is not None:
            success = merger.remove(body1)

    def __create_constraints(self, items: List[str]) -> None:
        """Create locks so that task items can be attached to the gripper."""
        for obj in items:
            joint = agx.LockJoint(self.gripper_ctrl, self.sim.getRigidBody(obj))
            self.sim.add(joint)
            joint.setEnable(False)
            self.item_locks[obj] = joint

    def __round(self, vec3: agx.Vec3) -> agx.Vec3:
        """Round function for Vec3."""
        return agx.Vec3(round(vec3.x(), 3), round(vec3.y(), 3), round(vec3.z(), 3))

    def __as_array(self,  vec3: agx.Vec3) -> np.ndarray:
        """Return the Vec3 object as Numpy Array."""
        try:
            return np.array([vec3.x(), vec3.y(), vec3.z(), vec3.w()])
        except AttributeError:
            return np.array([vec3.x(), vec3.y(), vec3.z()])
