"""Implementation of robot skills for the simulation."""

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

from simulation.algoryx.lfd.planning_itnerface import PlanningInterface
from simulation.algoryx.behaviors.agx_interface import AGXInterface
import py_trees as pt
import numpy as np
from behaviors.common_behaviors import ActionBehavior
import agx
from typing import Any, List
import random
import pickle

VERBOSE = False


class PickBehavior(ActionBehavior):
    def __init__(
        self,
        action_string: str,
        directory_path: str,
        name: str,
        world_interface: AGXInterface or PlanningInterface,
        target_object: str
    ):
        """
        Initialize the pick task.

        Args:
        ----
            - action_string: name of the action.
            - directory_path: path to the directory where the demonstration is stored.
            - name: name of the action file.
            - world_interface: interface to the robot.
            - target_object: name for the object to pick.

        """
        self.world_interface = world_interface

        if name is not None:
            try:
                with open(directory_path + '/' + name + '.pkl', 'rb') as f:
                    # this object contains fields defined in ActionInfo
                    self.action_info = pickle.load(f)
            except FileNotFoundError:
                pass

        self.target_obj = target_object

        self.max_ticks = 5
        self.counter = 0

        super().__init__(action_string, world_interface)

    def get_preconditions(self) -> List[str]:
        """Return the pre-conditions of the action."""
        preconditions = self.action_info.additional_preconditions + ['gripper_state open']
        return preconditions

    def get_postconditions(self) -> List[str]:
        """Return the post-conditions of the action."""
        postconditions = ['gripper_state closed', f'in_gripper {self.target_obj}']
        return postconditions

    def initialise(self):
        """Initialize the task as a thread."""
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

    def terminate(self, new_status: pt.common.Status):
        """Terminate the task thread and clear locks."""
        return super().terminate(new_status)

    def get_display_name(self) -> str:
        """Returnt the action name."""
        return f'Pick {self.target_obj}'

    def cost(self) -> int:
        """Define the cost of the action."""
        return 20


class PlaceBehavior(ActionBehavior):

    def __init__(
        self,
        action_string: str,
        directory_path: str,
        name: str,
        world_interface: AGXInterface or PlanningInterface,
        target_object: str,
        pose: agx.Vec3 or np.ndarray,
        reference_object: str,
        tolerance: float
    ):
        """
        Initialize the place task.

        Args:
        ----
            - action_string: name of the action.
            - directory_path: path to the directory where the demonstration is stored.
            - name: name of the action file.
            - world_interface: interface to the robot.
            - target_object: name for the object to pick.
            - reference_object: reference frame for the action.
            - pose: target pose in the given reference frame.

        """
        self.world_interface = world_interface

        if name is not None:
            try:
                with open(directory_path + '/' + name + '.pkl', 'rb') as f:
                    # this object contains fields defined in ActionInfo
                    self.action_info = pickle.load(f)
            except FileNotFoundError:
                pass

        self.target_obj = target_object
        self.holding = None
        self.ref_obj = reference_object
        self.pose = np.array(pose)
        self.tolerance = tolerance

        self.max_ticks = 5
        self.counter = 0

        super().__init__(action_string, world_interface)

    def get_preconditions(self) -> List[str]:
        """Return the pre-conditions of the action."""
        preconditions = self.action_info.equivalent_action.preconditions_with_additional()
        return preconditions

    def get_postconditions(self) -> List[str]:
        """Return the post-conditions of the action."""
        postconditions = self.action_info.equivalent_action.postconditions()
        return postconditions

    def initialise(self):
        """Initialize the task as a thread."""
        self.placing_task = self.world_interface.place(
            self.target_obj, self.ref_obj, self.pose)
        self.counter = 0
        self.holding = self.world_interface.holding
        if VERBOSE:
            print(self.counter)
        return super().initialise()

    def update(self) -> pt.common.Status:
        """Return the status of the behavior."""
        self.counter += 1
        if VERBOSE:
            print(self.counter)
        if self.world_interface.at_pose(self.target_obj, self.ref_obj, self.pose, self.tolerance):
            return pt.common.Status.SUCCESS
        elif self.holding != self.target_obj or self.counter > self.max_ticks:
            return pt.common.Status.FAILURE
        else:
            return pt.common.Status.RUNNING

    def terminate(self, new_status: pt.common.Status):
        """Terminate the task thread and clear locks."""
        return super().terminate(new_status)

    def get_display_name(self) -> str:
        """Returnt the action name."""
        name = 'Place %s at (%.2g, %.2g, %.2g) in %s' %\
            (
                self.target_obj, float(self.pose[0]), float(self.pose[1]),
                float(self.pose[2]), self.ref_obj
            )
        return name

    def cost(self) -> int:
        """Define the cost of the action."""
        return 20


class DropBehavior(PlaceBehavior):

    def update(self) -> pt.common.Status:
        """Return the status of the behavior."""
        self.counter += 1
        if self.world_interface.at_pose(
                self.target_obj, self.ref_obj, self.pose, self.tolerance, rough=True):
            return pt.common.Status.SUCCESS
        elif self.holding != self.target_obj or self.counter > self.max_ticks:
            return pt.common.Status.FAILURE
        else:
            return pt.common.Status.RUNNING

    def get_display_name(self) -> str:
        """Returnt the action name."""
        name = 'Drop %s at (%.2g, %.2g, %.2g) in %s' %\
            (
                self.target_obj, float(self.pose[0]), float(self.pose[1]),
                float(self.pose[2]), self.ref_obj
            )
        return name


class SetGripper(ActionBehavior):
    def __init__(
        self,
        action_string: str,
        world_interface: AGXInterface or PlanningInterface,
        state: str
    ):
        """
        Set the state for the gripper.

        Args:
        ----
            - action_string: name of the action.
            - world_interface: interface to the robot.
            - state: if the gripper is open or close.

        """
        super().__init__(action_string, world_interface)

        self.state = state
        self.world_interface = world_interface
        self.gripper_task = None

    def get_preconditions(self) -> List[str]:
        """Return the pre-conditions of the action."""
        return []

    def get_postconditions(self) -> List[str]:
        """Return the post-conditions of the action."""
        if self.state == 'open':
            return ['gripper_state open', 'in_gripper none']
        elif self.state == 'closed':
            return ['gripper_state closed']
        else:
            raise ValueError(f'Unknown gripper state "{self.state}".\
                The state must be either "open" or "closed".')

    def initialise(self):
        """Initialize the task as a thread."""
        self.gripper_task = self.world_interface.set_gripper(self.state)

    def update(self) -> pt.common.Status:
        """Return the status of the behavior."""
        if self.world_interface.is_gripper_state(self.state):
            return pt.common.Status.SUCCESS
        else:
            return pt.common.Status.RUNNING

    def terminate(self, new_status: pt.common.Status):
        """Terminate the task thread and clear locks."""
        return super().terminate(new_status)

    def get_display_name(self) -> str:
        """Returnt the action name."""
        return f'Set gripper {self.state}'

    def cost(self) -> int:
        """Define the cost of the action."""
        return 2


class MoveTo(ActionBehavior):
    def __init__(
        self,
        action_string: str,
        directory_path: str,
        name: str,
        world_interface: AGXInterface or PlanningInterface,
        target: np.ndarray,
        reference: str
    ):
        """
        Initialize the move task.

        Args:
        ----
            - action_string: name of the action.
            - directory_path: path to the directory where the demonstration is stored.
            - name: name of the action file.
            - world_interface: interface to the robot.
            - target: target goal for the robot.
            - reference: reference frame for the target.

        """
        self.world_interface = world_interface

        if name is not None:
            with open(directory_path + '/' + name + '.pkl', 'rb') as f:
                # this object contains fields defined in ActionInfo
                self.action_info = pickle.load(f)

        self.target = target
        self.reference = reference
        self.tolerance = 0.5
        self.moving_task = None

        super().__init__(action_string, world_interface)

    def get_preconditions(self) -> List[str]:
        """Return the pre-conditions of the action."""
        return []

    def get_postconditions(self) -> List[str]:
        """Return the post-conditions of the action."""
        return [
            str(f'robot_at {self.target[0]} {self.target[1]} {self.target[2]}' +
                f' {self.tolerance} {self.reference}')
        ]

    def initialise(self):
        """Initialize the task as a thread."""
        # Choose a goal randomically so that if the actions fails we have variation
        self.moving_task = self.world_interface.navigate(self.target, self.reference)

    def update(self) -> pt.common.Status:
        """Return the status of the behavior."""
        return pt.common.Status.SUCCESS

    def terminate(self,  new_status: pt.common.Status):
        """Terminate the task thread and clear locks."""
        return super().terminate(new_status)

    def get_display_name(self) -> str:
        """Returnt the action name."""
        return 'Move to (%.2g, %.2g, %.2g) in %s' %\
            (
                float(self.target[0]), float(self.target[1]), float(self.target[2]),
                self.reference
            )

    def cost(self) -> int:
        """Define the cost of the action."""
        return 30


class Approach(ActionBehavior):
    def __init__(
        self,
        action_string: str,
        world_interface: AGXInterface or PlanningInterface,
        target: List[float] = None,
        frame: str = None,
        target_object: str = None
    ):
        """
        Approach action for the mobile platform.

        If object is not None, x, y, z, and frame are ignored.
        If object is None, x, y, z, and frame must have values.

        Args:
        ----
            - action_string: name of the action.
            - directory_path: path to the directory where the demonstration is stored.
            - name: name of the action file.
            - world_interface: interface to the robot.
            - target: target goal for the robot.
            - reference: reference frame for the target.

        """
        self.world_interface = world_interface

        self.object = target_object
        self.target = None
        self.frame = world_interface.default_frame
        self.moving_task = None
        if self.object is None:
            self.target = np.array(target)
            self.frame = frame

        super().__init__(action_string, world_interface)

    def get_preconditions(self) -> List[str]:
        """Return the pre-conditions of the action."""
        return []

    def get_postconditions(self) -> List[str]:
        """Return the post-conditions of the action."""
        if self.object is None:
            return [f'reachable {self.target[0]} {self.target[1]} {self.target[2]} {self.frame}']
        else:
            return [f'reachable {self.object}']

    def initialise(self):
        """Initialize the task as a thread."""
        if self.object is not None:
            self.target, _ = self.world_interface.get_item_in_frame(self.object, self.frame)
        self.moving_task = self.world_interface.approach(self.target, self.frame)

    def update(self) -> pt.common.Status:
        """Return the status of the behavior."""
        return pt.common.Status.SUCCESS

    def terminate(self, new_status: pt.common.Status):
        """Terminate the task thread and clear locks."""
        return super().terminate(new_status)

    def get_display_name(self) -> str:
        """Returnt the action name."""
        if self.object is None:
            return 'Approach (%.2g, %.2g, %.2g) in %s' %\
                (self.target[0], self.target[1], self.target[2], self.frame)
        else:
            return 'Approach ' + self.object

    def cost(self) -> int:
        """Define the cost of the action."""
        return 30

# Conditions.
# Conditions don't need access to the configuration directory


class Unstack(pt.behaviour.Behaviour):
    """Returns SUCCESS if the items are unstacked."""

    def __init__(
        self,
        name: str,
        world_interface: AGXInterface or PlanningInterface,
        items: List[str]
    ) -> None:
        """Check if the input items are unstacked."""
        super().__init__(name)

        self.world_interface = world_interface
        self.items = items
        self.success = False
        self.world_interface.unstacked = False

    def update(self) -> pt.common.Status:
        """Return the status of the behavior."""
        unstacked = self.world_interface.items_unstacked(self.items)
        if unstacked or self.success:
            self.success = True
            self.world_interface.unstacked = True
            return pt.common.Status.SUCCESS
        else:
            return pt.common.Status.FAILURE

    def get_display_name(self) -> str:
        """Returnt the condition name."""
        return f'Boxes Unstacked?'


class InGripper(pt.behaviour.Behaviour):
    """
    Returns SUCCESS if the gripper is holding object.

    If object is None the behavior tests empty gripper.
    """

    def __init__(
        self,
        name: str,
        world_interface: AGXInterface or PlanningInterface,
        held_object: str
    ) -> None:
        """
        Initialize the condition.

        Args:
        ----
            - name: name of the condition
            - world_interface: interface to the robot.
            - held_object: name for the object in the gripper.

        """
        super().__init__(name)

        self.world_interface = world_interface
        self.object = held_object

    def update(self) -> pt.common.Status:
        """Return the status of the behavior."""
        if self.object is None or self.object == 'none':
            holding = self.world_interface.empty_gripper()
        else:
            holding = self.world_interface.grasped(self.object)

        if holding:
            return pt.common.Status.SUCCESS
        else:
            return pt.common.Status.FAILURE

    def get_display_name(self) -> str:
        """Returnt the condition name."""
        return f'In gripper {self.object}?'


class GripperState(pt.behaviour.Behaviour):
    """Returns SUCCESS if the gripper is in the desired state."""

    def __init__(
        self,
        name: str,
        world_interface: AGXInterface or PlanningInterface,
        state: Any
    ) -> None:
        """
        Initialize the condition.

        Args:
        ----
            - name: name of the condition
            - world_interface: interface to the robot.
            - state: state of the gripper.

        """
        super().__init__(name)

        self.world_interface = world_interface
        self.state = state

    def update(self) -> pt.common.Status:
        """Return the status of the behavior."""
        if self.world_interface.is_gripper_state(self.state):
            return pt.common.Status.SUCCESS
        else:
            return pt.common.Status.FAILURE

    def get_display_name(self) -> str:
        """Returnt the condition name."""
        return f'Gripper {self.state}?'


class ObjectAt(pt.behaviour.Behaviour):
    """Returns SUCCESS if the object is at a location within a tolerance."""

    def __init__(
        self,
        name: str,
        world_interface: AGXInterface or PlanningInterface,
        held_object: str,
        pose: agx.Vec3 or np.ndarray,
        frame: str
    ) -> None:
        """
        Initialize the condition.

        Args:
        ----
            - name: name of the condition
            - world_interface: interface to the robot.
            - held_object: name for the object in the gripper.
            - pose: desired position of the target object
            - frame: reference frame in which the position is defined.

        """
        super().__init__(name)

        self.world_interface = world_interface
        self.object = held_object
        self.position = np.array(pose)
        self.frame = frame
        self.tolerance = 0.03

    def update(self) -> pt.common.Status:
        """Return the status of the behavior."""
        pose_ok = self.world_interface.at_pose(
            self.object, self.frame, self.position, self.tolerance)
        if pose_ok:
            return pt.common.Status.SUCCESS
        else:
            return pt.common.Status.FAILURE

    def get_display_name(self) -> str:
        """Returnt the condition name."""
        return '%s at (%.2g, %.2g, %.2g) in %s?' %\
            (self.object, self.position[0], self.position[1], self.position[2], self.frame)


class ObjectRoughlyAt(ObjectAt):

    def __init__(self, *args, **kwargs) -> None:
        """Initialize the condition."""
        super().__init__(*args, **kwargs)

        self.tolerance = 0.1

    def update(self) -> pt.common.Status:
        """Return the status of the behavior."""
        pose_ok = self.world_interface.at_pose(
            self.object, self.frame, self.position, self.tolerance, rough=True)
        if pose_ok:
            return pt.common.Status.SUCCESS
        else:
            return pt.common.Status.FAILURE

    def get_display_name(self) -> str:
        """Returnt the condition name."""
        return '%s roughly at (%.2g, %.2g, %.2g) in %s?' %\
            (self.object, self.position[0], self.position[1], self.position[2], self.frame)


class RobotAt(pt.behaviour.Behaviour):
    """Returns SUCCESS if the object is at a location within a tolerance."""

    def __init__(
        self,
        name: str,
        world_interface: AGXInterface or PlanningInterface,
        target: List[float],
        frame: str
    ) -> None:
        """
        Condition to determine if the robot is a specific position.

        Args:
        ----
            - name: name of the action.
            - world_interface: interface to the robot.
            - target: target goal for the action.
            - tolerance: error in the robot position.
            - frame: reference frame for robot.

        """
        super().__init__(name)

        self.world_interface = world_interface
        self.position = np.array(target)
        self.tolerance = 0.5
        self.frame = frame

    def update(self) -> pt.common.Status:
        """Return the status of the behavior."""
        current_position = self.world_interface.robot_position(self.frame)

        distance = np.linalg.norm(self.position - current_position)
        if distance <= self.tolerance:
            return pt.common.Status.SUCCESS
        else:
            return pt.common.Status.FAILURE

    def get_display_name(self) -> str:
        """Returnt the action name."""
        return 'Robot at (%.2g, %.2g, %.2g) in %s?' %\
            (self.position[0], self.position[1], self.position[2], self.frame)


class Reachable(pt.behaviour.Behaviour):
    def __init__(
        self,
        name: str,
        world_interface: AGXInterface or PlanningInterface,
        target: List[float] = None,
        frame: str = None,
        target_object: str = None
    ) -> None:
        """
        Determine if the target pose or object is reachable.

        If object is not None, target, and frame are ignored.
        If object is None, target, and frame must have values.

        Args:
        ----
            - name: name of the condition.
            - world_interface: interface to the robot.
            - target: target goal for the action.
            - frame: reference frame for the object to approach.
            - target_object: name for the object to approach.

        """
        self.world_interface = world_interface
        self.target = None
        self.frame = world_interface.default_frame
        self.object = target_object

        if self.object is None:
            self.target = np.array([target])
            self.frame = frame

        super().__init__(name)

    def update(self) -> pt.common.Status:
        """Return the status of the behavior."""
        if self.object is not None:
            self.target, _ = self.world_interface.get_item_in_frame(self.object, self.frame)

        if self.world_interface.reachable(self.target, self.frame):
            return pt.common.Status.SUCCESS
        else:
            return pt.common.Status.FAILURE

    def get_display_name(self) -> str:
        """Returnt the action name."""
        if self.object is None:
            return 'Reachable (%.2g, %.2g, %.2g) in %s?' %\
                (self.target[0], self.target[1], self.target[2], self.frame)
        else:
            return f'Reachable {self.object}?'
