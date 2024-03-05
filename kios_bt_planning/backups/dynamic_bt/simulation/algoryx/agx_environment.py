"""Define the objects in the AGX simulaiton."""

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
from typing import Any, Dict, List

# AGX API
import agx
from agxBrick.brickLoaderUtils import extractModelFilePathAndName, setup_camera_and_lights
import agxOSG
from agxPythonModules.utils.environment import simulation, root, application
import numpy as np
import random


class AGXEnvironment():

    def __init__(self, args: Any = None, objects: Dict = None, debug: bool = False) -> None:
        self.sim = None

        self.debug = debug

        if args is not None:
            self.args = args
            if self.args.model == '':
                print('No brick model defined!')
                return

            # Load the brick model
            self.file_path, self.model_name = extractModelFilePathAndName(self.args)

        self.brickSimulation = None
        self.scene = None

        # parse config file where the user defines the items
        # objects is a dicitonary where key is the obj type and value is a list of objs of that type
        # every obj has a name and an reference location for the spawning
        ex_objects = {
            'Box1': {'type': 'box', 'reference': 'Table1', 'size': 0.1, 'color': 'yellow'},
            'Box2': {'type': 'box', 'reference': 'Table2', 'size': 0.1, 'color': 'green'},
            'Box3': {'type': 'box', 'reference': 'Table1', 'size': 0.1, 'color': 'blue'},
        }

        # Reference objects in the scene that do not move
        self.references = {
            'Table1': {'type': 'table', 'origin': [-1, 0, 0.375], 'size': [0.75, 0.75, 0.75]},
            'Table2': {'type': 'table', 'origin': [1, 0, 0.375], 'size': [0.75, 0.75, 0.75]},
            'Table3': {'type': 'table', 'origin': [0, 0, 0.375], 'size': [0.75, 0.75, 0.75]},
        }

        self.init_objects = ex_objects if objects is None else objects
        # Keep track of the objects
        self.objects = copy(self.init_objects)

    def get_objects(self) -> Dict:
        """Return the objects in the scene."""
        return self.objects

    def get_bodies(self) -> List[str]:
        """Return the names of the RigidBodies in the scene."""
        bodies = self.sim.getRigidBodies()
        names = []
        for body in bodies:
            names.append(str(body.getName()))
        return names

    def get_brickSimulation(self) -> Any:
        """Return the BrickSimulation object."""
        return self.brickSimulation

    def get_scene(self) -> Any:
        """Return the scene object."""
        return self.scene

    def shutdown(self):
        """Set internal variables to None."""
        self.sim.cleanup()
        self.sim.clearContactData()
        del self.sim
        del self.brickSimulation
        del self.scene
        self.sim = None
        self.brickSimulation = None
        self.scene = None
        print('Simulation DOWN!')

    def randomize_spawn(self, target: str, reference: str) -> np.ndarray:
        """Randomize the initial XYZ pose of the target object with respect to the reference."""
        candidate_pose = self._sample_pose(
            self.references[reference]['size'][0] - 3*self.objects[target]['size'],
            self.references[reference]['size'][1] - 3*self.objects[target]['size'],
            self.references[reference]['origin'][0:2]
        )
        colliding = self._is_colliding(target, reference, candidate_pose)

        attempts = 200
        while colliding and attempts > 0:
            # Sample a new pose
            candidate_pose = self._sample_pose(
                self.references[reference]['size'][0] - 3*self.objects[target]['size'],
                self.references[reference]['size'][1] - 3*self.objects[target]['size'],
                self.references[reference]['origin'][0:2]
            )
            # Check again for collision
            colliding = self._is_colliding(target, reference, candidate_pose)
            attempts -= 1

        z_val = (2*self.references[reference]['size'][2] + self.objects[target]['size'])/2
        pose = np.append(candidate_pose, z_val)
        return pose

    def spawn_kitbox(self, reference: str) -> np.ndarray:
        """Randomize the initial XYZ pose of the target object with respect to the reference."""
        candidate_pose = self.references[reference]['origin'][0:2]

        kitbox = self.sim.getRigidBody('KittingBox')
        ref_size = round(
            kitbox.getGeometries()[0].getShape().asBox().getHalfExtents()[0], 3)*2

        z_val = (2*self.references[reference]['size'][2] + ref_size)/2
        pose = np.append(candidate_pose, z_val)
        return pose

    def spawn(self, brickSimulation) -> None:
        """Spawn the objects randomically in the environment, given their origin."""
        from Brick.Math import Vec3
        from Brick.Physics import Component

        if brickSimulation is None:
            raise EnvironmentError(
                'No Brick Simulation found. Remember to build the AGX Scene first.')

        for obj in self.objects:
            # print(f'\nAdding {obj}')
            box_component = Component.CreateFromFile(
                self.file_path, self.objects[obj]['color'].capitalize() + 'BoxComponent')
            box_component.Bodies[0].Name = obj
            pose = self.randomize_spawn(obj, self.objects[obj]['reference'])
            # print(f'With pose {pose}')
            box_component.Bodies[0].LocalTransform.Position = Vec3(
                float(pose[0]), float(pose[1]), float(pose[2]))

            # Add the component to the simulation
            brickSimulation.AddComponent(box_component)

    def respawn(self) -> None:
        """Respawn the objects in the scene."""
        # respawn KittingBox
        body = self.sim.getRigidBody('KittingBox')
        table = random.choice(list(self.references.keys()))
        remaining_tables = list(self.references.keys())
        remaining_tables.remove(table)
        pose = self.spawn_kitbox(table)
        # print(f'With pose {pose}')
        body.setPosition(agx.Vec3(float(pose[0]), float(pose[1]), float(pose[2])))

        # respawn boxes
        for obj in self.objects:
            # print(f'\nRespawning {obj}')
            box_obj = self.sim.getRigidBody(obj)
            # Choose a reference object at random among the references
            reference = random.choice(remaining_tables)
            self.objects[obj]['reference'] = reference
            # Spawn the box in the reference
            pose = self.randomize_spawn(obj, reference)
            # print(f'With pose {pose}')
            box_obj.setPosition(agx.Vec3(float(pose[0]), float(pose[1]), float(pose[2])))

    def apply_configuration(self, target: Dict) -> None:
        """Move objects in the given configuration."""
        for obj in target:
            try:
                box_obj = self.sim.getRigidBody(obj)
            except AttributeError:
                print('The taget item does not exist!')
            try:
                reference = target[obj]['reference']
                # if the item is spwan in a table, then ignore it to allow randomization
                if reference.startswith('Table'):
                    continue
                pose = target[obj]['pose']
            except AttributeError:
                print('Target dictionary with wrong format!')

            ref_obj = self.sim.getRigidBody(reference)
            ref_pose = [
                ref_obj.getPosition().x(), ref_obj.getPosition().y(), ref_obj.getPosition().z()]
            # print(f'{reference} with pose {ref_pose}')

            # print(
            #     f'{obj} with pose {[pose[0] + ref_pose[0], pose[1] + ref_pose[1], pose[2] + ref_pose[2]]}')
            box_obj.setPosition(agx.Vec3(
                float(pose[0] + ref_pose[0]),
                float(pose[1] + ref_pose[1]),
                float(pose[2] + ref_pose[2])
            ))

    def build_scene(self) -> Any:
        """Build the AGX scene with the given arguments."""
        # With the current Brick modules it is necessary to import the API inside the function
        from Brick.AGXBrick import BrickSimulation
        from Brick.Physics import Component

        self.sim = simulation()
        self.sim.cleanup()
        self.sim.clearContactData()

        self.scene = Component.CreateFromFile(self.file_path, self.model_name)
        if self.scene is None:
            raise Exception(f'Failed to load scene')
        self.brickSimulation = BrickSimulation.Default
        self.brickSimulation.AddComponent(self.scene)

        if self.debug:
            application().setEnableDebugRenderer(True)
            application().setEnableOSGRenderer(False)

        if self.args.decorate:
            setup_camera_and_lights(
                application(),
                self.sim,
                root(),
                self.brickSimulation,
                self.scene,
                False
            )

        return self.brickSimulation

    def reset_simulation(self) -> None:
        """Reset the Brick Simulation."""
        from Brick.Physics import ComponentLoader

        self.scene.UpdateParameterSpace()
        ComponentLoader.RepositionComponent(self.scene)

        self.brickSimulation.ResetAgx()
        self.brickSimulation.SyncOutputParameters()

        self.respawn()

    def create_visual(self) -> None:
        """Enable the visualization of the environment."""
        agxOSG.createVisual(self.sim, application().getSceneRoot())

    def _sample_pose(self, width: float, length: float, origin: np.ndarray) -> np.ndarray:
        """Sample the pose from a uniform distribution."""
        x = round(random.uniform(-width/2 + 0.05, width/2 - 0.05), 3)
        y = round(random.uniform(-length/2 + 0.05, length/2 - 0.05), 3)
        return np.add(np.array([x, y]), origin)

    def _is_colliding(
        self,
        target: str,
        reference: str,
        target_pose: np.ndarray
    ) -> bool:
        """Check if the target pose is colliding with another element."""
        colliding = False
        # Check for rejection sampling only the objects that share the same origin
        if target not in self.objects:
            return False
        for obj in self.objects:
            # Get object from simulation
            try:
                body: agx.RigidBody = self.sim.getRigidBody(obj)
                body_pose = np.array([body.getPosition().x(), body.getPosition().y()])
            except AttributeError:
                # In case the object doesn't exist yet, skip
                continue

            if obj == target or self.objects[obj]['reference'] != reference:
                continue
            else:
                if np.linalg.norm(target_pose - body_pose) < 0.15:
                    colliding = True
        return colliding
