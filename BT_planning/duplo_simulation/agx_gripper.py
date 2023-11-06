# pylint: disable=too-many-arguments
"""
A simple two finger prismatic gripper in agx
"""
from dataclasses import dataclass
import math

import agx
import agxSDK
import agxOSG
import agxCollide
from agxPythonModules.utils.callbacks import KeyboardCallback as Input

from duplo_simulation.agx_interface import GripperRef

@dataclass
class GripperProps:
    """Properties of gripper """
    radius: float = 0.003
    height: float = 0.03
    mass: float = 0.1 #Mass per body

@dataclass
class PosRef:
    """Gripper position reference """
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    rotz: float = 0.0
    grip1: float = 0.02
    grip2: float = -0.02

class Gripper:
    # pylint: disable=too-many-instance-attributes
    """ A simple two finger prismatic gripper in agx """
    def __init__(self, sim, root, timestep, material, posref, visual):
        self.props = GripperProps()
        self.gripper = agxSDK.Assembly()
        self.gripper_ref = None
        self.posref = posref
        self.z_force_ref = 0.0
        self.z_force = 0.0
        self.force_duration = 0.2 / timestep #Lasts 0.2s
        self.force_applied = False
        self.force_count = 0
        self.z_offs = self.props.radius + self.props.height / 2
        self.gripper.setPosition(posref.x, posref.y, posref.z + self.z_offs)
        self.move_constraints = []
        self.sim = sim
        self.setup_geometries(root, material, visual)
        self.update_pos()
        if visual:
            self.setup_keyboard_listener()

    def setup_geometries(self, root, material, visual):
        # pylint: disable=too-many-statements, too-many-locals
        """ Create bodies and constraints so that we can move the gripper left/right/front/back """
        translate_bodies = [agx.RigidBody(), agx.RigidBody()]

        move_direction = [agx.Vec3(-1, 0, 0), agx.Vec3(0, -1, 0)]
        for i in range(0, 2):
            body = translate_bodies[i]
            body.setLocalPosition(0, 0, 0)
            body.getMassProperties().setMass(self.props.mass)
            self.gripper.add(body)

            if i == 0:
                frame1 = agx.Frame()
                frame1.setLocalTranslate(body.getPosition())
                frame1.setLocalRotate(agx.Quat(agx.Vec3(0, 0, -1), move_direction[i]))
                prismatic = agx.Prismatic(body, frame1)
            else:
                frame1 = agx.Frame()
                frame2 = agx.Frame()
                assert agx.Constraint.calculateFramesFromBody(agx.Vec3(), \
                    move_direction[i], translate_bodies[0], frame1, body, frame2)
                prismatic = agx.Prismatic(translate_bodies[0], frame1, body, frame2)

            self.gripper.add(prismatic)
            prismatic.getMotor1D().setLockedAtZeroSpeed(True)
            prismatic.getMotor1D().setForceRange(agx.RangeReal(50)) # N
            prismatic.getMotor1D().setSpeed(0)
            prismatic.getMotor1D().setEnable(True)
            prismatic.setSolveType(agx.Constraint.DIRECT_AND_ITERATIVE)
            prismatic.setCompliance(1e-15)
            self.move_constraints.append(prismatic)

        # Create a Cylindrical constraint that can be used for lifting AND rotating
        lift_rotate_body = agx.RigidBody()
        lift_rotate_body.setLocalPosition(0, 0, 0)
        lift_rotate_body.getMassProperties().setMass(self.props.mass)
        self.gripper.add(lift_rotate_body)
        frame1 = agx.Frame()
        frame2 = agx.Frame()
        assert agx.Constraint.calculateFramesFromBody( \
            agx.Vec3(), agx.Vec3(0, 0, -1), translate_bodies[1], frame1, lift_rotate_body, frame2)
        self.cylindrical = agx.CylindricalJoint(translate_bodies[1], frame1, lift_rotate_body, frame2)
        self.cylindrical.setSolveType(agx.Constraint.DIRECT_AND_ITERATIVE)
        self.cylindrical.setCompliance(1e-15)
        # Enable the motors and set some properties on the motors
        for d in [agx.Constraint2DOF.FIRST, agx.Constraint2DOF.SECOND]:
            self.cylindrical.getMotor1D(d).setEnable(True)
            self.cylindrical.getMotor1D(d).setLockedAtZeroSpeed(True)
            self.cylindrical.getMotor1D(d).setSpeed(0)
            if d == agx.Constraint2DOF.FIRST:
                self.cylindrical.getMotor1D(d).setForceRange(agx.RangeReal(15)) # N
            else:
                self.cylindrical.getMotor1D(d).setForceRange(agx.RangeReal(50)) # Nm
        self.gripper.add(self.cylindrical)

        # Create the actual fingers that is used for picking up screws.
        capsule1 = agxCollide.Geometry(agxCollide.Capsule(self.props.radius, self.props.height))
        capsule1.setLocalRotation(agx.EulerAngles(math.radians(90), 0, 0))
        capsule1.setMaterial(material)
        capsule2 = capsule1.clone()
        capsule1.setLocalPosition(0, -self.props.radius, 0)
        capsule2.setLocalPosition(0, self.props.radius, 0)
        self.finger1 = agx.RigidBody()
        self.finger1.add(capsule1)
        self.finger1.add(capsule2)
        self.finger2 = self.finger1.clone()
        fingers = [self.finger1, self.finger2]

        # Create the constraints that is used for open/close the picking device
        direction = [1, -1]
        self.grip_constraints = []
        grip_range = 0.025
        self.grip_offs = self.props.radius
        for i in range(0, 2):
            finger = fingers[i]
            finger.setLocalPosition(direction[i] * (self.grip_offs + self.posref.grip1), 0, 0)
            self.gripper.add(finger)
            finger.getMassProperties().setMass(self.props.mass)
            frame1 = agx.Frame()
            frame2 = agx.Frame()
            assert agx.Constraint.calculateFramesFromBody( \
                agx.Vec3(), agx.Vec3(-1, 0, 0) * direction[i], lift_rotate_body, frame1, finger, frame2)
            prismatic = agx.Prismatic(lift_rotate_body, frame1, finger, frame2)
            prismatic.getMotor1D().setForceRange(agx.RangeReal(-20, 20))
            prismatic.getMotor1D().setLockedAtZeroSpeed(True)
            prismatic.getMotor1D().setEnable(True)
            prismatic.getRange1D().setRange(agx.RangeReal(-self.posref.grip1, grip_range - self.posref.grip1))
            prismatic.getRange1D().setEnable(True)
            prismatic.setSolveType(agx.Constraint.DIRECT_AND_ITERATIVE)
            prismatic.setCompliance(1e-15)
            self.gripper.add(prismatic)
            self.grip_constraints.append(prismatic)

        self.sim.add(self.gripper)
        if visual:
            agxOSG.createVisual(self.gripper, root)

    def update_pos(self):
        """ Update local variables for position and rotation """
        self.current_pos_x = self.gripper.getRigidBodies()[0].getPosition().x()
        self.current_pos_y = self.gripper.getRigidBodies()[1].getPosition().y()
        self.current_pos_z = self.gripper.getRigidBodies()[2].getPosition().z() - self.z_offs
        self.current_rot_z = agx.EulerAngles(self.gripper.getRigidBodies()[2].getRotation()).z()

    def handle_key(self, data):
        # pylint: disable=too-many-branches
        """
        Callback function for keyboard events related to the gripper
        Used for debugging by joystick control
        """

        if data.isKeyDown:
            if data.key == ord('x'):
                self.posref.grip1 -= 0.005
                self.posref.grip2 += 0.005
            if data.key == ord('z'):
                self.posref.grip1 += 0.005
                self.posref.grip2 -= 0.005
            if data.key == agxSDK.GuiEventListener.KEY_Down:
                self.posref.x += 0.01
            if data.key == agxSDK.GuiEventListener.KEY_Up:
                self.posref.x -= 0.01
            if data.key == agxSDK.GuiEventListener.KEY_Left:
                self.posref.y -= 0.01
            if data.key == agxSDK.GuiEventListener.KEY_Right:
                self.posref.y += 0.01
            if data.key == agxSDK.GuiEventListener.KEY_Page_Up:
                self.posref.z += 0.01
            if data.key == agxSDK.GuiEventListener.KEY_Page_Down:
                self.posref.z -= 0.01
            if data.key == agxSDK.GuiEventListener.KEY_Home:
                self.posref.rotz += 0.1
                while self.posref.rotz > math.pi:
                    self.posref.rotz -= 2*math.pi
            if data.key == agxSDK.GuiEventListener.KEY_End:
                self.posref.rotz -= 0.1
                while self.posref.rotz < -math.pi:
                    self.posref.rotz += 2*math.pi

    def setup_keyboard_listener(self):
        """ Sets up listener for keyboard commands to control gripper """
        Input.bind(name="Left", key=agxSDK.GuiEventListener.KEY_Left, callback=self.handle_key)
        Input.bind(name="Right", key=agxSDK.GuiEventListener.KEY_Right, callback=self.handle_key)
        Input.bind(name="Back", key=agxSDK.GuiEventListener.KEY_Up, callback=self.handle_key)
        Input.bind(name="Forward", key=agxSDK.GuiEventListener.KEY_Down, callback=self.handle_key)
        Input.bind(name="Up", key=agxSDK.GuiEventListener.KEY_Page_Up, callback=self.handle_key)
        Input.bind(name="Down", key=agxSDK.GuiEventListener.KEY_Page_Down, callback=self.handle_key)
        Input.bind(name="Home", key=agxSDK.GuiEventListener.KEY_Home, callback=self.handle_key)
        Input.bind(name="End", key=agxSDK.GuiEventListener.KEY_End, callback=self.handle_key)
        Input.bind(name="Grip", key='z', callback=self.handle_key)
        Input.bind(name="Open", key='x', callback=self.handle_key)

    def set_ref(self, gripper_ref):
        """ Set gripper reference """
        self.gripper_ref = gripper_ref

    def handle_ref(self):
        """ Handle gripper reference, if any """
        if self.gripper_ref is not None:
            self.posref.x = self.gripper_ref[GripperRef.X_REF]
            self.posref.y = self.gripper_ref[GripperRef.Y_REF]
            self.posref.z = self.gripper_ref[GripperRef.Z_REF]

            if self.gripper_ref[GripperRef.CLOSE_GRIPPER] == 1.0:
                self.close()
            elif self.gripper_ref[GripperRef.CLOSE_GRIPPER] == 0.0:
                self.open()

            if self.gripper_ref[GripperRef.PRESS] == 1.0:
                self.force_applied = False
                self.force_count = 0

            self.z_force_ref = self.gripper_ref[GripperRef.FORCE]
            self.cylindrical.getMotor1D(agx.Constraint2DOF.FIRST).setForceRange(
                agx.RangeReal(self.z_force_ref))

    def is_gripping(self):
        """ Checks if gripper is closed, i.e. applying force between fingers """
        if self.grip_constraints[0].getMotor1D().getCurrentForce() > 15 or \
           self.grip_constraints[1].getMotor1D().getCurrentForce() < -15:
            return True
        return False

    def is_open(self):
        """ Checks if gripper is open, i.e not trying to move anywhere """
        if abs(self.grip_constraints[0].getMotor1D().getSpeed()) < 0.001 and \
           abs(self.grip_constraints[1].getMotor1D().getSpeed()) < 0.001:
            return True
        return False

    def close(self):
        """ Close gripper """
        self.posref.grip1 = -0.001
        self.posref.grip2 = 0.001

    def open(self):
        """ Open gripper """
        self.posref.grip1 = 0.02
        self.posref.grip2 = -0.02

    def controller_step(self):
        # pylint: disable=too-many-locals
        """ Step controller that makes gripper follow the references """
        translation_p = 10
        rotation_p = 10
        grip_p = 10
        grip_center_p = 10
        max_speed_trans = 1.5
        max_speed_rot = 1.5
        max_speed_grip = 0.1

        self.update_pos()

        self.z_force = self.cylindrical.getMotor1D(agx.Constraint2DOF.FIRST).getCurrentForce()
        if self.z_force_ref < 0 and self.z_force == self.z_force_ref:
            self.force_count += 1
            if self.force_count %self.force_duration == 0:
                self.force_count = 0
                self.force_applied = True
        else:
            self.force_count = 0

        self.gripper.getRigidBodies()[3].setParentFrame(self.gripper.getRigidBodies()[2].getFrame())
        self.gripper.getRigidBodies()[4].setParentFrame(self.gripper.getRigidBodies()[2].getFrame())
        current_grip1 = self.gripper.getRigidBodies()[3].getLocalPosition().x() - self.grip_offs
        current_grip2 = self.gripper.getRigidBodies()[4].getLocalPosition().x() + self.grip_offs

        diff_rot_z = (self.posref.rotz - self.current_rot_z)
        if diff_rot_z > math.pi:
            diff_rot_z -= 2*math.pi
        elif diff_rot_z < -math.pi:
            diff_rot_z += 2*math.pi

        diff_grip_center = (self.posref.grip1 + self.posref.grip2) - (current_grip1 + current_grip2)

        speed_ref_x = max(-max_speed_trans, min(max_speed_trans, translation_p * (self.posref.x - self.current_pos_x)))
        speed_ref_y = max(-max_speed_trans, min(max_speed_trans, translation_p * (self.posref.y - self.current_pos_y)))
        speed_ref_z = max(-max_speed_trans, min(max_speed_trans, translation_p * (self.posref.z - self.current_pos_z)))
        speed_ref_rotz = max(-max_speed_rot, min(max_speed_rot, rotation_p * diff_rot_z))

        speed_ref_grip_center = max(-max_speed_trans, min(max_speed_trans, grip_center_p * diff_grip_center))

        if speed_ref_grip_center > 0:
            speed_ref_grip1 = max(0, min(max_speed_grip, \
                grip_p * (self.posref.grip1 - current_grip1))) + speed_ref_grip_center
            speed_ref_grip2 = max(-max_speed_grip, min(0, \
                grip_p * -(self.posref.grip2 - current_grip2))) - speed_ref_grip_center
        else:
            speed_ref_grip1 = max(-max_speed_grip, min(0, \
                grip_p * (self.posref.grip1 - current_grip1))) + speed_ref_grip_center
            speed_ref_grip2 = max(0, min(max_speed_grip, \
                grip_p * -(self.posref.grip2 - current_grip2))) - speed_ref_grip_center

        self.move_constraints[0].getMotor1D().setSpeed(speed_ref_x)
        self.move_constraints[1].getMotor1D().setSpeed(speed_ref_y)
        self.cylindrical.getMotor1D(agx.Constraint2DOF.FIRST).setSpeed(speed_ref_z)
        self.cylindrical.getMotor1D(agx.Constraint2DOF.SECOND).setSpeed(speed_ref_rotz)
        self.grip_constraints[0].getMotor1D().setSpeed(speed_ref_grip1)
        self.grip_constraints[1].getMotor1D().setSpeed(speed_ref_grip2)
