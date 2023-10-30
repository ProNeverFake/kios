# pylint: disable=too-few-public-methods, broad-except
"""
Common functions for different agx simulations
"""
import sys
import traceback
import math
from dataclasses import dataclass

import agx
import agxSDK
import agxRender
import agxPython
import agxUtil

import duplo_simulation.agx_interface as interface
import duplo_simulation.agx_common_geometries as common_geometries
import duplo_simulation.agx_duplo_bricks as duplo_bricks
import duplo_simulation.agx_gripper as gripper

try:
    import rclpy #pylint: disable=import-error
except Exception as e:
    print("ROS2 could not be imported. Please ensure that the ROS2 script 'local_setup.bat' has been run.\n")
    traceback.print_exc()
    sys.exit(2)

class HoleContactListener(agxSDK.ContactEventListener):
    #pylint: disable=arguments-differ
    """
    A contact event listener that listen to the sensor contacts
    so anything interacting with the sensor will be triggered.
    Inspired by agx peg-in-hole example
    """
    def __init__(self):
        super().__init__()
        self.constraint = None
        self.stud_sensor = None
        self.hole_depth = None
        self.setEnable(False)

    def reset(self):
        """ Reset the listener """
        self.setEnable(False)
        self.constraint = None
        self.stud_sensor = None
        self.hole_depth = None

    def impact(self, time, geometry_contact):
        """ Called upon contact between sensors and anything else """
        if self.constraint:
            return agxSDK.ContactEventListener.KEEP_CONTACT

        stud_index = 0
        hole_index = 1
        # Make sure it is a "stud" that is colliding with this sensor
        if not (geometry_contact.geometry(0).hasGroup("stud") or geometry_contact.geometry(1).hasGroup("stud")):
            return agxSDK.ContactEventListener.KEEP_CONTACT
        # Check which of the geometries/bodies that is the stud
        if not geometry_contact.geometry(stud_index).hasGroup("stud"):
            stud_index = 1
            hole_index = 0

        self.stud_sensor = geometry_contact.geometry(stud_index)

        hole_body = geometry_contact.geometry(hole_index).getRigidBody()
        stud_body = geometry_contact.geometry(stud_index).getRigidBody()

        hole_frame = geometry_contact.geometry(hole_index).getFrame()
        stud_frame = geometry_contact.geometry(stud_index).getFrame()
        self.constraint = agx.SlackCylindricalJoint(hole_body, hole_frame, stud_body, stud_frame)

        self.constraint.setCompliance(1.e-10)
        self.constraint.setSlackParameters(agx.Vec2(0.00001), 0.001)
        self.constraint.setDamping(0.5, -1)

        #This motor simulates the friction in the rotational direction
        motor = self.constraint.getMotor1D(agx.Constraint2DOF.SECOND)
        motor.setEnable(True)
        motor.setForceRange(agx.RangeReal(0.5))

        #This motor simulates the friction in the translational direction
        motor = self.constraint.getMotor1D(agx.Constraint2DOF.FIRST)
        motor.setEnable(True)
        motor.setCompliance(0)
        motor.setForceRange(agx.RangeReal(0.5))
        # Add the constraint to the simulation
        self.getSimulation().add(self.constraint)

        # Limit the linear motion according to the hole radius
        range_1d = self.constraint.getRange1D(agx.Constraint2DOF.FIRST)
        range_1d.setEnable(True)
        range_1d.setRange(agx.RangeReal(-1.0 * self.hole_depth, math.inf))

        return agxSDK.ContactEventListener.KEEP_CONTACT

    def separation(self, time, geometry_pair):
        """ Called upon separation between the sensor and something else """
        if self.stud_sensor in geometry_pair and self.constraint is not None:
            self.getSimulation().remove(self.constraint)
            self.constraint = None
            self.stud_sensor = None

class ListenerPool():
    """
    Pool of listeners to use.
    Because of the way agx works, listeners cannot be removed so
    in order to avoid memory leaks we keep a constant list of listeners that
    may be used by the simulations
    """
    def __init__(self, size):
        self.size = size
        self.hole_listeners = []
        for _ in range(size):
            self.hole_listeners.append(HoleContactListener())

    def reset(self):
        """ Reset all listeners in pool """
        for listener in self.hole_listeners:
            listener.reset()

    def get_listener(self):
        """ Returns a free listener from pool """
        for listener in self.hole_listeners:
            if not listener.isEnabled():
                return listener
        print("Error, listener pool empty!")
        return None

class GuiListener(agxSDK.GuiEventListener):
    """ Guilistener for printing statistics on screen """
    def __init__(self, app, timestep):
        super().__init__(agxSDK.GuiEventListener.UPDATE + agxSDK.GuiEventListener.KEYBOARD)
        agx.Statistics.instance().setEnable(True)

        self.app = app
        self.timestep = timestep

        self.avg_time = agxUtil.MedianStatistic(100)
        self.avg_interstep = agxUtil.MedianStatistic(100)

    def update(self, x, y):
        # pylint: disable=line-too-long
        sim = agxPython.getContext().environment.getSimulation()
        agx.Statistics.instance().setEnable(True)
        step_forward = agx.Statistics.instance().getTimingInfo("Simulation", "Step forward time")
        interstep = agx.Statistics.instance().getTimingInfo("Simulation", "Inter-step time")
        self.avg_time.update(step_forward.current)
        self.avg_interstep.update(interstep.current)

        num_islands = agx.Statistics.instance().getTimingInfo("DynamicsSystem", "Num solve islands")
        if num_islands.current > -1:
            self.app.getSceneDecorator().setText(0, "Simulation time             : {0:1.2f}".format(self.avg_time.get()))
            self.app.getSceneDecorator().setText(1, "Interstep time              : {0:1.2f}".format(self.avg_interstep.get()))
            self.app.getSceneDecorator().setText(2, "Total time                  : {0:1.2f}".format(self.avg_interstep.get()+self.avg_time.get()))
            self.app.getSceneDecorator().setText(3, "Timestep                    : {0:1.4f}".format(self.timestep))
            self.app.getSceneDecorator().setText(4, "Number of simulation islands: {0:1.0f}".format(num_islands.current))
            self.app.getSceneDecorator().setText(5, "Number of threads           : {}".format(agx.getNumThreads()))
            self.app.getSceneDecorator().setText(6, "% of real time              : {0:1.0f}".format((self.avg_interstep.get()+self.avg_time.get()) / self.timestep / 10))
            self.app.getSceneDecorator().setText(7, "AMOR:                       : {}".format(sim.getMergeSplitHandler().getEnable()))

def setup_camera(app):
    """ Setup the default camera angle etc. """
    camera_data = app.getCameraData()
    camera_data.eye = agx.Vec3(0.5, 0, 0.25)
    camera_data.center = agx.Vec3(0, 0, 0)
    camera_data.up = agx.Vec3(0, 0, 1)
    camera_data.nearClippingPlane = 0.1
    camera_data.farClippingPlane = 5000
    app.applyCameraData(camera_data)

def start_video_capture():
    """ Start capturing video """
    app = agxPython.getContext().environment.getApplication()
    app.setupVideoCaptureRenderTotexture()
    app.setAllowWindowResizing(False)
    vc = app.getVideoServerCapture()
    vc.setFilename("agx_movie")
    vc.setEnableSyncWithSimulation(True)
    vc.setVideoFPS(30)
    vc.setImageCaptureFPS(30)
    vc.startCapture()

def stop_video_capture():
    """ Stop the video capture process (FFMPEG). This allows a new video to be created. """
    app = agxPython.getContext().environment.getApplication()
    vc = app.getVideoServerCapture()
    vc.stopCapture()
    vc.stopProcess()
    app.setAllowWindowResizing(True)

class RosInterface():
    """
    Class containing the ros interface, which is the only thing never shut down or reset
    Because of that, this class also keep the scene object and settings
    """
    def __init__(self, rosid, ros_ticks, scene_settings):
        self.scene_settings = scene_settings
        self.scene = Scene(scene_settings)
        self.scene_ref = None
        self.rosid = str(rosid) #Id of ROS interface, if we are running multiple in parallell
        self.ros_ticks = ros_ticks #Simulation ticks per ros communication
        self.ticks = 0.0
        self.reset_ack = 0.0

        if scene_settings.visual:
            self.step_listener = StepListener(self.step)
            self.scene_settings.sim.add(self.step_listener)

        self.ros_publisher = interface.RosPublisherArray("agx_pub_" + self.rosid, "sensor_data_" + self.rosid)
        self.ros_subscriber = interface.RosSubscriberArray("agx_sub_" + self.rosid)
        self.ros_subscriber.register_callback("scene_ref_" + self.rosid, self.callback_scene_ref)
        self.ros_subscriber.register_callback("gripper_ref_" + self.rosid, self.callback_gripper_ref)

        self.running = True

    def reset(self):
        """ Resets simulation to make it possible to rerun simulation without having to restart agx and ROS """
        if self.scene_settings.visual:
            self.scene_settings.sim.remove(self.step_listener)
            del self.step_listener

        self.scene_settings.sim.cleanup(self.scene_settings.sim.CLEANUP_ALL, True)
        self.scene_settings.listener_pool.reset()

        #Rebuild everything
        self.scene = Scene(self.scene_settings)
        self.scene_ref = None
        self.ticks = 0.0

        if self.scene_settings.visual:
            self.step_listener = StepListener(self.step)
            self.scene_settings.sim.add(self.step_listener)
            self.scene_settings.sim.add(GuiListener(
                agxPython.getContext().environment.getApplication(),
                self.scene_settings.timestep))

        self.reset_ack = 1.0

    def shutdown(self):
        """ Destroys all ROS nodes """
        self.ros_subscriber.destroy_node()
        self.ros_publisher.destroy_node()

    def step(self):
        """ Called after every agx step """
        if self.ticks % self.ros_ticks == 0:
            self.publish_sensors()
        elif (self.ticks - 2) % self.ros_ticks == 0:
            rclpy.spin_once(self.ros_subscriber)
            self.check_scene_ref()
            self.scene.gripper.handle_ref()
        if self.reset_ack != 1.0:
            self.ticks += 1.0
            self.scene.gripper.controller_step()

        return self.running

    def callback_scene_ref(self, msg):
        """ Save scene reference message """
        self.scene_ref = msg.data

    def callback_gripper_ref(self, msg):
        """ Save gripper reference message """
        self.scene.gripper.set_ref(msg.data)

    def check_scene_ref(self):
        """
        Checks for any scene reference
        index 0: Reset if larger than 0
        """
        if self.scene_ref is not None:
            if self.scene_ref[interface.SceneRef.SHUTDOWN] > 0.0:
                self.running = False
            elif self.scene_ref[interface.SceneRef.RESET] > 0.0:
                self.reset()
            elif self.scene_settings.visual:
                if self.scene_ref[interface.SceneRef.VIDEO] > 0.0:
                    start_video_capture()
                elif self.scene_ref[interface.SceneRef.VIDEO] == 0.0:
                    stop_video_capture()

            self.scene_ref = None

    def publish_sensors(self):
        """
        Publish (virtual) sensor data to other ros nodes
        """
        sensor_data = [0.0] * interface.SensorData.BRICKS_START

        sensor_data[interface.SensorData.RESET] = self.reset_ack
        self.reset_ack = 0.0

        sensor_data[interface.SensorData.TICKS] = self.ticks

        if self.scene.gripper.is_gripping():
            sensor_data[interface.SensorData.GRIPPING] = 1.0
        else:
            sensor_data[interface.SensorData.GRIPPING] = 0.0

        if self.scene.gripper.is_open():
            sensor_data[interface.SensorData.OPEN] = 1.0
        else:
            sensor_data[interface.SensorData.OPEN] = 0.0

        if self.scene.gripper.force_applied:
            sensor_data[interface.SensorData.PRESSED] = 1.0
            self.scene.gripper.force_applied = False
        else:
            sensor_data[interface.SensorData.PRESSED] = 0.0

        sensor_data[interface.SensorData.GRIPPER_X] = self.scene.gripper.current_pos_x
        sensor_data[interface.SensorData.GRIPPER_Y] = self.scene.gripper.current_pos_y
        sensor_data[interface.SensorData.GRIPPER_Z] = self.scene.gripper.current_pos_z

        for i in range(len(self.scene.bricks)):
            pos = self.scene.bricks[i].brick_rb.getPosition()
            sensor_data.append(pos[0])
            sensor_data.append(pos[1])
            sensor_data.append(pos[2])
            sensor_data.append(self.scene.bricks[i].height) #We need also height sometimes

        self.ros_publisher.send_data(sensor_data)

class StepListener(agxSDK.StepEventListener):
    """ Step listener for agx """
    def __init__(self, post):
        super().__init__()
        self.post_func = post

    def post(self, time):
        self.post_func()

@dataclass
class SceneSettings():
    """ Settings for the scene class """
    sim = None
    root = None
    start_positions = None
    brick_types = None
    timestep = None
    visual = None
    listener_pool = None

class Scene():
    """ Create the scene and everything in it """
    def __init__(self, settings):
        table_material, gripper_material, brick_material = setup_material_coff(settings.sim)

        colors = [agxRender.Color.Green(), agxRender.Color.RoyalBlue(), agxRender.Color.Red(), \
            agxRender.Color.Yellow(), agxRender.Color.HotPink(), agxRender.Color.WhiteSmoke(), \
            agxRender.Color.Turquoise(), agxRender.Color.DeepSkyBlue(), agxRender.Color.Orange()]

        settings.sim.getMergeSplitHandler().setEnable(True)

        table = common_geometries.add_table(settings.sim, settings.root, \
            height=0.01, length=0.4, width=0.4, material=table_material, visual=settings.visual)

        self.bricks = []
        for i in range(len(settings.start_positions)):
            self.bricks.append(duplo_bricks.new_brick(settings.sim, settings.root, settings.brick_types[i],\
                settings.start_positions[i], brick_material, settings.visual, colors[i], settings.listener_pool))
        for brick in self.bricks:
            for other_brick in self.bricks:
                brick.remove_hole_collisions(other_brick.brick_rb)
            brick.remove_hole_collisions(table)

        start_posref = gripper.PosRef(x=0.0, y=-0.0, z=0.05, rotz=math.radians(0), grip1=0.02, grip2=-0.02)
        self.gripper = gripper.Gripper(settings.sim, settings.root, \
            settings.timestep, gripper_material, start_posref, settings.visual)

def setup_material_coff(sim):
    """ Set up all the materials properties for the simulation """
    brick_damping = 1.0/4
    brick_youngs = 2.0E9

    brick_material = agx.Material("brick_material")
    brick_material.getSurfaceMaterial().setRoughness(0.5)
    brick_material.getSurfaceMaterial().setViscosity(1.e-15)
    brick_bulk = brick_material.getBulkMaterial()
    brick_bulk.setDamping(brick_damping)
    brick_bulk.setYoungsModulus(brick_youngs)
    gripper_material = agx.Material("gripper_material")
    gripper_material.getSurfaceMaterial().setRoughness(50)
    gripper_bulk = gripper_material.getBulkMaterial()
    gripper_bulk.setDamping(brick_damping)
    gripper_bulk.setYoungsModulus(brick_youngs)
    table_material = agx.Material("table_material")
    table_material.getSurfaceMaterial().setRoughness(10)
    table_bulk = table_material.getBulkMaterial()
    table_bulk.setDamping(brick_damping * 100)
    table_bulk.setYoungsModulus(brick_youngs * 100)

    friction_model = agx.IterativeProjectedConeFriction()
    friction_model.setSolveType(agx.FrictionModel.SPLIT)

    cm_brick_2_brick = sim.getMaterialManager().getOrCreateContactMaterial(brick_material, brick_material)
    cm_brick_2_brick.setFrictionModel(friction_model)
    cm_brick_2_brick.setRestitution(0.5)

    cm_brick_2_table = sim.getMaterialManager().getOrCreateContactMaterial(brick_material, table_material)
    cm_brick_2_table.setFrictionModel(friction_model)
    cm_brick_2_table.setRestitution(0.5)

    cm_gripper_2_brick = sim.getMaterialManager().getOrCreateContactMaterial(gripper_material, brick_material)
    friction_model_gripper = agx.IterativeProjectedConeFriction()
    friction_model_gripper.setSolveType(agx.FrictionModel.DIRECT) #Gripper may slip with inaccurate SPLIT solver
    cm_gripper_2_brick.setFrictionModel(friction_model_gripper)
    cm_gripper_2_brick.setRestitution(0.1)

    return table_material, gripper_material, brick_material

def init(visual, timestep):
    """ Initialize ROS2 and various other things common to all agx simulations """
    try:
        rclpy.init()
    except Exception:
        if not rclpy.ok():
            print("Init error, not because of multiple init")
    if visual:
        app = agxPython.getContext().environment.getApplication()
        sim = agxPython.getContext().environment.getSimulation()
        root = agxPython.getContext().environment.getSceneRoot()
    else:
        app = []
        sim = agxSDK.Simulation()
        root = []

    sim.setTimeStep(timestep)
    if visual:
        setup_camera(app)
        sim.add(GuiListener(app, timestep))
        sim.getRenderManager().setScaleFactor(0.03)
        app.getSceneDecorator().setBackgroundColor(agxRender.Color.SkyBlue(), agxRender.Color.DodgerBlue())

    agx.setNumThreads(2)

    return sim, root

def main_loop(scene_settings, rosid, ros_step):
    """
    Main loop that keeps simulation running until interrupted
    """
    sim, root = init(scene_settings.visual, scene_settings.timestep)
    scene_settings.sim = sim
    scene_settings.root = root

    ros_ticks = ros_step / scene_settings.timestep
    ros_interface = RosInterface(rosid, ros_ticks, scene_settings)
    if scene_settings.visual:
        return root
    try:
        running = True
        while running:
            scene_settings.sim.stepForward()
            running = ros_interface.step()
        print("Running loop ended, shutting down")
    except KeyboardInterrupt:
        ros_interface.shutdown()
        rclpy.shutdown() # Shut down ROS2
        print("Stopped by keyboard")
    except Exception as ex:
        template = "An exception of type {0} occurred. Arguments:\n{1!r}"
        message = template.format(type(ex).__name__, ex.args)
        print(message)

    return root
