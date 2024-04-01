#pylint: disable=broad-except, too-many-public-methods, too-few-public-methods, too-many-instance-attributes
"""
ROS interface to agx
"""
import sys
import traceback
import time
from math import sqrt
from dataclasses import dataclass
from enum import IntEnum

try:
    #pylint: disable=import-error
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import Float32MultiArray
except Exception as e:
    print("ROS2 could not be imported. Please ensure that the ROS2 script 'local_setup.bat' has been run.\n")
    traceback.print_exc()
    sys.exit(2)

class RosPublisherArray(Node):
    """ ROS publisher class """
    def __init__(self, node_name, topic_name):
        super().__init__(node_name)
        self.publisher = self.create_publisher(Float32MultiArray, topic_name, 1)

    def send_data(self, data):
        """ Sends data to topic """
        msg = Float32MultiArray()
        msg.data = data
        self.publisher.publish(msg)

class RosSubscriberArray(Node):
    """ ROS subscriber class """
    def register_callback(self, topic, callback_function):
        """ Note: callback function must have an input parameter for the message """
        self.create_subscription(Float32MultiArray, topic, callback_function, 1)

@dataclass
class Pos:
    """
    Cartesian position
    """
    x: float = 0
    y: float = 0
    z: float = 0
    def __add__(self, other):
        return Pos(self.x + other.x, self.y + other.y, self.z + other.z)

    def __str__(self):
        return '(' + str(self.x) + ', ' + str(self.y) + ', ' + str(self.z) + ')'

    def distance(self, other):
        """ Returns distance to other position """
        return sqrt((self.x - other.x)**2 + \
            (self.y - other.y)**2 + \
            (self.z - other.z)**2)

class SceneRef(IntEnum):
    """ Indices for data in scene ref message """
    RESET = 0 #Set >0 to reset scene
    VIDEO = 1 #Set >0 to start video recording, back to 0 to stop video
    SHUTDOWN = 2 #Shut down simulation completely. The process will have to be restarted

class GripperRef(IntEnum):
    """ Indices for data in gripper ref message """
    X_REF = 0
    Y_REF = 1
    Z_REF = 2
    CLOSE_GRIPPER = 3
    PRESS = 4
    FORCE = 5

class SensorData(IntEnum):
    """
    Indices for data in sensor data message
    Number of bricks may vary so only start of
    that block is given
    """
    RESET = 0
    TICKS = 1
    GRIPPING = 2
    OPEN = 3
    PRESSED = 4
    GRIPPER_X = 5
    GRIPPER_Y = 6
    GRIPPER_Z = 7
    BRICKS_START = 8 #Every brick has four data values: x,y,z and height

class LauncherFeedback(IntEnum):
    """
    Indices for data in feedback received from agx launcher
    """
    ACK = 0
    ERROR = 1

class AgxInterface:
    """
    Class for handling the interface to the agx simulation from the behavior tree side
    """
    def __init__(self, rosid=1):
        self.rosid = str(rosid)
        self.sensor_data = None
        self.gripper_ref = [0.0] * len(GripperRef)
        self.gripper_ref[GripperRef.FORCE] = 30.0
        self.picked = None
        self.hold_picked = False
        self.force_applied = None
        self.launcher_fdb = None
        self.crash_count = 0

        try:
            rclpy.init()
        except Exception:
            if not rclpy.ok():
                print("Init error, not because of multiple init")

        self.ros_pub_sceneref = RosPublisherArray("bt_pub_scene_" + self.rosid, "scene_ref_" + self.rosid)
        self.ros_pub_gripref = RosPublisherArray("bt_pub_grip_" + self.rosid, "gripper_ref_" + self.rosid)
        self.ros_pub_restart = RosPublisherArray("bt_pub_restart_" + self.rosid, "restart_agx_" + self.rosid)
        self.ros_sub_sensor = RosSubscriberArray("bt_sub_sensor_" + self.rosid)
        self.ros_sub_sensor.register_callback("sensor_data_" + self.rosid, self.callback_sensor_data)
        self.ros_sub_launcher = RosSubscriberArray("bt_sub_launcher_" + self.rosid)
        self.ros_sub_launcher.register_callback("launcher_fdb_" + self.rosid, self.callback_launcher_fdb)

    def shutdown(self):
        """ Close agx ROS interfaces """
        self.ros_pub_sceneref.destroy_node()
        self.ros_pub_gripref.destroy_node()
        self.ros_sub_sensor.destroy_node()

    def reinit(self):
        """ Reinitialize agx ROS interfaces """
        self.ros_pub_sceneref = RosPublisherArray("bt_pub_scene_" + self.rosid, "scene_ref_" + self.rosid)
        self.ros_pub_gripref = RosPublisherArray("bt_pub_grip_" + self.rosid, "gripper_ref_" + self.rosid)
        self.ros_sub_sensor = RosSubscriberArray("bt_sub_sensor_" + self.rosid)
        self.ros_sub_sensor.register_callback("sensor_data_" + self.rosid, self.callback_sensor_data)

    def get_feedback(self, max_attempts=10):
        """
        Check ROS subscriber for new sensor feeback
        Checks for ERROR every ten seconds if no sensor feedback is received
        """
        self.clear_feedback()
        feedback_received = False
        attempts = 0
        while not feedback_received:
            rclpy.spin_once(self.ros_sub_sensor, timeout_sec=10.0)
            if self.sensor_data is None:
                attempts += 1
                rclpy.spin_once(self.ros_sub_launcher, timeout_sec=0.0)
                if self.launcher_fdb is not None and self.launcher_fdb[LauncherFeedback.ERROR] > 0.0:
                    print("Unexcepted ERROR, agx must be restarted: ", self.crash_count)
                    self.crash_count += 1
                    self.launcher_fdb = None
                    break
                if attempts >= max_attempts:
                    print("ERROR: No connection with agx, restarting. ", attempts)
                    break
            else:
                feedback_received = True

        return feedback_received

    def clear_feedback(self):
        """ Clear last received feedback """
        self.sensor_data = None

    def get_sensor_data(self):
        """ Returns last sensor data received """
        return self.sensor_data

    def send_references(self):
        """ Sends references via ROS """
        self.ros_pub_gripref.send_data(self.gripper_ref)

    def restart(self):
        """ Restarts agx process completely """
        scene_ref = [0.0] * len(SceneRef)
        scene_ref[SceneRef.SHUTDOWN] = 1.0

        while self.launcher_fdb is None:
            self.ros_pub_sceneref.send_data(scene_ref)
            self.ros_pub_restart.send_data([1.0])
            time.sleep(0.5)
            self.shutdown()
            self.reinit()
            rclpy.spin_once(self.ros_sub_launcher, timeout_sec=30)
            if self.launcher_fdb is not None and self.launcher_fdb[LauncherFeedback.ERROR] > 0.0:
                print("ERROR, launcher did not receive message, try again")
                self.launcher_fdb = None
            print("Attempted restart: ", self.launcher_fdb)
        self.launcher_fdb = None

        #Make sure ROS is up and running in the agx process
        while self.ros_pub_sceneref.count_subscribers("scene_ref_" + self.rosid) == 1:
            time.sleep(0.1)

        self.reset()

    def callback_sensor_data(self, msg):
        """ Save new sensor data from agx """
        self.sensor_data = msg.data

    def callback_launcher_fdb(self, msg):
        """ Save new feedback from launcher """
        self.launcher_fdb = msg.data

    def reset(self):
        """ Reset the scene """
        self.gripper_ref = [0.0] * len(GripperRef)
        self.gripper_ref[GripperRef.FORCE] = 30.0
        self.picked = None
        self.hold_picked = False
        self.force_applied = None

        scene_ref = [0.0] * len(SceneRef)
        scene_ref[SceneRef.RESET] = 1.0
        self.ros_pub_sceneref.send_data(scene_ref)
        #Wait for ack
        while True:
            self.get_feedback(max_attempts=1)
            if self.sensor_data is not None and self.sensor_data[SensorData.RESET] == 1.0:
                break
            #Reset package might have been lost, try again
            self.ros_pub_sceneref.send_data(scene_ref)

        scene_ref = [0.0] * len(SceneRef)
        self.ros_pub_sceneref.send_data(scene_ref)

    def at_standstill(self, old_sensor_data):
        """ Checks if system is at standstill by checking if the sensor data changes """
        if self.sensor_data[SensorData.GRIPPING] != old_sensor_data[SensorData.GRIPPING] or \
           self.sensor_data[SensorData.OPEN] != old_sensor_data[SensorData.OPEN] or \
           self.sensor_data[SensorData.PRESSED] != old_sensor_data[SensorData.PRESSED]:
            return False

        if not self.is_gripper_at(Pos(old_sensor_data[SensorData.GRIPPER_X], \
                                      old_sensor_data[SensorData.GRIPPER_Y], \
                                      old_sensor_data[SensorData.GRIPPER_Z]), atol=1.e-4):
            return False

        for brick in range(int((len(self.sensor_data) - SensorData.BRICKS_START) / 4)):
            old_position = self.get_brick_position(brick, sensor_data=old_sensor_data)
            if self.distance(brick, old_position) > 1.e-5:
                return False
        return True

    def start_video(self):
        """ Starts video capture """
        scene_ref = [0.0] * len(SceneRef)
        scene_ref[SceneRef.VIDEO] = 1.0
        self.ros_pub_sceneref.send_data(scene_ref)

    def stop_video(self):
        """ Stops video capture and saves film """
        scene_ref = [0.0] * len(SceneRef)
        self.ros_pub_sceneref.send_data(scene_ref)

    def set_gripper_ref(self, position=None, close_gripper=None, press=False, force=None):
        """ Sets gripper reference """
        if position is not None:
            self.gripper_ref[GripperRef.X_REF] = position.x
            self.gripper_ref[GripperRef.Y_REF] = position.y
            self.gripper_ref[GripperRef.Z_REF] = position.z
        if close_gripper is not None:
            self.gripper_ref[GripperRef.CLOSE_GRIPPER] = close_gripper
        if press:
            self.gripper_ref[GripperRef.PRESS] = 1.0
        else:
            self.gripper_ref[GripperRef.PRESS] = 0.0
        if force is not None:
            self.gripper_ref[GripperRef.FORCE] = force

    def close_gripper(self):
        """ Close gripper """
        self.set_gripper_ref(close_gripper=1.0)

    def open_gripper(self):
        """ Open gripper """
        self.set_gripper_ref(close_gripper=0.0)

    def move_to(self, position):
        """ Move gripper to given position """
        self.set_gripper_ref(position=position)

    def apply_force(self, brick):
        """ Apply force to given brick """
        self.set_gripper_ref(press=True, force=-1.0)
        self.force_applied = brick

    def release_force(self):
        """ Stop applying force and reset gripper force limits """
        self.set_gripper_ref(force=30.0)

    def is_gripper_closed(self):
        """ Checks if gripper is currently closed """
        if self.sensor_data[SensorData.GRIPPING] == 1.0:
            return True
        return False

    def is_gripper_open(self):
        """ Checks if gripper is currently open """
        if self.sensor_data[SensorData.OPEN] == 1.0:
            return True
        return False

    def set_picked(self, brick):
        """Simply memorizes which brick was picked for lack of better sensor"""
        self.picked = brick

    def set_hold_picked(self, hold=True):
        """
        Forcibly hold the picked state, used so that we don't prematurely
        call the hand empty while placing it in orderly fashion
        """
        self.hold_picked = hold

    def get_picked(self):
        """ Returns the index of the currently picked brick """
        if self.picked is not None and not self.hold_picked:
            brick_position = self.get_brick_position(self.picked)
            gripper_position = Pos(self.sensor_data[SensorData.GRIPPER_X], \
                self.sensor_data[SensorData.GRIPPER_Y], \
                self.sensor_data[SensorData.GRIPPER_Z])
            if self.is_gripper_open() or brick_position.distance(gripper_position) > 0.02:
                self.picked = None
        return self.picked

    def get_brick_position(self, brick, sensor_data=None):
        """ Returns position of given brick """
        brick_index = SensorData.BRICKS_START + 4 * brick
        if sensor_data is None:
            return Pos(self.sensor_data[brick_index],
                       self.sensor_data[brick_index + 1],
                       self.sensor_data[brick_index + 2])
        return Pos(sensor_data[brick_index],
                   sensor_data[brick_index + 1],
                   sensor_data[brick_index + 2])

    def get_brick_height(self, brick):
        """ Returns height of given brick """
        brick_index = SensorData.BRICKS_START + 4 * brick
        return self.sensor_data[brick_index + 3]

    def get_force_applied(self):
        """ Checks if force has been applied successfully """
        return self.sensor_data[SensorData.PRESSED] == 1

    def is_gripper_at(self, position, atol):
        """ Checks if gripper is within atol distance from given position """
        gripper_position = Pos(self.sensor_data[SensorData.GRIPPER_X],
                               self.sensor_data[SensorData.GRIPPER_Y],
                               self.sensor_data[SensorData.GRIPPER_Z])
        return gripper_position.distance(position) <= atol

    def distance(self, brick, position):
        """ Returns distance between given brick and given position """
        brick_position = self.get_brick_position(brick)
        return brick_position.distance(position)
