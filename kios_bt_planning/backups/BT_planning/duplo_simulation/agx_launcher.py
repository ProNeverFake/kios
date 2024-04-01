"""
This file is used for launching agx simulation environments
and handling them if they crash for some reason or
need to be restarted.

This is needed because every once in a while agx will crash
and after running a few thousand simulations it gets slow,
most likely due to memory leaks and needs to be restarted

This is not needed if only running once or so, then the
simulation can be called directly instead
"""
import sys
import traceback
import subprocess
import time
import duplo_simulation.agx_interface as interface

try:
    import rclpy #pylint: disable=import-error
except Exception as e: #pylint: disable=broad-except
    print("ROS2 could not be imported. Please ensure that the ROS2 script 'local_setup.bat' has been run.\n")
    traceback.print_exc()
    sys.exit(2)

class AgxLauncher():
    """ Main launcher class """
    def __init__(self, rosid):
        self.rosid = str(rosid)
        #Path to the simulation to run, change this to run a different simulation
        #self.simulation_path = "duplo_simulation/agx_duplo_tower.py"
        #self.simulation_path = "duplo_simulation/agx_duplo_croissant.py"
        #self.simulation_path = "duplo_simulation/agx_duplo_balance.py"
        self.simulation_path = "duplo_simulation/agx_duplo_blocking.py"

        rclpy.init()
        self.ros_publisher = interface.RosPublisherArray("agx_launcher_pub_" + self.rosid, "launcher_fdb_" + self.rosid)
        self.ros_subscriber = interface.RosSubscriberArray("agx_launcher_sub_" + self.rosid)
        self.ros_subscriber.register_callback("restart_agx_" + self.rosid, self.restart_agx_callback)
        self.agx_process = None
        self.restart = False

    def restart_agx_callback(self, _msg):
        """ Callback for restart commands """
        self.restart = True

    def open_subprocess(self):
        """ Opens agx subprocess """
        self.agx_process = subprocess.Popen(['python', self.simulation_path, self.rosid, "1"])

    def run(self):
        """
        Main procedure, will loop until stopped manually
        """
        self.open_subprocess()
        try:
            while True:
                if self.restart:
                    #Call to close procedure is handled elsewhere
                    #Here we just make sure process is not already running
                    while self.agx_process.poll() is None:
                        time.sleep(0.1)
                    self.open_subprocess()
                    while self.agx_process.poll() is not None: #Make sure process is running
                        time.sleep(0.1)
                    feedback = [0.0] * len(interface.LauncherFeedback)
                    feedback[interface.LauncherFeedback.ACK] = 1.0
                    self.ros_publisher.send_data(feedback) #Acknowledge reset
                    self.restart = False

                rclpy.spin_once(self.ros_subscriber, timeout_sec=1.0)
                if not self.restart and self.agx_process.poll() is not None:
                    feedback = [0.0] * len(interface.LauncherFeedback)
                    feedback[interface.LauncherFeedback.ERROR] = 1.0
                    self.ros_publisher.send_data(feedback)

        except KeyboardInterrupt:
            self.agx_process.kill()
            self.ros_subscriber.destroy_node()
            self.ros_publisher.destroy_node()

        time.sleep(2)
        print("Launcher done", self.agx_process.poll())

INSTANCE_ID = "1"
if len(sys.argv) > 1:
    INSTANCE_ID = sys.argv[1]

LAUNCHER = AgxLauncher(INSTANCE_ID)
LAUNCHER.run()
rclpy.shutdown()
