import socket
import struct
import threading
import rclpy
from rclpy.node import Node
import json
import time

from bt_mios_ros2_interface.msg import RobotState


class SharedData:
    def __init__(self):
        self.lock = threading.Lock()
        self.data = None

    def update(self, new_data):
        with self.lock:
            self.data = new_data

    def read(self):
        with self.lock:
            return self.data


class UDPReceiver:
    def __init__(self, shared_data, host='localhost', port=12346):
        self.shared_data = shared_data
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((host, port))

    def run(self):
        while True:
            data, addr = self.sock.recvfrom(1024)
            decoded_data = json.loads(data.decode())
            self.shared_data.update(decoded_data)
            time.sleep(0.1)


class ROSPublisher(Node):
    def __init__(self, shared_data):
        super().__init__('ros_publisher')
        self.shared_data = shared_data
        self.pub = self.create_publisher(RobotState, 'mios_state_topic', 10)

    def run(self):
        while rclpy.ok():
            data = self.shared_data.read()
            if data is not None:
                msg = RobotState()
                print("state read: ", data["TF_F_ext_K"][2])
                time.sleep(1)
            rclpy.spin_once(self)


def main(args=None):
    rclpy.init(args=args)

    shared_data = SharedData()

    udp_receiver = UDPReceiver(shared_data)
    ros_publisher = ROSPublisher(shared_data)

    t1 = threading.Thread(target=udp_receiver.run)
    t2 = threading.Thread(target=ros_publisher.run)

    t1.start()
    t2.start()

    t1.join()
    t2.join()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
