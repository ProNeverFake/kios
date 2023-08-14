import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.qos import qos_profile_sensor_data

import json

from .resource.mongodb_client import MongoDBClient

from kios_interface.srv import GetObject


class MongoReader(Node):

    udp_subscriber = ''

    def __init__(self):
        super().__init__('mongo_reader')

        # declare flag
        self.is_running = True

        # declare parameters
        self.declare_parameter('power_on', True)

        # intialize mongoDB client
        self.mongo_client_ = MongoDBClient(port=27017)

        # initialize timer
        timer_callback_group = ReentrantCallbackGroup()
        self.timer_ = self.create_timer(
            2,  # sec
            self.timer_callback,
            callback_group=timer_callback_group)
        
        # initialize get_object server
        server_callback_group = MutuallyExclusiveCallbackGroup()
        self.server_ = self.create_service(
            GetObject,
            "get_object_service",
            self.server_callback,
            callback_group=server_callback_group
        )


    def timer_callback(self):
        if self.is_running:
            self.get_logger().info('start')
            result = self.mongo_client_.read("miosL", "environment", {"name": "housing"})
            print(result[0]['O_T_OB'])
            self.get_logger().info('stop')

        else:
            self.get_logger().error('not runnning, timer pass ...')
            pass

    def server_callback(self, request, response):
        response.object_data = ""
        response.error_message = ""
        response.is_success = False
        if self.is_running:
            object_list = request.object_list
            if len(object_list) <= 0:
                self.get_logger().error('null object list!')
                response.error_message = "null object list"
                return response
            else:
                object_result = {}
                for object_name in object_list:
                    try:
                        result = self.mongo_client_.read("miosL", "environment", {"name": object_name})[0]
                        object_result[object_name] = result
                    except:
                        self.get_logger().error('UNKNOWN ERROR IN MONGO CLIENT READ() OF OBEJCT %s', object_name)
                
                if len(object_result) <= 0:
                    response.error_message = "read result == null"
                    return

                response.is_success = True
                response.object_data = json.dumps(object_result)
                return response

        else:
            self.get_logger().error('not runnning, server pass ...')
            response.error_message = "is_running == False"
            return response 


def main(args=None):
    rclpy.init(args=args)

    mongo_reader = MongoReader()

    executor = MultiThreadedExecutor()
    executor.add_node(mongo_reader)

    executor.spin()

    mongo_reader.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
