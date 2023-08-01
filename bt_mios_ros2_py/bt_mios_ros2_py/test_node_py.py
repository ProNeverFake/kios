# from example_interfaces.srv import AddTwoInts

# import rclpy
# from rclpy.node import Node


# class TestNode(Node):

#     def __init__(self):
#         super().__init__('test_node_py')
#         self.srv = self.create_service(
#             AddTwoInts, 'add_two_ints', self.add_two_ints_callback)

#     def add_two_ints_callback(self, request, response):
#         response.sum = request.a + request.b
#         self.get_logger().info('Incoming request\na: %d b: %d' % (request.a, request.b))

#         return response


# def main(args=None):
#     rclpy.init(args=args)

#     test_node_py = TestNode()

#     rclpy.spin(test_node_py)

#     rclpy.shutdown()


# if __name__ == '__main__':
#     main()

#     import rclpy
# from rclpy.node import Node
# from std_srvs.srv import Empty

# class ServiceNode(Node):
#     def __init__(self):
#         super().__init__('service_node')
#         self.srv = self.create_service(Empty, 'test_service', callback=self.service_callback)

#     def service_callback(self, request, result):
#         self.get_logger().info('Received request, responding...')
#         return result


# if __name__ == '__main__':
#     rclpy.init()
#     node = ServiceNode()
#     try:
#         node.get_logger().info("Starting server node, shut down with CTRL-C")
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         node.get_logger().info('Keyboard interrupt, shutting down.\n')
#     node.destroy_node()
#     rclpy.shutdown()


import rclpy
from rclpy.node import Node
from std_srvs.srv import Empty
from example_interfaces.srv import AddTwoInts
from bt_mios_ros2_interface.srv import RequestState


class ServiceNode(Node):
    def __init__(self):
        super().__init__('service_node')
        # self.srv = self.create_service(
        #     Empty, 'test_service', callback=self.service_callback)
        self.srv = self.create_service(
            RequestState, 'request_state', callback=self.service_callback)

    def service_callback(self, request, result):
        self.get_logger().info('Received request, responding...')
        result.tf_f_ext_k = [0.001, 0.001, 0.001, 0.001, 0.001, 0.001]
        return result


def main():
    rclpy.init()
    node = ServiceNode()
    try:
        node.get_logger().info("Starting server node, shut down with CTRL-C")
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard interrupt, shutting down.\n')
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
