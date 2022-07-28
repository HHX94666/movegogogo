'''
Date: 2022-03-10 17:01:04
LastEditors: houhuixiang
'''
from chassis_interfaces.srv import GetString

import rclpy
from rclpy.node import Node


class MinimalService(Node):

    def __init__(self):
        super().__init__('minimal_service')
        self.srv = self.create_service(GetString, '/auto_charge/cancel', self.add_two_ints_callback)

    def add_two_ints_callback(self, request, response):
        response.data = "succese"
        self.get_logger().info('Incoming request')

        return response


def main(args=None):
    rclpy.init(args=args)

    minimal_service = MinimalService()

    rclpy.spin(minimal_service)

    rclpy.shutdown()


if __name__ == '__main__':
    main()