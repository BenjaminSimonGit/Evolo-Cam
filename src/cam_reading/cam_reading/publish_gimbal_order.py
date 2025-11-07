# This script is used for testing read_and_publish.py

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Vector3

import math


class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(Vector3, 'desired_gimbal_euler', 10)
        timer_period = 1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = Vector3()
        msg.x = 0.0
        msg.y = 180*math.sin(self.i / 5)
        msg.z = 0.0
        self.publisher_.publish(msg)
        print("Published order")
        self.i += 1


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()