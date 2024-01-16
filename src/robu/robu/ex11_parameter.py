import rclpy
from rclpy.node import Node

class MinimalParameter(Node):
    def __init__(self):
        super().__init__('MinimalParameter')

        self.declare_parameter('my_parameter')

        my_param = self.get_parameter('my_parameter')

        print("my_parameter: ", my_param)


def main():
    rclpy.init()

    node = MinimalParameter()

    rclpy.spin(node)
