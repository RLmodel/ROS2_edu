import rclpy
from rclpy.node import Node

class MyNode(Node):
    def __init__(self):
        super().__init__("first_node")
        self.get_logger().info("Hello RLmodel ROS2")

def main(args=None):
    rclpy.init(args=args)
    node = MyNode()
    rclpy.shutdown()

if __name__== '__main__':
    main()