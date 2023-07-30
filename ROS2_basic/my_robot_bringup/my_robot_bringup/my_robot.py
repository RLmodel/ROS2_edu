import rclpy
from rclpy.node import Node

class MyNode(Node):
    def __init__(self):
        super().__init__("first_node")
        self.counter = 0
        self.get_logger().info("Hello RLmodel ROS2")
        self.create_timer(1.0, self.timer_callback)

    def timer_callback(self): # inside node
        self.get_logger().info("hi log "+str(self.counter))
        self.counter +=1

def main(args=None):
    rclpy.init(args=args)
    node = MyNode()
    rclpy.spin(node) # continuously running
    rclpy.shutdown()

if __name__== '__main__':
    main()