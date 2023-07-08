import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


class ConrolUgv(Node):
    def __init__(self):
        super().__init__('twist_publisher')
        self.twist_publisher = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.create_timer(0.2, self.timer_callback)

    def timer_callback(self):
        msg = Twist()
        msg.linear.x = 0.5
        msg.angular.z = 0.0
        self.twist_publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    move_node = ConrolUgv()
    rclpy.spin(move_node)

    rclpy.shutdown()

if __name__ == '__main__':
    main()
