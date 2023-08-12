import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from sensor_msgs.msg import LaserScan

class ScanSub(Node):
    def __init__(self):
        super().__init__('scan_sub')
        self.subscription = self.create_subscription(
            LaserScan,
            '/diffbot/scan',
            self.listener_callback,
            10
        )

    def listener_callback(self, msg):
        dist_back = format(msg.ranges[180], '.2f')
        dist_left = format(msg.ranges[90], '.2f')
        dist_right = format(msg.ranges[0], '.2f')
        dist_head = format(msg.ranges[0], '.2f')
        self.get_logger().info(f'{dist_back} {dist_left} {dist_right} {dist_head}')

def main(args=None):
    rclpy.init(args=args)
    laser_sub = ScanSub()
    rclpy.spin(laser_sub)

    laser_sub.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()