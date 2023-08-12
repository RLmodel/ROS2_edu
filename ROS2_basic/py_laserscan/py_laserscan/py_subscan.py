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
        len0_left = format(msg.ranges[0], '.2f')
        len720_right = format(msg.ranges[720], '.2f')
        len360_front = format(msg.ranges[360], '.2f')
        self.get_logger().info(f'{len0_left} {len720_right} {len360_front}')
        self.get_logger().info("Left / Right / Front")
        print("Left / Right / Front")

def main(args=None):
    rclpy.init(args=args)
    laser_sub = ScanSub()
    rclpy.spin(laser_sub)

    laser_sub.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()