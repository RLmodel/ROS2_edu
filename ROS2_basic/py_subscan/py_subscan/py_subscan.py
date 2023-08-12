import sys
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
        len720_right = format(msg.ranges[719], '.2f')
        len360_front = format(msg.ranges[359], '.2f')
        self.get_logger().info(f'{len0_left} {len720_right}  {len360_front}')
        self.get_logger().info("Left Right Front")

def main(args=None):
    rclpy.init(args=args)
    laser_sub = ScanSub()
    '''
    rclpy.spin(laser_sub)

    laser_sub.destroy_node()
    rclpy.shutdown()
    '''
    try:
        rclpy.spin(laser_sub)
    except KeyboardInterrupt:
        laser_sub.get_logger().info('==== Server stopped cleanly ====')
    except BaseException:
        laser_sub.get_logger().info('!! Exception in server:', file=sys.stderr)
        raise
    finally:
        # (optional - Done automatically when node is garbage collected)
        rclpy.shutdown()

if __name__=='__main__':
    main()