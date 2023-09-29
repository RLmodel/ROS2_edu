# !/usr/bin/env/ python3

import sys
import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.node import Node

class getpose(Node):
    def __init__(self):
        super().__init__('getamclpose')
        self.subscriber = self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.timer_callback, 10)

        self.subscriber  # prevent unused variable warning
        self.get_logger().info('==== Pose subscribe start ====\n')

    def timer_callback(self, msg):
        print("position X: ","{:.2f}".format(msg.pose.pose.position.x),
                "/ position Y: ","{:.2f}".format(msg.pose.pose.position.y),
                "/ orientation z: ","{:.2f}".format(msg.pose.pose.orientation.z),
                "/ orientation w: ","{:.2f}".format(msg.pose.pose.orientation.w),
                
                )
        #msg.pose.position.z=0

def main(args=None):
    rclpy.init(args=args)
    getpose_node = getpose()

    try:
        rclpy.spin(getpose_node)
    except KeyboardInterrupt:
        getpose_node.get_logger().info('==== Server stopped cleanly ====')
    except BaseException:
        getpose_node.get_logger().info('!! Exception in server:', file=sys.stderr)
        raise
    finally:
        # (optional - Done automatically when node is garbage collected)
        rclpy.shutdown()

if __name__ == '__main__':
    main()

