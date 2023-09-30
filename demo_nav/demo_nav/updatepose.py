# !/usr/bin/env/ python3

import sys
import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.node import Node
from std_msgs.msg import String

class updatepose(Node):
    def __init__(self):
        super().__init__('poseupdate')
        self.subscriber = self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.timer_callback, 10)
        self.publisher_ = self.create_publisher(String, '/tracer_pose', 10)

        self.subscriber  # prevent unused variable warning
        self.get_logger().info('==== Pose subscribe start ====\n')
        self.i =0

    def timer_callback(self, msg):
        print(  "index :",str(self.i), 
                "/ pos.x:","{:.2f}".format(msg.pose.pose.position.x),
                "/ pos.y:","{:.2f}".format(msg.pose.pose.position.y),
                "/ ori.z:","{:.2f}".format(msg.pose.pose.orientation.z),
                "/ ori.w:","{:.2f}".format(msg.pose.pose.orientation.w),
                )
        
        #msg.pose.position.z=0
        msg = String()
        msg.data = str(self.i)
        self.publisher_.publish(msg)

        self.i +=1

def main(args=None):
    rclpy.init(args=args)
    posehandle_node = updatepose()

    try:
        rclpy.spin(posehandle_node)
    except KeyboardInterrupt:
        posehandle_node.get_logger().info('==== Server stopped cleanly ====')
    except BaseException:
        posehandle_node.get_logger().info('!! Exception in server:', file=sys.stderr)
        raise
    finally:
        # (optional - Done automatically when node is garbage collected)
        rclpy.shutdown()

if __name__ == '__main__':
    main()

