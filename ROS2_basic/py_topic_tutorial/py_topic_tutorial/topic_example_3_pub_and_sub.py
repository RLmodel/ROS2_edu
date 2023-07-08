# Copyright 2021 @RoadBalance
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.


import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan


class ParkingNode(Node):

    def __init__(self):

        super().__init__('parking_node')
        queue_size = 10  # Queue Size

        self.twist_publisher = self.create_publisher(Twist, '/diffbot/cmd_vel', queue_size)
        self.scan_subscriber = self.create_subscription(
            LaserScan, '/diffbot/scan', self.sub_callback, queue_size
        )

    def sub_callback(self, msg):

        twist_msg = Twist()
        distance_forward = msg.ranges[60]
        print('distance ', distance_forward)

        if distance_forward > 0.5:
            self.get_logger().info(f'Distance from Front Object : {distance_forward}')
            twist_msg.linear.x = 0.5
            self.twist_publisher.publish(twist_msg)
        else:
            self.get_logger().info('==== Parking Done!!! ====\n')
            twist_msg.linear.x = 0.0
            self.twist_publisher.publish(twist_msg)

def main(args=None):
    """Do enter into this main function first."""
    rclpy.init(args=args)

    parking_node = ParkingNode()

    rclpy.spin(parking_node)

    parking_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    """main function"""
    main()
