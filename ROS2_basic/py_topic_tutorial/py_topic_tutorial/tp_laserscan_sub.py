# Copyright 2023 @RLmodel
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


"""
This is second example code for ROS 2 topic subscriber.

Let's learn about those things.

Create topic subscriber then check the value from that with ros2 command line tools.
Listen to pose of turtle in the turtlesim.
"""

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan

from rclpy.qos import qos_profile_sensor_data
from rclpy.qos import qos_profile_services_default

from rclpy.qos import QoSDurabilityPolicy
from rclpy.qos import QoSHistoryPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import QoSReliabilityPolicy

my_profile = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT, #RELIABLE,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=10,
    durability=QoSDurabilityPolicy.VOLATILE
)

class ScanSubNode(Node):
    """turtlesim/Pose msg Subscriber Node.

    This node will listen pose topic from turtlesim.
    Then just print them on terminal.
    """

    def __init__(self):
        """Node Initialization.

        You must type name of the node in inheritanced initializer.
        """
        super().__init__('scan_sub_node')

        queue_size = 10  # Queue Size
        # You can create subscriber with create_subscription function
        # this function get those params
        #
        # msg type, topic name, callback function, queue_size
        #
        # topic name must exists and coincident with exact topic name
        self.pose_subscriber = self.create_subscription(
            LaserScan, 'scan', self.sub_callback, my_profile
        )

    def sub_callback(self, msg):
        """Timer will run this function periodically."""
        self.get_logger().info(f' \
            \nmsg.ranges[0] : {round(msg.ranges[0], 2)} \
            \nmsg.ranges[30] : {round(msg.ranges[30], 2)} \
            \nmsg.ranges[60] : {round(msg.ranges[60], 2)} \
            \nmsg.ranges[90] : {round(msg.ranges[90], 2)} \
            \nmsg.ranges[119] : {round(msg.ranges[119], 2)} \
        ')
        # self.get_logger().info(f"""x : {msg.x:.3f} / y : {msg.y:.3f} / theta : {msg.theta:.3f}
        # linear_velocity : {msg.linear_velocity} / angular_velocity : {msg.angular_velocity }""")


class ScanSubNode2 (Node):
    """turtlesim/Pose msg Subscriber Node.

    This node will listen pose topic from turtlesim.
    Then just print them on terminal.
    """

    def __init__(self):
        """Node Initialization.

        You must type name of the node in inheritanced initializer.
        """
        super().__init__('scan_sub_node')

        queue_size = 10  # Queue Size
        # You can create subscriber with create_subscription function
        # this function get those params
        #
        # msg type, topic name, callback function, queue_size
        #
        # topic name must exists and coincident with exact topic name
        self.pose_subscriber = self.create_subscription(
            LaserScan, 'scan', self.sub_callback, queue_size
        )

    def sub_callback(self, msg):
        """Timer will run this function periodically."""
        self.get_logger().info(f' \
            \nmsg.ranges[0] : {msg.ranges[0]} \
            \nmsg.ranges[10] : {msg.ranges[10]} \
            \nmsg.ranges[20] : {msg.ranges[20]} \
            \nmsg.ranges[30] : {msg.ranges[30]} \
            \nmsg.ranges[40] : {msg.ranges[40]} \
        ')
        # self.get_logger().info(f"""x : {msg.x:.3f} / y : {msg.y:.3f} / theta : {msg.theta:.3f}
        # linear_velocity : {msg.linear_velocity} / angular_velocity : {msg.angular_velocity }""")


def main(args=None):
    """Do enter into this main function first."""
    rclpy.init(args=args)

    scan_sub_node = ScanSubNode()
    rclpy.spin(scan_sub_node)

    scan_sub_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    """main function"""
    main()
