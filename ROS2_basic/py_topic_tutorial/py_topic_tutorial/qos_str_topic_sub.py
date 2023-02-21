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
from std_msgs.msg import String

from rclpy.qos import qos_profile_sensor_data
from rclpy.qos import qos_profile_services_default

from rclpy.qos import QoSDurabilityPolicy
from rclpy.qos import QoSHistoryPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import QoSReliabilityPolicy

my_profile = QoSProfile(
    reliability=QoSReliabilityPolicy.RELIABLE,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=10,
    durability=QoSDurabilityPolicy.VOLATILE
)

class StringSubNode(Node):

    def __init__(self):

        super().__init__('strsub_node')

        self.pose_subscriber = self.create_subscription(
            String, 'qos_substrtopic', self.sub_callback, my_profile
        )
        # self.pose_subscriber = self.create_subscription(
        #     String, 'qos_test_topic', self.sub_callback, my_profile
        # )

    def sub_callback(self, msg):
        self.get_logger().info(msg.data)
    

def main(args=None):
    """Do enter into this main function first."""
    rclpy.init(args=args)

    string_sub_node = StringSubNode()

    rclpy.spin(string_sub_node)

    string_sub_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    """main function"""
    main()
