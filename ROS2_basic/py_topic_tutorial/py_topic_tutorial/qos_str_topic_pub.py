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
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=10,
    durability=QoSDurabilityPolicy.VOLATILE
)

class StringPubNode(Node):

    def __init__(self):

        super().__init__('strpub_node')

        self.string_publisher = self.create_publisher(String, 'qos_pubstrtopic', qos_profile_sensor_data)
        # self.string_publisher = self.create_publisher(String, 'qos_test_topic', my_profile)
        self.create_timer(0.2, self.timer_callback)

        self.count = 0

    def timer_callback(self):
        msg = String()
        msg.data = f"Hello {self.count}"

        self.get_logger().info(msg.data)
        self.string_publisher.publish(msg)

        self.count += 1

def main(args=None):
    """Do enter into this main function first."""
    rclpy.init(args=args)

    string_pub_node = StringPubNode()
    rclpy.spin(string_pub_node)
    string_pub_node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    """main function"""
    main()
