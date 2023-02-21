# !/usr/bin/env/ python3
#
# Copyright 2022 @RoadBalance
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

import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor

from action_tutorials_interfaces.action import Fibonacci
from custom_interfaces.action import Parking
from sensor_msgs.msg import LaserScan

"""Parking.action
#goal definition
bool start_flag
---
#result definition
string message 
---
#feedback definition
float32 distance
"""

class ParkingActionServer(Node):

    def __init__(self):
        super().__init__('parking_action_server')
        
        self.laser_sub = self.create_subscription(
            LaserScan, 'scan', self.sub_callback, 10
        )

        self.action_server = ActionServer(
            self, Parking, 'src_parking', self.execute_callback
        )

        self.get_logger().info("Action Ready...")

        self.is_sub = False
        # distance from forward obstacles
        self.f_obs_distance = 100.0

        # distance from L/R obstacles
        self.r_obs_distance = 100.0
        self.l_obs_distance = 100.0

    def sub_callback(self, data):
        
        if self.is_sub:
            self.f_obs_distance = data.ranges[60]
            self.r_obs_distance = data.ranges[30]
            self.l_obs_distance = data.ranges[90]
            self.get_logger().info("sub success")

    def execute_callback(self, goal_handle):

        self.is_sub = True

        self.get_logger().info('Executing goal...')

        feedback_msg = Parking.Feedback()

        while self.f_obs_distance > 0.5:
            feedback_msg.distance = self.f_obs_distance
            goal_handle.publish_feedback(feedback_msg)
            self.get_logger().info(
                f"Distance from forward obstacle : {self.f_obs_distance}"
            )
            time.sleep(1)

        goal_handle.succeed()

        result = Parking.Result()
        lr_diff = abs(self.r_obs_distance - self.l_obs_distance)
        print(lr_diff)
        if lr_diff < 0.15:
            result.message = "[Success!] Oh... Teach me how you did :0"
        else:
            result.message = "[Fail] Be careful, Poor Driver! "
        return result


def main(args=None):

    rclpy.init(args=args)

    # parking_action_server = ParkingActionServer()
    # rclpy.spin(parking_action_server)
    # parking_action_server.destroy_node()
    # rclpy.shutdown()

    try:
        parking_action_server = ParkingActionServer()
        # MultiThreadedExecutor  ref
        # https://url.kr/x4kf2b
        executor = MultiThreadedExecutor()
        executor.add_node(parking_action_server)
        try:
            executor.spin()
        except KeyboardInterrupt:
            parking_action_server.get_logger().info('Keyboard Interrupt (SIGINT)')
        finally:
            executor.shutdown()
            parking_action_server.destroy_node()
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()