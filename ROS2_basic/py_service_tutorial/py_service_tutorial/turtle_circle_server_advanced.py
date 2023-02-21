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

from geometry_msgs.msg import Twist
from custom_interfaces.srv import CircleTurtle
import rclpy
from rclpy.node import Node


# custom_interfaces/srv/CircleTurtle srv Description.
#
# float32 time   # Turtle will turn during this seconds
# ---
# bool success   # Success or Not
# string message # Any optional message

class TurtleCircleNodeAdvanced(Node):

    def __init__(self):
        super().__init__('turtle_circle_server_advanced')
        self.publisher = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.srv = self.create_service(
            CircleTurtle, 'turtle_circle_advanced', self.turtle_circle_callback
        )

        self.moving_time = 0.0
        self.twist_msg = Twist()
        self.get_logger().info('==== Robot Turning Server Started, Waiting for Request ====')

    def turtle_circle(self):
        self.twist_msg.linear.x = 2.0
        self.twist_msg.angular.z = 1.0

        time_start = self.get_clock().now().to_msg().sec
        time_now = self.get_clock().now().to_msg().sec

        while (time_now - time_start) < self.moving_time:
            self.publisher.publish(self.twist_msg)
            time_now = self.get_clock().now().to_msg().sec

        self.twist_msg.linear.x = 0.0
        self.twist_msg.angular.z = 0.0
        self.publisher.publish(self.twist_msg)

        self.get_logger().info('Turtle Stop')

    def turtle_circle_callback(self, request, response):
        self.start_time = self.get_clock().now().to_msg().sec

        self.moving_time = request.time
        self.turtle_circle()

        response.success = True
        response.message = "Turtle successfully drawed Circle"

        return response

def main(args=None):
    rclpy.init(args=args)

    turtle_circle_server = TurtleCircleNodeAdvanced()

    rclpy.spin(turtle_circle_server)

    turtle_circle_server.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
