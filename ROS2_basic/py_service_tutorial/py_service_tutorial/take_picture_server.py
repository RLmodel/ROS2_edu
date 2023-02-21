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

import cv2
import rclpy

from geometry_msgs.msg import Twist
from example_interfaces.srv import SetBool

from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image 

# example_interfaces/srv/SetBool srv Description.
#
# bool data # e.g. for hardware enabling / disabling
# ---
# bool success   # indicate successful run of triggered service
# string message # informational, e.g. for error messages


class PictureNode(Node):

    def __init__(self):
        super().__init__('turtle_circle_server')

        self.server = self.create_service(
            SetBool, 'take_picture', self.take_picture_callback
        )

        self.subscriber = self.create_subscription(
            Image, 'logi_camera_sensor/image_raw', self.sub_callback, 10
        )

        self.br = CvBridge()
        self.is_request = False

    def sub_callback(self, data):

        if self.is_request:
            current_frame = self.br.imgmsg_to_cv2(data, "bgr8")

            file_name = str(self.get_clock().now().to_msg().sec) + '.png'
            cv2.imwrite(file_name, current_frame)
            self.get_logger().info(f'Image saved in {file_name}')
            
            self.is_request = False

    def take_picture_callback(self, request, response):

        if request.data is True:
            self.get_logger().info('KimChi~')
            self.is_request = True

        response.success = True
        response.message = "Successfully image written"

        return response

def main(args=None):
    rclpy.init(args=args)

    picture_node = PictureNode()

    rclpy.spin(picture_node)

    picture_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
