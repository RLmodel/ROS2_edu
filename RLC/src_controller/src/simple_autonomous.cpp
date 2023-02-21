// Copyright 2022 @RoadBalance
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.


#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

using Twist = geometry_msgs::msg::Twist;
using LaserScan = sensor_msgs::msg::LaserScan;

class SimpleAutonomous : public rclcpp::Node {
private:
  rclcpp::Publisher<Twist>::SharedPtr twist_publisher;
  rclcpp::Subscription<LaserScan>::SharedPtr laser_subscriber;

  Twist twist_pub_msg;

  void sub_callback(const LaserScan::SharedPtr msg) {
    RCLCPP_INFO(this->get_logger(), "msg->ranges.size()=%d", msg->ranges.size());

    RCLCPP_INFO(this->get_logger(), "msg->ranges[0]=%f", msg->ranges[0]);
    RCLCPP_INFO(this->get_logger(), "msg->ranges[29]=%f", msg->ranges[29]);
    RCLCPP_INFO(this->get_logger(), "msg->ranges[59]=%f", msg->ranges[59]);
    RCLCPP_INFO(this->get_logger(), "msg->ranges[89]=%f", msg->ranges[89]);
    RCLCPP_INFO(this->get_logger(), "msg->ranges[119]=%f", msg->ranges[119]);

    // TODO: Make your own logic with Lidar, then calculate cmd_vel!
    // twist_publisher->publish();
  }

public:
  SimpleAutonomous() : Node("simple_autonomous") {
    laser_subscriber = create_subscription<LaserScan>(
      "scan", 10,
      std::bind(&SimpleAutonomous::sub_callback, this, std::placeholders::_1)
    );
    twist_publisher = create_publisher<Twist>("cmd_vel", 10);
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto quiz_node = std::make_shared<SimpleAutonomous>();

  rclcpp::spin(quiz_node);
  rclcpp::shutdown();

  return 0;
}
