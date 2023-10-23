// Copyright 2022 Stogl Robotics Consulting UG (haftungsbeschränkt)
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
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"

#include "std_msgs/msg/float64_multi_array.hpp"

std::shared_ptr<rclcpp::Node> node;

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  node = std::make_shared<rclcpp::Node>("gripper_test_node");

  auto publisher = node->create_publisher<std_msgs::msg::Float64MultiArray>(
    "/egp64_finger_controller/commands", 10);

  RCLCPP_INFO(node->get_logger(), "node created");
// ros2 topic pub --once /egp64_finger_controller/commands std_msgs/Float64MultiArray "{layout: {dim: [{label: '', size: 2, stride: 1}], data_offset: 0}, data: [10, 10]}"
// ros2 topic pub --once /egp64_finger_controller/commands std_msgs/Float64MultiArray "{layout: {dim: [{label: '', size: 2, stride: 1}], data_offset: 0}, data: [-10, -10]}"

  std_msgs::msg::Float64MultiArray commands;

  using namespace std::chrono_literals;
  std::this_thread::sleep_for(1s);
  commands.data.push_back(-10);
  commands.data.push_back(-10);
  // commands.data.push_back(0);
  // commands.data.push_back(0);
  // commands.data.push_back(0);
  // commands.data.push_back(0);
  // commands.data.push_back(0);
  // commands.data.push_back(0);
  // commands.data.push_back(0);
  // commands.data.push_back(0);
  publisher->publish(commands);
  std::this_thread::sleep_for(1s);

  /* Close gripper */
  commands.data[0] = 10;
  commands.data[1] = 10;
  // commands.data[2] = 10;
  // commands.data[3] = 10;
  // commands.data[4] = -10;
  // commands.data[5] = -10;
  // commands.data[6] = 0;
  // commands.data[7] = 0;
  // commands.data[8] = 0;
  // commands.data[9] = 100;
  publisher->publish(commands);
  std::this_thread::sleep_for(4s);

  /* Open gripper */
  // commands.data[0] = -4;
  // commands.data[1] = -4;
  // commands.data[2] = -4;
  // commands.data[3] = -4;
  // commands.data[4] = 4;
  // commands.data[5] = 4;
  // publisher->publish(commands);
  // std::this_thread::sleep_for(1s);

  rclcpp::shutdown();

  return 0;
}
