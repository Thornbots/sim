// Copyright 2026 Thornbots
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

#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2_msgs/msg/tf_message.hpp"

class LatchHolder final : public rclcpp::Node
{
public:
  LatchHolder()
  : Node("foxglove_latch_holder")
  {
    const auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
    tf_static_ = create_publisher<tf2_msgs::msg::TFMessage>("/tf_static", qos);
    map_ = create_publisher<nav_msgs::msg::OccupancyGrid>("/map", qos);
    robot_description_ = create_publisher<std_msgs::msg::String>("/robot_description", qos);
  }

private:
  rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr tf_static_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr robot_description_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LatchHolder>());
  rclcpp::shutdown();
  return 0;
}
