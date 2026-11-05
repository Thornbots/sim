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

// ros_gz_bridge's parameter_bridge with the arguments sim uses
// (`topic@ros_type[gz_type`, `]` for ROS to gz, `@` for both, and the
// SetEntityPose service), and one difference: after spinning it calls
// rclcpp::shutdown(). parameter_bridge returns from main without it, so on
// SIGINT main exits while rclcpp's signal-handler thread is still inside
// Context::shutdown. Static destruction then tears down the Fast DDS
// participant under that thread (SIGSEGV in PDP::~PDP, or a glibc mutex
// assertion), a crash seen in about one shutdown in five. shutdown() blocks
// on the context lock until the handler thread finishes.
#include "ros_gz_bridge/ros_gz_bridge.hpp"
#include "rclcpp/rclcpp.hpp"
#include <cstdio>
#include <iostream>
#include <stdexcept>
#include <string>

namespace {
using ros_gz_bridge::BridgeDirection;

void usage() {
  std::cerr << "usage: gz_bridge TOPIC@ROS_TYPE[GZ_TYPE | TOPIC@ROS_TYPE]GZ_TYPE |\n"
               "                 TOPIC@ROS_TYPE@GZ_TYPE | SERVICE@ros_gz_interfaces/srv/SetEntityPose ...\n"
               "  [ gz to ROS, ] ROS to gz, @ both; ROS arguments such as -r and -p go after --ros-args\n";
}

void add(ros_gz_bridge::RosGzBridge &node, const std::string &spec) {
  const auto at = spec.find('@');
  if (at == std::string::npos || at == 0) {
    throw std::invalid_argument("bridge '" + spec + "' needs TOPIC@TYPE");
  }
  const std::string name = spec.substr(0, at), rest = spec.substr(at + 1);
  const auto split = rest.find_first_of("[]@");
  if (split == std::string::npos) {
    // Only the service sim bridges; the request and reply are gz.msgs.Pose/Boolean.
    if (rest != "ros_gz_interfaces/srv/SetEntityPose") {
      throw std::invalid_argument("bridge '" + spec + "': unsupported service type");
    }
    node.add_service_bridge(rest, "gz.msgs.Pose", "gz.msgs.Boolean", name);
    return;
  }
  ros_gz_bridge::BridgeConfig config;
  config.ros_topic_name = config.gz_topic_name = name;
  config.ros_type_name = rest.substr(0, split);
  config.gz_type_name = rest.substr(split + 1);
  switch (rest[split]) {
    case '[': config.direction = BridgeDirection::GZ_TO_ROS; break;
    case ']': config.direction = BridgeDirection::ROS_TO_GZ; break;
    default: config.direction = BridgeDirection::BIDIRECTIONAL; break;
  }
  if (config.ros_type_name.empty() || config.gz_type_name.empty()) {
    throw std::invalid_argument("bridge '" + spec + "' needs both types");
  }
  node.add_bridge(config);
}
}  // namespace

int main(int argc, char **argv) {
  setvbuf(stdout, nullptr, _IONBF, BUFSIZ);
  const auto args = rclcpp::init_and_remove_ros_arguments(argc, argv);
  if (args.size() < 2) {
    usage();
    rclcpp::shutdown();
    return 1;
  }
  try {
    auto node = std::make_shared<ros_gz_bridge::RosGzBridge>(rclcpp::NodeOptions());
    for (size_t i = 1; i < args.size(); ++i) {
      add(*node, args[i]);
    }
    rclcpp::spin(node);
  } catch (const std::exception &e) {
    std::cerr << "gz_bridge: " << e.what() << '\n';
    usage();
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
