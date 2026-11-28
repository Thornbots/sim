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

// The bridge owns its ROS context and removes its Gazebo callbacks before
// shutting it down. Signal handling must not tear down DDS while a Gazebo
// receive callback can still publish through it.
#include "ros_gz_bridge/ros_gz_bridge.hpp"
#include "rclcpp/rclcpp.hpp"
#include <chrono>
#include <cstdio>
#include <csignal>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <pthread.h>

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
  if (config.ros_type_name == "rosgraph_msgs/msg/Clock") {
    // The last clock at a hold may be the only one that makes a timer runnable.
    config.qos_profile = rclcpp::QoS(1).reliable().transient_local();
    config.publisher_queue_size = 1;
  }
  node.add_bridge(config);
}

// The node is referenced from its own handles, services and heartbeat timer
// (measured with ros_gz_bridge 1.0.24: use_count 2-3 after spin, 1 after
// release()), so it would never be destroyed and would hold the context, and
// with it rmw's graph-listener thread and the Fast DDS participant, until
// static destruction. release() drops them so the node, and then the context,
// are freed on the main thread.
class Bridge : public ros_gz_bridge::RosGzBridge {
 public:
  using RosGzBridge::RosGzBridge;
  void release() {
    for (const auto &topic : gz_node_->SubscribedTopics()) {
      gz_node_->Unsubscribe(topic);
    }
    heartbeat_timer_.reset();
    services_.clear();
    handles_.clear();
  }
};

// Everything that uses the context lives in this scope, so it is gone before
// the caller releases the context.
int run(const std::shared_ptr<rclcpp::Context> &context, const std::vector<std::string> &args,
        const sigset_t &stop_signals) {
  std::shared_ptr<Bridge> node;
  int status = 0;
  try {
    rclcpp::NodeOptions node_options;
    node_options.context(context);
    node = std::make_shared<Bridge>(node_options);
    for (size_t i = 1; i < args.size(); ++i) {
      add(*node, args[i]);
    }
    rclcpp::ExecutorOptions executor_options;
    executor_options.context = context;
    rclcpp::executors::SingleThreadedExecutor executor(executor_options);
    executor.add_node(node);
    const timespec no_wait{0, 0};
    while (sigtimedwait(&stop_signals, nullptr, &no_wait) < 0) {
      executor.spin_once(std::chrono::milliseconds(100));
    }
  } catch (const std::exception &e) {
    std::cerr << "gz_bridge: " << e.what() << '\n';
    usage();
    status = 1;
  }
  if (node) node->release();
  return status;
}
}  // namespace

int main(int argc, char **argv) {
  setvbuf(stdout, nullptr, _IONBF, BUFSIZ);
  sigset_t stop_signals;
  sigemptyset(&stop_signals);
  sigaddset(&stop_signals, SIGINT);
  sigaddset(&stop_signals, SIGTERM);
  if (pthread_sigmask(SIG_BLOCK, &stop_signals, nullptr) != 0) {
    std::cerr << "gz_bridge: could not block stop signals\n";
    return 1;
  }
  auto context = std::make_shared<rclcpp::Context>();
  context->init(argc, argv);
  const auto args = rclcpp::remove_ros_arguments(argc, argv);
  int status = 1;
  if (args.size() < 2) {
    usage();
  } else {
    status = run(context, args, stop_signals);
  }
  context->shutdown("gz_bridge exit");
  context.reset();
  return status;
}
