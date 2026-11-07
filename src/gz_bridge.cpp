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
// SetEntityPose service), and one difference: it owns its rclcpp::Context and
// destroys it before main returns. parameter_bridge uses the global default
// context, which is only finalised by static destruction, so on SIGINT main
// returns while rclcpp's signal-handler thread is still inside
// Context::shutdown and rmw's graph-listener thread is still taking from a
// reader. Static destruction then deletes the Fast DDS participant under both
// (SIGSEGV in a DataReaderImpl mutex, "pure virtual method called", PDP::~PDP
// or a glibc mutex assertion, about 2% of early interrupts). Here shutdown()
// blocks on the context lock until the handler thread finishes, and
// destroying the last reference runs rcl_context_fini, which joins rmw's
// threads and deletes the participant, on the main thread before exit.
#include "ros_gz_bridge/ros_gz_bridge.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rcutils/logging.h"
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include <unistd.h>

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
    heartbeat_timer_.reset();
    services_.clear();
    handles_.clear();
  }
};

// Bounded teardown. Destroying the Fast DDS participant has rarely blocked here (1 bridge
// of about 1,000 exits in the lifecycle matrix: SIGINT seen, no exit for 15 s, SIGTERM
// ignored), and the bridge holds nothing that needs saving. Three seconds after spin()
// returns the process leaves without finishing the destruction, and says so on stderr.
// GZ_BRIDGE_ABORT_ON_HANG=1 aborts instead, to take a core of the stuck teardown.
void start_teardown_watchdog() {
  std::thread([] {
    std::this_thread::sleep_for(std::chrono::seconds(3));
    static const char message[] =
        "gz_bridge: teardown exceeded 3 s; leaving without destroying the DDS participant\n";
    const ssize_t written = write(STDERR_FILENO, message, sizeof(message) - 1);
    (void)written;
    if (std::getenv("GZ_BRIDGE_ABORT_ON_HANG")) std::abort();
    std::_Exit(0);
  }).detach();
}

// Everything that uses the context lives in this scope, so it is gone before
// the caller releases the context.
int run(const std::shared_ptr<rclcpp::Context> &context, const std::vector<std::string> &args) {
  try {
    rclcpp::NodeOptions node_options;
    node_options.context(context);
    auto node = std::make_shared<Bridge>(node_options);
    for (size_t i = 1; i < args.size(); ++i) {
      add(*node, args[i]);
    }
    rclcpp::ExecutorOptions executor_options;
    executor_options.context = context;
    rclcpp::executors::SingleThreadedExecutor executor(executor_options);
    executor.add_node(node);
    executor.spin();
    start_teardown_watchdog();
    executor.remove_node(node);
    node->release();
    return 0;
  } catch (const std::exception &e) {
    std::cerr << "gz_bridge: " << e.what() << '\n';
    usage();
    return 1;
  }
}
}  // namespace

int main(int argc, char **argv) {
  setvbuf(stdout, nullptr, _IONBF, BUFSIZ);
  // Logging first: SignalHandler::install() starts the handler thread before it
  // initialises logging, and an interrupt in that window made both threads run
  // rcutils_logging_initialize (SIGSEGV in rcutils_hash_map_init, 1 in 300).
  rcutils_logging_initialize();
  // rclcpp::init() installs the SIGINT/SIGTERM handler; Context::init() does not.
  rclcpp::install_signal_handlers();
  auto context = std::make_shared<rclcpp::Context>();
  context->init(argc, argv);
  const auto args = rclcpp::remove_ros_arguments(argc, argv);
  int status = 1;
  if (args.size() < 2) {
    usage();
  } else {
    status = run(context, args);
  }
  context->shutdown("gz_bridge exit");  // waits for the signal-handler thread's own shutdown
  context.reset();                      // rcl_context_fini: joins rmw's threads, deletes the participant
  return status;
}
