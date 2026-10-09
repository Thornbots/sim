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

#ifndef SIM__SUITE_NODE_HPP_
#define SIM__SUITE_NODE_HPP_
#include <algorithm>
#include <chrono>
#include <functional>
#include <iostream>
#include <memory>
#include <rcl_interfaces/srv/set_parameters.hpp>
#include <rclcpp/rclcpp.hpp>
#include <stdexcept>
#include <string>
#include <vector>
namespace sim {
class SimTimeNode : public rclcpp::Node {
public:
  explicit SimTimeNode(const std::string &name)
      : Node(name, rclcpp::NodeOptions().parameter_overrides(
                       {rclcpp::Parameter("use_sim_time", true)})) {
    executor_.add_node(get_node_base_interface());
  }
  template <class Stamp> static double stamp_s(const Stamp &stamp) {
    return stamp.sec + stamp.nanosec * 1e-9;
  }
  double now_s() { return get_clock()->now().seconds(); }
  void spin_once() { executor_.spin_once(std::chrono::milliseconds(100)); }
  bool wait_until(const std::function<bool()> &predicate, double timeout,
                  const std::string &description) {
    const auto end = std::chrono::steady_clock::now() +
                     std::chrono::duration<double>(timeout);
    while (rclcpp::ok() && std::chrono::steady_clock::now() < end) {
      if (predicate())
        return true;
      spin_once();
    }
    std::cout << "[wait_until] timed out after " << timeout
              << "s waiting for: " << description << std::endl;
    return false;
  }
  virtual void spin_for(double seconds) {
    const auto end =
        std::chrono::steady_clock::now() +
        std::chrono::duration<double>(std::max(3 * seconds, seconds + 5));
    double start = -1;
    while (rclcpp::ok() && std::chrono::steady_clock::now() < end) {
      spin_once();
      const double now = now_s();
      if (start < 0 && now > 0)
        start = now;
      if (start >= 0 && now - start >= seconds)
        return;
    }
    std::cout << "[spin_for] wall-clock cap hit before " << seconds
              << "s of sim time elapsed" << std::endl;
  }
  bool nodes_up(const std::vector<std::string> &names) {
    const auto live = get_node_names();
    return std::all_of(names.begin(), names.end(), [&](const auto &n) {
      return std::find(live.begin(), live.end(), "/" + n) != live.end() ||
             std::find(live.begin(), live.end(), n) != live.end();
    });
  }
  void check_nodes(const std::vector<std::string> &names) {
    if (!nodes_up(names))
      throw std::runtime_error("stack nodes not running; see stack.log in "
                               "--log-dir, or the launch log");
  }
  void set_params(const std::string &node,
                  const std::vector<rclcpp::Parameter> &params) {
    auto client = create_client<rcl_interfaces::srv::SetParameters>(
        "/" + node + "/set_parameters");
    if (!client->wait_for_service(std::chrono::seconds(10)))
      throw std::runtime_error("/" + node + "/set_parameters not available");
    auto req = std::make_shared<rcl_interfaces::srv::SetParameters::Request>();
    for (const auto &p : params)
      req->parameters.push_back(p.to_parameter_msg());
    auto future = client->async_send_request(req);
    if (executor_.spin_until_future_complete(future,
                                             std::chrono::seconds(10)) !=
        rclcpp::FutureReturnCode::SUCCESS)
      throw std::runtime_error(node + " parameter request timed out");
    const auto response = future.get();
    for (const auto &r : response->results)
      if (!r.successful)
        throw std::runtime_error(node + " rejected parameters: " + r.reason);
  }

private:
  rclcpp::executors::SingleThreadedExecutor executor_;
};
} // namespace sim
#endif
