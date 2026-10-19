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

#include <gz/msgs/boolean.pb.h>
#include <gz/msgs/empty.pb.h>
#include <gz/msgs/scene.pb.h>
#include <gz/msgs/world_control.pb.h>
#include <gz/msgs/world_stats.pb.h>
#include <gz/transport/Node.hh>
#include <lifecycle_msgs/msg/state.hpp>
#include <lifecycle_msgs/srv/get_state.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/header.hpp>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <memory>
#include <set>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace
{
using namespace std::chrono_literals;
using GetState = lifecycle_msgs::srv::GetState;

std::string without_outer_slashes(std::string name)
{
  while (!name.empty() && name.front() == '/') {
    name.erase(name.begin());
  }
  while (!name.empty() && name.back() == '/') {
    name.pop_back();
  }
  return name;
}

class E2EStartup : public rclcpp::Node
{
public:
  E2EStartup()
  : Node("e2e_startup"), started_(std::chrono::steady_clock::now())
  {
    required_nodes_ = declare_parameter<std::vector<std::string>>(
      "required_nodes", std::vector<std::string>{});
    models_ = declare_parameter<std::vector<std::string>>(
      "models", std::vector<std::string>{});
    auto lifecycle_nodes = declare_parameter<std::vector<std::string>>(
      "lifecycle_nodes", std::vector<std::string>{"map_server", "amcl"});
    timeout_s_ = declare_parameter<double>("timeout_s", 30.0);
    world_ = declare_parameter<std::string>("world", "ARCC_Field_2026");
    ready_topics_ = declare_parameter<std::vector<std::string>>(
      "ready_topics", std::vector<std::string>{});
    for (const auto & topic : ready_topics_) {
      if (topic.empty()) {
        throw std::invalid_argument("ready_topics entries cannot be empty");
      }
      pending_ready_topics_.insert(topic);
    }

    if (required_nodes_.empty()) {
      throw std::invalid_argument("required_nodes must contain at least one node");
    }
    if (models_.empty()) {
      throw std::invalid_argument("models must contain at least one model");
    }
    if (!std::isfinite(timeout_s_) || timeout_s_ <= 0.0 || world_.empty()) {
      throw std::invalid_argument("timeout_s and world must be valid");
    }
    normalize_names(required_nodes_, "required_nodes");
    normalize_names(models_, "models");
    normalize_names(lifecycle_nodes, "lifecycle_nodes");

    lifecycle_probes_.reserve(lifecycle_nodes.size());
    for (const auto & name : lifecycle_nodes) {
      LifecycleProbe probe;
      probe.name = name;
      probe.client = create_client<GetState>("/" + name + "/get_state");
      lifecycle_probes_.push_back(std::move(probe));
    }
    for (const auto & topic : pending_ready_topics_) {
      ready_subs_.push_back(create_subscription<std_msgs::msg::Header>(
          topic, rclcpp::QoS(1).reliable().transient_local(),
          [this, topic](std_msgs::msg::Header::ConstSharedPtr) {
            pending_ready_topics_.erase(topic);
          }));
    }

    if (!std::getenv("GZ_IP")) {
      setenv("GZ_IP", "127.0.0.1", 0);
    }
    gz_node_ = std::make_unique<gz::transport::Node>();
    gz_node_->Subscribe<gz::msgs::WorldStatistics>("/world/" + world_ + "/stats",
      [this](const gz::msgs::WorldStatistics & stats) {
        paused_ = stats.paused();
        sim_ns_ = stats.sim_time().sec() * 1000000000LL + stats.sim_time().nsec();
      });
    timer_ = create_wall_timer(100ms, [this] { poll_startup(); });
    RCLCPP_INFO(get_logger(), "waiting for startup nodes and world models");
  }

  bool done() const {return done_;}
  bool failed() const {return failed_;}

private:
  struct LifecycleProbe
  {
    std::string name;
    rclcpp::Client<GetState>::SharedPtr client;
    bool pending = false;
    bool active = false;
  };

  static void normalize_names(std::vector<std::string> & names,
                              const std::string & parameter)
  {
    for (auto & name : names) {
      name = without_outer_slashes(std::move(name));
      if (name.empty()) {
        throw std::invalid_argument(parameter + " contains an empty name");
      }
    }
  }

  bool timed_out() const
  {
    return std::chrono::steady_clock::now() - started_ >=
           std::chrono::duration<double>(timeout_s_);
  }

  void timeout_failure()
  {
    const auto present = normalized_node_names();
    std::vector<std::string> missing;
    for (const auto & name : required_nodes_) {
      if (!present.count(name)) {
        missing.push_back(name);
      }
    }
    for (const auto & probe : lifecycle_probes_) {
      if (!probe.active) {
        missing.push_back(probe.name + " (lifecycle not active)");
      }
    }
    std::string message = "startup timed out after " +
      std::to_string(timeout_s_) + " s";
    message += "; " + waiting_for_;
    if (!missing.empty()) {
      message += "; still waiting for ";
      for (size_t i = 0; i < missing.size(); ++i) {
        if (i != 0) {
          message += ", ";
        }
        message += missing[i];
      }
    }
    fail(message);
  }

  std::set<std::string> normalized_node_names()
  {
    std::set<std::string> names;
    for (const auto & name : get_node_graph_interface()->get_node_names()) {
      names.insert(without_outer_slashes(name));
    }
    return names;
  }

  void poll_startup()
  {
    if (done_) {
      return;
    }
    if (timed_out()) {
      timeout_failure();
      return;
    }

    const auto present = normalized_node_names();
    for (const auto & name : required_nodes_) {
      if (!present.count(name)) {
        return;
      }
    }

    poll_lifecycle_states();
    if (failed_ || !all_lifecycle_active()) {
      return;
    }
    if (!pending_ready_topics_.empty()) {
      waiting_for_ = "readiness on";
      for (const auto & topic : pending_ready_topics_) {
        waiting_for_ += " " + topic;
      }
      return;
    }
    waiting_for_ = "world statistics and models at paused sim zero";
    verify_models_and_unpause();
  }

  void poll_lifecycle_states()
  {
    for (size_t i = 0; i < lifecycle_probes_.size(); ++i) {
      auto & probe = lifecycle_probes_[i];
      if (probe.active || probe.pending || !probe.client->service_is_ready()) {
        continue;
      }
      probe.pending = true;
      try {
        probe.client->async_send_request(
          std::make_shared<GetState::Request>(),
          [this, i](rclcpp::Client<GetState>::SharedFuture future) {
            auto & result = lifecycle_probes_[i];
            result.pending = false;
            try {
              const auto response = future.get();
              result.active = response && response->current_state.id ==
                lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE;
            } catch (const std::exception & error) {
              RCLCPP_WARN(get_logger(), "GetState failed for %s: %s",
                result.name.c_str(), error.what());
            }
          });
      } catch (const std::exception & error) {
        probe.pending = false;
        RCLCPP_WARN(get_logger(), "could not request state for %s: %s",
          probe.name.c_str(), error.what());
      }
    }
  }

  bool all_lifecycle_active() const
  {
    return std::all_of(lifecycle_probes_.begin(), lifecycle_probes_.end(),
      [](const LifecycleProbe & probe) {return probe.active;});
  }

  void verify_models_and_unpause()
  {
    if (sim_ns_ < 0) {waiting_for_ = "world statistics"; return;}
    if (!paused_ || sim_ns_ != 0) {
      fail("world advanced before startup readiness; sim_ns=" + std::to_string(sim_ns_));
      return;
    }
    gz::msgs::Scene scene;
    bool result = false;
    const std::string scene_service = "/world/" + world_ + "/scene/info";
    if (!gz_node_->Request(scene_service, gz::msgs::Empty{}, 500u, scene, result) ||
        !result)
    {
      waiting_for_ = "Gazebo scene query: " + scene_service;
      return;
    }

    std::set<std::string> present;
    for (const auto & model : scene.model()) {
      present.insert(model.name());
    }
    std::vector<std::string> missing;
    for (const auto & model : models_) {
      if (!present.count(model)) {
        missing.push_back(model);
      }
    }
    if (!missing.empty()) {
      std::string message = "models missing from Gazebo world " + world_ + ": ";
      for (size_t i = 0; i < missing.size(); ++i) {
        if (i != 0) {
          message += ", ";
        }
        message += missing[i];
      }
      waiting_for_ = message;
      return;
    }
    if (timed_out()) {
      timeout_failure();
      return;
    }

    gz::msgs::WorldControl request;
    request.set_pause(false);
    gz::msgs::Boolean response;
    result = false;
    const std::string control_service = "/world/" + world_ + "/control";
    if (!gz_node_->Request(control_service, request, 2000u, response, result) ||
        !result || !response.data())
    {
      fail("Gazebo refused to unpause world " + world_);
      return;
    }
    RCLCPP_INFO(get_logger(), "startup ready at sim zero; unpaused world %s", world_.c_str());
    done_ = true;
  }

  void fail(const std::string & message)
  {
    failed_ = true;
    done_ = true;
    RCLCPP_ERROR(get_logger(), "%s", message.c_str());
  }

  std::vector<std::string> required_nodes_;
  std::vector<std::string> models_;
  std::vector<std::string> ready_topics_;
  std::set<std::string> pending_ready_topics_;
  std::vector<LifecycleProbe> lifecycle_probes_;
  std::vector<rclcpp::Subscription<std_msgs::msg::Header>::SharedPtr> ready_subs_;
  double timeout_s_ = 30.0;
  std::string world_;
  std::chrono::steady_clock::time_point started_;
  std::atomic<int64_t> sim_ns_{-1};
  std::atomic<bool> paused_{false};
  std::string waiting_for_ = "nodes and lifecycle activation";
  rclcpp::TimerBase::SharedPtr timer_;
  bool done_ = false;
  bool failed_ = false;
  std::unique_ptr<gz::transport::Node> gz_node_;
};
}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  int result = 1;
  try {
    auto startup = std::make_shared<E2EStartup>();
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(startup);
    while (rclcpp::ok() && !startup->done()) {
      executor.spin_once(std::chrono::milliseconds(100));
    }
    result = !startup->done() || startup->failed() ? 1 : 0;
    if (!startup->done()) {
      RCLCPP_ERROR(startup->get_logger(), "startup interrupted before readiness");
    }
    executor.remove_node(startup);
  } catch (const std::exception & error) {
    RCLCPP_ERROR(rclcpp::get_logger("e2e_startup"),
      "startup gate failed: %s", error.what());
  }
  rclcpp::shutdown();
  return result;
}
