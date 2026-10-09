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

// /clock for gz-free stacks. A positive rate follows wall time; rate zero
// stops at each paced topic's next stamped deadline.
// see README.md for design rationale

#include <algorithm>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/serialization.hpp"
#include "rclcpp/typesupport_helpers.hpp"
#include "rosgraph_msgs/msg/clock.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"

namespace
{
using Clock = std::chrono::steady_clock;
using MessageMember = rosidl_typesupport_introspection_cpp::MessageMember;
using MessageMembers = rosidl_typesupport_introspection_cpp::MessageMembers;

const MessageMember * find_member(const MessageMembers * members, const char * name)
{
  for (uint32_t i = 0; i < members->member_count_; ++i) {
    if (std::string(members->members_[i].name_) == name) {
      return &members->members_[i];
    }
  }
  return nullptr;
}

const MessageMembers * nested_members(const MessageMember * member)
{
  if (!member || member->type_id_ != rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE ||
    member->is_array_ || !member->members_)
  {
    throw std::runtime_error("pace_topics message has no usable stamp");
  }
  return static_cast<const MessageMembers *>(member->members_->data);
}

struct PaceType
{
  explicit PaceType(const std::string & type)
  : introspection_library(rclcpp::get_typesupport_library(
        type, "rosidl_typesupport_introspection_cpp")),
    cpp_library(rclcpp::get_typesupport_library(type, "rosidl_typesupport_cpp")),
    members(static_cast<const MessageMembers *>(rclcpp::get_message_typesupport_handle(
        type, "rosidl_typesupport_introspection_cpp", *introspection_library)->data)),
    serializer(rclcpp::get_message_typesupport_handle(
        type, "rosidl_typesupport_cpp", *cpp_library))
  {
    const MessageMember * stamp = find_member(members, "stamp");
    if (!stamp) {
      const MessageMember * header = find_member(members, "header");
      const auto * header_members = nested_members(header);
      stamp = find_member(header_members, "stamp");
      stamp_offset = header->offset_;
    }
    const auto * time_members = nested_members(stamp);
    stamp_offset += stamp->offset_;
    const auto * sec = find_member(time_members, "sec");
    const auto * nanosec = find_member(time_members, "nanosec");
    if (!sec || !nanosec ||
      sec->type_id_ != rosidl_typesupport_introspection_cpp::ROS_TYPE_INT32 ||
      nanosec->type_id_ != rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT32 ||
      sec->is_array_ || nanosec->is_array_)
    {
      throw std::runtime_error("pace_topics stamp lacks sec/nanosec fields");
    }
    sec_offset = sec->offset_;
    nanosec_offset = nanosec->offset_;
  }

  int64_t stamp_ns(const rclcpp::SerializedMessage & serialized) const
  {
    void * buffer = ::operator new(members->size_of_);
    members->init_function(buffer, rosidl_runtime_cpp::MessageInitialization::ALL);
    try {
      serializer.deserialize_message(&serialized, buffer);
      const auto * stamp = static_cast<const uint8_t *>(buffer) + stamp_offset;
      const auto sec = *reinterpret_cast<const int32_t *>(stamp + sec_offset);
      const auto nanosec = *reinterpret_cast<const uint32_t *>(stamp + nanosec_offset);
      members->fini_function(buffer);
      ::operator delete(buffer);
      return static_cast<int64_t>(sec) * 1000000000LL + nanosec;
    } catch (...) {
      members->fini_function(buffer);
      ::operator delete(buffer);
      throw;
    }
  }

  std::shared_ptr<rcpputils::SharedLibrary> introspection_library;
  std::shared_ptr<rcpputils::SharedLibrary> cpp_library;
  const MessageMembers * members;
  rclcpp::SerializationBase serializer;
  uint32_t stamp_offset = 0;
  uint32_t sec_offset = 0;
  uint32_t nanosec_offset = 0;
};

struct Gate
{
  int64_t period_ns;
  std::optional<int64_t> last_ns;
  bool live = false;
};
}  // namespace

class SimClock : public rclcpp::Node
{
public:
  SimClock()
  : Node("sim_clock")
  {
    rate_ = declare_parameter("rate", 1.0);
    const double publish_rate_hz = declare_parameter("publish_rate_hz", 1000.0);
    step_ns_ = static_cast<int64_t>(declare_parameter("step_s", 0.002) * 1e9);
    const auto pace_topics = declare_parameter<std::vector<std::string>>(
      "pace_topics", std::vector<std::string>{""});
    max_wait_s_ = declare_parameter("max_wait_s", 0.5);
    publisher_ = create_publisher<rosgraph_msgs::msg::Clock>("/clock", 10);
    last_wall_ = Clock::now();

    if (rate_ > 0.0) {
      timer_ = create_wall_timer(
        std::chrono::duration<double>(1.0 / publish_rate_hz),
        [this] {on_timer();});
      RCLCPP_INFO(get_logger(), "sim_clock: /clock at %gx wall time", rate_);
      return;
    }

    for (const auto & spec : pace_topics) {
      std::istringstream stream(spec);
      std::string topic, type_name, period, extra;
      if (!(stream >> topic)) {
        continue;
      }
      if (!(stream >> type_name >> period) || (stream >> extra)) {
        throw std::runtime_error("pace_topics entry must be 'topic pkg/msg/Type period_s'");
      }
      gates_[topic] = {static_cast<int64_t>(std::stod(period) * 1e9), std::nullopt, false};
      auto pace_type = std::make_shared<PaceType>(type_name);
      subscriptions_.push_back(create_generic_subscription(
        topic, type_name, rclcpp::SensorDataQoS(),
        [this, topic, pace_type](const rclcpp::SerializedMessage & message) {
          on_paced(topic, pace_type->stamp_ns(message));
        }));
    }
    std::string topics = "[";
    for (const auto & [topic, gate] : gates_) {
      (void)gate;
      if (topics.size() > 1) {
        topics += ", ";
      }
      topics += "'" + topic + "'";
    }
    topics += "]";
    RCLCPP_INFO(
      get_logger(), "sim_clock: as fast as %s keep up, %g ms steps",
      topics.c_str(), step_ns_ / 1e6);
  }

  bool paced() const {return rate_ <= 0.0;}

  void run_paced()
  {
    auto report_wall = Clock::now();
    int64_t report_sim = 0;
    while (rclcpp::ok()) {
      bool idle;
      int64_t next_ns;
      {
        std::unique_lock<std::mutex> lock(mutex_);
        const auto deadline = Clock::now() + std::chrono::duration<double>(max_wait_s_);
        auto waiting = blocking();
        while (!waiting.empty() && Clock::now() < deadline) {
          for (const auto & topic : waiting) {
            if (count_publishers(topic) == 0) {
              gates_.at(topic).live = false;
            }
          }
          const auto remaining = std::chrono::duration<double>(deadline - Clock::now());
          condition_.wait_for(lock, std::min(remaining, std::chrono::duration<double>(0.01)));
          waiting = blocking();
        }
        for (const auto & topic : waiting) {
          gates_.at(topic).live = false;
          RCLCPP_WARN(get_logger(), "sim_clock: %s silent, no longer pacing on it", topic.c_str());
        }
        idle = std::none_of(gates_.begin(), gates_.end(),
            [](const auto & item) {return item.second.live;});
        next_ns = next_step();
      }
      if (idle) {
        std::this_thread::sleep_for(std::chrono::duration<double>(step_ns_ / 1e9));
      }
      sim_ns_ = next_ns;
      publish();
      const auto wall = Clock::now();
      const double elapsed = std::chrono::duration<double>(wall - report_wall).count();
      if (elapsed >= 10.0) {
        RCLCPP_INFO(
          get_logger(), "sim_clock: %.1fx", (sim_ns_ - report_sim) / 1e9 / elapsed);
        report_wall = wall;
        report_sim = sim_ns_;
      }
    }
  }

private:
  void on_timer()
  {
    const auto wall = Clock::now();
    sim_ns_ += static_cast<int64_t>(
      std::chrono::duration<double>(wall - last_wall_).count() * rate_ * 1e9);
    last_wall_ = wall;
    publish();
  }

  void publish()
  {
    rosgraph_msgs::msg::Clock message;
    int64_t sec = sim_ns_ / 1000000000LL;
    int64_t nanosec = sim_ns_ % 1000000000LL;
    if (nanosec < 0) {
      --sec;
      nanosec += 1000000000LL;
    }
    message.clock.sec = static_cast<int32_t>(sec);
    message.clock.nanosec = static_cast<uint32_t>(nanosec);
    publisher_->publish(message);
  }

  void on_paced(const std::string & topic, int64_t stamp_ns)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    auto & gate = gates_.at(topic);
    gate.last_ns = gate.last_ns ? std::max(*gate.last_ns, stamp_ns) : stamp_ns;
    gate.live = true;
    condition_.notify_one();
  }

  std::vector<std::string> blocking() const
  {
    std::vector<std::string> result;
    for (const auto & [topic, gate] : gates_) {
      if (gate.live && *gate.last_ns + gate.period_ns <= sim_ns_) {
        result.push_back(topic);
      }
    }
    return result;
  }

  int64_t next_step() const
  {
    int64_t next = sim_ns_ + step_ns_;
    for (const auto & [topic, gate] : gates_) {
      (void)topic;
      if (gate.live) {
        next = std::min(next, *gate.last_ns + gate.period_ns);
      }
    }
    return next;
  }

  double rate_ = 1.0;
  int64_t step_ns_ = 0;
  double max_wait_s_ = 0.5;
  int64_t sim_ns_ = 0;
  Clock::time_point last_wall_;
  rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::vector<rclcpp::GenericSubscription::SharedPtr> subscriptions_;
  mutable std::mutex mutex_;
  std::condition_variable condition_;
  std::map<std::string, Gate> gates_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SimClock>();
  if (node->paced()) {
    std::thread thread([node] {
      try {
        node->run_paced();
      } catch (const std::exception &) {
        // Context shutdown can race a graph query in the pacing loop.
        if (rclcpp::ok()) {throw;}
      }
    });
    rclcpp::spin(node);
    rclcpp::shutdown();
    thread.join();
  } else {
    rclcpp::spin(node);
    rclcpp::shutdown();
  }
  return 0;
}
