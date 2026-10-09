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

// Scripted spawn-to-center routes; the real MCB retains aim and fire.
// see README.md for design rationale

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "builtin_interfaces/msg/time.hpp"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "gz/msgs/double.pb.h"
#include "gz/msgs/pose.pb.h"
#include "gz/msgs/pose_v.pb.h"
#include "gz/transport/Node.hh"
#include "nav_msgs/msg/odometry.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sim/combat.hpp"
#include "sim/cv_head_aim_core.hpp"
#include "sim/match_scenario.hpp"
#include "std_msgs/msg/header.hpp"
#include "urdf/model.h"

namespace {
using Vector2 = Eigen::Vector2d;
using Vector3 = Eigen::Vector3d;
using Matrix4 = Eigen::Matrix4d;
using Route = std::vector<Vector2>;

struct PipeCloser {
  void operator()(FILE *file) const { pclose(file); }
};

builtin_interfaces::msg::Time message_time(const rclcpp::Time &time) {
  return rclcpp::convert_rcl_time_to_sec_nanos(time.nanoseconds());
}

const std::map<std::string, Route> kRoutes = [] {
  std::map<std::string, Route> result;
  for (const auto &[name, points] : sim::match_scenario::routes()) {
    for (const auto &point : points) {
      result[name].emplace_back(point[0], point[1]);
    }
  }
  return result;
}();
const std::vector<std::string> kRouteNames = {"sentry", "opponent_0", "ally_0",
                                              "opponent_1"};
const auto &kTeams = sim::match_scenario::teams();
using sim::match_scenario::ingress_duration;
using sim::match_scenario::match_duration;
struct MatchSample {
  Vector2 position, velocity;
  double spin;
};
struct RobotSample {
  Vector2 position, velocity;
  double yaw, spin;
};
MatchSample sample_match(double seconds, const std::string &stage) {
  const auto sample = sim::match_scenario::sample_match(seconds, stage);
  return {{sample.position[0], sample.position[1]},
          {sample.velocity[0], sample.velocity[1]},
          sample.spin};
}
RobotSample sample_robot(const std::string &name, double seconds,
                         const std::string &stage) {
  const auto sample = sim::match_scenario::sample_robot(name, seconds, stage);
  return {{sample.position[0], sample.position[1]},
          {sample.velocity[0], sample.velocity[1]},
          sample.yaw,
          sample.spin};
}

Matrix4 pose_matrix(const gz::msgs::Pose &pose) {
  const auto &p = pose.position();
  const auto &q = pose.orientation();
  return sim::combat::pose_matrix(
      Vector3(p.x(), p.y(), p.z()),
      Eigen::Quaterniond(q.w(), q.x(), q.y(), q.z()));
}

// CPython Random's integer seeding, 53-bit random(), and cached Box-Muller
// gauss().
class PythonRandom {
public:
  explicit PythonRandom(int64_t seed) {
    uint64_t magnitude = seed < 0 ? static_cast<uint64_t>(-(seed + 1)) + 1 : static_cast<uint64_t>(seed);
    std::vector<uint32_t> key;
    do {
      key.push_back(static_cast<uint32_t>(magnitude));
      magnitude >>= 32;
    } while (magnitude);
    init_by_array(key);
  }

  double gauss(double sigma) {
    double z;
    if (next_gauss_) {
      z = *next_gauss_;
      next_gauss_.reset();
    } else {
      const double angle = random() * (2.0 * M_PI);
      const double radius = std::sqrt(-2.0 * std::log(1.0 - random()));
      z = std::cos(angle) * radius;
      next_gauss_ = std::sin(angle) * radius;
    }
    return z * sigma;
  }

private:
  static constexpr size_t kN = 624;
  static constexpr size_t kM = 397;

  void init_by_array(const std::vector<uint32_t> &key) {
    state_[0] = 19650218u;
    for (size_t i = 1; i < kN; ++i) {
      state_[i] = 1812433253u * (state_[i - 1] ^ (state_[i - 1] >> 30)) + i;
    }
    size_t i = 1, j = 0;
    for (size_t k = std::max(kN, key.size()); k > 0; --k) {
      state_[i] =
          (state_[i] ^ ((state_[i - 1] ^ (state_[i - 1] >> 30)) * 1664525u)) +
          key[j] + j;
      if (++i >= kN) {
        state_[0] = state_[kN - 1];
        i = 1;
      }
      if (++j >= key.size()) {
        j = 0;
      }
    }
    for (size_t k = kN - 1; k > 0; --k) {
      state_[i] = (state_[i] ^
                   ((state_[i - 1] ^ (state_[i - 1] >> 30)) * 1566083941u)) -
                  i;
      if (++i >= kN) {
        state_[0] = state_[kN - 1];
        i = 1;
      }
    }
    state_[0] = 0x80000000u;
    index_ = kN;
  }

  uint32_t next_uint32() {
    if (index_ >= kN) {
      for (size_t i = 0; i < kN; ++i) {
        const uint32_t y =
            (state_[i] & 0x80000000u) | (state_[(i + 1) % kN] & 0x7fffffffu);
        state_[i] =
            state_[(i + kM) % kN] ^ (y >> 1) ^ ((y & 1u) ? 0x9908b0dfu : 0u);
      }
      index_ = 0;
    }
    uint32_t y = state_[index_++];
    y ^= y >> 11;
    y ^= (y << 7) & 0x9d2c5680u;
    y ^= (y << 15) & 0xefc60000u;
    y ^= y >> 18;
    return y;
  }

  double random() {
    const uint32_t a = next_uint32() >> 5;
    const uint32_t b = next_uint32() >> 6;
    return (a * 67108864.0 + b) / 9007199254740992.0;
  }

  std::array<uint32_t, kN> state_{};
  size_t index_ = kN;
  std::optional<double> next_gauss_;
};

std::vector<Matrix4> armor_offsets() {
  const std::string path = ament_index_cpp::get_package_share_directory("sim") +
                           "/urdf/sentry_v2.urdf.xacro";
  const std::string command = "xacro '" + path + "'";
  std::unique_ptr<FILE, PipeCloser> pipe(popen(command.c_str(), "r"));
  if (!pipe) {
    throw std::runtime_error("could not run xacro for armor offsets");
  }
  std::string xml;
  std::array<char, 4096> buffer{};
  while (const size_t count =
             fread(buffer.data(), 1, buffer.size(), pipe.get())) {
    xml.append(buffer.data(), count);
  }
  urdf::Model model;
  if (!model.initString(xml)) {
    throw std::runtime_error("could not parse expanded sentry URDF");
  }
  std::vector<Matrix4> offsets;
  for (int i = 0; i < 4; ++i) {
    const auto link = model.getLink("armor_" + std::to_string(i));
    if (!link || !link->parent_joint) {
      throw std::runtime_error("sentry URDF is missing an armor joint");
    }
    const auto &origin = link->parent_joint->parent_to_joint_origin_transform;
    offsets.push_back(sim::combat::pose_matrix(
        Vector3(origin.position.x, origin.position.y, origin.position.z),
        Eigen::Quaterniond(origin.rotation.w, origin.rotation.x,
                           origin.rotation.y, origin.rotation.z)));
  }
  return offsets;
}
} // namespace

class MatchDriver : public rclcpp::Node {
public:
  MatchDriver()
      : Node("match_driver"), random_(declare_parameter("seed", 2026)) {
    declare_parameter("active", false);
    stage_ = declare_parameter("stage", std::string("mcb_drive"));
    for (const auto &name : kRouteNames) {
      hp_[name] = 400;
    }
    position_ = kRoutes.at("sentry").front();
    command_pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    reference_pub_ =
        create_publisher<nav_msgs::msg::Odometry>("/sim/match/reference", 10);
    path_names_ =
        stage_ == "mcb_match"
            ? std::vector<std::string>{"opponent_0", "opponent_1", "ally_0"}
            : std::vector<std::string>{"opponent_0"};
    for (const auto &name : path_names_) {
      path_pubs_[name] = create_publisher<nav_msgs::msg::Odometry>(
          "/sim/match/" + name + "/path", 10);
    }
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        "/sim/raw_odom", 10,
        [this](nav_msgs::msg::Odometry::ConstSharedPtr message) {
          on_odom(*message);
        });
    if (stage_ == "mcb_match") {
      setup_combat();
    }
    param_callback_ = add_on_set_parameters_callback(
        [this](const std::vector<rclcpp::Parameter> &params) {
          return on_params(params);
        });
    timer_ =
        create_timer(std::chrono::duration<double>(0.01), [this] { tick(); });
    RCLCPP_INFO(get_logger(), "spawn-to-center scenario: %.2f sim seconds",
                match_duration(stage_));
  }

private:
  struct Truth {
    double stamp_s;
    Matrix4 root;
  };

  rcl_interfaces::msg::SetParametersResult
  on_params(const std::vector<rclcpp::Parameter> &params) {
    for (const auto &param : params) {
      if (param.get_name() == "active") {
        start_s_ = param.as_bool() ? std::optional<double>(now().seconds())
                                   : std::nullopt;
        velocity_.setZero();
        spin_ = 0.0;
      }
    }
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    return result;
  }

  void on_odom(const nav_msgs::msg::Odometry &message) {
    const auto &p = message.pose.pose.position;
    const auto &q = message.pose.pose.orientation;
    const auto &v = message.twist.twist.linear;
    position_ = Vector2(p.x, p.y);
    const double tx = 2.0 * (q.y * v.z - q.z * v.y);
    const double ty = 2.0 * (q.z * v.x - q.x * v.z);
    const double tz = 2.0 * (q.x * v.y - q.y * v.x);
    world_velocity_ = Vector2(v.x + q.w * tx + q.y * tz - q.z * ty,
                              v.y + q.w * ty + q.z * tx - q.x * tz);
    yaw_ = std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z));
  }

  nav_msgs::msg::Odometry
  path_message(const std::string &name, const Vector2 &position,
               const Vector2 &velocity, double yaw, double spin,
               const builtin_interfaces::msg::Time &stamp) const {
    nav_msgs::msg::Odometry message;
    message.header.stamp = stamp;
    message.header.frame_id = "map";
    message.child_frame_id = name;
    message.pose.pose.position.x = position.x();
    message.pose.pose.position.y = position.y();
    message.pose.pose.orientation.z = std::sin(yaw / 2.0);
    message.pose.pose.orientation.w = std::cos(yaw / 2.0);
    const double c = std::cos(yaw), s = std::sin(yaw);
    message.twist.twist.linear.x = c * velocity.x() + s * velocity.y();
    message.twist.twist.linear.y = -s * velocity.x() + c * velocity.y();
    message.twist.twist.angular.z = spin;
    return message;
  }

  void tick() {
    const rclcpp::Time current = now();
    const double seconds = current.seconds();
    const double previous = last_s_ && *last_s_ != 0.0 ? *last_s_ : seconds;
    const double dt = std::min(std::max(seconds - previous, 0.0), 0.1);
    last_s_ = seconds;
    const double elapsed = start_s_ ? std::max(seconds - *start_s_, 0.0) : 0.0;
    auto path = sample_match(elapsed, stage_);
    if (!start_s_) {
      path = {kRoutes.at("sentry").front(), Vector2::Zero(), 0.0};
    }
    reference_pub_->publish(path_message("root", path.position, path.velocity,
                                         yaw_, path.spin,
                                         message_time(current)));
    if (hp_.at("sentry") == 0) {
      path.velocity.setZero();
      path.spin = 0.0;
      path.position = position_;
    }
    Vector2 desired = path.velocity + 2.0 * (path.position - position_);
    const double norm = desired.norm();
    if (norm > 2.0) {
      desired *= 2.0 / norm;
    }
    const Vector2 delta = desired - velocity_;
    const double length = delta.norm();
    const double scale = length != 0.0 ? std::min(1.0, 2.0 * dt / length) : 1.0;
    velocity_ += scale * delta;
    spin_ += std::clamp(path.spin - spin_, -20.0 * dt, 20.0 * dt);
    geometry_msgs::msg::Twist command;
    const double c = std::cos(yaw_), s = std::sin(yaw_);
    command.linear.x = c * velocity_.x() + s * velocity_.y();
    command.linear.y = -s * velocity_.x() + c * velocity_.y();
    command.angular.z = spin_;
    command_pub_->publish(command);

    std::map<std::string, std::pair<Vector2, Vector2>> samples;
    samples["sentry"] = {position_, world_velocity_};
    for (const auto &name : path_names_) {
      auto robot = sample_robot(name, elapsed, stage_);
      samples[name] = {robot.position, robot.velocity};
      if (hp_.at(name) == 0) {
        std::lock_guard<std::mutex> lock(truth_mutex_);
        const auto found = truth_.find(name);
        if (found != truth_.end()) {
          robot.position = found->second.root.topRightCorner<2, 1>();
          robot.yaw =
              std::atan2(found->second.root(1, 0), found->second.root(0, 0));
        }
        robot.velocity.setZero();
        robot.spin = 0.0;
      }
      path_pubs_.at(name)->publish(
          path_message(name, robot.position, robot.velocity, robot.yaw,
                       robot.spin, message_time(current)));
    }
    if (stage_ == "mcb_match" && start_s_ && elapsed < match_duration(stage_)) {
      combat(elapsed, current, samples);
    }
  }

  void setup_combat() {
    gz_ = std::make_unique<gz::transport::Node>();
    armor_offsets_ = armor_offsets();
    for (const auto &name : path_names_) {
      head_commands_[name] = {
          gz_->Advertise<gz::msgs::Double>("/model/" + name +
                                           "/joint/headlink/cmd_pos"),
          gz_->Advertise<gz::msgs::Double>("/model/" + name +
                                           "/joint/headpitch/cmd_pos")};
      shot_pubs_[name] = create_publisher<std_msgs::msg::Header>(
          "/sim/match/" + name + "/shot", 10);
    }
    for (const auto &name : kRouteNames) {
      gz_->Subscribe<gz::msgs::Pose_V>(
          "/model/" + name + "/pose",
          [this, name](const gz::msgs::Pose_V &message) {
            on_truth(name, message);
          });
    }
    referee_sub_ = create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
        "/sim/match/referee", 10,
        [this](diagnostic_msgs::msg::DiagnosticArray::ConstSharedPtr message) {
          on_referee(*message);
        });
  }

  void on_referee(const diagnostic_msgs::msg::DiagnosticArray &message) {
    for (const auto &status : message.status) {
      for (const auto &value : status.values) {
        if (value.key == "hp" && hp_.count(status.name)) {
          hp_.at(status.name) = std::stoi(value.value);
        }
      }
    }
  }

  void on_truth(const std::string &name, const gz::msgs::Pose_V &message) {
    const gz::msgs::Pose *model = nullptr;
    const gz::msgs::Pose *root = nullptr;
    for (const auto &pose : message.pose()) {
      if (pose.name() == name) {
        model = &pose;
      }
      if (pose.name() == name + "::root") {
        root = &pose;
      }
    }
    if (!model || !root) {
      return;
    }
    const double stamp_s =
        model->header().stamp().sec() + model->header().stamp().nsec() * 1e-9;
    const Truth sample{stamp_s, pose_matrix(*model) * pose_matrix(*root)};
    std::lock_guard<std::mutex> lock(truth_mutex_);
    if (!truth_.count(name)) {
      truth_order_.push_back(name);
    }
    truth_[name] = sample;
  }

  void
  combat(double elapsed, const rclcpp::Time &current,
         const std::map<std::string, std::pair<Vector2, Vector2>> &samples) {
    std::map<std::string, Truth> truth;
    std::vector<std::string> order;
    {
      std::lock_guard<std::mutex> lock(truth_mutex_);
      truth = truth_;
      order = truth_order_;
    }
    const double seconds = current.seconds();
    for (const auto &name : path_names_) {
      if (hp_.at(name) == 0 || !truth.count(name) ||
          seconds - truth.at(name).stamp_s > 0.1) {
        continue;
      }
      const Matrix4 &root = truth.at(name).root;
      std::optional<std::string> target;
      double nearest = 0.0;
      for (const auto &enemy : order) {
        if (kTeams.at(enemy) == kTeams.at(name) || hp_.at(enemy) <= 0 ||
            seconds - truth.at(enemy).stamp_s > 0.1) {
          continue;
        }
        const double distance = (truth.at(enemy).root.topRightCorner<2, 1>() -
                                 root.topRightCorner<2, 1>())
                                    .norm();
        if (!target || distance < nearest) {
          target = enemy;
          nearest = distance;
        }
      }
      if (!target) {
        continue;
      }
      const Matrix4 &target_root = truth.at(*target).root;
      Matrix4 panel = Matrix4::Zero();
      double facing = 0.0;
      bool first = true;
      for (const auto &offset : armor_offsets_) {
        const Matrix4 candidate = target_root * offset;
        const double score = candidate.topLeftCorner<3, 1>().dot(
            root.topRightCorner<3, 1>() - candidate.topRightCorner<3, 1>());
        if (first || score > facing) {
          panel = candidate;
          facing = score;
          first = false;
        }
      }
      Vector3 aim = panel.topRightCorner<3, 1>();
      const double flight = (aim - root.topRightCorner<3, 1>()).norm() / 25.0;
      aim.head<2>() += samples.at(*target).second * flight;
      aim.z() += 9.80665 * flight * flight / 2.0;
      const Vector3 relative =
          (root.inverse() * Eigen::Vector4d(aim.x(), aim.y(), aim.z(), 1.0))
              .head<3>();
      auto [yaw, pitch] = sim::cv_head_aim::solve_head_angles(relative);
      yaw += random_.gauss(0.015);
      pitch += random_.gauss(0.015);
      gz::msgs::Double yaw_message;
      yaw_message.set_data(yaw);
      head_commands_.at(name)[0].Publish(yaw_message);
      gz::msgs::Double pitch_message;
      pitch_message.set_data(std::clamp(pitch, -0.6, 0.6));
      head_commands_.at(name)[1].Publish(pitch_message);
      if (elapsed < ingress_duration(stage_)) {
        continue;
      }
      if (seconds >= next_fire_[name]) {
        std_msgs::msg::Header shot;
        shot.stamp = message_time(current);
        shot.frame_id = name + "/muzzle";
        shot_pubs_.at(name)->publish(shot);
        next_fire_[name] = seconds + 0.5;
      }
    }
  }

  std::string stage_;
  PythonRandom random_;
  std::map<std::string, int> hp_;
  Vector2 world_velocity_ = Vector2::Zero();
  std::map<std::string, Truth> truth_;
  std::vector<std::string> truth_order_;
  std::mutex truth_mutex_;
  std::map<std::string, double> next_fire_;
  std::optional<double> start_s_;
  std::optional<double> last_s_;
  Vector2 position_ = Vector2::Zero();
  double yaw_ = M_PI;
  Vector2 velocity_ = Vector2::Zero();
  double spin_ = 0.0;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr command_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr reference_pub_;
  std::vector<std::string> path_names_;
  std::map<std::string, rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr>
      path_pubs_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr
      referee_sub_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr
      param_callback_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::unique_ptr<gz::transport::Node> gz_;
  std::vector<Matrix4> armor_offsets_;
  std::map<std::string, std::array<gz::transport::Node::Publisher, 2>>
      head_commands_;
  std::map<std::string, rclcpp::Publisher<std_msgs::msg::Header>::SharedPtr>
      shot_pubs_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MatchDriver>());
  rclcpp::shutdown();
  return 0;
}
