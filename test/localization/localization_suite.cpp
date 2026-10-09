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

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rcl_interfaces/srv/get_parameters.hpp"
#include "rcl_interfaces/srv/set_parameters.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sim/gazebo_helpers.hpp"
#include "sim/process.hpp"
#include "sim/suite_timing.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include <algorithm>
#include <array>
#include <cctype>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <functional>
#include <gtest/gtest.h>
#include <iomanip>
#include <iostream>
#include <limits>
#include <map>
#include <memory>
#include <mutex>
#include <numeric>
#include <optional>
#include <sstream>
#include <string>
#include <thread>
#include <tuple>
#include <utility>
#include <vector>
namespace {
using Point = std::array<double, 2>;
using Pose = std::array<double, 3>;
using Leg = std::array<double, 3>;
using Clock = std::chrono::steady_clock;
namespace timing = sim::suite_timing;
constexpr double kPi = 3.14159265358979323846;
constexpr double kMaxDelta = 0.40;
constexpr char kLogDir[] = "/tmp/localization_drift_tests";
struct Options {
  std::string suite = "drift", backend = "amcl", scenario, rtf = "0";
  bool gui = true, rf2o = true, restart = false;
  double speed = 4.0, accel = 20.0, spawn_yaw = sim::gazebo::kSpawnYaw;
  double slip = 0.05, drift = 0.002, seconds = 45.0;
} options;
std::vector<std::string> scenario_names = {
    "baseline",         "noise_correction",
    "drift_correction", "drift_correction_obstacle",
    "moving_obstacles", "real_accel",
    "jerk_with_motion", "odom_stuck",
    "scan_degraded"};
std::array<Leg, 4> loop_legs() {
  const double s = options.speed, d = 3.0 / s;
  return {{{0.0, -s, d}, {s, 0.0, d}, {0.0, s, d}, {-s, 0.0, d}}};
}
std::string fixed(double value, int precision = 4) {
  std::ostringstream out;
  out << std::fixed << std::setprecision(precision) << value;
  return out.str();
}
std::string elapsed(double value) {
  std::ostringstream out;
  out << std::fixed << std::setprecision(1) << std::setw(5) << value;
  return out.str();
}
std::string pose_string(const Pose &p) {
  return "(" + fixed(p[0], 6) + ", " + fixed(p[1], 6) + ", " + fixed(p[2], 6) +
         ")";
}
double distance(const Point &a, const Point &b) {
  return std::hypot(a[0] - b[0], a[1] - b[1]);
}
Point xy(const Pose &p) { return {p[0], p[1]}; }
std::vector<std::string> log_errors(const std::string &text) {
  std::vector<std::string> errors;
  std::istringstream input(text);
  std::string line;
  while (std::getline(input, line)) {
    auto low = line;
    std::transform(low.begin(), low.end(), low.begin(),
                   [](unsigned char c) { return std::tolower(c); });
    if (low.find("failed to meet update rate") != std::string::npos) {
      continue;
    }
    if (low.find("[error]") != std::string::npos ||
        low.find("traceback") != std::string::npos ||
        low.find("segmentation fault") != std::string::npos) {
      errors.push_back(line);
    }
  }
  return errors;
}
struct Scenario {
  std::string name;
  bool passed = false, skipped = false;
  std::vector<std::string> details;
  explicit Scenario(std::string value) : name(std::move(value)) {}
  void log(const std::string &text) {
    std::cout << "    " << text << std::endl;
    details.push_back(text);
  }
  void result(bool ok, const std::string &text) {
    passed = ok;
    const std::string status = ok ? "PASS" : "FAIL";
    std::cout << "  [" << status << "] " << name << ": " << text << std::endl;
    details.push_back(status + ": " + text);
  }
};
class Helper : public rclcpp::Node {
public:
  Helper(const std::string &parent_frame, const std::string &child_frame)
      : Node("localization_drift_test_helper",
             rclcpp::NodeOptions().parameter_overrides(
                 {rclcpp::Parameter("use_sim_time", true)})),
        parent(parent_frame), child(child_frame), buffer(get_clock()),
        listener(buffer) {
    command = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    scan_sub = create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", 10,
        [this](sensor_msgs::msg::LaserScan::ConstSharedPtr) { ++scan_count; });
    truth_sub = create_subscription<nav_msgs::msg::Odometry>(
        "/sim/raw_odom", 10,
        [this](nav_msgs::msg::Odometry::ConstSharedPtr msg) {
          truth = Point{msg->pose.pose.position.x, msg->pose.pose.position.y};
          const auto &q = msg->pose.pose.orientation;
          truth_yaw = std::atan2(2 * (q.w * q.z + q.x * q.y),
                                 1 - 2 * (q.y * q.y + q.z * q.z));
          yaw_range[0] = std::min(yaw_range[0], truth_yaw);
          yaw_range[1] = std::max(yaw_range[1], truth_yaw);
        });
    joint_sub = create_subscription<sensor_msgs::msg::JointState>(
        "/sim/raw_joint_states", 10,
        [this](sensor_msgs::msg::JointState::ConstSharedPtr msg) {
          const auto found =
              std::find(msg->name.begin(), msg->name.end(), "headlink");
          if (found != msg->name.end()) {
            head_yaw = msg->position.at(
                static_cast<size_t>(found - msg->name.begin()));
            head_range[0] = std::min(head_range[0], *head_yaw);
            head_range[1] = std::max(head_range[1], *head_yaw);
          }
        });
    reset_yaw_range();
  }
  std::string parent, child;
  tf2_ros::Buffer buffer;
  tf2_ros::TransformListener listener;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr command;
  int scan_count = 0;
  std::optional<Point> truth;
  double truth_yaw = 0.0;
  std::optional<double> head_yaw;
  Pose yaw_range{}, head_range{};
  double now_s() { return get_clock()->now().seconds(); }
  void spin_wall(double seconds) {
    if (!rclcpp::ok()) {
      throw std::runtime_error("suite interrupted");
    }
    const auto end = Clock::now() + std::chrono::duration<double>(seconds);
    while (rclcpp::ok() && Clock::now() < end) {
      rclcpp::spin_some(shared_from_this());
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
  }
  void spin_for(double seconds) {
    if (!rclcpp::ok()) {
      throw std::runtime_error("suite interrupted");
    }
    const auto end = Clock::now() + std::chrono::duration<double>(
                                        std::max(3.0 * seconds, seconds + 5.0));
    std::optional<double> start;
    while (rclcpp::ok() && Clock::now() < end) {
      rclcpp::spin_some(shared_from_this());
      const double t = now_s();
      if (!start && t > 0.0) {
        start = t;
      }
      if (start && t - *start >= seconds) {
        return;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    if (!rclcpp::ok()) {
      throw std::runtime_error("suite interrupted");
    }
    RCLCPP_WARN(get_logger(),
                "spin_for: wall-clock cap hit before %gs of sim time", seconds);
  }
  bool wait_truth(double timeout = 10.0) {
    const auto end = Clock::now() + std::chrono::duration<double>(timeout);
    while (Clock::now() < end) {
      if (truth) {
        return true;
      }
      spin_wall(0.1);
    }
    return false;
  }
  bool scans_flowing(int count = 10, double timeout = 60.0) {
    scan_count = 0;
    const auto end = Clock::now() + std::chrono::duration<double>(timeout);
    while (Clock::now() < end) {
      if (scan_count >= count) {
        return true;
      }
      spin_wall(0.1);
    }
    return false;
  }
  std::optional<Pose> transform(const std::string &from, const std::string &to,
                                double timeout = 2.0) {
    try {
      const auto tf = buffer.lookupTransform(from, to, tf2::TimePointZero,
                                             tf2::durationFromSec(timeout));
      const auto &t = tf.transform.translation;
      const auto &q = tf.transform.rotation;
      return Pose{t.x, t.y,
                  std::atan2(2 * (q.w * q.z + q.x * q.y),
                             1 - 2 * (q.y * q.y + q.z * q.z))};
    } catch (const tf2::TransformException &) {
      return std::nullopt;
    }
  }
  std::optional<Pose> correction(double timeout = 2.0) {
    return transform(parent, child, timeout);
  }
  std::optional<Point> root(double timeout = 2.0) {
    const auto p = transform(parent, "root", timeout);
    return p ? std::optional<Point>(xy(*p)) : std::nullopt;
  }
  std::optional<Pose> wait_correction(double timeout = 30.0) {
    const auto end = Clock::now() + std::chrono::duration<double>(timeout);
    while (Clock::now() < end) {
      const auto p = correction(0.0);
      if (p) {
        return p;
      }
      spin_wall(0.1);
    }
    return std::nullopt;
  }
  template <typename Service>
  typename Service::Response::SharedPtr
  call(const std::string &name, typename Service::Request::SharedPtr request,
       double timeout = 10.0) {
    const auto client = create_client<Service>(name);
    if (!client->wait_for_service(std::chrono::duration<double>(timeout))) {
      throw std::runtime_error(name + " not available");
    }
    auto future = client->async_send_request(request);
    if (rclcpp::spin_until_future_complete(
            shared_from_this(), future,
            std::chrono::duration<double>(timeout)) !=
        rclcpp::FutureReturnCode::SUCCESS) {
      throw std::runtime_error(name + " call timed out");
    }
    return future.get();
  }
  std::string trigger(const std::string &name) {
    return call<std_srvs::srv::Trigger>(
               name, std::make_shared<std_srvs::srv::Trigger::Request>())
        ->message;
  }
  void set_params(const std::string &name,
                  const std::vector<rclcpp::Parameter> &parameters) {
    auto req = std::make_shared<rcl_interfaces::srv::SetParameters::Request>();
    for (const auto &param : parameters) {
      req->parameters.push_back(param.to_parameter_msg());
    }
    const auto reply = call<rcl_interfaces::srv::SetParameters>(name, req);
    for (const auto &result : reply->results) {
      if (!result.successful) {
        throw std::runtime_error(name + " rejected params: " + result.reason);
      }
    }
  }
  void reset_yaw_range() {
    yaw_range = {INFINITY, -INFINITY, 0};
    head_range = {INFINITY, -INFINITY, 0};
  }
  std::string yaw_ranges() {
    const auto fmt = [](const Pose &range) {
      return range[0] > range[1] ? "none"
                                 : fixed(range[0] * 180 / kPi, 2) + ".." +
                                       fixed(range[1] * 180 / kPi, 2) + " deg";
    };
    return "true yaw " + fmt(yaw_range) + ", head yaw " + fmt(head_range);
  }
  void drive(double vx, double vy, double duration, double accel = -1.0) {
    if (accel < 0.0) {
      accel = options.accel;
    }
    const double speed = std::hypot(vx, vy), period = 0.1;
    double safety = now_s() + std::max(duration * 3.0, duration + 5.0);
    if (accel != 0.0) {
      safety += 2 * std::sqrt(speed * duration / accel);
    }
    if (speed <= 1e-6 || !wait_truth()) {
      geometry_msgs::msg::Twist msg;
      msg.linear.x = vx;
      msg.linear.y = vy;
      const double end = now_s() + duration;
      while (now_s() < end) {
        command->publish(msg);
        spin_for(0.1);
      }
      command->publish(geometry_msgs::msg::Twist{});
      spin_for(0.2);
      return;
    }
    const Point target{(*truth)[0] + vx * duration,
                       (*truth)[1] + vy * duration};
    double speed_cmd = 0.0;
    while (now_s() < safety) {
      const double dx = target[0] - (*truth)[0], dy = target[1] - (*truth)[1],
                   dist = std::hypot(dx, dy);
      if (dist <= 0.03) {
        break;
      }
      double speed_now = std::min(speed, dist / period);
      if (accel != 0.0) {
        const double at = accel * period;
        speed_now = std::min({speed_now, speed_cmd + at,
                              std::sqrt(at * at + 2 * accel * dist) - at});
      }
      speed_cmd = speed_now;
      const double c = std::cos(truth_yaw), s = std::sin(truth_yaw);
      geometry_msgs::msg::Twist msg;
      msg.linear.x = speed_now * (c * dx + s * dy) / dist;
      msg.linear.y = speed_now * (-s * dx + c * dy) / dist;
      command->publish(msg);
      spin_for(period);
    }
    if (now_s() >= safety) {
      RCLCPP_WARN(get_logger(),
                  "drive(%g, %g, %g): hit safety cap %.3fm short of intended "
                  "endpoint (%.3f, %.3f)",
                  vx, vy, duration, distance(target, *truth), target[0],
                  target[1]);
    }
    command->publish(geometry_msgs::msg::Twist{});
    spin_for(0.2);
  }

private:
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr truth_sub;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub;
};
std::unique_ptr<sim::LaunchTree> sim_tree, actor_tree;
std::shared_ptr<Helper> shared_helper;
std::vector<rclcpp::Parameter> defaults;
std::vector<std::string> spawned;
int robot_runs = 0, actor_runs = 0;
std::vector<std::string>
launch_command(const std::map<std::string, std::string> &args) {
  std::vector<std::string> command{"ros2", "launch", "sim",
                                   "localization_tests.launch.py"};
  for (const auto &entry : args) {
    command.push_back(entry.first + ":=" + entry.second);
  }
  return command;
}
void stop_actor() { actor_tree.reset(); }
void stop_sim() {
  timing::Phase phase("teardown");
  stop_actor();
  timing::set_sim_clock({});
  shared_helper.reset();
  sim_tree.reset();
  spawned.clear();
}
class Stack {
public:
  std::unique_ptr<sim::LaunchTree> robot;
  std::shared_ptr<Helper> helper;
  size_t sim_log_start = 0;
  ~Stack() {
    timing::Phase phase("teardown");
    stop_actor();
    if (helper != shared_helper) {
      timing::set_sim_clock({});
      helper.reset();
    }
    robot.reset();
  }
  std::string log_text() {
    std::string text = robot ? robot->read_log() : "";
    if (sim_tree) {
      std::istringstream input(sim_tree->read_log().substr(sim_log_start));
      std::string line;
      while (std::getline(input, line)) {
        if (line.rfind("[rviz2", 0) != 0) {
          text += line + '\n';
        }
      }
    }
    return text;
  }
};
std::unique_ptr<Stack> run_stack(bool noise,
                                 std::vector<rclcpp::Parameter> params = {},
                                 std::string backend = "") {
  if (backend.empty()) {
    backend = options.backend;
  }
  const std::string parent = backend == "none" ? "odom" : "map",
                    child = backend == "none" ? "root" : "odom";
  std::filesystem::create_directories(kLogDir);
  auto stack = std::make_unique<Stack>();
  const auto helper_for = [&] {
    if (shared_helper &&
        (shared_helper->parent != parent || shared_helper->child != child)) {
      timing::set_sim_clock({});
      shared_helper.reset();
    }
    if (!shared_helper) {
      shared_helper = std::make_shared<Helper>(parent, child);
    } else {
      shared_helper->spin_wall(0.1);
      shared_helper->buffer.clear();
    }
    return shared_helper;
  };
  std::map<std::string, std::string> args{
      {"run_tests", "false"},
      {"headless", options.gui ? "false" : "true"},
      {"real_time_factor", options.rtf},
      {"backend", backend},
      {"use_rf2o", options.rf2o ? "true" : "false"}};
  params.emplace_back("odom_noise_enabled", noise);
  if (std::none_of(params.begin(), params.end(), [](const auto &p) {
        return p.get_name() == "odom_slip_ratio";
      })) {
    params.emplace_back("odom_slip_ratio", 0.02);
  }
  if (options.restart) {
    stack->helper = std::make_shared<Helper>(parent, child);
    timing::set_sim_clock([helper = stack->helper] { return helper->now_s(); });
    for (const auto &p : params) {
      args[p.get_name()] = p.value_to_string();
    }
    stack->robot = std::make_unique<sim::LaunchTree>(
        "stack", launch_command(args), std::string(kLogDir) + "/stack.log");
    return stack;
  }
  if (sim_tree &&
      (!sim_tree->alive() ||
       sim_tree->read_log().find("process has died") != std::string::npos)) {
    std::cout
        << "[sim] the shared sim exited or lost a process; starting a new one"
        << std::endl;
    stop_sim();
  }
  {
    timing::Phase phase("reset");
    stack->helper = helper_for();
  }
  timing::set_sim_clock([helper = stack->helper] { return helper->now_s(); });
  auto &helper = *stack->helper;
  if (!sim_tree) {
    timing::Phase phase("sim_start");
    sim_tree = std::make_unique<sim::LaunchTree>(
        "sim",
        launch_command({{"run_tests", "false"},
                        {"part", "sim"},
                        {"headless", options.gui ? "false" : "true"},
                        {"real_time_factor", options.rtf}}),
        std::string(kLogDir) + "/sim.log");
    const auto end = Clock::now() + std::chrono::seconds(90);
    while (!helper.wait_truth(1.0)) {
      if (sim_tree->read_log().find("process has died") != std::string::npos ||
          Clock::now() > end) {
        sim_tree.reset();
        throw std::runtime_error(
            "sim never published /sim/raw_odom; see sim.log");
      }
    }
    auto req = std::make_shared<rcl_interfaces::srv::GetParameters::Request>();
    req->names = {"odom_noise_enabled",     "odom_drift_stddev",
                  "odom_jitter_stddev",     "odom_jerk_stddev",
                  "odom_jerk_bias_enabled", "odom_jerk_bias_x",
                  "odom_jerk_bias_y",       "odom_slip_ratio"};
    const auto reply = helper.call<rcl_interfaces::srv::GetParameters>(
        "/pose_emulator/get_parameters", req, 30.0);
    defaults.clear();
    for (size_t i = 0; i < req->names.size(); ++i) {
      rcl_interfaces::msg::Parameter msg;
      msg.name = req->names[i];
      msg.value = reply->values.at(i);
      defaults.push_back(rclcpp::Parameter::from_parameter_msg(msg));
    }
  }
  {
    timing::Phase phase("reset");
    helper.command->publish(geometry_msgs::msg::Twist{});
    helper.spin_for(0.2);
    stop_actor();
    for (const auto &name : spawned) {
      if (!sim::gazebo::remove_model(name)) {
        throw std::runtime_error("could not remove '" + name +
                                 "' from the world");
      }
    }
    if (!sim::gazebo::teleport(0, 0, sim::gazebo::kZ, options.spawn_yaw)) {
      throw std::runtime_error("teleport back to spawn failed");
    }
    helper.spin_for(0.5);
    const auto names = sim::gazebo::model_names();
    if (!names) {
      throw std::runtime_error("gz scene/info did not answer");
    }
    for (const auto &name : spawned) {
      if (std::find(names->begin(), names->end(), name) != names->end()) {
        throw std::runtime_error("still in the world after removal: " + name);
      }
    }
    spawned.clear();
    auto values = defaults;
    for (const auto &p : params) {
      const auto found =
          std::find_if(values.begin(), values.end(), [&](const auto &value) {
            return value.get_name() == p.get_name();
          });
      if (found == values.end()) {
        values.push_back(p);
      } else {
        *found = p;
      }
    }
    helper.set_params("/pose_emulator/set_parameters", values);
    helper.trigger("/pose_emulator/reset");
  }
  {
    timing::Phase phase("bringup");
    for (int attempt = 1; attempt <= 2; ++attempt) {
      args["part"] = "robot";
      ++robot_runs;
      stack->robot = std::make_unique<sim::LaunchTree>(
          "robot", launch_command(args),
          std::string(kLogDir) + "/robot_" + std::to_string(robot_runs) + ".log");
      const auto end = Clock::now() + std::chrono::seconds(30);
      bool ready = false;
      if (attempt == 1) {
        while (Clock::now() < end) {
          if (helper.root(0.0)) {
            ready = true;
            break;
          }
          helper.spin_wall(0.1);
        }
      }
      if (attempt == 2 || ready) {
        stack->sim_log_start = sim_tree->read_log().size();
        return stack;
      }
      std::cout << "[robot] " << parent
                << "->root not up after 30s; restarting the robot stack once "
                   "(bring-up race, see _wait_for_root_chain)"
                << std::endl;
      stack->robot.reset();
    }
  }
  return stack;
}
bool ready(Scenario &sc, Helper &helper) {
  timing::Phase phase("bringup");
  const bool ok = helper.scans_flowing();
  sc.log(ok ? "stack ready: >= 10 /scan messages received"
            : "stack NOT ready: fewer than 10 /scan messages received within "
              "60.0s -- treating as an unhealthy/too-slow run, not a "
              "correctness result");
  if (!ok) {
    sc.result(
        false,
        "stack failed to reach a healthy /scan rate in time -- see log above");
  }
  return ok;
}
bool correction_ready(Scenario &sc, Helper &helper, std::optional<Pose> &p) {
  p = helper.wait_correction(45.0);
  if (!p) {
    sc.result(false, helper.parent + "->" + helper.child +
                         " never became available within 45s");
  }
  return p.has_value();
}
void reposition(Helper &helper) {
  helper.drive(0.0, options.speed, 1.5 / options.speed);
  helper.drive(-options.speed, 0.0, 1.5 / options.speed);
}
std::optional<double> truth_error(Helper &helper) {
  const auto root = helper.root();
  return root && helper.truth
             ? std::optional<double>(distance(*root, *helper.truth))
             : std::nullopt;
}
bool truth_scored() {
  return options.backend == "none" || options.backend == "mapping";
}
void baseline(Scenario &sc) {
  auto stack = run_stack(false);
  auto &helper = *stack->helper;
  if (!ready(sc, helper)) {
    return;
  }
  std::optional<Pose> p;
  if (!correction_ready(sc, helper, p)) {
    return;
  }
  const auto edge = helper.parent + "->" + helper.child;
  const double mag = std::hypot((*p)[0], (*p)[1]);
  sc.log(edge + " = (x=" + fixed((*p)[0]) + ", y=" + fixed((*p)[1]) +
         ", yaw=" + fixed((*p)[2]) + "), |xy|=" + fixed(mag) + " m");
  helper.spin_for(10.0);
  const auto second = helper.wait_correction(5.0).value_or(*p);
  const double drift = distance(xy(second), xy(*p));
  sc.log("after +10s: " + edge + " = (x=" + fixed(second[0]) +
         ", y=" + fixed(second[1]) +
         "), drift from first sample = " + fixed(drift) + " m");
  const auto errors = log_errors(stack->log_text());
  for (size_t i = 0; i < std::min<size_t>(10, errors.size()); ++i) {
    sc.log("log error: " + errors[i]);
  }
  sc.result(drift < 0.05 && errors.empty(),
            edge + " drift over 10s = " + fixed(drift) +
                " m (threshold 0.05 m; absolute offset " + fixed(mag) +
                " m is expected/normal, see note above), log_errors=" +
                std::to_string(errors.size()));
}
void noise_correction(Scenario &sc) {
  auto stack = run_stack(true);
  auto &helper = *stack->helper;
  if (!ready(sc, helper)) {
    return;
  }
  std::optional<Pose> p;
  if (!correction_ready(sc, helper, p)) {
    return;
  }
  reposition(helper);
  sc.log("repositioned to OBSTACLE_LOOP_LEGS's start corner (-1.5,-1.5) before "
         "tracing it");
  const auto edge = helper.parent + "->" + helper.child,
             metric = truth_scored() ? "truth error" : "|" + edge + " xy|";
  std::vector<double> samples;
  const double t0 = helper.now_s();
  size_t i = 0;
  while (helper.now_s() - t0 < 30.0) {
    const auto leg = loop_legs()[i++ % 4];
    helper.drive(leg[0], leg[1], leg[2]);
    const auto p_now = helper.correction();
    const auto mag =
        truth_scored()
            ? truth_error(helper)
            : (p_now
                   ? std::optional<double>(std::hypot((*p_now)[0], (*p_now)[1]))
                   : std::nullopt);
    if (mag) {
      samples.push_back(*mag);
      sc.log("t=" + elapsed(helper.now_s() - t0) + "s  " + metric + "=" +
             fixed(*mag) + " m");
    }
  }
  if (samples.size() < 3) {
    sc.result(false, "too few " + metric + " samples (" +
                         std::to_string(samples.size()) +
                         ") to assess boundedness");
    return;
  }
  const auto half =
      samples.begin() + static_cast<ptrdiff_t>(samples.size() / 2);
  const double first = *std::max_element(samples.begin(), half),
               second = *std::max_element(half, samples.end()),
               max_mag = *std::max_element(samples.begin(), samples.end()),
               growth = second / std::max(first, 1e-6);
  const auto errors = log_errors(stack->log_text());
  sc.result(
      growth < 2.0 && errors.empty(),
      "max " + metric + "=" + fixed(max_mag) + " m, first_half_max=" +
          fixed(first) + ", second_half_max=" + fixed(second) +
          ", growth_ratio=" + fixed(growth, 2) +
          " (threshold 2.0), log_errors=" + std::to_string(errors.size()));
}
void spawn_box() {
  const std::string name = "unmapped_test_obstacle";
  const std::string sdf =
      "<sdf version=\"1.6\"><model name=\"" + name +
      "\"><static>true</static><pose>0 0 0.4 0 0 0</pose><link "
      "name=\"link\"><collision name=\"collision\"><geometry><box><size>0.3 "
      "0.3 0.8</size></box></geometry></collision><visual "
      "name=\"visual\"><geometry><box><size>0.3 0.3 "
      "0.8</size></box></geometry><material><ambient>0.1 0.1 0.8 "
      "1</ambient><diffuse>0.1 0.1 0.8 "
      "1</diffuse></material></visual></link></model></sdf>";
  if (!sim::gazebo::spawn_model(name, sdf, 0.0, 0.0, 0.4)) {
    throw std::runtime_error("spawning obstacle '" + name + "' failed");
  }
  spawned.push_back(name);
}
bool start_actor(Helper &helper) {
  stop_actor();
  for (int i = 0; i < 3; ++i) {
    spawned.push_back("moving_actor_" + std::to_string(i));
  }
  ++actor_runs;
  actor_tree = std::make_unique<sim::LaunchTree>(
      "actor_driver",
      std::vector<std::string>{
          "ros2", "run", "sim", "actor_driver", "--ros-args", "-p",
          "use_sim_time:=true", "-p", "count:=3", "-p",
          "name_prefix:=moving_actor", "-p",
          "route:=[-1.5, 1.5, -1.5, -1.5, 1.5, -1.5, 1.5, 1.5]", "-p",
          "route_speed:=" + fixed(options.speed, 1)},
      std::string(kLogDir) + "/actor_driver_" + std::to_string(actor_runs) + ".log");
  const auto end = Clock::now() + std::chrono::seconds(60);
  while (Clock::now() < end) {
    if (actor_tree->read_log().find("all 3 actors spawned") !=
        std::string::npos) {
      return true;
    }
    if (!actor_tree->alive()) {
      return false;
    }
    helper.spin_wall(0.5);
  }
  return false;
}
void cornering(Scenario &sc, const std::string &obstacle, double accel = -1.0) {
  auto stack = run_stack(false, {rclcpp::Parameter("odom_slip_ratio", 0.15)});
  auto &helper = *stack->helper;
  if (!ready(sc, helper)) {
    return;
  }
  std::optional<Pose> p;
  if (!correction_ready(sc, helper, p)) {
    return;
  }
  const auto edge = helper.parent + "->" + helper.child;
  sc.log(edge + " before loop start = " + pose_string(*p));
  const int scans_before = helper.scan_count;
  reposition(helper);
  sc.log("repositioned to the loop's start corner (-1.5,-1.5) before tracing "
         "its perimeter");
  if (obstacle == "box") {
    spawn_box();
    sc.log("spawned 0.3x0.3x0.8m box obstacle at (0.0, 0.0) (not present in "
           "the saved map) -- at the center of the loop this scenario is about "
           "to drive, see OBSTACLE_LOOP_LEGS");
  }
  if (obstacle == "actors") {
    if (!start_actor(helper)) {
      sc.result(false,
                "actor_driver did not spawn its actors; see actor_driver_" +
                    std::to_string(actor_runs) + ".log");
      return;
    }
    sc.log("actor_driver spawned 3 moving boxes crossing the loop");
  }
  const auto metric =
      truth_scored() ? "truth error" : "|" + edge + " - pre-loop " + edge + "|";
  std::vector<double> samples;
  helper.reset_yaw_range();
  const double t0 = helper.now_s();
  size_t i = 0;
  while (helper.now_s() - t0 < 30.0) {
    const auto leg = loop_legs()[i++ % 4];
    helper.drive(leg[0], leg[1], leg[2], accel);
    helper.spin_for(1.0);
    const auto now = helper.correction();
    const auto delta =
        truth_scored()
            ? truth_error(helper)
            : (now ? std::optional<double>(distance(xy(*now), xy(*p)))
                   : std::nullopt);
    if (delta) {
      samples.push_back(*delta);
      const auto truth = truth_scored() ? std::nullopt : truth_error(helper);
      sc.log("t=" + elapsed(helper.now_s() - t0) + "s  " + metric + "=" +
             fixed(*delta) + " m" +
             (truth ? "  ground_truth_error=" + fixed(*truth) + " m" : ""));
    }
  }
  sc.log("over the loop, every message: " + helper.yaw_ranges());
  if (samples.size() < 3) {
    sc.result(false, "too few " + metric + " samples (" +
                         std::to_string(samples.size()) +
                         ") to assess boundedness");
    return;
  }
  if (helper.scan_count <= scans_before) {
    sc.result(false, "scan count did not advance while driving the loop -- "
                     "backend may have stalled");
    return;
  }
  if (obstacle == "actors" && (!actor_tree || !actor_tree->alive())) {
    sc.result(false, "actor_driver exited mid-loop; see actor_driver_" +
                         std::to_string(actor_runs) + ".log");
    return;
  }
  const double maximum = *std::max_element(samples.begin(), samples.end());
  auto errors = log_errors(stack->log_text());
  if (obstacle == "actors") {
    const auto more = log_errors(actor_tree->read_log());
    errors.insert(errors.end(), more.begin(), more.end());
  }
  const std::string note = obstacle == "box"      ? " past the obstacle"
                           : obstacle == "actors" ? " among moving actors"
                                                  : "";
  sc.result(
      maximum < kMaxDelta && errors.empty(),
      "max " + metric + " = " + fixed(maximum) +
          " m over 30s driving the cornering loop" + note +
          " (threshold 0.4 m), log_errors=" + std::to_string(errors.size()));
}
void jerk_with_motion(Scenario &sc) {
  if (options.backend == "none") {
    sc.skipped = true;
    sc.details.push_back("no map layer under test (backend none); jerk "
                         "response is not characterized");
    std::cout << "  [SKIP] jerk_with_motion: " << sc.details.back()
              << std::endl;
    return;
  }
  auto stack =
      run_stack(false, {rclcpp::Parameter("odom_jerk_stddev", 0.24),
                        rclcpp::Parameter("odom_jerk_bias_enabled", true),
                        rclcpp::Parameter("odom_jerk_bias_x", 0.0),
                        rclcpp::Parameter("odom_jerk_bias_y", 0.0)});
  auto &helper = *stack->helper;
  if (!ready(sc, helper)) {
    return;
  }
  reposition(helper);
  sc.log("repositioned to OBSTACLE_LOOP_LEGS's start corner (-1.5,-1.5) before "
         "tracing it");
  int passed = 0;
  std::string summary;
  const auto edge = helper.parent + "->" + helper.child;
  for (int trial = 1; trial <= 8; ++trial) {
    sc.log("--- trial " + std::to_string(trial) + "/8 ---");
    const auto before = helper.wait_correction(45.0);
    if (!before) {
      summary += (summary.empty() ? "" : "; ") + std::string("trial ") +
                 std::to_string(trial) + ": " + edge +
                 " never became available within 45s";
      continue;
    }
    sc.log(edge + " before jerk = " + pose_string(*before));
    const auto response = helper.trigger("/pose_emulator/trigger_jerk");
    double dx = 0, dy = 0, magnitude = 0.24;
    try {
      const auto dx_start = response.find("dx="),
                 dy_start = response.find("dy=");
      if (dx_start == std::string::npos || dy_start == std::string::npos) {
        throw std::runtime_error("no offsets");
      }
      dx = std::stod(response.substr(dx_start + 3, dy_start - dx_start - 3));
      dy = std::stod(response.substr(dy_start + 3));
      magnitude = std::hypot(dx, dy);
      sc.log("trigger_jerk called, actual applied (dx, dy) = (" + fixed(dx) +
             ", " + fixed(dy) + "), |jerk| = " + fixed(magnitude) + " m");
    } catch (const std::exception &) {
      dx = dy = 0;
      sc.log("trigger_jerk called (could not parse actual applied dx/dy from "
             "response; falling back to a stddev-based magnitude estimate, no "
             "position correction possible this trial)");
    }
    const auto leg = loop_legs()[static_cast<size_t>(trial - 1) % 4];
    const double x = leg[0] * leg[2] - dx, y = leg[1] * leg[2] - dy,
                 dist = std::hypot(x, y);
    if (dist < 1e-6) {
      helper.drive(0, 0, 0);
    } else {
      helper.drive(4 * x / dist, 4 * y / dist, dist / 4);
    }
    const auto p = helper.correction(5.0);
    const double delta = p ? distance(xy(*p), xy(*before)) : 0.0;
    if (p) {
      sc.log(edge + " after driving to next corner: " + pose_string(*p) +
             " (|" + edge + " - pre-jerk " + edge + "|=" + fixed(delta) +
             " m)");
    } else {
      sc.log(edge + " unavailable after driving to next corner");
    }
    const double threshold = magnitude * 0.3;
    if (delta > threshold || delta <= kMaxDelta) {
      ++passed;
    }
    summary += (summary.empty() ? "" : "; ") + std::string("trial ") +
               std::to_string(trial) + ": delta " + fixed(delta) +
               " m after one leg (threshold " + fixed(threshold) +
               " m = 0.3x applied jerk " + fixed(magnitude) +
               " m, OR within MAX_DELTA_THRESHOLD 0.4 m)";
  }
  sc.log("--- extra lap around the square after all 8 trials ---");
  for (const auto &leg : loop_legs()) {
    helper.drive(leg[0], leg[1], leg[2]);
  }
  const auto p = helper.correction(5.0);
  sc.log(edge + " after extra lap = " + (p ? pose_string(*p) : "None"));
  const auto errors = log_errors(stack->log_text());
  sc.result(
      passed == 8 && errors.empty(),
      std::to_string(passed) + "/8 trials passed -- " + summary +
          " -- plus one extra closing lap around the square -- log_errors=" +
          std::to_string(errors.size()));
}
void odom_stuck(Scenario &sc) {
  auto stack = run_stack(false);
  auto &helper = *stack->helper;
  if (!ready(sc, helper)) {
    return;
  }
  std::optional<Pose> p;
  if (!correction_ready(sc, helper, p)) {
    return;
  }
  const auto edge = helper.parent + "->" + helper.child;
  sc.log(edge + " before trigger = " + pose_string(*p));
  reposition(helper);
  sc.log("repositioned to OBSTACLE_LOOP_LEGS's start corner (-1.5,-1.5) before "
         "tracing it");
  if (options.backend == "mapping") {
    for (int i = 0; i < 8; ++i) {
      const auto leg = loop_legs()[static_cast<size_t>(i) % 4];
      helper.drive(leg[0], leg[1], leg[2]);
      helper.spin_for(1.0);
    }
    sc.log("mapping window: 2 laps before the trigger");
  }
  helper.trigger("/pose_emulator/trigger_odom_stuck");
  sc.log("triggered odom_stuck: /dji_serial_bridge/pose now pinned at (0, 0)");
  helper.reset_yaw_range();
  const int scans_before = helper.scan_count;
  const double t0 = helper.now_s();
  size_t i = 0;
  std::vector<Pose> samples;
  while (helper.now_s() - t0 < 30.0) {
    const auto leg = loop_legs()[i++ % 4];
    helper.drive(leg[0], leg[1], leg[2]);
    const auto p_now = helper.correction();
    if (p_now) {
      samples.push_back(*p_now);
      const auto truth = truth_error(helper);
      const auto odom = helper.transform("odom", "root", 0.0);
      std::string detail =
          truth ? "  ground_truth_error=" + fixed(*truth) + " m" : "";
      if (odom && helper.truth) {
        detail += "  odom->root=(" + fixed((*odom)[0], 3) + ", " +
                  fixed((*odom)[1], 3) + ")  truth=(" +
                  fixed((*helper.truth)[0]) + ", " + fixed((*helper.truth)[1]) +
                  ")  true_yaw=" + fixed(helper.truth_yaw * 180 / kPi, 2) +
                  " deg";
      }
      sc.log("t=" + elapsed(helper.now_s() - t0) + "s  " + edge +
             " = (x=" + fixed((*p_now)[0]) + ", y=" + fixed((*p_now)[1]) +
             ", yaw=" + fixed((*p_now)[2]) + ")" + detail);
    }
  }
  sc.log("after odom_stuck, every message: " + helper.yaw_ranges());
  if (samples.size() < 3) {
    sc.result(false, "too few " + edge + " samples (" +
                         std::to_string(samples.size()) +
                         ") to assess liveness");
    return;
  }
  if (helper.scan_count <= scans_before) {
    sc.result(false, "scan count did not advance after odom went stuck -- "
                     "backend may have stalled");
    return;
  }
  double spread = 0;
  for (size_t j = 0; j < samples.size(); ++j) {
    for (size_t k = j + 1; k < samples.size(); ++k) {
      spread = std::max(spread, distance(xy(samples[j]), xy(samples[k])));
    }
  }
  const auto errors = log_errors(stack->log_text());
  sc.result(spread >= 0.01 && errors.empty(),
            "max pairwise " + edge +
                " spread over 30s after odom_stuck = " + fixed(spread) +
                " m (threshold 0.01 m -- proves the backend is still "
                "attempting corrections, not latched), log_errors=" +
                std::to_string(errors.size()));
}
void scan_degraded(Scenario &sc) {
  auto stack = run_stack(false, {rclcpp::Parameter("odom_slip_ratio", 0.15)});
  auto &helper = *stack->helper;
  if (!ready(sc, helper)) {
    return;
  }
  if (!helper.wait_correction(45.0)) {
    sc.result(false, "correction TF never became available within 45s");
    return;
  }
  std::string phase = "before";
  std::map<std::pair<std::string, std::string>, int> grades;
  auto quality =
      helper.create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
          "/scan_odom/quality", 50,
          [&](diagnostic_msgs::msg::DiagnosticArray::ConstSharedPtr msg) {
            for (const auto &status : msg->status) {
              std::string tier = "?";
              for (const auto &value : status.values) {
                if (value.key == "tier") {
                  tier = value.value;
                  break;
                }
              }
              ++grades[{phase, tier}];
            }
          });
  const auto sector = [&](double start, double end) {
    helper.set_params("/lidar_self_filter/set_parameters",
                      {rclcpp::Parameter("blind_angle_start", start),
                       rclcpp::Parameter("blind_angle_end", end)});
  };
  reposition(helper);
  std::vector<std::pair<std::string, double>> samples;
  const auto leg = [&](int i, const std::string &label) {
    const auto value = loop_legs()[static_cast<size_t>(i) % 4];
    helper.drive(value[0], value[1], value[2]);
    helper.spin_for(1.0);
    const auto err = truth_error(helper);
    if (err) {
      samples.emplace_back(label, *err);
      std::ostringstream out;
      out << std::left << std::setw(6) << label << " leg " << i
          << ": truth error " << fixed(*err) << " m";
      sc.log(out.str());
    }
  };
  bool blanked = false;
  try {
    for (int i = 0; i < 4; ++i) {
      leg(i, "before");
    }
    phase = "during";
    sector(0.5, 0.5 + 300.0 * kPi / 180.0);
    blanked = true;
    sc.log("blanked /scan outside a 60 deg arc");
    for (int i = 4; i < 6; ++i) {
      leg(i, "during");
    }
    sector(0.09, 1.41);
    blanked = false;
    phase = "after";
    for (int i = 6; i < 14; ++i) {
      leg(i, "after");
    }
  } catch (...) {
    if (blanked) {
      try {
        sector(0.09, 1.41);
      } catch (...) {
      }
    }
    throw;
  }
  std::string grade_text;
  for (const auto &item : grades) {
    grade_text += (grade_text.empty() ? "" : ", ") + item.first.first + "/" +
                  item.first.second + "=" + std::to_string(item.second);
  }
  sc.log("/scan_odom/quality grades: " + grade_text);
  if (samples.size() < 10) {
    sc.result(false, "too few truth-error samples (" +
                         std::to_string(samples.size()) + ")");
    return;
  }
  std::vector<std::pair<std::string, double>> worst;
  for (const auto &sample : samples) {
    auto found =
        std::find_if(worst.begin(), worst.end(), [&](const auto &value) {
          return value.first == sample.first;
        });
    if (found == worst.end()) {
      worst.push_back(sample);
    } else {
      found->second = std::max(found->second, sample.second);
    }
  }
  bool ok = true;
  std::string text;
  for (const auto &item : worst) {
    const double limit = item.first == "during" ? 0.5 : kMaxDelta;
    ok = ok && item.second < limit;
    text += (text.empty() ? "" : ", ") + item.first + " " + fixed(item.second) +
            " m (< " + fixed(limit, 1) + ")";
  }
  const auto errors = log_errors(stack->log_text());
  sc.result(ok && errors.empty(), "max truth error " + text + ", log_errors=" +
                                      std::to_string(errors.size()));
}
struct Stats {
  double mean, rms, maximum;
};
Stats stats(const std::vector<double> &samples) {
  const double mean =
      std::accumulate(samples.begin(), samples.end(), 0.0) / samples.size();
  double square = 0;
  for (double value : samples) {
    square += value * value;
  }
  return {mean, std::sqrt(square / samples.size()),
          *std::max_element(samples.begin(), samples.end())};
}
Point leg_error(const Point &start, const Point &end, const Point &truth_start,
                const Point &truth_end) {
  const double dx = end[0] - start[0], dy = end[1] - start[1],
               tx = truth_end[0] - truth_start[0],
               ty = truth_end[1] - truth_start[1], length = std::hypot(tx, ty);
  if (length < 1e-6 || std::hypot(dx, dy) < 1e-6) {
    return {NAN, NAN};
  }
  return {std::atan2(tx * dy - ty * dx, tx * dx + ty * dy) * 180 / kPi,
          std::hypot(dx, dy) / length};
}
class DriftSuite : public testing::TestWithParam<std::string> {};
TEST_P(DriftSuite, Scenario) {
  const auto name = GetParam();
  timing::Case timed("test_scenario[" + name + "]");
  std::cout << "\n=== Running scenario: " << name
            << " (backend=" << options.backend
            << ", use_rf2o=" << (options.rf2o ? "True" : "False")
            << ") ===" << std::endl;
  Scenario sc(name);
  try {
    if (name == "baseline") {
      baseline(sc);
    } else if (name == "noise_correction") {
      noise_correction(sc);
    } else if (name == "jerk_with_motion") {
      jerk_with_motion(sc);
    } else if (name == "odom_stuck") {
      odom_stuck(sc);
    } else if (name == "scan_degraded") {
      scan_degraded(sc);
    } else {
      cornering(sc,
                name == "drift_correction_obstacle" ? "box"
                : name == "moving_obstacles"        ? "actors"
                                                    : "",
                name == "real_accel" ? 1.2 : -1.0);
    }
  } catch (const std::exception &e) {
    sc.result(false, e.what());
  }
  if (sc.skipped) {
    GTEST_SKIP() << sc.details.back();
  }
  std::string detail;
  for (const auto &line : sc.details) {
    detail += line + '\n';
  }
  EXPECT_TRUE(sc.passed) << name << " (backend=" << options.backend
                         << ", use_rf2o=" << options.rf2o << ") failed:\n"
                         << detail;
}
INSTANTIATE_TEST_SUITE_P(Localization, DriftSuite,
                         testing::ValuesIn(scenario_names),
                         [](const testing::TestParamInfo<std::string> &info) {
                           return info.param;
                         });
TEST(EkfGroundTruth, BeatsRawOdometry) {
  timing::Case timed("test_ekf_beats_raw_odom");
  const bool saved_rf2o = options.rf2o;
  options.rf2o = true;
  auto stack = run_stack(true,
                         {rclcpp::Parameter("odom_drift_stddev", options.drift),
                          rclcpp::Parameter("odom_slip_ratio", options.slip)},
                         "none");
  options.rf2o = saved_rf2o;
  struct StackGuard {
    std::unique_ptr<Stack> &stack;
    ~StackGuard() {
      stack.reset();
      stop_sim();
    }
  } stack_guard{stack};
  auto &helper = *stack->helper;
  auto probe = std::make_shared<rclcpp::Node>(
      "ekf_ground_truth_probe", rclcpp::NodeOptions().parameter_overrides(
                                    {rclcpp::Parameter("use_sim_time", true)}));
  std::mutex mutex;
  std::optional<Point> truth, odom, scan;
  std::optional<double> head;
  const auto save_position = [&](std::optional<Point> &dest,
                                 nav_msgs::msg::Odometry::ConstSharedPtr msg) {
    std::lock_guard<std::mutex> lock(mutex);
    dest = Point{msg->pose.pose.position.x, msg->pose.pose.position.y};
  };
  const auto truth_sub = probe->create_subscription<nav_msgs::msg::Odometry>(
      "/sim/raw_odom", 10, [&](nav_msgs::msg::Odometry::ConstSharedPtr msg) {
        save_position(truth, msg);
      });
  const auto odom_sub = probe->create_subscription<nav_msgs::msg::Odometry>(
      "/odom", 10, [&](nav_msgs::msg::Odometry::ConstSharedPtr msg) {
        save_position(odom, msg);
      });
  const auto scan_sub = probe->create_subscription<nav_msgs::msg::Odometry>(
      "/scan_odom", 10, [&](nav_msgs::msg::Odometry::ConstSharedPtr msg) {
        save_position(scan, msg);
      });
  const auto joint_sub =
      probe->create_subscription<sensor_msgs::msg::JointState>(
          "/sim/raw_joint_states", 10,
          [&](sensor_msgs::msg::JointState::ConstSharedPtr msg) {
            const auto found =
                std::find(msg->name.begin(), msg->name.end(), "headlink");
            if (found != msg->name.end()) {
              std::lock_guard<std::mutex> lock(mutex);
              head = msg->position.at(
                  static_cast<size_t>(found - msg->name.begin()));
            }
          });
  tf2_ros::Buffer buffer(probe->get_clock());
  tf2_ros::TransformListener listener(buffer);
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(probe);
  std::thread thread([&] { executor.spin(); });
  struct SpinGuard {
    rclcpp::Executor &executor;
    std::thread &thread;
    ~SpinGuard() {
      executor.cancel();
      if (thread.joinable()) {
        thread.join();
      }
    }
  } guard{executor, thread};
  const auto sample = [&]() -> std::optional<std::array<Point, 3>> {
    std::optional<Point> truth_copy, odom_copy;
    {
      std::lock_guard<std::mutex> lock(mutex);
      truth_copy = truth;
      odom_copy = odom;
    }
    try {
      const auto tf = buffer.lookupTransform("odom", "root", tf2::TimePointZero,
                                             tf2::durationFromSec(0.5));
      if (truth_copy && odom_copy) {
        return std::array<Point, 3>{
            *truth_copy, *odom_copy,
            Point{tf.transform.translation.x, tf.transform.translation.y}};
      }
    } catch (const tf2::TransformException &) {
    }
    return std::nullopt;
  };
  const auto scan_now = [&] {
    std::lock_guard<std::mutex> lock(mutex);
    return scan;
  };
  Scenario sc("ekf_ground_truth");
  ASSERT_TRUE(ready(sc, helper))
      << "FAIL: stack never reached a healthy /scan rate";
  ASSERT_TRUE(helper.wait_correction(45.0))
      << "FAIL: odom->root never became available within 45s";
  const auto end = Clock::now() + std::chrono::seconds(15);
  while (Clock::now() < end && !sample()) {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
  const auto first = sample();
  ASSERT_TRUE(first) << "FAIL: never got a simultaneous truth/odom/ekf sample";
  std::cout << "initial truth=(" << (*first)[0][0] << ", " << (*first)[0][1]
            << ")  odom=(" << (*first)[1][0] << ", " << (*first)[1][1]
            << ")  ekf=(" << (*first)[2][0] << ", " << (*first)[2][1] << ")"
            << std::endl;
  reposition(helper);
  std::vector<double> odom_errors, ekf_errors;
  std::map<std::string, std::vector<double>> angles;
  const double t0 = helper.now_s();
  size_t i = 0;
  auto previous = sample();
  auto previous_scan = scan_now();
  while (helper.now_s() - t0 < options.seconds) {
    const auto leg = loop_legs()[i++ % 4];
    helper.drive(leg[0], leg[1], leg[2]);
    helper.spin_for(1.0);
    const auto current = sample();
    const auto current_scan = scan_now();
    if (!current) {
      previous.reset();
      previous_scan = current_scan;
      continue;
    }
    if (previous) {
      std::vector<std::tuple<std::string, Point, Point>> sources{
          {"odom", (*previous)[1], (*current)[1]},
          {"ekf", (*previous)[2], (*current)[2]}};
      if (previous_scan && current_scan) {
        sources.insert(sources.begin(),
                       {"scan_odom", *previous_scan, *current_scan});
      }
      std::cout << "  leg (" << std::showpos << std::fixed
                << std::setprecision(1) << leg[0] << ',' << leg[1]
                << std::noshowpos << ") vs truth: ";
      bool comma = false;
      for (const auto &source : sources) {
        const auto error = leg_error(std::get<1>(source), std::get<2>(source),
                                     (*previous)[0], (*current)[0]);
        angles[std::get<0>(source)].push_back(error[0]);
        std::cout << (comma ? ", " : "") << std::get<0>(source) << ' '
                  << std::showpos << std::setw(6) << std::setprecision(1)
                  << error[0] << std::noshowpos << "deg x"
                  << fixed(error[1], 2);
        comma = true;
      }
      {
        std::lock_guard<std::mutex> lock(mutex);
        if (head) {
          std::cout << "  head_yaw=" << std::showpos << std::setprecision(2)
                    << *head << std::noshowpos;
        }
      }
      std::cout << std::endl;
    }
    previous = current;
    previous_scan = current_scan;
    const double e_odom = distance((*current)[1], (*current)[0]),
                 e_ekf = distance((*current)[2], (*current)[0]);
    odom_errors.push_back(e_odom);
    ekf_errors.push_back(e_ekf);
    std::cout << "t=" << elapsed(helper.now_s() - t0) << "s  truth=("
              << fixed((*current)[0][0], 3) << ',' << fixed((*current)[0][1], 3)
              << ")  odom=(" << fixed((*current)[1][0], 3) << ','
              << fixed((*current)[1][1], 3) << ")  ekf=("
              << fixed((*current)[2][0], 3) << ',' << fixed((*current)[2][1], 3)
              << ")  err_odom=" << fixed(e_odom) << "  err_ekf=" << fixed(e_ekf)
              << std::endl;
  }
  ASSERT_GE(odom_errors.size(), 3U)
      << "FAIL: too few samples (" << odom_errors.size() << ")";
  for (const auto &item : angles) {
    std::vector<double> finite;
    for (const auto value : item.second) {
      if (!std::isnan(value)) {
        finite.push_back(std::abs(value));
      }
    }
    if (!finite.empty()) {
      const auto data = stats(finite);
      std::cout << item.first << " leg direction error: mean "
                << fixed(data.mean, 1) << " deg, max " << fixed(data.maximum, 1)
                << " deg over " << finite.size() << " legs" << std::endl;
    }
  }
  const auto raw = stats(odom_errors), fused = stats(ekf_errors);
  const double improvement =
      raw.mean > 0 ? (raw.mean - fused.mean) / raw.mean * 100 : NAN;
  std::cout << "\n=== " << odom_errors.size()
            << " samples, slip_ratio=" << options.slip
            << ", drift_stddev=" << options.drift
            << " ===\nraw /odom   vs truth: mean=" << fixed(raw.mean)
            << " m  rms=" << fixed(raw.rms) << "  max=" << fixed(raw.maximum)
            << "\nekf fused   vs truth: mean=" << fixed(fused.mean)
            << " m  rms=" << fixed(fused.rms)
            << "  max=" << fixed(fused.maximum)
            << "\nEKF improvement over raw /odom: " << std::showpos
            << std::fixed << std::setprecision(1) << improvement
            << std::noshowpos << '%' << std::endl;
  EXPECT_GT(improvement, 0.0)
      << "EKF did not beat raw /odom over " << odom_errors.size()
      << " samples: mean error " << fixed(fused.mean) << " m fused vs "
      << fixed(raw.mean) << " m raw (" << improvement << "%)";
}
void parse_options(int &argc, char **argv) {
  int write = 1;
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    const auto value = [&] {
      if (i + 1 >= argc) {
        throw std::invalid_argument("missing value for " + arg);
      }
      return std::string(argv[++i]);
    };
    if (arg == "--suite") {
      options.suite = value();
    } else if (arg == "--backend") {
      options.backend = value();
    } else if (arg == "--scenario") {
      options.scenario = value();
    } else if (arg == "--real-time-factor") {
      options.rtf = value();
    } else if (arg == "--headless") {
      options.gui = false;
    } else if (arg == "--restart-sim") {
      options.restart = true;
    } else if (arg == "--use-rf2o") {
      options.rf2o = true;
    } else if (arg == "--no-use-rf2o") {
      options.rf2o = false;
    } else if (arg == "--speed") {
      options.speed = std::stod(value());
    } else if (arg == "--drive-accel") {
      options.accel = std::stod(value());
    } else if (arg == "--spawn-yaw-deg") {
      options.spawn_yaw =
          sim::gazebo::kSpawnYaw + std::stod(value()) * kPi / 180;
    } else if (arg == "--ekf-slip-ratio") {
      options.slip = std::stod(value());
    } else if (arg == "--ekf-drift-stddev") {
      options.drift = std::stod(value());
    } else if (arg == "--ekf-seconds") {
      options.seconds = std::stod(value());
    } else if (arg == "--run-on-demand") {
    } else {
      argv[write++] = argv[i];
    }
  }
  argc = write;
  if (!options.scenario.empty()) {
    if (std::find(scenario_names.begin(), scenario_names.end(),
                  options.scenario) == scenario_names.end()) {
      throw std::invalid_argument("unknown --scenario: " + options.scenario);
    }
    scenario_names = {options.scenario};
  }
  if (options.suite != "drift" && options.suite != "ekf") {
    throw std::invalid_argument("unknown --suite: " + options.suite);
  }
  if (options.backend != "slam" && options.backend != "mapping" &&
      options.backend != "amcl" && options.backend != "none") {
    throw std::invalid_argument("unknown --backend: " + options.backend);
  }
}
} // namespace
int main(int argc, char **argv) {
  try {
    parse_options(argc, argv);
  } catch (const std::exception &e) {
    std::cerr << e.what() << std::endl;
    return 2;
  }
  testing::InitGoogleTest(&argc, argv);
  if (testing::GTEST_FLAG(filter) == "*") {
    testing::GTEST_FLAG(filter) =
        options.suite == "ekf" ? "EkfGroundTruth.*" : "Localization/*";
  }
  const bool list_only = testing::GTEST_FLAG(list_tests);
  if (list_only) {
    return RUN_ALL_TESTS();
  }
  if (options.gui) {
    const auto error = sim::display_error();
    if (!error.empty()) {
      options.gui = false;
      std::cout << error
                << ": no gz or rviz2 windows. Watch in Foxglove on port 8765 "
                   "(foxglove.launch.py)."
                << std::endl;
    }
  }
  rclcpp::init(argc, argv);
  const auto start = Clock::now();
  int result = 1;
  {
    timing::Suite suite(options.suite == "ekf" ? "test_ekf_ground_truth"
                                               : "test_localization_drift");
    sim::check_no_orphans("pre-flight");
    result = RUN_ALL_TESTS();
    stop_sim();
    sim::check_no_orphans("post-flight (should be empty if teardown worked)");
  }
  std::cout << "============================= suite timing "
               "============================="
            << std::endl;
  for (const auto &line : timing::report()) {
    std::cout << line << std::endl;
  }
  const auto *unit = testing::UnitTest::GetInstance();
  std::cout << "=== " << unit->successful_test_count() << " passed";
  if (unit->failed_test_count()) {
    std::cout << ", " << unit->failed_test_count() << " failed";
  }
  if (unit->skipped_test_count()) {
    std::cout << ", " << unit->skipped_test_count() << " skipped";
  }
  std::cout << " in "
            << fixed(
                   std::chrono::duration<double>(Clock::now() - start).count(),
                   2)
            << "s ===" << std::endl;
  timing::set_sim_clock({});
  rclcpp::shutdown();
  return result;
}
