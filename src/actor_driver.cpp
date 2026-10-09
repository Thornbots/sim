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

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <iomanip>
#include <limits>
#include <memory>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "gz/msgs/boolean.pb.h"
#include "gz/msgs/entity_factory.pb.h"
#include "gz/transport/Node.hh"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "ros_gz_interfaces/msg/entity.hpp"
#include "ros_gz_interfaces/srv/set_entity_pose.hpp"

namespace
{
using Point = std::pair<double, double>;
using Segment = std::pair<Point, Point>;
constexpr double kStep = 0.05;
constexpr double kOnRoute = 0.5;
constexpr const char * kWorld = "/world/ARCC_Field_2026/";

double modulo(double value, double divisor)
{
  if (divisor == 0.0) {
    throw std::runtime_error("float modulo by zero");
  }
  double remainder = std::fmod(value, divisor);
  if (remainder != 0.0) {
    if ((divisor < 0.0) != (remainder < 0.0)) {remainder += divisor;}
  } else {
    remainder = std::copysign(0.0, divisor);
  }
  return remainder;
}

std::pair<double, double> project(Point p, Point a, Point b)
{
  const double dx = b.first - a.first;
  const double dy = b.second - a.second;
  const double len2 = dx * dx + dy * dy;
  const double t = len2 < 1e-12 ? 0.0 : std::clamp(
    ((p.first - a.first) * dx + (p.second - a.second) * dy) / len2, 0.0, 1.0);
  return {std::hypot(p.first - (a.first + t * dx), p.second - (a.second + t * dy)), t};
}

struct Route
{
  explicit Route(const std::vector<double> & flat)
  {
    std::vector<Point> points;
    for (std::size_t i = 0; i + 1 < flat.size(); i += 2) {
      points.emplace_back(flat[i], flat[i + 1]);
    }
    for (std::size_t i = 0; i < points.size(); ++i) {
      segments.emplace_back(points[i], points[(i + 1) % points.size()]);
      lengths.push_back(std::hypot(
        segments.back().second.first - segments.back().first.first,
        segments.back().second.second - segments.back().first.second));
      total += lengths.back();
    }
  }

  std::pair<double, double> locate(Point p) const
  {
    std::pair<double, double> best{std::numeric_limits<double>::infinity(), 0.0};
    double s0 = 0.0;
    for (std::size_t i = 0; i < segments.size(); ++i) {
      const auto [d, t] = project(p, segments[i].first, segments[i].second);
      if (d < best.first) {best = {d, s0 + t * lengths[i]};}
      s0 += lengths[i];
    }
    return best;
  }

  Point point(double s) const
  {
    s = modulo(s, total);
    for (std::size_t i = 0; i < segments.size(); ++i) {
      const double n = lengths[i];
      if (s <= n) {
        const double f = n > 0.0 ? s / n : 0.0;
        const auto & [a, b] = segments[i];
        return {a.first + (b.first - a.first) * f, a.second + (b.second - a.second) * f};
      }
      s -= n;
    }
    return segments.back().second;
  }

  std::vector<Segment> arc(double s, double length) const
  {
    if (total == 0.0) {throw std::runtime_error("float division by zero");}
    std::vector<double> cuts{s};
    double c = total * std::floor(s / total);
    while (c < s + length) {
      for (double n : lengths) {
        c += n;
        if (s < c && c < s + length) {cuts.push_back(c);}
      }
    }
    cuts.push_back(s + length);
    std::vector<Segment> result;
    for (std::size_t i = 0; i + 1 < cuts.size(); ++i) {
      result.emplace_back(point(cuts[i]), point(cuts[i + 1]));
    }
    return result;
  }

  std::vector<Segment> segments;
  std::vector<double> lengths;
  double total{0.0};
};

struct Actor
{
  Actor(std::string actor_name, double x0, double y0, double x1, double y1, double actor_speed)
  : name(std::move(actor_name)), a(x0, y0), b(x1, y1),
    length(std::hypot(x1 - x0, y1 - y0)), speed(actor_speed) {}

  Point position(double progress) const
  {
    progress = modulo(progress, 2.0 * length);
    const double d = progress < length ? progress : 2.0 * length - progress;
    const double f = length > 0.0 ? d / length : 0.0;
    return {a.first + (b.first - a.first) * f, a.second + (b.second - a.second) * f};
  }

  std::string name;
  Point a, b;
  double length, speed, u{0.0};
  bool spawned{false};
};

std::string box_sdf(const Actor & actor, Point p, double size, double height, double mass)
{
  const double i_xy = mass * (size * size + height * height) / 12.0;
  const double i_z = mass * size * size / 6.0;
  std::ostringstream xml;
  xml << std::setprecision(17) << "<sdf version=\"1.6\"><model name=\"" << actor.name << "\">"
      << "<pose>" << p.first << ' ' << p.second << ' ' << height / 2.0
      << " 0 0 0</pose><link name=\"link\"><gravity>false</gravity>"
      << "<inertial><mass>" << mass << "</mass><inertia><ixx>" << i_xy
      << "</ixx><iyy>" << i_xy << "</iyy><izz>" << i_z
      << "</izz><ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia></inertial>"
      << "<visual name=\"visual\"><geometry><box><size>" << size << ' ' << size << ' ' << height
      << "</size></box></geometry><material><ambient>0.8 0.4 0.1 1</ambient>"
      << "<diffuse>0.8 0.4 0.1 1</diffuse></material></visual></link></model></sdf>";
  return xml.str();
}

class ActorDriver : public rclcpp::Node
{
public:
  ActorDriver() : Node("actor_driver")
  {
    const auto count = declare_parameter<int64_t>("count", 3);
    const auto speeds = declare_parameter<std::vector<double>>("speeds", {1.0, 2.0, 0.5});
    const auto paths = declare_parameter<std::vector<double>>("paths",
      {-0.2, -0.4, -2.1, -0.4, -0.4, 0.2, -0.4, 2.3, 0.2, 0.4, 2.1, 0.4});
    const double rate = declare_parameter("rate_hz", 10.0);
    size_ = declare_parameter("size", 0.3);
    height_ = declare_parameter("height", 0.8);
    mass_ = declare_parameter("mass", 20.0);
    const double radius = declare_parameter("robot_radius", 0.5);
    const double margin = declare_parameter("margin", 0.3);
    lookahead_ = declare_parameter("lookahead_s", 1.0);
    const auto route = declare_parameter<std::vector<double>>("route",
      {-1.5, 1.5, -1.5, -1.5, 1.5, -1.5, 1.5, 1.5});
    if (route.size() >= 4) {route_.emplace(route);}
    route_speed_ = declare_parameter("route_speed", 4.0);
    const auto prefix = declare_parameter<std::string>("name_prefix", "moving_actor");
    if (static_cast<int64_t>(paths.size()) < 4 * count ||
      static_cast<int64_t>(speeds.size()) < count)
    {
      throw std::invalid_argument("count=" + std::to_string(count) + " needs " +
        std::to_string(4 * count) + " path values and " + std::to_string(count) +
        " speeds; got " + std::to_string(paths.size()) + ", " + std::to_string(speeds.size()));
    }
    for (int64_t i = 0; i < count; ++i) {
      actors_.emplace_back(prefix + "_" + std::to_string(i), paths[4 * i], paths[4 * i + 1],
        paths[4 * i + 2], paths[4 * i + 3], speeds[i]);
    }
    keep_out_ = radius + size_ / std::sqrt(2.0) + margin;
    set_pose_ = create_client<ros_gz_interfaces::srv::SetEntityPose>(
      std::string(kWorld) + "set_pose");
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>("/sim/raw_odom", 10,
      [this](nav_msgs::msg::Odometry::ConstSharedPtr msg) {on_odom(*msg);});
    timer_ = create_timer(std::chrono::duration<double>(1.0 / rate), [this]() {tick();});
  }

  void log_gap() const
  {
    RCLCPP_INFO(get_logger(), "closest box to the robot: %.2f m", min_gap_);
  }

private:
  void on_odom(const nav_msgs::msg::Odometry & msg)
  {
    const auto & q = msg.pose.pose.orientation;
    const double yaw = std::atan2(2.0 * (q.w * q.z + q.x * q.y),
      1.0 - 2.0 * (q.y * q.y + q.z * q.z));
    const auto & v = msg.twist.twist.linear;
    robot_ = std::array<double, 5>{msg.pose.pose.position.x, msg.pose.pose.position.y,
      v.x * std::cos(yaw) - v.y * std::sin(yaw),
      v.x * std::sin(yaw) + v.y * std::cos(yaw), now().seconds()};
  }

  std::vector<Segment> danger(double lookahead) const
  {
    const auto & r = *robot_;
    std::vector<Segment> result{{{r[0], r[1]}, {r[0] + r[2] * lookahead, r[1] + r[3] * lookahead}}};
    if (route_) {
      const auto [d, s] = route_->locate({r[0], r[1]});
      if (d <= kOnRoute) {
        const auto ahead = route_->arc(
          s, std::max(route_speed_, std::hypot(r[2], r[3])) * lookahead);
        result.insert(result.end(), ahead.begin(), ahead.end());
      }
    }
    return result;
  }

  bool clear(Point xy, const std::vector<Segment> & danger_segments) const
  {
    return std::all_of(danger_segments.begin(), danger_segments.end(),
      [this, xy](const Segment & s) {return project(xy, s.first, s.second).first >= keep_out_;});
  }

  std::optional<double> nearest_clear(
    const Actor & actor, double u, const std::vector<Segment> & danger_segments) const
  {
    const int end = static_cast<int>(2.0 * actor.length / kStep) + 1;
    for (int k = 0; k < end; ++k) {
      for (double candidate : {u + k * kStep, u - k * kStep}) {
        if (clear(actor.position(candidate), danger_segments)) {
          return modulo(candidate, 2.0 * actor.length);
        }
      }
    }
    return std::nullopt;
  }

  bool spawn()
  {
    const auto swept = danger(lookahead_);
    for (auto & actor : actors_) {
      if (actor.spawned) {continue;}
      const auto progress = nearest_clear(actor, 0.0, swept);
      if (!progress) {return false;}
      actor.u = *progress;
      const auto xy = actor.position(actor.u);
      gz::msgs::EntityFactory request;
      request.set_sdf(box_sdf(actor, xy, size_, height_, mass_));
      request.set_name(actor.name);
      request.set_allow_renaming(false);
      request.mutable_pose()->mutable_position()->set_x(xy.first);
      request.mutable_pose()->mutable_position()->set_y(xy.second);
      request.mutable_pose()->mutable_position()->set_z(height_ / 2.0);
      if (!gz_node_) {
        if (!std::getenv("GZ_IP")) {setenv("GZ_IP", "127.0.0.1", 0);}
        gz_node_ = std::make_unique<gz::transport::Node>();
      }
      gz::msgs::Boolean reply;
      bool result = false;
      if (!gz_node_->Request(std::string(kWorld) + "create", request, 2000u, reply, result) ||
        !result || !reply.data())
      {
        RCLCPP_ERROR(get_logger(), "spawning %s failed", actor.name.c_str());
        throw std::runtime_error("spawning " + actor.name + " failed");
      }
      actor.spawned = true;
      RCLCPP_INFO(get_logger(), "spawned %s at (%.2f, %.2f)",
        actor.name.c_str(), xy.first, xy.second);
    }
    return true;
  }

  void tick()
  {
    const double current = now().seconds();
    if (!robot_ || current - (*robot_)[4] > 1.0) {return;}
    if (!set_pose_->service_is_ready()) {
      RCLCPP_WARN_THROTTLE(get_logger(), throttle_clock_, 5000,
        "waiting for set_pose_bridge (sim.launch.py)");
      return;
    }
    if (!spawned_) {
      spawned_ = spawn();
      if (spawned_) {
        RCLCPP_INFO(get_logger(), "all %zu actors spawned", actors_.size());
        last_tick_ = now().seconds();
      }
      return;
    }
    const double dt = std::max(0.0, current - last_tick_);
    last_tick_ = current;
    ++ticks_;
    tick_dt_sum_ += dt;
    if (ticks_ % 100 == 0) {
      RCLCPP_INFO(get_logger(), "%zu ticks, mean %.3f s sim per tick, closest box %.2f m",
        ticks_, tick_dt_sum_ / ticks_, min_gap_);
    }
    const auto swept = danger(std::max(lookahead_, 3.0 * dt));
    std::vector<std::pair<std::string, Point>> moves;
    for (auto & actor : actors_) {
      const auto progress = nearest_clear(actor, actor.u + actor.speed * dt, swept);
      if (!progress) {continue;}
      actor.u = *progress;
      moves.emplace_back(actor.name, actor.position(actor.u));
    }
    for (const auto & actor : actors_) {
      const auto xy = actor.position(actor.u);
      min_gap_ = std::min(min_gap_, std::hypot(xy.first - (*robot_)[0], xy.second - (*robot_)[1]));
    }
    for (const auto & [name, xy] : moves) {
      if (pending_[name]) {continue;}
      auto request = std::make_shared<ros_gz_interfaces::srv::SetEntityPose::Request>();
      request->entity.name = name;
      request->entity.type = ros_gz_interfaces::msg::Entity::MODEL;
      request->pose.position.x = xy.first;
      request->pose.position.y = xy.second;
      request->pose.position.z = height_ / 2.0;
      request->pose.orientation.w = 1.0;
      pending_[name] = true;
      set_pose_->async_send_request(request,
        [this, name](rclcpp::Client<ros_gz_interfaces::srv::SetEntityPose>::SharedFuture future) {
          pending_[name] = false;
          bool success = false;
          try {success = future.get()->success;} catch (const std::exception &) {}
          if (!success) {
            RCLCPP_WARN(get_logger(), "set_pose %s failed (%zu so far)", name.c_str(), ++failures_);
          }
        });
    }
  }

  double size_, height_, mass_, lookahead_, route_speed_, keep_out_, last_tick_{0.0};
  double tick_dt_sum_{0.0}, min_gap_{std::numeric_limits<double>::infinity()};
  bool spawned_{false};
  std::size_t ticks_{0}, failures_{0};
  std::optional<Route> route_;
  std::optional<std::array<double, 5>> robot_;
  std::vector<Actor> actors_;
  std::unordered_map<std::string, bool> pending_;
  std::unique_ptr<gz::transport::Node> gz_node_;
  rclcpp::Client<ros_gz_interfaces::srv::SetEntityPose>::SharedPtr set_pose_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Clock throttle_clock_{RCL_STEADY_TIME};
};
}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ActorDriver>();
  try {
    rclcpp::spin(node);
  } catch (...) {
    node->log_gap();
    node.reset();
    rclcpp::shutdown();
    throw;
  }
  node->log_gap();
  node.reset();
  rclcpp::shutdown();
  return 0;
}
