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

#include "sim/match_scenario.hpp"
#include <algorithm>
#include <cmath>
#include <optional>
#include <stdexcept>
namespace sim::match_scenario {
namespace {
constexpr double kPi = 3.14159265358979323846;
const Point center{1.55, -0.65};
struct CenterLeg {
  std::string name;
  Route points;
  double speed;
  std::optional<double> duration;
};
const std::vector<CenterLeg> center_legs = {
    {"parked", {center}, 1.0, 5.0},
    {"straight_1", {center, {1.55, -0.05}, center}, 1.0, std::nullopt},
    {"turn",
     {center, {0.65, -0.65}, {0.65, -0.05}, {1.55, -0.05}, center},
     1.0,
     std::nullopt},
    {"spin", {center}, 1.0, 6.0}};
double duration(const CenterLeg &leg) {
  return leg.duration.value_or(route_duration(leg.points, leg.speed));
}
} // namespace
const std::map<std::string, ParkedPath> &parked_paths() {
  static const std::map<std::string, ParkedPath> data = {
      {"lateral", {0.0, 1.3, 1.9}},
      {"radial", {90.0, 1.3, 0.4}},
      {"diagonal", {45.0, 1.3, 0.65}}};
  return data;
}
const std::map<std::string, Route> &routes() {
  static const std::map<std::string, Route> data = {
      {"sentry", {{4.625, 0.0}, {4.625, -2.35}, {1.55, -2.35}, {1.55, -0.65}}},
      {"opponent_0",
       {{-4.625, 0.0}, {-4.625, -2.35}, {-1.55, -2.35}, {-1.55, -0.65}}},
      {"ally_0",
       {{5.0, 0.9},
        {5.0, -2.9},
        {2.15, -2.9},
        {2.15, -1.5},
        {0.65, -1.5},
        {0.65, 1.05}}},
      {"opponent_1",
       {{-5.0, 0.9},
        {-5.0, -2.9},
        {-2.15, -2.9},
        {-2.15, -1.5},
        {-0.65, -1.5},
        {-0.65, 1.05}}}};
  return data;
}
const std::map<std::string, std::string> &teams() {
  static const std::map<std::string, std::string> data = {
      {"sentry", "blue"},
      {"ally_0", "blue"},
      {"opponent_0", "red"},
      {"opponent_1", "red"}};
  return data;
}
const std::map<std::string, double> &route_delays() {
  static const std::map<std::string, double> data = {{"sentry", 0.0},
                                                     {"opponent_0", 0.0},
                                                     {"ally_0", 4.0},
                                                     {"opponent_1", 4.0}};
  return data;
}
double leg_duration(double length, double speed, double accel) {
  const double peak = std::min(speed, std::sqrt(length * accel));
  return length != 0.0 ? length / peak + peak / accel : 0.0;
}
double route_duration(const Route &points, double speed) {
  double result = 0.0;
  for (size_t i = 1; i < points.size(); ++i) {
    result += leg_duration(std::hypot(points[i][0] - points[i - 1][0],
                                      points[i][1] - points[i - 1][1]),
                           speed);
  }
  return result;
}
RouteSample sample_route(const Route &points, double seconds, double speed) {
  seconds = std::max(seconds, 0.0);
  for (size_t i = 1; i < points.size(); ++i) {
    const auto &a = points[i - 1];
    const auto &b = points[i];
    const double length = std::hypot(b[0] - a[0], b[1] - a[1]);
    const double span = leg_duration(length, speed);
    if (seconds > span) {
      seconds -= span;
      continue;
    }
    const double peak = std::min(speed, std::sqrt(length * kRouteAccel));
    const double ramp = peak / kRouteAccel;
    double distance, velocity;
    if (seconds < ramp) {
      distance = kRouteAccel * seconds * seconds / 2;
      velocity = kRouteAccel * seconds;
    } else if (seconds <= span - ramp) {
      distance = peak * (seconds - ramp / 2);
      velocity = peak;
    } else {
      const double remaining = span - seconds;
      distance = length - kRouteAccel * remaining * remaining / 2;
      velocity = kRouteAccel * remaining;
    }
    const Point direction{(b[0] - a[0]) / length, (b[1] - a[1]) / length};
    return {{a[0] + direction[0] * distance, a[1] + direction[1] * distance},
            {direction[0] * velocity, direction[1] * velocity},
            false};
  }
  return {points.back(), {0.0, 0.0}, true};
}
double ingress_duration(const std::string &stage) {
  if (stage != "mcb_match") {
    return route_duration(routes().at("sentry"));
  }
  double result = 0.0;
  for (const auto &item : routes()) {
    result = std::max(result, route_duration(item.second) +
                                  route_delays().at(item.first));
  }
  return result;
}
double match_duration(const std::string &stage) {
  double result = ingress_duration(stage);
  for (const auto &leg : center_legs) {
    result += duration(leg);
  }
  return result;
}
MatchSample sample_match(double seconds, const std::string &stage) {
  const double approach = ingress_duration(stage);
  if (seconds < approach) {
    const auto sample = sample_route(routes().at("sentry"), seconds);
    return {sample.position, sample.velocity, 0.0, "approach_2"};
  }
  seconds -= approach;
  for (const auto &leg : center_legs) {
    const double span = duration(leg);
    if (seconds < span) {
      const auto sample = sample_route(leg.points, seconds, leg.speed);
      return {sample.position, sample.velocity, leg.name == "spin" ? 9.0 : 0.0,
              leg.name};
    }
    seconds -= span;
  }
  return {center, {0.0, 0.0}, 0.0, "finished"};
}
RobotSample sample_robot(const std::string &name, double seconds,
                         const std::string &stage) {
  const double delay = stage == "mcb_match" ? route_delays().at(name) : 0.0;
  auto sample = sample_route(routes().at(name), std::max(0.0, seconds - delay));
  double yaw = teams().at(name) == "blue" ? kPi : 0.0, spin = 0.0;
  const double fight = std::max(0.0, seconds - ingress_duration(stage));
  if (sample.finished && fight > 0.0) {
    const double omega = 3.0 * kPi, accel = 20.0, ramp = omega / accel;
    yaw +=
        fight < ramp ? accel * fight * fight / 2 : omega * (fight - ramp / 2);
    spin = std::min(accel * fight, omega);
    if (teams().at(name) == "red") {
      const double amplitude = name == "opponent_0" ? 0.3 : 0.2,
                   frequency = name == "opponent_0" ? 2.0 : 1.5;
      sample.position[1] += amplitude * (1 - std::cos(frequency * fight));
      sample.velocity = {0.0,
                         amplitude * frequency * std::sin(frequency * fight)};
    }
  }
  return {sample.position, sample.velocity, yaw, spin};
}
} // namespace sim::match_scenario
