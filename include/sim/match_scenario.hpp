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

#ifndef SIM__MATCH_SCENARIO_HPP_
#define SIM__MATCH_SCENARIO_HPP_
#include <array>
#include <map>
#include <string>
#include <vector>
namespace sim::match_scenario {
using Point = std::array<double, 2>;
using Route = std::vector<Point>;
struct ParkedPath {
  double path_angle_deg, center_x, half_width;
};
const std::map<std::string, ParkedPath> &parked_paths();
const std::map<std::string, Route> &routes();
const std::map<std::string, std::string> &teams();
const std::map<std::string, double> &route_delays();
constexpr double kRouteAccel = 2.0, kRouteSpeed = 2.0;
double leg_duration(double length, double speed = kRouteSpeed,
                    double accel = kRouteAccel);
double route_duration(const Route &points, double speed = kRouteSpeed);
struct RouteSample {
  Point position, velocity;
  bool finished;
};
RouteSample sample_route(const Route &points, double seconds,
                         double speed = kRouteSpeed);
double ingress_duration(const std::string &stage = "mcb_drive");
double match_duration(const std::string &stage = "mcb_drive");
struct MatchSample {
  Point position, velocity;
  double spin;
  std::string segment;
};
MatchSample sample_match(double seconds,
                         const std::string &stage = "mcb_drive");
struct RobotSample {
  Point position, velocity;
  double yaw, spin;
};
RobotSample sample_robot(const std::string &name, double seconds,
                         const std::string &stage = "mcb_match");
} // namespace sim::match_scenario
#endif // SIM__MATCH_SCENARIO_HPP_
