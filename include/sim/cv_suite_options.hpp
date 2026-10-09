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

#ifndef SIM__CV_SUITE_OPTIONS_HPP_
#define SIM__CV_SUITE_OPTIONS_HPP_
#include "sim/cv_bench.hpp"
#include <cctype>
#include <filesystem>
#include <fstream>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <set>
namespace sim::cv_bench {
struct Options {
  std::vector<double> speeds{.5, 1, 2, 4};
  std::vector<std::string> paths{"lateral", "radial", "diagonal"};
  std::string layout = "both", path = "lateral", log_dir, stage = "mcb_parked",
              rtf = "0", filter;
  double duration = 0, shooter = 0, chassis = 0, latency = 0;
  std::optional<double> spin;
  bool headless = false, external = false, skip_stationary = false,
       only_stationary = false, blackout = false, no_firmware_fixes = false,
       fail_fast = false;
};
inline Options options;
inline std::vector<std::string> split(const std::string &s) {
  std::vector<std::string> out;
  std::istringstream in(s);
  std::string p;
  while (std::getline(in, p, ','))
    if (!p.empty())
      out.push_back(p);
  return out;
}
inline void parse_options(int &argc, char **argv) {
  int keep = 1;
  for (int i = 1; i < argc; ++i) {
    std::string arg = argv[i];
    auto value = [&]() {
      if (i + 1 >= argc)
        throw std::runtime_error("missing value for " + arg);
      return std::string(argv[++i]);
    };
    if (arg == "--headless")
      options.headless = true;
    else if (arg == "--external-stack")
      options.external = true;
    else if (arg == "--skip-stationary")
      options.skip_stationary = true;
    else if (arg == "--only-stationary")
      options.only_stationary = true;
    else if (arg == "--blackout")
      options.blackout = true;
    else if (arg == "--no-firmware-fixes")
      options.no_firmware_fixes = true;
    else if (arg == "--shot-speeds" || arg == "--e2e-speeds") {
      options.speeds.clear();
      for (const auto &s : split(value()))
        options.speeds.push_back(std::stod(s));
    } else if (arg == "--e2e-paths")
      options.paths = split(value());
    else if (arg == "--e2e-spin")
      options.spin = std::stod(value());
    else if (arg == "--shot-duration" || arg == "--e2e-duration")
      options.duration = std::stod(value());
    else if (arg == "--panel-layout")
      options.layout = value();
    else if (arg == "--target-path")
      options.path = value();
    else if (arg == "--shooter-speed")
      options.shooter = std::stod(value());
    else if (arg == "--chassis-spin")
      options.chassis = std::stod(value());
    else if (arg == "--camera-latency")
      options.latency = std::stod(value());
    else if (arg == "--stage")
      options.stage = value();
    else if (arg == "--real-time-factor")
      options.rtf = value();
    else if (arg == "--log-dir")
      options.log_dir = value();
    else if (arg == "-x")
      options.fail_fast = true;
    else if (arg == "-k")
      options.filter = value();
    else if (arg == "-s" || arg == "-v") {
    } else
      argv[keep++] = argv[i];
  }
  argc = keep;
  if (options.log_dir.empty())
    options.log_dir =
        "/tmp/" +
        std::string(options.stage == "shot_hit"     ? "shot_hit"
                    : options.stage == "estimation" ? "estimation"
                    : options.stage == "mcb_parked" ? "e2e"
                                                    : options.stage) +
        "_test_logs";
  std::filesystem::create_directories(options.log_dir);
}
inline void append_json(const std::string &path, const Json &record) {
  std::ofstream(path, std::ios::app) << record.dump() << '\n';
}
inline void truncate(const std::string &path) {
  std::ofstream(path, std::ios::trunc);
}
inline std::vector<rclcpp::Parameter> path_params(const std::string &path) {
  if (path == "lateral")
    return {rclcpp::Parameter("path_angle_deg", 0.),
            rclcpp::Parameter("center_x", 3.),
            rclcpp::Parameter("half_width", 2.4)};
  if (path == "radial")
    return {rclcpp::Parameter("path_angle_deg", 90.),
            rclcpp::Parameter("center_x", 3.5),
            rclcpp::Parameter("half_width", 2.)};
  if (path == "diagonal")
    return {rclcpp::Parameter("path_angle_deg", 45.),
            rclcpp::Parameter("center_x", 3.),
            rclcpp::Parameter("half_width", 2.)};
  throw std::runtime_error("unknown target path " + path);
}
struct Cell {
  std::string layout, name;
  double speed = 0, yaw = 0, spin = 0, stagger = 0;
};
inline std::vector<Cell> cells(bool estimation) {
  std::vector<Cell> out;
  auto layouts = options.layout == "both"
                     ? std::vector<std::string>{"flat", "staggered"}
                     : std::vector<std::string>{options.layout};
  for (const auto &layout : layouts) {
    auto add = [&](double speed, double yaw) {
      Cell c;
      c.layout = layout;
      c.speed = speed;
      c.yaw = yaw;
      c.stagger = layout == "flat" ? 0 : kStagger;
      c.spin = speed == 0 ? 0
                          : spin_hz(speed,
                                    *std::min_element(options.speeds.begin(),
                                                      options.speeds.end()),
                                    *std::max_element(options.speeds.begin(),
                                                      options.speeds.end()));
      c.name = cell_id(layout, speed, options.path, options.shooter,
                       estimation && options.blackout,
                       estimation ? options.latency : 0, yaw,
                       estimation ? options.chassis : 0);
      if (options.filter.empty() ||
          c.name.find(options.filter) != std::string::npos)
        out.push_back(c);
    };
    if (!options.skip_stationary) {
      add(0, 0);
      if (estimation)
        add(0, 45);
    }
    if (!options.only_stationary)
      for (double s : options.speeds)
        add(s, 0);
  }
  return out;
}
inline std::string gtest_name(const std::string &name) {
  std::string out;
  for (char c : name)
    out += std::isalnum(static_cast<unsigned char>(c)) ? c : '_';
  return out;
}
} // namespace sim::cv_bench
#endif
