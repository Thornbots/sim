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

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <rclcpp/rclcpp.hpp>
#include <iomanip>
#include <iostream>
#include <map>
#include "sim/tool_io.hpp"

namespace {
void summary(const std::string &path) {
  const auto rows = sim::tools::json_lines(path);
  std::cout << path << ": " << rows.size() << " matches\n";
  if (rows.empty()) return;
  std::map<std::string, int> tiers;
  std::vector<std::pair<std::string, int>> reasons;
  for (const auto &r : rows) {
    ++tiers[r.at("tier").get<std::string>()];
    auto message = r.at("message").get<std::string>();
    auto start = message.find('('), end = message.rfind(')');
    if (start == std::string::npos) continue;
    std::istringstream input(message.substr(start + 1, end - start - 1));
    std::string why;
    while (std::getline(input, why, ',')) {
      auto found = std::find_if(reasons.begin(), reasons.end(), [&](const auto &v) { return v.first == why; });
      if (found == reasons.end()) reasons.emplace_back(why, 1); else ++found->second;
    }
  }
  const auto count = [&](const std::string &key, int n, int width, const std::string &indent) {
    std::cout << indent << std::left << std::setw(width) << key << std::right << ' '
              << std::setw(6) << n << "  " << std::fixed << std::setprecision(2)
              << std::setw(6) << 100.0 * n / rows.size() << "%\n";
  };
  for (const auto &v : tiers) count(v.first, v.second, 10, "  ");
  std::stable_sort(reasons.begin(), reasons.end(), [](const auto &a, const auto &b) { return a.second > b.second; });
  for (const auto &v : reasons) count(v.first, v.second, 20, "    ");
  std::cout << "  signal            ";
  for (const auto label : {"p50", "p90", "p99", "p99.9", "p100", "min"}) std::cout << std::setw(10) << label;
  std::cout << '\n';
  for (const std::string key : {"valid_fraction", "sigma_max_m", "sigma_min_m", "speed_mps",
                                "scan_gap_periods", "levels_solved", "extrinsic_age_s"}) {
    std::vector<double> values;
    for (const auto &r : rows) if (r.contains(key)) {
      const auto &v = r.at(key);
      values.push_back(v.is_string() ? std::stod(v.get<std::string>()) : v.get<double>());
    }
    if (values.empty()) continue;
    std::cout << "  " << std::left << std::setw(18) << key << std::right << std::setprecision(4);
    for (const double p : {50.0, 90.0, 99.0, 99.9, 100.0}) std::cout << std::setw(10) << sim::tools::percentile(values, p);
    std::cout << std::setw(10) << *std::min_element(values.begin(), values.end()) << '\n';
  }
}
void record(const std::string &path) {
  rclcpp::init(0, nullptr);
  auto node = std::make_shared<rclcpp::Node>("rf2o_quality_recorder");
  std::ofstream out(path, std::ios::app);
  if (!out) throw std::runtime_error("cannot open " + path);
  auto sub = node->create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
    "/scan_odom/quality", rclcpp::SensorDataQoS(), [&](const diagnostic_msgs::msg::DiagnosticArray &message) {
      for (const auto &status : message.status) {
        sim::tools::Json row = sim::tools::Json::object();
        for (const auto &kv : status.values) row[kv.key] = kv.value;
        row["stamp"] = message.header.stamp.sec + message.header.stamp.nanosec * 1e-9;
        row["message"] = status.message;
        out << row.dump() << '\n' << std::flush;
      }
    });
  rclcpp::spin(node);
  rclcpp::shutdown();
}
}  // namespace
int main(int argc, char **argv) {
  if (argc < 3) { std::cerr << "usage: rf2o_quality record OUT | summary FILE [FILE ...]\n"; return 2; }
  try {
    const std::string command = argv[1];
    if (command == "record" && argc == 3) record(argv[2]);
    else if (command == "summary") for (int i = 2; i < argc; ++i) summary(argv[i]);
    else return 2;
    return 0;
  } catch (const std::exception &e) { std::cerr << e.what() << '\n'; return 1; }
}
