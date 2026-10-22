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

#ifndef SIM__E2E_TEST_CORE_HPP_
#define SIM__E2E_TEST_CORE_HPP_
#include "sim/cv_bench.hpp"
#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>
#include <vector>
namespace sim::e2e {
inline cv_bench::Json localization_accuracy(
    const std::vector<cv_bench::Json> &records) {
  std::vector<double> errors;
  for (const auto &record : records) {
    const auto error = record.value("localization_error_m", cv_bench::Json(nullptr));
    if (error.is_number() && std::isfinite(error.get<double>()) && error.get<double>() >= 0)
      errors.push_back(error.get<double>());
  }
  const double coverage = records.empty() ? 0. : static_cast<double>(errors.size()) / records.size();
  const double p95 = cv_bench::percentile(errors, 95);
  const double maximum = errors.empty() ? std::numeric_limits<double>::infinity()
                                        : *std::max_element(errors.begin(), errors.end());
  // Fixed-stack Mac mini runs reached cell p95 0.107 m, max within 3 mm of
  // p95: scan odometry drifts uncorrected while parked (docs/ROADMAP.md
  // T39). 0.15/0.20 m leave that margin and still fail 0.4 m bring-up errors.
  return {{"samples", records.size()}, {"available", errors.size()},
          {"coverage", coverage}, {"p95_m", p95}, {"max_m", maximum},
          {"passed", coverage >= .95 && p95 <= .15 && maximum <= .20}};
}
inline void score_window(double seconds, const std::function<double()> &sim_now,
                         const std::function<double()> &wall_now,
                         const std::function<void()> &spin_once,
                         double wall_seconds_per_sim_second = 20) {
  double start = sim_now(), previous = start, wall_start = wall_now(),
         last_progress = wall_start;
  while (sim_now() - start < seconds) {
    spin_once();
    double now = sim_now(), wall = wall_now();
    if (now < previous - 1e-6)
      throw std::runtime_error("E2E clock moved backwards during scoring");
    if (now > previous)
      last_progress = wall;
    if (wall - last_progress > 5)
      throw std::runtime_error("E2E clock stalled for five wall seconds");
    if (wall - wall_start > std::max(30., seconds * wall_seconds_per_sim_second))
      throw std::runtime_error("E2E scoring exceeded its wall-time budget");
    previous = now;
  }
}
inline cv_bench::Json
diagnosis(const std::vector<cv_bench::Json> &route_records,
          const std::string &segment,
          const std::vector<cv_bench::Json> &shots) {
  if (shots.empty())
    return "target_tracker/point_to_cv_target: no firmware shots during this "
           "segment";
  std::vector<double> aim, head, barrel;
  for (const auto &r : shots) {
    if (r.contains("aim_off_panel_m"))
      aim.push_back(r["aim_off_panel_m"]);
    if (r.contains("barrel_off_aim_deg"))
      barrel.push_back(r["barrel_off_aim_deg"]);
  }
  for (const auto &r : route_records)
    if (r.value("segment", std::string()) == segment &&
        r.contains("head_tf_error_deg") && !r["head_tf_error_deg"].is_null())
      head.push_back(r["head_tf_error_deg"]);
  if (!aim.empty() && cv_bench::percentile(aim, 50) > .10)
    return "target_tracker/point_to_cv_target: median aim error exceeds 0.10 m";
  if (!head.empty() && cv_bench::percentile(head, 95) > 1)
    return "pose/TF stamps: head_pitch p95 attitude error exceeds 1 degree";
  if (!barrel.empty() && cv_bench::percentile(barrel, 50) > 1)
    return "MCB/gimbal: median barrel error exceeds 1 degree";
  return nullptr;
}
} // namespace sim::e2e
#endif
