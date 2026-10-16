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
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <set>
#include <sstream>
#include <vector>
#include "sim/combat_repeats.hpp"

namespace {
using sim::combat_repeats::Audit;
using sim::combat_repeats::Group;
using sim::combat_repeats::Metric;
using sim::combat_repeats::Run;

std::string pct(double value) {
  std::ostringstream out;
  out << std::fixed << std::setprecision(1) << value * 100 << '%';
  return out.str();
}

std::string metric(const Metric &value) {
  std::ostringstream out;
  out << value.numeric << " values, " << value.nulls << " null, "
      << value.missing << " missing";
  if (value.numeric)
    out << ", p50/p95 " << std::fixed << std::setprecision(4) << value.p50
        << '/' << value.p95;
  return out.str();
}

std::string tracker(const Audit &audit) {
  return "true/false/null/missing " + std::to_string(audit.tracker_true) + "/" +
         std::to_string(audit.tracker_false) + "/" +
         std::to_string(audit.tracker_null) + "/" +
         std::to_string(audit.tracker_missing);
}

std::string rejection(const Run &run) {
  std::vector<std::string> reasons;
  if (run.runner_exit) reasons.push_back("runner exit " + std::to_string(run.runner_exit));
  if (run.checker_exit) reasons.push_back("checker exit " + std::to_string(run.checker_exit));
  if (run.timeout_lines)
    reasons.push_back(std::to_string(run.timeout_lines) + " distinct lockstep timeout line(s)");
  for (const auto &name : run.missing_artifacts)
    reasons.push_back("missing " + name);
  if (run.missing_artifacts.empty() && run.shots == 0)
    reasons.push_back("no native MCB shots");
  if (run.stage == "mcb_parked" && run.parked_cells == 0 &&
      std::find(run.missing_artifacts.begin(), run.missing_artifacts.end(),
                "scores.jsonl") == run.missing_artifacts.end())
    reasons.push_back("no parked score cells");
  if (reasons.empty()) return "accepted";
  std::string out = "rejected: ";
  for (std::size_t i = 0; i < reasons.size(); ++i) {
    if (i) out += ", ";
    out += reasons[i];
  }
  return out;
}

void print_run(const Run &run) {
  std::cout << "RUN " << run.profile << '/' << run.backend << '/' << run.stage
            << ' ' << run.directory.string() << "\n  status=" << rejection(run)
            << " workspace=" << run.workspace << " pins=" << run.pins_identity
            << " counts=" << run.hits << '/' << run.shots << " rate=";
  if (run.shots) std::cout << pct(run.rate());
  else std::cout << "n/a";
  if (run.stage == "mcb_parked")
    std::cout << " parked_cells=" << run.parked_cells;
  std::cout << '\n';
  for (const auto &cell : run.cells) {
    std::cout << "  cell " << cell.cell << ' ' << cell.hits << '/' << cell.shots
              << " rate=";
    if (cell.shots) std::cout << pct(static_cast<double>(cell.hits) / cell.shots);
    else std::cout << "n/a";
    std::cout << '\n';
  }
  std::cout << "  shots: detection_age_s " << metric(run.shot_audit.detection_age)
            << "; tracker_age_s " << metric(run.shot_audit.tracker_age)
            << "; tracker_valid " << tracker(run.shot_audit)
            << "; odom_disagreement_m " << metric(run.shot_audit.odom_disagreement)
            << "\n  states: ";
  if (run.states_present) {
    std::cout << "detection_age_s " << metric(run.state_audit.detection_age)
              << "; tracker_age_s " << metric(run.state_audit.tracker_age)
              << "; tracker_valid " << tracker(run.state_audit)
              << "; odom_disagreement_m " << metric(run.state_audit.odom_disagreement);
  } else {
    std::cout << "not present";
  }
  std::cout << "\n  route: ";
  if (run.route_present) {
    std::cout << run.route_source << " navigation_error_m "
              << metric(run.route_audit.navigation_error)
              << "; localization_error_m "
              << metric(run.route_audit.localization_error);
  } else {
    std::cout << "not present";
  }
  std::cout << '\n';
}

void print_group(const Group &group) {
  std::cout << "GROUP " << group.profile << '/' << group.backend << '/'
            << group.stage << " workspace=" << group.workspace
            << " pins=" << group.pins_identity
            << " accepted_runs=" << group.accepted_runs
            << " pooled=" << group.hits << '/' << group.shots;
  if (group.shots) std::cout << " (" << pct(static_cast<double>(group.hits) / group.shots) << ')';
  if (group.rate_runs)
    std::cout << " min_run=" << pct(group.minimum_rate)
              << " across " << group.rate_runs << " scored run(s)";
  if (const auto floor = sim::combat_repeats::tentative_floor(group))
    std::cout << " tentative_floor=" << pct(*floor)
              << " (min accepted run rate - 10 points; not final calibration)";
  else if (group.stage == "mcb_drive" || group.stage == "mcb_match")
    std::cout << " tentative_floor=unavailable (need >=3 accepted runs with shots"
              << (group.rate_runs && group.minimum_rate < .10
                      ? "; minimum accepted run rate is below 10%"
                      : "")
              << ')';
  std::cout << '\n';
}
}  // namespace

int main(int argc, char **argv) {
  if (argc < 2) {
    std::cerr << "usage: combat_repeats RUN_DIR [RUN_DIR ...]\n";
    return 2;
  }
  try {
    std::vector<Run> runs;
    bool artifact_errors = false;
    std::set<std::filesystem::path> seen;
    for (int i = 1; i < argc; ++i) {
      try {
        const auto canonical = std::filesystem::weakly_canonical(argv[i]);
        if (!seen.insert(canonical).second) {
          artifact_errors = true;
          std::cout << "DUPLICATE " << argv[i] << " resolves to "
                    << canonical.string() << "; ignored\n";
          continue;
        }
        runs.push_back(sim::combat_repeats::read_run(canonical));
      } catch (const std::exception &error) {
        artifact_errors = true;
        std::cout << "RUN " << argv[i] << " artifact_error=\""
                  << error.what() << "\"; excluded from every denominator\n";
      }
    }
    for (const auto &run : runs) print_run(run);
    const auto groups = sim::combat_repeats::group_accepted(runs);
    for (const auto &group : groups) print_group(group);

    std::cout << "T17 status: unverified by this artifact aggregation; it cannot "
                 "establish repeat-sequence equivalence or complete the required "
                 "tracker and stamped odometry investigation.\n";
    return artifact_errors ? 1 : 0;
  } catch (const std::exception &error) {
    std::cerr << "combat_repeats: " << error.what() << '\n';
    return 1;
  }
}
