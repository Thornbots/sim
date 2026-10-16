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

#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <map>
#include <optional>
#include <regex>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <tuple>
#include <vector>
#include "sim/tool_io.hpp"

namespace sim::combat_repeats {
using Json = sim::tools::Json;

struct Metric {
  std::size_t numeric{0}, nulls{0}, missing{0};
  double p50{0}, p95{0};
};

struct Audit {
  Metric detection_age, tracker_age, odom_disagreement;
  Metric navigation_error, localization_error;
  std::size_t tracker_true{0}, tracker_false{0}, tracker_null{0},
      tracker_missing{0};
};

struct CellScore {
  std::string cell;
  std::size_t hits{0}, shots{0};
};

struct Run {
  std::filesystem::path directory;
  std::string profile, backend, stage, workspace, pins_identity;
  int runner_exit{0}, checker_exit{0};
  std::size_t timeout_lines{0}, hits{0}, shots{0}, parked_cells{0};
  std::vector<CellScore> cells;
  std::vector<std::string> missing_artifacts;
  bool states_present{false}, route_present{false};
  std::string route_source;
  std::optional<Json> pins;
  Audit shot_audit, state_audit, route_audit;

  bool accepted() const {
    return runner_exit == 0 && checker_exit == 0 && timeout_lines == 0 &&
           missing_artifacts.empty() && shots > 0 &&
           (stage != "mcb_parked" || parked_cells > 0);
  }
  double rate() const {
    return shots ? static_cast<double>(hits) / shots : 0.0;
  }
};

struct Group {
  std::string profile, backend, stage, workspace, pins_identity;
  std::size_t accepted_runs{0}, rate_runs{0}, hits{0}, shots{0};
  double minimum_rate{0};
};

inline int required_int(const Json &object, const std::string &key) {
  const auto &value = object.at(key);
  if (!value.is_number_integer() || value.is_boolean())
    throw std::runtime_error("outcome." + key + " must be an integer");
  return value.get<int>();
}

inline std::string required_string(const Json &object,
                                   const std::string &key) {
  const auto &value = object.at(key);
  if (!value.is_string())
    throw std::runtime_error("outcome." + key + " must be a string");
  return value.get<std::string>();
}

inline void require_file(const std::filesystem::path &directory,
                         const std::string &name) {
  if (!std::filesystem::is_regular_file(directory / name))
    throw std::runtime_error("missing required artifact " +
                             (directory / name).string());
}

inline std::vector<Json> json_lines(const std::filesystem::path &path) {
  auto records = sim::tools::json_lines(path);
  for (std::size_t i = 0; i < records.size(); ++i)
    if (!records[i].is_object())
      throw std::runtime_error(path.string() + ": record " +
                               std::to_string(i + 1) + " must be an object");
  return records;
}

inline bool has_timeout(const std::filesystem::path &path,
                        std::set<std::string> &unique) {
  static const std::regex pattern(
      R"((lockstep timeout\b(?!s)|lockstep[^\n]*timed out|lockstep[^\n]*\b[1-9][0-9]*\s+timeouts?\b|\b[1-9][0-9]*\s+lockstep\s+timeouts?\b))",
      std::regex::icase);
  std::ifstream input(path);
  if (!input) throw std::runtime_error("cannot open " + path.string());
  std::string line;
  while (std::getline(input, line))
    if (std::regex_search(line, pattern)) unique.insert(line);
  return !unique.empty();
}

inline double linear_percentile(std::vector<double> values, double p) {
  if (values.empty()) return 0;
  std::sort(values.begin(), values.end());
  const double position = p * (values.size() - 1);
  const auto low = static_cast<std::size_t>(std::floor(position));
  const auto high = static_cast<std::size_t>(std::ceil(position));
  return values[low] + (position - low) * (values[high] - values[low]);
}

inline Metric read_numeric_metric(const std::vector<Json> &records,
                                  const std::string &key) {
  Metric out;
  std::vector<double> values;
  for (const auto &record : records) {
    if (!record.contains(key)) {
      ++out.missing;
    } else if (record.at(key).is_null()) {
      ++out.nulls;
    } else {
      const auto &value = record.at(key);
      if (!value.is_number() || value.is_boolean() ||
          !std::isfinite(value.get<double>()) || value.get<double>() < 0)
        throw std::runtime_error(key + " must be null or a finite nonnegative number");
      values.push_back(value.get<double>());
    }
  }
  out.numeric = values.size();
  out.p50 = linear_percentile(values, .5);
  out.p95 = linear_percentile(values, .95);
  return out;
}

inline Audit read_audit(const std::vector<Json> &records) {
  Audit out;
  out.detection_age = read_numeric_metric(records, "detection_age_s");
  out.tracker_age = read_numeric_metric(records, "tracker_age_s");
  out.odom_disagreement = read_numeric_metric(records, "odom_disagreement_m");
  out.navigation_error = read_numeric_metric(records, "navigation_error_m");
  out.localization_error = read_numeric_metric(records, "localization_error_m");
  for (const auto &record : records) {
    const std::string key = record.contains("valid") ? "valid" : "tracker_valid";
    if (!record.contains(key)) {
      ++out.tracker_missing;
    } else if (record.at(key).is_null()) {
      ++out.tracker_null;
    } else if (!record.at(key).is_boolean()) {
      throw std::runtime_error("tracker_valid must be boolean or null");
    } else if (record.at(key).get<bool>()) {
      ++out.tracker_true;
    } else {
      ++out.tracker_false;
    }
  }
  return out;
}

inline Run read_run(const std::filesystem::path &directory) {
  const auto canonical = std::filesystem::weakly_canonical(directory);
  require_file(canonical, "outcome.json");
  require_file(canonical, "launch.log");
  require_file(canonical, "checker.log");

  const Json outcome =
      sim::tools::parse(sim::tools::read(canonical / "outcome.json"));
  if (!outcome.is_object()) throw std::runtime_error("outcome.json must be an object");
  Run run;
  run.directory = canonical;
  run.profile = required_string(outcome, "profile");
  run.backend = required_string(outcome, "backend");
  run.stage = required_string(outcome, "stage");
  run.workspace = required_string(outcome, "workspace");
  run.runner_exit = required_int(outcome, "runner_exit");
  run.checker_exit = required_int(outcome, "checker_exit");
  if (run.profile != "nightly" && run.profile != "branch")
    throw std::runtime_error("outcome.profile must be nightly or branch");
  if (run.backend != "hosted" && run.backend != "renode")
    throw std::runtime_error("outcome.backend must be hosted or renode");
  if (run.stage != "mcb_parked" && run.stage != "mcb_drive" &&
      run.stage != "mcb_match")
    throw std::runtime_error("outcome.stage is not an MCB stage");
  if (run.workspace.empty()) throw std::runtime_error("outcome.workspace must not be empty");
  if (outcome.contains("pins")) run.pins = outcome.at("pins");
  else if (outcome.contains("pin_manifest")) run.pins = outcome.at("pin_manifest");
  run.pins_identity = run.pins ? run.pins->dump() : "<no pin manifest>";

  std::set<std::string> timeouts;
  has_timeout(canonical / "launch.log", timeouts);
  const auto stack_path = canonical / "stack.log";
  if (std::filesystem::exists(stack_path)) {
    has_timeout(stack_path, timeouts);
  } else {
    run.missing_artifacts.push_back("stack.log");
  }
  run.timeout_lines = timeouts.size();

  const auto shots_path = canonical / "shots.jsonl";
  const auto scores_path = canonical / "scores.jsonl";
  if (!std::filesystem::exists(shots_path))
    run.missing_artifacts.push_back("shots.jsonl");
  if (!std::filesystem::exists(scores_path))
    run.missing_artifacts.push_back("scores.jsonl");
  const auto shots = std::filesystem::exists(shots_path)
                         ? json_lines(shots_path)
                         : std::vector<Json>{};
  const auto scores = std::filesystem::exists(scores_path)
                          ? json_lines(scores_path)
                          : std::vector<Json>{};
  for (const auto &shot : shots)
    if (!shot.contains("kind") || !shot.at("kind").is_string())
      throw std::runtime_error("shots.jsonl record requires string kind");
  if (run.stage == "mcb_parked") {
    run.parked_cells = scores.size();
    for (const auto &score : scores) {
      if (!score.contains("cell") || !score.at("cell").is_string())
        throw std::runtime_error("scores.jsonl record requires string cell");
      const auto &count = score.at("mcb_shots");
      const auto &rate = score.at("mcb_hit_rate");
      if (!count.is_number_integer() || count.is_boolean() ||
          count.get<int64_t>() < 0 || !rate.is_number() || rate.is_boolean() ||
          !std::isfinite(rate.get<double>()) || rate.get<double>() < 0 ||
          rate.get<double>() > 1)
        throw std::runtime_error("scores.jsonl requires valid mcb_shots and mcb_hit_rate");
      const auto cell_shots = static_cast<std::size_t>(count.get<int64_t>());
      const auto cell_hits = static_cast<std::size_t>(std::llround(
          count.get<double>() * rate.get<double>()));
      run.cells.push_back({score.at("cell").get<std::string>(), cell_hits,
                           cell_shots});
      run.shots += cell_shots;
      run.hits += cell_hits;
    }
  } else {
    for (const auto &shot : shots) {
      if (shot.at("kind") != "mcb") continue;
      const std::string result_key = run.stage == "mcb_match" ? "enemy_hit" : "hit";
      if (!shot.contains(result_key) || !shot.at(result_key).is_boolean())
        throw std::runtime_error("MCB shot is missing boolean " + result_key);
      ++run.shots;
      run.hits += shot.at(result_key).get<bool>() ? 1 : 0;
    }
  }

  std::vector<Json> mcb_shots;
  for (const auto &shot : shots)
    if (shot.value("kind", std::string()) == "mcb") mcb_shots.push_back(shot);
  run.shot_audit = read_audit(mcb_shots);
  const auto state_path = canonical / "states.jsonl";
  if (std::filesystem::exists(state_path)) {
    run.states_present = true;
    run.state_audit = read_audit(json_lines(state_path));
  }
  auto route_path = canonical / "route.jsonl";
  if (!std::filesystem::exists(route_path)) {
    route_path = canonical / "poses.jsonl";
    if (std::filesystem::exists(route_path)) run.route_source = "poses.jsonl";
  } else {
    run.route_source = "route.jsonl";
  }
  if (std::filesystem::exists(route_path)) {
    run.route_present = true;
    run.route_audit = read_audit(json_lines(route_path));
  }
  if (!run.missing_artifacts.empty()) {
    run.hits = 0;
    run.shots = 0;
    run.parked_cells = 0;
    run.cells.clear();
  }
  return run;
}

inline std::vector<Group> group_accepted(const std::vector<Run> &runs) {
  using Key = std::tuple<std::string, std::string, std::string, std::string,
                         std::string>;
  struct Accum { Group group; bool initialized{false}; };
  std::map<Key, Accum> groups;
  std::set<std::filesystem::path> seen;
  for (const auto &run : runs) {
    if (!run.accepted() || !seen.insert(std::filesystem::weakly_canonical(run.directory)).second)
      continue;
    auto &acc = groups[{run.profile, run.backend, run.stage, run.workspace,
                        run.pins_identity}];
    auto &group = acc.group;
    group.profile = run.profile;
    group.backend = run.backend;
    group.stage = run.stage;
    group.workspace = run.workspace;
    group.pins_identity = run.pins_identity;
    ++group.accepted_runs;
    group.hits += run.hits;
    group.shots += run.shots;
    if (run.shots) {
      ++group.rate_runs;
      if (!acc.initialized || run.rate() < group.minimum_rate)
        group.minimum_rate = run.rate();
      acc.initialized = true;
    }
  }
  std::vector<Group> out;
  for (const auto &entry : groups) out.push_back(entry.second.group);
  return out;
}

inline std::optional<double> tentative_floor(const Group &group) {
  if ((group.stage != "mcb_drive" && group.stage != "mcb_match") ||
      group.accepted_runs < 3 || group.rate_runs != group.accepted_runs ||
      !group.shots || group.minimum_rate < .10)
    return std::nullopt;
  return std::max(.10, group.minimum_rate - .10);
}

}  // namespace sim::combat_repeats
