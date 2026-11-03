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

#include <fstream>
#include <iomanip>
#include <iostream>
#include <set>
#include <utility>
#include "sim/tool_io.hpp"

using sim::tools::Json;
namespace {
std::string repr(const Json &value) {
  if (value.is_null()) return "None";
  if (value.is_boolean()) return value.get<bool>() ? "True" : "False";
  if (value.is_number_float()) {
    double number = value.get<double>();
    if (std::isnan(number)) return "nan";
    if (std::isinf(number)) return number < 0 ? "-inf" : "inf";
  }
  if (value.is_string()) {
    const auto raw = value.get<std::string>();
    const char quote = raw.find('\'') != std::string::npos && raw.find('"') == std::string::npos ? '"' : '\'';
    std::string text(1, quote);
    for (char c : raw) {
      if (c == quote || c == '\\') text += '\\';
      if (c == '\n') text += "\\n";
      else if (c == '\r') text += "\\r";
      else if (c == '\t') text += "\\t";
      else text += c;
    }
    return text + quote;
  }
  if (value.is_structured()) {
    std::string text = value.is_array() ? "[" : "{";
    bool first = true;
    for (const auto &item : value.items()) {
      if (!first) text += ", ";
      first = false;
      if (value.is_object()) text += repr(Json(item.key())) + ": ";
      text += repr(item.value());
    }
    return text + (value.is_array() ? "]" : "}");
  }
  return value.dump();
}
struct Difference {
  std::string key;
  Json a, b;
  bool missing_a = false, missing_b = false;
};
void differ(const Json &a, const Json &b, double tolerance, const std::string &key,
            std::vector<Difference> &out) {
  if (a.is_object() && b.is_object()) {
    std::set<std::string> keys;
    for (const auto &v : a.items()) keys.insert(v.key());
    for (const auto &v : b.items()) keys.insert(v.key());
    for (const auto &k : keys) {
      const auto field = key.empty() ? k : key + '.' + k;
      if (!a.contains(k) || !b.contains(k))
        out.push_back({field, a.value(k, Json()), b.value(k, Json()), !a.contains(k), !b.contains(k)});
      else differ(a.at(k), b.at(k), tolerance, field, out);
    }
  } else if (a.is_array() && b.is_array() && a.size() == b.size()) {
    for (std::size_t i = 0; i < a.size(); ++i)
      differ(a[i], b[i], tolerance, key + '[' + std::to_string(i) + ']', out);
  } else if (a.is_number() && b.is_number()) {
    const double x = a.get<double>(), y = b.get<double>();
    if (x != y && !(std::isfinite(x) && std::isfinite(y) && std::abs(x - y) <= tolerance))
      out.push_back({key, a, b});
  } else if (a.type() != b.type() || a != b) out.push_back({key, a, b});
}
bool finite_number(const Json &value) {
  return value.is_number() && std::isfinite(value.get<double>());
}
bool finite_vector(const Json &value, size_t size) {
  return value.is_array() && value.size() == size &&
         std::all_of(value.begin(), value.end(), finite_number);
}
void validate_record(const Json &row, const std::string &name) {
  // Validate the core emitted by shot_hit_suite and Scorer::{judge_state,pose_errors}.
  // Extra diagnostics can be absent or null (for example unavailable stamped TF).
  const auto stamp = name == "shots.jsonl" && !row.contains("t") ? "t_fire" : "t";
  if (!row.contains(stamp) || !finite_number(row.at(stamp)))
    throw std::runtime_error("unusable record: expected finite numeric " + std::string(stamp));
  if (name == "shots.jsonl") {
    if (!row.contains("hit") || !row.at("hit").is_boolean())
      throw std::runtime_error("unusable shot record: expected boolean hit");
  } else if (name == "states.jsonl") {
    if (!row.contains("valid") || !row.at("valid").is_boolean() ||
        !row.contains("centre_xy_m") || !finite_number(row.at("centre_xy_m")) ||
        !row.contains("vel_err") || !finite_vector(row.at("vel_err"), 3))
      throw std::runtime_error("unusable state record: expected boolean valid, finite centre_xy_m "
                               "and three finite vel_err components");
  } else if (!row.contains("truth_xy") || !finite_vector(row.at("truth_xy"), 2)) {
    throw std::runtime_error("unusable pose/route record: expected two finite truth_xy components");
  }
}
std::vector<Json> trace_rows(const std::filesystem::path &path) {
  std::ifstream input(path);
  if (!input) throw std::runtime_error("cannot open trace " + path.string());
  std::vector<Json> rows;
  std::string line;
  size_t number = 0;
  while (std::getline(input, line)) {
    ++number;
    if (line.find_first_not_of(" \t\r\n") == std::string::npos) continue;
    try {
      auto row = sim::tools::parse(line);
      if (!row.is_object()) throw std::runtime_error("expected a JSON object record");
      validate_record(row, path.filename().string());
      rows.push_back(std::move(row));
    } catch (const std::exception &e) {
      throw std::runtime_error(path.string() + ":" + std::to_string(number) + ": " + e.what());
    }
  }
  if (input.bad()) throw std::runtime_error("cannot read trace " + path.string());
  return rows;
}
}  // namespace
int main(int argc, char **argv) {
  try {
    std::vector<std::string> runs;
    double tolerance = 0;
    for (int i = 1; i < argc; ++i) {
      std::string arg = argv[i];
      if (arg == "--tol") {
        if (i + 1 >= argc) throw std::runtime_error("--tol requires a finite nonnegative number");
        const std::string value = argv[++i];
        size_t used = 0;
        tolerance = std::stod(value, &used);
        if (used != value.size() || !std::isfinite(tolerance) || tolerance < 0)
          throw std::runtime_error("--tol requires a finite nonnegative number");
      }
      else runs.push_back(arg);
    }
    if (runs.size() != 2) { std::cerr << "usage: compare_runs RUN_A RUN_B [--tol NUMBER]\n"; return 2; }
    for (const auto &run : runs)
      if (!std::filesystem::is_directory(run))
        throw std::runtime_error("not a run directory: " + run);
    bool same = true;
    size_t compared_records = 0;
    for (const std::string name : {"shots.jsonl", "states.jsonl", "poses.jsonl", "route.jsonl"}) {
      const auto pa = std::filesystem::path(runs[0]) / name, pb = std::filesystem::path(runs[1]) / name;
      const bool exists_a = std::filesystem::exists(pa), exists_b = std::filesystem::exists(pb);
      if (!exists_a && !exists_b) {
        std::cout << name << ": absent in both runs, not compared\n";
        continue;
      }
      if (!exists_a || !exists_b) {
        std::cout << name << ": only in run " << (exists_a ? 'A' : 'B') << '\n'; same = false; continue;
      }
      if (!std::filesystem::is_regular_file(pa) || !std::filesystem::is_regular_file(pb))
        throw std::runtime_error(name + ": expected regular trace files in both runs");
      const auto a = trace_rows(pa), b = trace_rows(pb);
      compared_records += std::min(a.size(), b.size());
      bool different = false;
      for (std::size_t i = 0; i < std::min(a.size(), b.size()); ++i) {
        std::vector<Difference> diffs;
        differ(a[i], b[i], tolerance, "", diffs);
        if (diffs.empty()) continue;
        std::cout << name << ": first divergence at record " << i << " (t="
                  << repr(a[i].value("t", Json())) << " / " << repr(b[i].value("t", Json())) << ")\n";
        for (const auto &diff : diffs)
          std::cout << "  " << diff.key << ": " << (diff.missing_a ? "<missing>" : repr(diff.a))
                    << " != " << (diff.missing_b ? "<missing>" : repr(diff.b)) << '\n';
        different = true; break;
      }
      if (different) same = false;
      else if (a.size() != b.size()) {
        std::cout << name << ": identical for " << std::min(a.size(), b.size())
                  << " records, then A has " << a.size() << ", B " << b.size() << '\n'; same = false;
      } else if (a.empty()) std::cout << name << ": empty in both runs, no records compared\n";
      else std::cout << name << ": identical, " << a.size() << " records\n";
    }
    if (!compared_records) {
      std::cerr << "no comparable records in shots.jsonl, states.jsonl, poses.jsonl or route.jsonl; "
                   "equality not established\n";
      return 1;
    }
    return same ? 0 : 1;
  } catch (const std::exception &e) { std::cerr << e.what() << '\n'; return 1; }
}
