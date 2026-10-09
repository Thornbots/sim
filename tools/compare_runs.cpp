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

#include <iomanip>
#include <iostream>
#include <set>
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
struct Difference { std::string key; Json a, b; };
void differ(const Json &a, const Json &b, double tolerance, const std::string &key,
            std::vector<Difference> &out) {
  if (a.is_object() && b.is_object()) {
    std::set<std::string> keys;
    for (const auto &v : a.items()) keys.insert(v.key());
    for (const auto &v : b.items()) keys.insert(v.key());
    for (const auto &k : keys) differ(a.value(k, Json()), b.value(k, Json()), tolerance,
                                     key.empty() ? k : key + '.' + k, out);
  } else if (a.is_array() && b.is_array() && a.size() == b.size()) {
    for (std::size_t i = 0; i < a.size(); ++i)
      differ(a[i], b[i], tolerance, key + '[' + std::to_string(i) + ']', out);
  } else if (a.is_number() && b.is_number()) {
    const double x = a.get<double>(), y = b.get<double>();
    if (x != y && !(std::isfinite(x) && std::isfinite(y) && std::abs(x - y) <= tolerance))
      out.push_back({key, a, b});
  } else if ((a.is_boolean() && b.is_number()) || (a.is_number() && b.is_boolean())) {
    const double x = a.is_boolean() ? static_cast<int>(a.get<bool>()) : a.get<double>();
    const double y = b.is_boolean() ? static_cast<int>(b.get<bool>()) : b.get<double>();
    if (x != y) out.push_back({key, a, b});
  } else if (a != b) out.push_back({key, a, b});
}
}  // namespace
int main(int argc, char **argv) {
  try {
    std::vector<std::string> runs;
    double tolerance = 0;
    for (int i = 1; i < argc; ++i) {
      std::string arg = argv[i];
      if (arg == "--tol" && i + 1 < argc) tolerance = std::stod(argv[++i]);
      else runs.push_back(arg);
    }
    if (runs.size() != 2) { std::cerr << "usage: compare_runs RUN_A RUN_B [--tol NUMBER]\n"; return 2; }
    bool same = true;
    for (const std::string name : {"shots.jsonl", "states.jsonl", "poses.jsonl", "route.jsonl"}) {
      const auto pa = std::filesystem::path(runs[0]) / name, pb = std::filesystem::path(runs[1]) / name;
      const bool exists_a = std::filesystem::exists(pa), exists_b = std::filesystem::exists(pb);
      if (!exists_a && !exists_b) continue;
      if (!exists_a || !exists_b) {
        std::cout << name << ": only in run " << (exists_a ? 'A' : 'B') << '\n'; same = false; continue;
      }
      const auto a = sim::tools::json_lines(pa), b = sim::tools::json_lines(pb);
      bool different = false;
      for (std::size_t i = 0; i < std::min(a.size(), b.size()); ++i) {
        std::vector<Difference> diffs;
        differ(a[i], b[i], tolerance, "", diffs);
        if (diffs.empty()) continue;
        std::cout << name << ": first divergence at record " << i << " (t="
                  << repr(a[i].value("t", Json())) << " / " << repr(b[i].value("t", Json())) << ")\n";
        for (const auto &diff : diffs)
          std::cout << "  " << diff.key << ": " << repr(diff.a) << " != " << repr(diff.b) << '\n';
        different = true; break;
      }
      if (different) same = false;
      else if (a.size() != b.size()) {
        std::cout << name << ": identical for " << std::min(a.size(), b.size())
                  << " records, then A has " << a.size() << ", B " << b.size() << '\n'; same = false;
      } else std::cout << name << ": identical, " << a.size() << " records\n";
    }
    return same ? 0 : 1;
  } catch (const std::exception &e) { std::cerr << e.what() << '\n'; return 1; }
}
