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

#include "sim/suite_timing.hpp"
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <limits>
#include <map>
#include <sstream>
#include <utility>
namespace sim::suite_timing {
namespace {
using Mark = std::array<double, 2>;
using Row = std::array<double, 3>;
using Phases = std::map<std::string, Row>;
struct Entry {
  std::string suite, name;
  Phases phases;
};
std::vector<Entry> totals;
std::function<double()> sim_now, wall_now;
struct Names {
  std::string suite, test_case;
} names;
Mark mark{};
constexpr std::array<const char *, 6> phases = {
    "sim_start", "bringup", "reset", "settle", "scored", "teardown"};
Mark now() {
  const double sim = sim_now ? sim_now() : 0.0;
  return {wall_now ? wall_now()
                   : std::chrono::duration<double>(
                         std::chrono::steady_clock::now().time_since_epoch())
                         .count(),
          sim != 0.0 ? sim : std::numeric_limits<double>::quiet_NaN()};
}
void add(const std::string &phase, const Mark &start, const Mark &end) {
  const auto name = names.test_case.empty() ? "(setup)" : names.test_case;
  auto found = std::find_if(totals.begin(), totals.end(), [&](const Entry &e) {
    return e.suite == names.suite && e.name == name;
  });
  if (found == totals.end()) {
    totals.push_back({names.suite, name, {}});
    found = totals.end() - 1;
  }
  auto &row = found->phases[phase];
  row[0] += end[0] - start[0];
  if (!std::isnan(start[1]) && !std::isnan(end[1])) {
    row[1] += end[1] - start[1];
    row[2] += end[0] - start[0];
  }
}
std::string line(const std::string &name, const Phases &data, size_t width) {
  double wall = 0.0, sim = 0.0, sim_wall = 0.0;
  for (const auto &item : data) {
    wall += item.second[0];
    sim += item.second[1];
    sim_wall += item.second[2];
  }
  std::ostringstream out;
  out << "  " << std::left << std::setw(static_cast<int>(width)) << name << ' '
      << std::right;
  for (const auto p : phases) {
    const auto row = data.find(p);
    if (row == data.end()) {
      out << std::setw(9) << "";
    } else {
      out << std::fixed << std::setprecision(1) << std::setw(9)
          << row->second[0];
    }
    out << ' ';
  }
  out << std::fixed << std::setprecision(1) << std::setw(8) << wall << ' ';
  if (sim_wall > 0.0) {
    out << std::setprecision(2) << std::setw(6) << sim / sim_wall;
  } else {
    out << std::setw(6) << "-";
  }
  return out.str();
}
} // namespace
void set_sim_clock(std::function<double()> clock) {
  sim_now = std::move(clock);
}
void set_wall_clock_for_testing(std::function<double()> clock) {
  wall_now = std::move(clock);
}
void reset() {
  totals.clear();
  sim_now = {};
  wall_now = {};
  names.suite.clear();
  names.test_case.clear();
  mark = {};
}
Suite::Suite(std::string name) : outer_(names.suite) {
  names.suite = std::move(name);
}
Suite::~Suite() { names.suite = outer_; }
Case::Case(std::string name) {
  names.test_case = std::move(name);
  mark = now();
}
Case::~Case() {
  add("scored", mark, now());
  names.test_case.clear();
  mark = {};
}
Phase::Phase(std::string name) : name_(std::move(name)) {
  const auto start = now();
  wall_ = start[0];
  sim_ = start[1];
  if (!names.test_case.empty()) {
    add("scored", mark, start);
  }
}
Phase::~Phase() {
  const auto end = now();
  add(name_, {wall_, sim_}, end);
  if (!names.test_case.empty()) {
    mark = end;
  }
}
std::vector<std::string> report() {
  std::vector<std::string> suites, lines;
  for (const auto &entry : totals) {
    if (std::find(suites.begin(), suites.end(), entry.suite) == suites.end()) {
      suites.push_back(entry.suite);
    }
  }
  for (const auto &name : suites) {
    size_t width = 5;
    for (const auto &entry : totals) {
      if (entry.suite == name) {
        width = std::max(width, entry.name.size());
      }
    }
    lines.push_back(name +
                    ": wall seconds per phase; RTF over sim-timed spans");
    std::ostringstream header;
    header << "  " << std::left << std::setw(static_cast<int>(width)) << "case"
           << ' ' << std::right;
    for (const auto p : phases) {
      header << std::setw(9) << p << ' ';
    }
    header << std::setw(8) << "total" << ' ' << std::setw(6) << "RTF";
    lines.push_back(header.str());
    Phases grand;
    for (const auto &entry : totals) {
      if (entry.suite != name) {
        continue;
      }
      for (const auto &item : entry.phases) {
        for (size_t i = 0; i < 3; ++i) {
          grand[item.first][i] += item.second[i];
        }
      }
      lines.push_back(line(entry.name, entry.phases, width));
    }
    lines.push_back(line("total", grand, width));
  }
  return lines;
}
} // namespace sim::suite_timing
