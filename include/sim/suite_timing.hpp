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

#ifndef SIM__SUITE_TIMING_HPP_
#define SIM__SUITE_TIMING_HPP_
#include <functional>
#include <string>
#include <vector>
namespace sim::suite_timing {
void set_sim_clock(std::function<double()> clock);
void reset();
void set_wall_clock_for_testing(std::function<double()> clock);
std::vector<std::string> report();
class Suite {
public:
  explicit Suite(std::string name);
  ~Suite();
  Suite(const Suite &) = delete;
  Suite &operator=(const Suite &) = delete;

private:
  std::string outer_;
};
class Case {
public:
  explicit Case(std::string name);
  ~Case();
  Case(const Case &) = delete;
  Case &operator=(const Case &) = delete;
};
class Phase {
public:
  explicit Phase(std::string name);
  ~Phase();
  Phase(const Phase &) = delete;
  Phase &operator=(const Phase &) = delete;

private:
  std::string name_;
  double wall_, sim_;
};
} // namespace sim::suite_timing
#endif // SIM__SUITE_TIMING_HPP_
