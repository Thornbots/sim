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

#include "sim/process.hpp"
#include <filesystem>
#include <fstream>
#include <gtest/gtest.h>
#include <stdexcept>
#include <string>
#include <unistd.h>
namespace {
class SuiteExit : public testing::TestWithParam<int> {};
TEST_P(SuiteExit, PropagatesChildResultThroughLaunchService) {
  if (GetParam() == 0) {
    EXPECT_NO_THROW(sim::capture(
        {"ros2", "launch", "sim", "suite_exit_probe.launch.py", "code:=0"}));
  } else {
    EXPECT_THROW(
        sim::capture({"ros2", "launch", "sim", "suite_exit_probe.launch.py",
                      "code:=" + std::to_string(GetParam())}),
        std::runtime_error);
  }
}
INSTANTIATE_TEST_SUITE_P(ChildCodes, SuiteExit, testing::Values(0, 1, 2));
struct CheckCase {
  std::string text;
  bool failed;
};
class BenchLog : public testing::TestWithParam<CheckCase> {};
TEST_P(BenchLog, RejectsFailedTestsAndStalls) {
  const auto path = std::filesystem::path("/tmp/sim_checker_" +
                                          std::to_string(getpid()) + ".log");
  {
    std::ofstream output(path);
    output << "[tests-1] ==== suite timing ====\n[tests-1] " << GetParam().text
           << '\n';
  }
  bool failed = false;
  try {
    sim::capture({SIM_CHECK_BENCH_LOG, path.string()});
  } catch (const std::runtime_error &) {
    failed = true;
  }
  EXPECT_EQ(failed, GetParam().failed);
  std::filesystem::remove(path);
}
INSTANTIATE_TEST_SUITE_P(
    Summaries, BenchLog,
    testing::Values(
        CheckCase{"================ 12 passed in 1.0s ================", false},
        CheckCase{
            "================ 1 failed, 11 passed in 1.0s ================",
            true},
        CheckCase{"================ 1 error in 1.0s ================", true},
        CheckCase{"lockstep: 0 lockstep timeouts", false},
        CheckCase{"lockstep: 2 lockstep timeouts", true},
        CheckCase{"lockstep timeout: mcb_batch at 0.065 s, no ack", true},
        CheckCase{"LockstepGate: lockstep timeout 1 held at 0.065 s", true},
        CheckCase{"E2E clock stalled during scoring", true},
        CheckCase{"lockstep: /cv/target/state_ack timed out after 0.50 s wall",
                  true}));
} // namespace
