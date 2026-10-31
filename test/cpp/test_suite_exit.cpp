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
#include <cerrno>
#include <chrono>
#include <csignal>
#include <filesystem>
#include <fstream>
#include <gtest/gtest.h>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <unistd.h>
namespace {
class ProcessStop : public testing::Test {
protected:
  std::filesystem::path log;
  std::unique_ptr<sim::LaunchTree> child;
  void SetUp() override {
    log = std::filesystem::temp_directory_path() /
      ("sim_stop_" + std::to_string(getpid()) + "_" +
       testing::UnitTest::GetInstance()->current_test_info()->name() + ".log");
    child = std::make_unique<sim::LaunchTree>(std::vector<std::string>{
      "python3", "-c",
      "import signal, sys, time\n"
      "def stop(signum, frame):\n"
      "    time.sleep(0.15)\n"
      "    sys.exit(0)\n"
      "signal.signal(signal.SIGINT, stop)\n"
      "print('ready', flush=True)\n"
      "while True: signal.pause()\n"}, log.string());
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
    while (child->read_log().find("ready") == std::string::npos &&
           std::chrono::steady_clock::now() < deadline) {
      ASSERT_TRUE(child->alive());
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    ASSERT_NE(child->read_log().find("ready"), std::string::npos);
  }
  void TearDown() override {
    child.reset();
    std::filesystem::remove(log);
  }
  void expect_reaped(pid_t process) {
    EXPECT_FALSE(child->alive());
    errno = 0;
    EXPECT_EQ(kill(-process, 0), -1);
    EXPECT_EQ(errno, ESRCH);
  }
};
TEST_F(ProcessStop, PumpsProgressDuringGracefulShutdown) {
  const auto process = child->pid();
  int calls = 0;
  child->stop(5.0, [&] { ++calls; });
  EXPECT_GT(calls, 1);
  expect_reaped(process);
}
TEST_F(ProcessStop, ReapsChildBeforeRethrowingProgressFailure) {
  const auto process = child->pid();
  EXPECT_THROW(child->stop(5.0, [] { throw std::runtime_error("progress failed"); }),
               std::runtime_error);
  expect_reaped(process);
}
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
        CheckCase{"user interrupted with ctrl-c\n[ERROR] [launch]: Caught exception in launch: missing world", true},
        CheckCase{"lockstep: /cv/target/state_ack timed out after 0.50 s wall",
                  true}));
} // namespace
