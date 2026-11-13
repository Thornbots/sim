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
// A leader that starts one member and handles SIGINT as `leader_on_int` says.
std::string two_process_script(const std::string &member, const std::string &leader_on_int) {
  return "import signal, subprocess, sys, time\n"
         "member = subprocess.Popen(['python3', '-c', " + member + "])\n"
         "def stop(signum, frame):\n" + leader_on_int +
         "signal.signal(signal.SIGINT, stop)\n"
         "print('ready', flush=True)\n"
         "while True: signal.pause()\n";
}
bool wait_ready(sim::LaunchTree &tree) {
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
  while (std::chrono::steady_clock::now() < deadline) {
    if (tree.read_log().find("ready") != std::string::npos) {
      std::this_thread::sleep_for(std::chrono::milliseconds(200));
      return true;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  return false;
}
const char *kMemberExitsOnInt =
    "'import signal,sys,time\\nsignal.signal(signal.SIGINT,lambda s,f:sys.exit(0))\\n"
    "while True: time.sleep(1)'";
const char *kMemberIgnoresInt =
    "'import signal,time\\nsignal.signal(signal.SIGINT,signal.SIG_IGN)\\n"
    "while True: time.sleep(1)'";
TEST(LaunchTreeSignals, LeaderOnlySigintIsCleanWhenTheLeaderStopsItsMember) {
  const auto log = std::filesystem::temp_directory_path() / "sim_stop_leader_clean.log";
  sim::LaunchTree tree({"python3", "-c",
                        two_process_script(kMemberExitsOnInt,
                                           "    member.send_signal(signal.SIGINT)\n"
                                           "    member.wait()\n    sys.exit(0)\n")},
                       log.string());
  ASSERT_TRUE(wait_ready(tree));
  const auto group = tree.pid();
  tree.stop(5.0);
  EXPECT_TRUE(tree.clean());
  errno = 0;
  EXPECT_EQ(kill(-group, 0), -1);
  EXPECT_EQ(errno, ESRCH);
  std::filesystem::remove(log);
}
TEST(LaunchTreeSignals, MemberThatOutlivesTheLeaderIsKilledAndReported) {
  const auto log = std::filesystem::temp_directory_path() / "sim_stop_orphan.log";
  sim::LaunchTree tree({"python3", "-c",
                        two_process_script(kMemberIgnoresInt, "    sys.exit(0)\n")},
                       log.string());
  ASSERT_TRUE(wait_ready(tree));
  const auto group = tree.pid();
  tree.stop(5.0);
  EXPECT_FALSE(tree.clean());
  errno = 0;
  for (int attempt = 0; attempt < 100 && kill(-group, 0) == 0; ++attempt)
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  EXPECT_EQ(kill(-group, 0), -1);
  EXPECT_EQ(errno, ESRCH);
  EXPECT_NE(tree.read_log().find("ready"), std::string::npos);
  std::filesystem::remove(log);
}
TEST(LaunchTreeSignals, GroupModeReachesEveryMemberWithoutForwarding) {
  const auto log = std::filesystem::temp_directory_path() / "sim_stop_group.log";
  sim::LaunchTree tree("group", {"python3", "-c",
                                 two_process_script(kMemberExitsOnInt,
                                                    "    member.wait()\n    sys.exit(0)\n")},
                       log.string(), sim::LaunchOptions{true});
  ASSERT_TRUE(wait_ready(tree));
  tree.stop(5.0);
  EXPECT_TRUE(tree.clean());
  std::filesystem::remove(log);
}
// Renode: third-party SIGINT stays blocked, the owner stops it with SIGTERM.
TEST(LaunchTreeSignals, EmulatorOptionsBlockSigintAndStopOnSigterm) {
  const auto log = std::filesystem::temp_directory_path() / "sim_stop_emulator.log";
  sim::LaunchTree tree("emulator", {"python3", "-c",
      "import signal, sys, time\n"
      "signal.signal(signal.SIGINT, lambda s, f: sys.exit(1))\n"  // a runtime handler
      "signal.signal(signal.SIGTERM, lambda s, f: sys.exit(0))\n"
      "print('ready', flush=True)\n"
      "while True: time.sleep(1)\n"}, log.string(), sim::LaunchOptions::emulator());
  ASSERT_TRUE(wait_ready(tree));
  const auto group = tree.pid();
  EXPECT_EQ(kill(group, SIGINT), 0);  // what kill_launch.sh does
  std::this_thread::sleep_for(std::chrono::milliseconds(300));
  EXPECT_TRUE(tree.alive());
  tree.stop(5.0);
  EXPECT_TRUE(tree.clean());
  std::filesystem::remove(log);
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
        CheckCase{"lockstep: /bench/progress timed out after 0.50 s wall", true},
        CheckCase{"[wait_until] timed out after 10s waiting for: /bench/pacing acknowledgement", true},
        CheckCase{"stack NOT ready: /bench/pacing unavailable", true},
        CheckCase{"stack NOT ready: pacing cleanup: /bench/pacing rejected handoff", true},
        CheckCase{"lockstep: /cv/target/state_ack timed out after 0.50 s wall",
                  true},
        CheckCase{"[WARNING] [launch]: user interrupted with ctrl-c (SIGINT)\n"
                  "[INFO] [gazebo-1]: sending signal 'SIGTERM' to process[gazebo-1]", true},
        CheckCase{"[stack] did not exit within 15s, SIGKILLing process group 7.", true},
        CheckCase{"[stack] processes outlived the launch in group 7; SIGKILLing them.", true},
        CheckCase{"[WARNING] [launch]: user interrupted with ctrl-c (SIGINT)\n"
                  "[INFO] [gazebo-1]: sending signal 'SIGINT' to process[gazebo-1]\n"
                  "[INFO] [gazebo-1]: process has finished cleanly [pid 4]", false},
        // Exit -2 after an interrupt is only a note: the run is trusted, the
        // lifecycle check rejects it.
        CheckCase{"[WARNING] [launch]: user interrupted with ctrl-c (SIGINT)\n"
                  "[INFO] [x-1]: sending signal 'SIGINT' to process[x-1]\n"
                  "[ERROR] [x-1]: process has died [pid 4, exit code -2, cmd 'x'].", false}));
} // namespace
