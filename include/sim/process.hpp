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

#ifndef SIM__PROCESS_HPP_
#define SIM__PROCESS_HPP_
#include <csignal>
#include <functional>
#include <string>
#include <sys/types.h>
#include <vector>
namespace sim {
// A command in its own session. stop() sends SIGINT to the leader only, so a
// ros2 launch leader forwards it once to its children (a group-wide signal
// reaches every node twice and kills them with exit -2). A leader that does
// not forward (Renode, `ros2 run`) is signalled as a group: pass
// LaunchOptions::signal_group, which `ros2 run` commands get automatically.
// Members that outlive the leader are a defect: stop() SIGKILLs them, says so
// and leaves clean() false.
struct LaunchOptions {
  bool signal_group = false;  // signal the whole group, not the leader
  int stop_signal = SIGINT;   // what stop() sends
  int death_signal = SIGINT;  // what the kernel sends if the owner dies
  bool block_sigint = false;  // SIGINT stays pending in the child, never delivered
  // Renode: no state worth saving, and its runtime wedges if SIGINTed mid-command
  // (kill_launch.sh signals every descendant; a dying owner's parent-death SIGINT
  // does too). Its .NET runtime replaces an ignored SIGINT with its own handler,
  // but a blocked one stays blocked. It exits on SIGTERM.
  static LaunchOptions emulator() {
    return {true, SIGTERM, SIGKILL, true};
  }
};
class LaunchTree {
public:
  LaunchTree(std::vector<std::string> command, std::string logfile);
  LaunchTree(std::string name, std::vector<std::string> command,
             std::string logfile, LaunchOptions options = {});
  ~LaunchTree();
  LaunchTree(const LaunchTree &) = delete;
  LaunchTree &operator=(const LaunchTree &) = delete;
  bool alive();
  // Keep fixture I/O moving during shutdown; cleanup precedes callback rethrows.
  void stop(double timeout = 15.0,
            const std::function<void()> &progress = {});
  // False once stop() had to SIGKILL the leader or any group member.
  bool clean() const { return clean_; }
  std::string read_log() const;
  std::string log_text() const { return read_log(); }
  pid_t pid() const { return pid_; }

private:
  std::string name_, logfile_;
  pid_t pid_ = -1, group_ = -1;
  LaunchOptions options_;
  bool clean_ = true;
};
// Linux: deliver `signal` to this process when its parent dies, with the
// SIGINT disposition reset to default (a shell's `&` leaves it ignored).
// Raises it at once if the parent is already gone.
void die_with_parent(int signal = SIGINT);
std::string capture(const std::vector<std::string> &command,
                    double timeout = 10.0);
std::string display_error();
bool check_no_orphans(const std::string &label);
} // namespace sim
#endif // SIM__PROCESS_HPP_
