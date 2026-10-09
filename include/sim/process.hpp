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
#include <string>
#include <sys/types.h>
#include <vector>
namespace sim {
class LaunchTree {
public:
  LaunchTree(std::vector<std::string> command, std::string logfile);
  LaunchTree(std::string name, std::vector<std::string> command,
             std::string logfile);
  ~LaunchTree();
  LaunchTree(const LaunchTree &) = delete;
  LaunchTree &operator=(const LaunchTree &) = delete;
  bool alive();
  void stop(double timeout = 15.0);
  std::string read_log() const;
  std::string log_text() const { return read_log(); }
  pid_t pid() const { return pid_; }

private:
  std::string name_, logfile_;
  pid_t pid_ = -1, group_ = -1;
};
std::string capture(const std::vector<std::string> &command,
                    double timeout = 10.0);
std::string display_error();
bool check_no_orphans(const std::string &label);
} // namespace sim
#endif // SIM__PROCESS_HPP_
