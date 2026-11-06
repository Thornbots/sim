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

// guarded_exec COMMAND [ARGS...]: exec COMMAND so that it gets SIGINT when this
// process's parent dies, with default signal dispositions. tools/run_suite.sh
// starts the launch through it so a killed shell still ends in a graceful stop.
#include "sim/process.hpp"
#include <cstring>
#include <iostream>
#include <unistd.h>
int main(int argc, char **argv) {
  if (argc < 2) {
    std::cerr << "usage: guarded_exec COMMAND [ARGS...]\n";
    return 2;
  }
  sim::die_with_parent(SIGINT);
  execvp(argv[1], argv + 1);
  std::cerr << "guarded_exec: " << argv[1] << ": " << std::strerror(errno) << '\n';
  return 127;
}
