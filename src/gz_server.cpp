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

// `gz sim -s WORLD [-r]` without the ruby front end, stopping on SIGINT/SIGTERM
// from a normal thread. gz-common's SignalHandler runs Server::Stop() inside
// the asynchronous handler, on top of the interrupted sim thread, and Sensors'
// Stop then waits for its render thread. If the sim thread held a gz-transport
// lock the render thread needed, neither returned and the server ignored
// SIGINT and SIGTERM until SIGKILL. Here every thread blocks both signals and
// the main thread takes them with sigtimedwait. See the README's shutdown contract.
#include <gz/common/Console.hh>
#include <gz/sim/Server.hh>
#include <gz/sim/ServerConfig.hh>
#include <gz/sim/config.hh>

#include <csignal>
#include <cstdlib>
#include <ctime>
#include <iostream>
#include <string>

#include <pthread.h>

namespace {
int usage() {
  std::cerr << "usage: gz_server WORLD.sdf [-r] [-v LEVEL]\n"
               "  -r: start running (otherwise paused); -v: gz console verbosity 0-4\n";
  return 2;
}
}  // namespace

int main(int argc, char **argv) {
  // Before any thread exists, so every gz, transport and render thread inherits it.
  sigset_t stop_signals;
  sigemptyset(&stop_signals);
  sigaddset(&stop_signals, SIGINT);
  sigaddset(&stop_signals, SIGTERM);
  pthread_sigmask(SIG_BLOCK, &stop_signals, nullptr);

  std::string world;
  bool run = false;
  int verbosity = 1;  // gz sim's default
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    if (arg == "-r") {
      run = true;
    } else if (arg == "-v" && i + 1 < argc) {
      verbosity = std::atoi(argv[++i]);
    } else if (!arg.empty() && arg[0] != '-' && world.empty()) {
      world = arg;
    } else {
      return usage();
    }
  }
  if (world.empty()) return usage();

  gz::common::Console::SetVerbosity(verbosity);
  gz::sim::ServerConfig config;
  // Same console log as `gz sim -s`: <LogRecordPath>/server_console.log.
  gzLogInit(config.LogRecordPath(), "server_console.log");
  gzmsg << "Gazebo Sim Server v" << GZ_SIM_VERSION_FULL << " (sim gz_server)\n";
  if (!config.SetSdfFile(world)) {
    gzerr << "Failed to load world [" << world << "]\n";
    return 1;
  }

  gz::sim::Server server(config);
  if (!server.Run(false, 0, !run)) {
    gzerr << "Server failed to start\n";
    return 1;
  }
  // Poll so a server that stops itself (the /server_control stop request) also exits.
  const timespec poll{0, 100'000'000};
  while (server.Running()) {
    const int signal = sigtimedwait(&stop_signals, nullptr, &poll);
    if (signal == SIGINT || signal == SIGTERM) {
      gzmsg << "gz_server: received signal " << signal << ", stopping\n";
      server.Stop();
      break;
    }
  }
  return 0;  // ~Server joins the run thread and unloads the systems
}
