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
#include <cstdlib>
#include <cstring>
#include <dlfcn.h>
#include <fcntl.h>
#include <fstream>
#include <iostream>
#include <signal.h>
#include <sstream>
#include <stdexcept>
#include <sys/wait.h>
#include <thread>
#include <unistd.h>
#ifdef __linux__
#include <sys/prctl.h>
#endif
namespace sim {
namespace {
std::vector<char *> arguments(std::vector<std::string> &command) {
  std::vector<char *> args;
  for (auto &arg : command) {
    args.push_back(arg.data());
  }
  args.push_back(nullptr);
  return args;
}
void parent_death(pid_t parent) {
  signal(SIGINT, SIG_DFL);
#ifdef __linux__
  prctl(PR_SET_PDEATHSIG, SIGINT);
  if (getppid() != parent) {
    raise(SIGINT);
  }
#else
  static_cast<void>(parent);
#endif
}
} // namespace
LaunchTree::LaunchTree(std::vector<std::string> command, std::string logfile)
    : LaunchTree("stack", std::move(command), std::move(logfile)) {}
LaunchTree::LaunchTree(std::string name, std::vector<std::string> command,
                       std::string logfile)
    : name_(std::move(name)), logfile_(std::move(logfile)) {
  if (command.empty()) {
    throw std::invalid_argument("empty launch command");
  }
  auto args = arguments(command);
  const int log =
      open(logfile_.c_str(), O_WRONLY | O_CREAT | O_TRUNC | O_CLOEXEC, 0644);
  if (log < 0) {
    throw std::runtime_error("open launch log: " +
                             std::string(std::strerror(errno)));
  }
  const auto parent = getpid();
  pid_ = fork();
  if (pid_ == 0) {
    setsid();
    parent_death(parent);
    dup2(log, STDOUT_FILENO);
    dup2(log, STDERR_FILENO);
    close(log);
    execvp(args[0], args.data());
    _exit(127);
  }
  close(log);
  if (pid_ < 0) {
    throw std::runtime_error("fork launch: " +
                             std::string(std::strerror(errno)));
  }
  group_ = pid_;
  std::cout << '[' << name_ << "] started pid=" << pid_ << " log=" << logfile_
            << " cmd=";
  for (const auto &arg : command) {
    std::cout << arg << ' ';
  }
  std::cout << std::endl;
}
LaunchTree::~LaunchTree() { stop(); }
bool LaunchTree::alive() {
  if (pid_ < 0) {
    return false;
  }
  int status;
  if (waitpid(pid_, &status, WNOHANG) == pid_) {
    pid_ = -1;
    return false;
  }
  return true;
}
void LaunchTree::stop(double timeout) {
  if (group_ < 0) {
    return;
  }
  if (alive()) {
    std::cout << '[' << name_ << "] sending SIGINT to process group " << group_
              << "..." << std::endl;
    kill(-group_, SIGINT);
    const auto deadline = std::chrono::steady_clock::now() +
                          std::chrono::duration<double>(timeout);
    while (alive() && std::chrono::steady_clock::now() < deadline) {
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    if (alive()) {
      std::cout << '[' << name_ << "] did not exit within " << timeout
                << "s, SIGKILLing process group " << group_ << '.' << std::endl;
      kill(-group_, SIGKILL);
      int status;
      while (waitpid(pid_, &status, 0) < 0 && errno == EINTR) {
      }
      pid_ = -1;
    } else {
      std::cout << '[' << name_ << "] exited cleanly." << std::endl;
    }
  }
  if (kill(-group_, 0) == 0) {
    std::cout << '[' << name_ << "] processes outlived the launch in group "
              << group_ << "; SIGKILLing them." << std::endl;
    kill(-group_, SIGKILL);
  }
  group_ = -1;
}
std::string LaunchTree::read_log() const {
  std::ifstream input(logfile_);
  return {std::istreambuf_iterator<char>(input), {}};
}
std::string capture(const std::vector<std::string> &command, double timeout) {
  int pipefd[2];
  if (pipe(pipefd) < 0) {
    throw std::runtime_error("pipe failed");
  }
  auto mutable_command = command;
  auto args = arguments(mutable_command);
  const auto parent = getpid();
  const auto pid = fork();
  if (pid == 0) {
    parent_death(parent);
    close(pipefd[0]);
    dup2(pipefd[1], STDOUT_FILENO);
    close(pipefd[1]);
    execvp(args[0], args.data());
    _exit(127);
  }
  close(pipefd[1]);
  if (pid < 0) {
    close(pipefd[0]);
    throw std::runtime_error("fork failed");
  }
  fcntl(pipefd[0], F_SETFL, O_NONBLOCK);
  const auto deadline =
      std::chrono::steady_clock::now() + std::chrono::duration<double>(timeout);
  std::string result;
  char data[4096];
  int status = 0;
  while (true) {
    const auto count = read(pipefd[0], data, sizeof(data));
    if (count > 0) {
      result.append(data, static_cast<size_t>(count));
      continue;
    }
    if (waitpid(pid, &status, WNOHANG) == pid) {
      while (true) {
        const auto n = read(pipefd[0], data, sizeof(data));
        if (n <= 0) {
          break;
        }
        result.append(data, static_cast<size_t>(n));
      }
      break;
    }
    if (std::chrono::steady_clock::now() >= deadline) {
      kill(pid, SIGKILL);
      while (waitpid(pid, &status, 0) < 0 && errno == EINTR) {
      }
      close(pipefd[0]);
      throw std::runtime_error("command timed out");
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  close(pipefd[0]);
  if (!WIFEXITED(status) || WEXITSTATUS(status) != 0) {
    throw std::runtime_error("command failed: " + result);
  }
  return result;
}
std::string display_error() {
#ifdef __APPLE__
  return {};
#else
  const char *display = std::getenv("DISPLAY");
  if (!display || !*display) {
    return "DISPLAY is unset";
  }
  void *library = dlopen("libX11.so.6", RTLD_LAZY);
  if (!library) {
    return {};
  }
  const auto open_display =
      reinterpret_cast<void *(*)(const char *)>(dlsym(library, "XOpenDisplay"));
  const auto close_display =
      reinterpret_cast<int (*)(void *)>(dlsym(library, "XCloseDisplay"));
  void *handle = open_display ? open_display(display) : nullptr;
  if (handle && close_display) {
    close_display(handle);
  }
  dlclose(library);
  return handle ? std::string{} : "cannot open DISPLAY=" + std::string(display);
#endif
}
bool check_no_orphans(const std::string &label) {
  std::string listing;
  try {
    listing = capture({"ps", "-eo", "pid,ppid,args"});
  } catch (const std::exception &e) {
    listing = e.what();
  }
  std::istringstream input(listing);
  std::string line, matches;
  while (std::getline(input, line)) {
    if (line.find("<defunct>") != std::string::npos ||
        line.find("localization_suite") != std::string::npos ||
        line.find("localization_tests.launch.py") != std::string::npos ||
        line.find("pytest") != std::string::npos) {
      continue;
    }
    for (const auto needle :
         {"gz sim", "slam_toolbox", "amcl", "map_server", "ekf_filter_node",
          "pose_translator", "pose_emulator"}) {
      if (line.find(needle) != std::string::npos) {
        matches += line + '\n';
        break;
      }
    }
  }
  double load[3];
  if (getloadavg(load, 3) == 3) {
    std::cout << '[' << label << "] host load average: " << load[0] << ' '
              << load[1] << ' ' << load[2] << " ("
              << sysconf(_SC_NPROCESSORS_ONLN) << " CPUs)" << std::endl;
  }
  if (!matches.empty()) {
    std::cout
        << '[' << label
        << "] WARNING: localization/sim-related processes already running:\n"
        << matches;
  }
  return matches.empty();
}
} // namespace sim
