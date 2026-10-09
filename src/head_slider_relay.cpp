// Copyright 2026 Thornbots
// Licensed under the Apache License, Version 2.0.

#include <cerrno>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <memory>
#include <mutex>
#include <regex>
#include <string>
#include <thread>

#include <fcntl.h>
#include <signal.h>
#include <sys/prctl.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <unistd.h>

#include "rclcpp/rclcpp.hpp"

namespace
{

void wait_for_child(pid_t pid)
{
  int status = 0;
  while (waitpid(pid, &status, 0) == -1 && errno == EINTR) {}
}

void suppress_stderr()
{
  const int null_fd = open("/dev/null", O_WRONLY);
  if (null_fd >= 0) {
    dup2(null_fd, STDERR_FILENO);
    close(null_fd);
  }
}

class Relay
{
public:
  Relay(const char * source, const char * destination, const rclcpp::Logger & logger)
  : source_(source), destination_(destination), logger_(logger) {}

  ~Relay()
  {
    stop();
  }

  void start()
  {
    reader_ = std::thread(&Relay::read_slider, this);
    publisher_ = std::thread(&Relay::publish_latest, this);
  }

  bool reader_done() const
  {
    return reader_done_.load();
  }

  void stop()
  {
    pid_t echo_pid = -1;
    pid_t publish_pid = -1;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      stopping_ = true;
      echo_pid = echo_pid_;
      publish_pid = publish_pid_;
    }
    updated_.notify_all();
    if (echo_pid > 0) {
      kill(echo_pid, SIGTERM);
    }
    if (publish_pid > 0) {
      kill(publish_pid, SIGTERM);
    }
    if (reader_.joinable()) {
      reader_.join();
    }
    if (publisher_.joinable()) {
      publisher_.join();
    }
  }

private:
  void read_slider()
  {
    int pipe_fds[2];
    if (pipe(pipe_fds) != 0) {
      RCLCPP_ERROR(logger_, "Cannot open slider pipe: %s", std::strerror(errno));
      reader_done_ = true;
      return;
    }

    const pid_t parent_pid = getpid();
    const pid_t pid = fork();
    if (pid == 0) {
      close(pipe_fds[0]);
      // The echo must die when a launched relay is killed without cleanup.
      prctl(PR_SET_PDEATHSIG, SIGTERM);
      if (getppid() != parent_pid) {
        _exit(1);
      }
      dup2(pipe_fds[1], STDOUT_FILENO);
      close(pipe_fds[1]);
      suppress_stderr();
      execlp("stdbuf", "stdbuf", "-oL", "gz", "topic", "-e", "-t",
        source_.c_str(), static_cast<char *>(nullptr));
      _exit(127);
    }
    close(pipe_fds[1]);
    if (pid < 0) {
      RCLCPP_ERROR(logger_, "Cannot start slider echo: %s", std::strerror(errno));
      close(pipe_fds[0]);
      reader_done_ = true;
      return;
    }

    {
      std::lock_guard<std::mutex> lock(mutex_);
      echo_pid_ = pid;
      if (stopping_) {
        kill(pid, SIGTERM);
      }
    }
    FILE * stream = fdopen(pipe_fds[0], "r");
    if (stream == nullptr) {
      RCLCPP_ERROR(logger_, "Cannot read slider echo: %s", std::strerror(errno));
      close(pipe_fds[0]);
      kill(pid, SIGTERM);
      wait_for_child(pid);
      {
        std::lock_guard<std::mutex> lock(mutex_);
        echo_pid_ = -1;
      }
      reader_done_ = true;
      return;
    }

    static const std::regex data_line(R"(^\s*data:\s*(-?[0-9.eE+-]+)\s*$)");
    char * line = nullptr;
    size_t capacity = 0;
    while (getline(&line, &capacity, stream) != -1) {
      std::cmatch match;
      if (!std::regex_match(line, match, data_line)) {
        continue;
      }
      {
        std::lock_guard<std::mutex> lock(mutex_);
        latest_ = match[1].str();
        has_update_ = true;
      }
      updated_.notify_one();
    }
    std::free(line);
    fclose(stream);
    wait_for_child(pid);
    {
      std::lock_guard<std::mutex> lock(mutex_);
      echo_pid_ = -1;
    }
    reader_done_ = true;
  }

  void publish_latest()
  {
    while (true) {
      std::string value;
      {
        std::unique_lock<std::mutex> lock(mutex_);
        updated_.wait(lock, [this] {return has_update_ || stopping_;});
        if (stopping_) {
          return;
        }
        has_update_ = false;
        value = latest_;
      }

      const std::string payload = "data: " + value;
      const pid_t pid = fork();
      if (pid == 0) {
        const int null_fd = open("/dev/null", O_WRONLY);
        if (null_fd >= 0) {
          dup2(null_fd, STDOUT_FILENO);
          dup2(null_fd, STDERR_FILENO);
          close(null_fd);
        }
        execlp("gz", "gz", "topic", "-t", destination_.c_str(), "-m",
          "gz.msgs.Double", "-p", payload.c_str(), static_cast<char *>(nullptr));
        _exit(127);
      }
      if (pid < 0) {
        RCLCPP_ERROR(logger_, "Cannot publish slider position: %s", std::strerror(errno));
        continue;
      }
      {
        std::lock_guard<std::mutex> lock(mutex_);
        publish_pid_ = pid;
        if (stopping_) {
          kill(pid, SIGTERM);
        }
      }
      wait_for_child(pid);
      {
        std::lock_guard<std::mutex> lock(mutex_);
        publish_pid_ = -1;
      }
    }
  }

  const std::string source_;
  const std::string destination_;
  const rclcpp::Logger logger_;
  std::mutex mutex_;
  std::condition_variable updated_;
  std::string latest_;
  bool has_update_ = false;
  bool stopping_ = false;
  pid_t echo_pid_ = -1;
  pid_t publish_pid_ = -1;
  std::thread reader_;
  std::thread publisher_;
  std::atomic<bool> reader_done_{false};
};

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("head_slider_relay");
  Relay pan(
    "/model/sentry/joint/headlink/0/cmd_pos",
    "/model/sentry/joint/headlink/cmd_pos", node->get_logger());
  Relay pitch(
    "/model/sentry/joint/headpitch/0/cmd_pos",
    "/model/sentry/joint/headpitch/cmd_pos", node->get_logger());
  pan.start();
  pitch.start();
  while (rclcpp::ok() && (!pan.reader_done() || !pitch.reader_done())) {
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }
  pan.stop();
  pitch.stop();
  rclcpp::shutdown();
  return 0;
}
