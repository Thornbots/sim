// Copyright 2026 Thornbots. SPDX-License-Identifier: Apache-2.0
#pragma once
#include <cerrno>
#include <chrono>
#include <cstring>
#include <deque>
#include <string>
#include <set>
#include <sys/socket.h>
#include <sys/un.h>
#include <poll.h>
#include <unistd.h>
#include <dji_serial_bridge/sim_uart.hpp>
#include "sim/mcb_firmware.hpp"
#include "sim/uart_timeline.hpp"

namespace sim {
class McbSimUart {
public:
  McbSimUart(const std::string &path, int64_t baud) : path_(path), rx_(baud), tx_(baud) {
    if (path.empty() || path.size() >= sizeof(sockaddr_un{}.sun_path))
      throw std::invalid_argument("invalid simulator UART socket path");
    try {
      if (socketpair(AF_UNIX, SOCK_STREAM | SOCK_CLOEXEC | SOCK_NONBLOCK, 0, firmware_) < 0)
        mcb_firmware::system_error("socketpair simulator UART");
      listener_ = socket(AF_UNIX, SOCK_SEQPACKET | SOCK_CLOEXEC | SOCK_NONBLOCK, 0);
      if (listener_ < 0) mcb_firmware::system_error("socket simulator UART");
      sockaddr_un address{};
      address.sun_family = AF_UNIX;
      std::memcpy(address.sun_path, path.c_str(), path.size() + 1);
      if (bind(listener_, reinterpret_cast<sockaddr *>(&address), sizeof(address)) < 0)
        mcb_firmware::system_error("bind simulator UART");
      bound_ = true;
      if (listen(listener_, 1) < 0) mcb_firmware::system_error("listen simulator UART");
    } catch (...) {close(); throw;}
  }
  ~McbSimUart() {close();}
  McbSimUart(const McbSimUart &) = delete;
  McbSimUart &operator=(const McbSimUart &) = delete;
  int firmware_fd() const {return firmware_[1];}
  void child_started() {mcb_firmware::close_fd(firmware_[1]);}
  void poll_input(int64_t now_ns) {accept_client(); receive(now_ns);}
  bool received_cv(int64_t stamp_ns) const {return cv_stamps_.count(stamp_ns) != 0;}
  std::vector<int64_t> take_pose_stamps() {
    std::vector<int64_t> result;
    result.swap(pose_stamps_);
    return result;
  }
  void wait_input() {
    if (client_ < 0) return;
    pollfd pfd{client_, POLLIN, 0};
    if (::poll(&pfd, 1, 1) < 0 && errno != EINTR)
      mcb_firmware::system_error("poll simulator UART input");
  }

  void before_tick(int64_t now_ns) {
    accept_client();
    receive(now_ns);
    const auto bytes = tx_.take_bytes(now_ns);
    if (!bytes.empty()) {
      const auto n = ::write(firmware_[0], bytes.data(), bytes.size());
      if (n != static_cast<ssize_t>(bytes.size()))
        throw std::runtime_error("short firmware UART delivery");
    }
    if (client_ < 0) return;
    for (const auto &frame : rx_.take_frames(now_ns)) {
      dji_serial_bridge::sim_uart::send_packet(client_,
        {frame.sequence, frame.capture_ns, frame.bytes});
      const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
      acked_ = 0;
      while (acked_ != frame.sequence) {
        receive(now_ns);
        if (acked_ == frame.sequence) break;
        if (std::chrono::steady_clock::now() >= deadline)
          throw std::runtime_error("lockstep timeout: simulated UART bridge consumption");
        pollfd pfd{client_, POLLIN, 0};
        if (poll(&pfd, 1, 1) < 0 && errno != EINTR)
          mcb_firmware::system_error("poll simulator UART acknowledgement");
      }
      if (frame.bytes.size() >= 7 && frame.bytes[5] == mcb_protocol::kPose &&
        frame.bytes[6] == 0 && frame.capture_ns > 0) pose_stamps_.push_back(frame.capture_ns);
    }
  }

  void after_tick(int64_t now_ns) {
    uint8_t bytes[4096];
    for (;;) {
      const auto count = ::read(firmware_[0], bytes, sizeof(bytes));
      if (count < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) break;
      if (count < 0 && errno == EINTR) continue;
      if (count <= 0) throw std::runtime_error("firmware UART socket closed");
      buffer_.insert(buffer_.end(), bytes, bytes + count);
    }
    while (buffer_.size() >= 7) {
      if (buffer_[0] != mcb_protocol::kHeadByte ||
        mcb_protocol::crc8(buffer_.data(), 4) != buffer_[4])
        throw std::runtime_error("invalid firmware UART header");
      const size_t length = 9 + static_cast<size_t>(buffer_[1]) +
        (static_cast<size_t>(buffer_[2]) << 8);
      if (length > 4096) throw std::runtime_error("oversized firmware UART frame");
      if (buffer_.size() < length) break;
      rx_.enqueue(++sequence_, now_ns, {buffer_.begin(), buffer_.begin() + length});
      buffer_.erase(buffer_.begin(), buffer_.begin() + length);
    }
  }
private:
  void accept_client() {
    if (client_ >= 0) return;
    client_ = accept4(listener_, nullptr, nullptr, SOCK_CLOEXEC | SOCK_NONBLOCK);
    if (client_ < 0 && errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR)
      mcb_firmware::system_error("accept simulator UART");
  }
  void receive(int64_t now_ns) {
    if (client_ < 0) return;
    while (const auto packet = dji_serial_bridge::sim_uart::receive_packet(client_, true)) {
      if (packet->bytes.empty()) {
        acked_ = packet->sequence;
      } else {
        const auto frame = tx_.enqueue(packet->sequence, packet->stamp_ns, packet->bytes);
        if (frame.ready_ns <= now_ns)
          throw std::runtime_error("lockstep timeout: late simulator UART transmit seq=" +
            std::to_string(packet->sequence) + " capture_ns=" + std::to_string(packet->stamp_ns) +
            " ready_ns=" + std::to_string(frame.ready_ns) + " now_ns=" + std::to_string(now_ns));
        if (packet->bytes.size() >= 7 && packet->bytes[5] == mcb_protocol::kCvTarget &&
          packet->bytes[6] == 0) {
          cv_stamps_.insert(packet->stamp_ns);
          if (cv_stamps_.size() > 3000) cv_stamps_.erase(cv_stamps_.begin());
        }
      }
    }
  }
  void close() {
    for (auto &fd : firmware_) mcb_firmware::close_fd(fd);
    mcb_firmware::close_fd(client_);
    mcb_firmware::close_fd(listener_);
    if (bound_) {unlink(path_.c_str()); bound_ = false;}
  }
  std::string path_;
  int firmware_[2]{-1, -1}, listener_ = -1, client_ = -1;
  bool bound_ = false;
  uint64_t sequence_ = 0, acked_ = 0;
  UartTimeline rx_, tx_;
  std::vector<uint8_t> buffer_;
  std::set<int64_t> cv_stamps_;
  std::vector<int64_t> pose_stamps_;
};
}  // namespace sim
