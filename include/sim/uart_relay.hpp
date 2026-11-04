// Copyright 2026 Thornbots. SPDX-License-Identifier: Apache-2.0
#ifndef SIM__UART_RELAY_HPP_
#define SIM__UART_RELAY_HPP_

#include <cerrno>
#include <cstdint>
#include <deque>
#include <stdexcept>
#include <system_error>
#include <vector>
#include <fcntl.h>
#include <sys/socket.h>
#include <unistd.h>
#include "sim/uart_wire.hpp"
#include "sim/uart_frame_assembler.hpp"

namespace sim {
// Both directions are nonblocking; queued bytes survive short writes.
class UartRelay {
public:
  UartRelay(int pty, uint32_t mcb_baud, uint32_t jetson_baud, int64_t read_delay_ns = 0, bool capture_frames = false)
  : pty_(pty), to_pty_(mcb_baud), to_firmware_(jetson_baud), delay_(read_delay_ns),
    capture_frames_(capture_frames) {
    if (delay_ < 0) throw std::invalid_argument("negative UART read delay");
    int sockets[2];
    if (socketpair(AF_UNIX, SOCK_STREAM, 0, sockets) < 0) fail("socketpair");
    socket_ = sockets[0]; child_ = sockets[1];
    for (int fd : sockets) {
      if (fcntl(fd, F_SETFL, fcntl(fd, F_GETFL) | O_NONBLOCK) < 0 ||
          fcntl(fd, F_SETFD, FD_CLOEXEC) < 0) {
        close(socket_); close(child_); fail("fcntl UART socket");
      }
    }
  }
  ~UartRelay() { if (socket_ >= 0) close(socket_); if (child_ >= 0) close(child_); }
  UartRelay(const UartRelay &) = delete;
  UartRelay &operator=(const UartRelay &) = delete;
  int child_fd() const { return child_; }
  void close_child() { if (child_ >= 0) close(child_); child_ = -1; }

  void pump(int64_t now_ns) {
    before_step(now_ns);
    after_step(now_ns);
  }
  // Observe Jetson input only before the firmware cycle. Input arriving while
  // it runs cannot have existed at that cycle's simulation time.
  void before_step(int64_t now_ns) {
    if (now_ns > INT64_MAX - delay_) throw std::overflow_error("UART relay time overflow");
    receive_firmware(now_ns, false);
    receive(pty_, to_firmware_, now_ns + delay_, false);
    deliver_to_pty(now_ns);
    deliver(socket_, to_firmware_, now_ns, true);
  }
  void after_step(int64_t now_ns) {
    receive_firmware(now_ns, true);
    deliver_to_pty(now_ns);
  }
  std::vector<CapturedUartFrame> take_delivered() {
    auto result = std::move(delivered_);
    delivered_.clear();
    return result;
  }
private:
  void receive_firmware(int64_t time, bool cycle_complete) {
    uint8_t buffer[4096];
    for (;;) {
      const auto n = ::read(socket_, buffer, sizeof(buffer));
      if (n > 0) {
        if (capture_frames_) {
          if (!cycle_complete) throw std::runtime_error("firmware UART outside a known cycle");
          for (auto &frame : assembler_.feed(buffer, static_cast<size_t>(n), time)) {
            complete_bytes_ += frame.bytes.size();
            frames_.push_back({complete_bytes_, std::move(frame)});
          }
        }
        to_pty_.enqueue(time, std::vector<uint8_t>(buffer, buffer + n));
      } else if (n == 0) throw std::runtime_error("firmware UART disconnected");
      else if (errno == EAGAIN || errno == EWOULDBLOCK) return;
      else if (errno != EINTR) fail("UART relay read");
    }
  }
  void deliver_to_pty(int64_t time) {
    auto bytes = to_pty_.ready(time);
    if (bytes.empty()) return;
    const auto n = ::write(pty_, bytes.data(), bytes.size());
    if (n > 0) {
      to_pty_.consume(static_cast<size_t>(n));
      sent_bytes_ += static_cast<size_t>(n);
      while (!frames_.empty() && frames_.front().end <= sent_bytes_) {
        delivered_.push_back(std::move(frames_.front().frame));
        frames_.pop_front();
      }
    } else if (n < 0 && errno != EAGAIN && errno != EWOULDBLOCK && errno != EIO && errno != EINTR)
      fail("UART relay write");
  }
  static void fail(const char *what) { throw std::system_error(errno, std::generic_category(), what); }
  static void receive(int fd, UartWire &wire, int64_t time, bool socket) {
    uint8_t buffer[4096];
    for (;;) {
      const auto n = ::read(fd, buffer, sizeof(buffer));
      if (n > 0) wire.enqueue(time, std::vector<uint8_t>(buffer, buffer + n));
      else if (n == 0) {
        if (socket) throw std::runtime_error("firmware UART disconnected");
        return;
      } else if (errno == EAGAIN || errno == EWOULDBLOCK || errno == EIO) return;
      else if (errno != EINTR) fail("UART relay read");
    }
  }
  static void deliver(int fd, UartWire &wire, int64_t time, bool socket) {
    auto bytes = wire.ready(time);
    if (bytes.empty()) return;
    const auto n = socket ? ::send(fd, bytes.data(), bytes.size(), MSG_NOSIGNAL) :
                            ::write(fd, bytes.data(), bytes.size());
    if (n > 0) wire.consume(static_cast<size_t>(n));
    else if (n < 0 && errno != EAGAIN && errno != EWOULDBLOCK && errno != EIO && errno != EINTR)
      fail("UART relay write");
  }
  int pty_, socket_ = -1, child_ = -1;
  UartWire to_pty_, to_firmware_;
  int64_t delay_;
  bool capture_frames_;
  struct PendingFrame { uint64_t end; CapturedUartFrame frame; };
  UartFrameAssembler assembler_;
  std::deque<PendingFrame> frames_;
  std::vector<CapturedUartFrame> delivered_;
  uint64_t complete_bytes_ = 0, sent_bytes_ = 0;
};
}  // namespace sim
#endif  // SIM__UART_RELAY_HPP_
