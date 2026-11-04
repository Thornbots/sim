// Copyright 2026 Thornbots. SPDX-License-Identifier: Apache-2.0
#ifndef SIM__UART_RELAY_HPP_
#define SIM__UART_RELAY_HPP_

#include <cerrno>
#include <cstdint>
#include <stdexcept>
#include <system_error>
#include <vector>
#include <fcntl.h>
#include <sys/socket.h>
#include <unistd.h>
#include "sim/uart_wire.hpp"

namespace sim {
// Both directions are nonblocking; queued bytes survive short writes.
class UartRelay {
public:
  UartRelay(int pty, uint32_t mcb_baud, uint32_t jetson_baud, int64_t read_delay_ns = 0)
  : pty_(pty), to_pty_(mcb_baud), to_firmware_(jetson_baud), delay_(read_delay_ns) {
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
    if (now_ns > INT64_MAX - delay_) throw std::overflow_error("UART relay time overflow");
    receive(socket_, to_pty_, now_ns, true);
    receive(pty_, to_firmware_, now_ns + delay_, false);
    deliver(pty_, to_pty_, now_ns, false);
    deliver(socket_, to_firmware_, now_ns, true);
  }
private:
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
};
}  // namespace sim
#endif  // SIM__UART_RELAY_HPP_
