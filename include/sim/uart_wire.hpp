// Copyright 2026 Thornbots. SPDX-License-Identifier: Apache-2.0
#ifndef SIM__UART_WIRE_HPP_
#define SIM__UART_WIRE_HPP_

#include <cstddef>
#include <cstdint>
#include <deque>
#include <limits>
#include <stdexcept>
#include <vector>

namespace sim {
// One direction of an 8N1 UART. Times are simulation nanoseconds.
class UartWire {
public:
  explicit UartWire(uint32_t baud) : baud_(baud) {
    if (baud == 0) throw std::invalid_argument("UART baud must be positive");
  }

  void enqueue(int64_t now_ns, const std::vector<uint8_t> &bytes) {
    if (now_ns < 0 || now_ns < last_enqueue_ns_)
      throw std::invalid_argument("UART enqueue time must be monotonic and nonnegative");
    last_enqueue_ns_ = now_ns;
    if (bytes.empty()) return;
    if (now_ns >= deadline_ns()) {
      finish_ns_ = now_ns;
      remainder_ = 0;
    }
    for (const auto byte : bytes) {
      constexpr int64_t bits_ns = 10'000'000'000LL;
      const int64_t delta = bits_ns / baud_;
      const uint64_t fraction = remainder_ + bits_ns % baud_;
      const int64_t advance = delta + static_cast<int64_t>(fraction / baud_);
      if (finish_ns_ > std::numeric_limits<int64_t>::max() - advance - 1)
        throw std::overflow_error("UART time overflow");
      finish_ns_ += advance;
      remainder_ = fraction % baud_;
      pending_.push_back({deadline_ns(), byte});
    }
  }

  // Peek and consume separately so a short nonblocking write loses no bytes.
  std::vector<uint8_t> ready(int64_t now_ns) const {
    std::vector<uint8_t> bytes;
    for (const auto &entry : pending_) {
      if (entry.when_ns > now_ns) break;
      bytes.push_back(entry.byte);
    }
    return bytes;
  }
  void consume(size_t count) {
    if (count > pending_.size()) throw std::out_of_range("UART consume exceeds queue");
    while (count--) pending_.pop_front();
  }
  bool empty() const { return pending_.empty(); }
  int64_t next_deadline_ns() const {
    if (empty()) throw std::logic_error("UART queue is empty");
    return pending_.front().when_ns;
  }

private:
  int64_t deadline_ns() const { return finish_ns_ + (remainder_ != 0); }
  struct Entry { int64_t when_ns; uint8_t byte; };
  uint32_t baud_;
  int64_t finish_ns_ = 0, last_enqueue_ns_ = 0;
  uint64_t remainder_ = 0;
  std::deque<Entry> pending_;
};
}  // namespace sim
#endif  // SIM__UART_WIRE_HPP_
