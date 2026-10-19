// Copyright 2026 Thornbots. SPDX-License-Identifier: Apache-2.0
#pragma once
#include <algorithm>
#include <cstdint>
#include <cstddef>
#include <deque>
#include <limits>
#include <stdexcept>
#include <vector>
#include <utility>

namespace sim {
class UartTimeline {
public:
  struct Frame {
    uint64_t sequence;
    int64_t capture_ns, ready_ns;
    std::vector<uint8_t> bytes;
  };
  explicit UartTimeline(int64_t baud = 115200) : baud_(baud) {
    if (baud <= 0) throw std::invalid_argument("UART baud must be positive");
  }
  Frame enqueue(uint64_t sequence, int64_t capture_ns, const std::vector<uint8_t> &bytes) {
    if (capture_ns < 0 || bytes.empty() || bytes.size() > 4096)
      throw std::invalid_argument("invalid simulated UART frame");
    const int64_t start = std::max(capture_ns, available_ns_);
    const auto duration = [this](size_t count) {
      const int64_t bits_ns = static_cast<int64_t>(count) * 10000000000LL;
      return bits_ns / baud_ + (bits_ns % baud_ != 0);
    };
    if (start > std::numeric_limits<int64_t>::max() - duration(bytes.size()))
      throw std::overflow_error("simulated UART deadline overflows timestamp");
    for (size_t i = 0; i < bytes.size(); ++i) {
      bytes_.push_back({start + duration(i + 1), bytes[i]});
    }
    available_ns_ = bytes_.back().first;
    Frame frame{sequence, capture_ns, available_ns_, bytes};
    frames_.push_back(frame);
    return frame;
  }
  std::vector<uint8_t> take_bytes(int64_t now_ns) {
    std::vector<uint8_t> out;
    while (!bytes_.empty() && bytes_.front().first <= now_ns) {
      out.push_back(bytes_.front().second);
      bytes_.pop_front();
    }
    while (!frames_.empty() && frames_.front().ready_ns <= now_ns) frames_.pop_front();
    return out;
  }
  std::vector<Frame> take_frames(int64_t now_ns) {
    std::vector<Frame> out;
    while (!frames_.empty() && frames_.front().ready_ns <= now_ns) {
      out.push_back(std::move(frames_.front()));
      frames_.pop_front();
    }
    while (!bytes_.empty() && bytes_.front().first <= now_ns) bytes_.pop_front();
    return out;
  }
private:
  int64_t baud_, available_ns_ = 0;
  std::deque<std::pair<int64_t, uint8_t>> bytes_;
  std::deque<Frame> frames_;
};
}  // namespace sim
