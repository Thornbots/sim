// Copyright 2026 Thornbots. SPDX-License-Identifier: Apache-2.0
#ifndef SIM__UART_FRAME_ASSEMBLER_HPP_
#define SIM__UART_FRAME_ASSEMBLER_HPP_

#include <cstdint>
#include <limits>
#include <stdexcept>
#include <vector>
#include <utility>
#include "sim/mcb_protocol.hpp"

namespace sim {
struct CapturedUartFrame {
  uint64_t id;
  int64_t capture_ns;
  std::vector<uint8_t> bytes;
};

// Input must be observed at a known firmware cycle; never resynchronize silently.
class UartFrameAssembler {
public:
  std::vector<CapturedUartFrame> feed(const uint8_t *data, size_t size, int64_t capture_ns) {
    if (failed_) throw std::runtime_error("UART frame assembler failed");
    try {
      return feed_checked(data, size, capture_ns);
    } catch (...) {
      failed_ = true;
      throw;
    }
  }
private:
  std::vector<CapturedUartFrame> feed_checked(const uint8_t *data, size_t size, int64_t capture_ns) {
    if (capture_ns < 0 || capture_ns < last_ns_)
      throw std::invalid_argument("UART capture time is not monotonic");
    last_ns_ = capture_ns;
    std::vector<CapturedUartFrame> frames;
    for (size_t i = 0; i < size; ++i) {
      if (buffer_.empty()) {
        if (data[i] != mcb_protocol::kHeadByte) throw std::runtime_error("invalid UART frame start");
        first_ns_ = capture_ns;
      }
      buffer_.push_back(data[i]);
      if (buffer_.size() == 5) {
        if (mcb_protocol::crc8(buffer_.data(), 4) != buffer_[4])
          throw std::runtime_error("invalid UART header CRC");
        length_ = static_cast<size_t>(buffer_[1]) | (static_cast<size_t>(buffer_[2]) << 8);
        if (length_ > mcb_protocol::kDebugMaxPayload) throw std::runtime_error("invalid UART length");
      }
      if (buffer_.size() >= 5 && buffer_.size() == length_ + 9) {
        const size_t end = buffer_.size() - 2;
        const auto crc = static_cast<uint16_t>(buffer_[end]) |
          (static_cast<uint16_t>(buffer_[end + 1]) << 8);
        if (crc != mcb_protocol::crc16(buffer_.data(), end))
          throw std::runtime_error("invalid UART frame CRC");
        const auto type = static_cast<uint16_t>(buffer_[5]) |
          (static_cast<uint16_t>(buffer_[6]) << 8);
        if (!((type == mcb_protocol::kPose && length_ == mcb_protocol::Pose::kSize) ||
              (type == mcb_protocol::kRefSys && length_ == mcb_protocol::RefSys::kSize) ||
              (type == mcb_protocol::kPing && length_ == mcb_protocol::Ping::kSize) ||
              type == mcb_protocol::kDebug))
          throw std::runtime_error("unsupported UART frame");
        if (next_id_ == 0) throw std::overflow_error("UART frame identity exhausted");
        frames.push_back({next_id_++, first_ns_, std::move(buffer_)});
        buffer_.clear();
      }
    }
    return frames;
  }
  bool failed_ = false;
  std::vector<uint8_t> buffer_;
  size_t length_ = 0;
  int64_t first_ns_ = 0, last_ns_ = 0;
  uint64_t next_id_ = 1;
};
}  // namespace sim
#endif  // SIM__UART_FRAME_ASSEMBLER_HPP_
