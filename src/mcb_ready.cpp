// Copyright 2026 Thornbots. SPDX-License-Identifier: Apache-2.0
// Exits 0 once the MCB PTY carries a CRC-valid POSE.
// Usage: mcb_ready <pty> [wall_timeout_s=30] [device_only]
#include <chrono>
#include <iostream>
#include <thread>
#include <fcntl.h>
#include "sim/mcb_protocol.hpp"
#include <unistd.h>
int main(int argc, char **argv) {
  if (argc < 2) return 2;
  const auto deadline = std::chrono::steady_clock::now() +
      std::chrono::seconds(argc > 2 ? std::stoi(argv[2]) : 30);
  const bool device_only = argc > 3 && std::string(argv[3]) == "device_only";
  sim::mcb_protocol::DJISerial parser;
  int fd = -1;
  while (std::chrono::steady_clock::now() < deadline) {
    if (device_only && access(argv[1], R_OK | W_OK) == 0) return 0;
    if (!device_only && fd < 0) fd = open(argv[1], O_RDONLY | O_NOCTTY | O_NONBLOCK);
    if (fd >= 0) {
      uint8_t bytes[1024];
      const auto count = read(fd, bytes, sizeof(bytes));
      if (count > 0) {
        for (const auto &frame : parser.feed(bytes, count)) {
          if (frame.msg_type == sim::mcb_protocol::kPose &&
              frame.payload.size() == sim::mcb_protocol::Pose::kSize) {
            close(fd);
            return 0;
          }
        }
      }
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  if (fd >= 0) close(fd);
  std::cerr << "MCB UART readiness timeout: " << argv[1] << std::endl;
  return 1;
}
