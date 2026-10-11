// Copyright 2026 Thornbots. SPDX-License-Identifier: Apache-2.0
#include <chrono>
#include <iostream>
#include <thread>
#include <unistd.h>
int main(int argc, char **argv) {
  if (argc < 2) return 2;
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(30);
  while (std::chrono::steady_clock::now() < deadline) {
    if (access(argv[1], R_OK | W_OK) == 0) return 0;
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  std::cerr << "MCB UART readiness timeout: " << argv[1] << std::endl;
  return 1;
}
