// Copyright 2026 Thornbots. SPDX-License-Identifier: Apache-2.0
#ifndef SIM__RENODE_BOARD_HPP_
#define SIM__RENODE_BOARD_HPP_
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>
#include "sim/mcb_firmware.hpp"
extern "C" {
#define _Alignas alignas
#include <renode_api.h>
#undef _Alignas
}

namespace sim::type_c {
inline bool listening(unsigned port) {
  for (const char *path : {"/proc/net/tcp", "/proc/net/tcp6"}) {
    std::ifstream input(path);
    std::string line;
    while (std::getline(input, line)) {
      std::istringstream fields(line);
      std::string slot, local, remote, state;
      fields >> slot >> local >> remote >> state;
      if (state == "0A" && std::stoul(local.substr(local.find(':') + 1), nullptr, 16) == port)
        return true;
    }
  }
  return false;
}
inline void check(renode_error_t *error) {
  if (!error) return;
  const std::string message = error->message;
  renode_free_error(error);
  throw std::runtime_error("Renode: " + message);
}

// Host side of renode/TypeC.cs. Motor feedback and DBUS repetition run inside
// Renode in virtual time; this class only exchanges state between run_for calls.
class Board {
public:
  explicit Board(std::string port) : port_(std::move(port)) { connect(); }
  ~Board() { close(); }
  Board(const Board &) = delete;
  Board &operator=(const Board &) = delete;

  void advance(unsigned microseconds) {
    renode_time_t duration;
    check(renode_create_time(microseconds, TU_MICROSECONDS, &duration));
    call(renode_run_for(connection_, duration));
  }
  void write(uint64_t address, const uint8_t *data, size_t size) {
    if (size) call(renode_sysbus_write(bus_, address, AW_BYTE, data, size));
  }
  void read(uint64_t address, uint8_t *data, size_t size) {
    if (size) call(renode_sysbus_read(bus_, address, AW_BYTE, data, size));
  }
  uint32_t read_register(uint64_t address) {
    uint32_t value = 0;
    call(renode_sysbus_read(bus_, address, AW_DOUBLE_WORD, &value, 1));
    return value;
  }
  void stream(uint64_t address, const uint8_t *data, size_t size) {
    for (size_t offset = 0; offset < size; offset += 256)
      write(address, data + offset, std::min(size - offset, size_t{256}));
  }
  void referee(const std::vector<uint8_t> &data) { stream(0xf0000200, data.data(), data.size()); }
  void jetson(const uint8_t *data, size_t size) { stream(0xf0000100, data, size); }
  // Starts or replaces the DBUS frame Renode repeats every 14 ms.
  void remote(bool drive, bool auto_fire = true) {
    // DBUS switches: left UP, right UP for auto-drive or MID for centred manual.
    const uint8_t data[] = {0, 4, 32, 0, 1, static_cast<uint8_t>(
        8 | (drive ? 1 : 3) << 4 | (auto_fire ? 1 : 3) << 6),
                            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 4};
    write(0xf0000300, data, sizeof(data));
  }
  void sensors(const std::array<double, 10> &readings) {
    std::vector<uint8_t> bytes;
    for (size_t i = 0; i < 4; ++i) mcb_firmware::append_float(bytes, readings[i]);
    write(0xf0000400, bytes.data(), bytes.size());
    bytes.clear();
    const auto word = [&bytes](double value) {
      mcb_firmware::append_u16(bytes, static_cast<uint16_t>(static_cast<int16_t>(
          std::clamp(std::round(value), -32768.0, 32767.0))));
    };
    // Yaw-only IMU mounting model; see README's Type C limits.
    word(0); word(0);
    word(readings[5] * 32767 / (2000 * mcb_motor::kPi / 180));
    word(0); word(0); word(32768 / 3.0);
    // The pod steering encoder is inverted by the board's hardware definition.
    const double turn = (readings[4] - readings[6]) / (2 * mcb_motor::kPi);
    mcb_firmware::append_u16(bytes, 16383 - static_cast<uint16_t>(
        (turn - std::floor(turn)) * 16384));
    write(0xf0000500, bytes.data(), bytes.size());
  }
  // Plant order: yaw, pitch, index, odo, flywheel 1/2, drive 1..4 (SentryHardware IDs).
  void feedback(const mcb_motor::Feedback &feedback) {
    for (size_t bus = 0; bus < kBuses.size(); ++bus) {
      std::array<uint8_t, 0x41> block{};
      for (size_t id = 0; id < 8; ++id) {
        const int motor = kBuses[bus][id];
        if (motor < 0) continue;
        std::copy(feedback[motor].begin(), feedback[motor].end(), block.begin() + 8 * id);
        block[0x40] |= 1 << id;
      }
      write(kMotors + 0x100 * bus, block.data(), block.size());
    }
  }
  std::vector<uint8_t> uart() {
    uint32_t size = 0;
    call(renode_sysbus_read(bus_, 0xf0000000, AW_DOUBLE_WORD, &size, 1));
    if (size > 65536) throw std::runtime_error("Type C UART queue overflow");
    std::vector<uint8_t> bytes(size);
    for (size_t offset = 0; offset < size; offset += 4096)
      read(0xf0000000, bytes.data() + offset, std::min(size - offset, size_t{4096}));
    return bytes;
  }
  mcb_motor::Commands motors() {
    mcb_motor::Commands result{};
    frames_ = 0;
    for (size_t bus = 0; bus < kBuses.size(); ++bus) {
      std::array<uint8_t, 20> data{};
      read(kMotors + 0x100 * bus + 0x80, data.data(), data.size());
      for (size_t id = 0; id < 8; ++id) {
        const int motor = kBuses[bus][id];
        if (motor >= 0)
          result[motor] = static_cast<int16_t>(data[2 * id] << 8 | data[2 * id + 1]);
      }
      frames_ += data[16] | data[17] << 8 | data[18] << 16 | static_cast<uint32_t>(data[19]) << 24;
    }
    return result;
  }
  // Board command frames counted by the latest motors() call.
  uint64_t frames() const { return frames_; }

private:
  // Motor index for CAN IDs 0x201..0x208 on CAN1 and CAN2; -1 is absent.
  static constexpr std::array<std::array<int, 8>, 2> kBuses{{
      {6, 7, 8, 9, 3, -1, 0, -1}, {-1, 5, 4, -1, 1, 2, -1, -1}}};
  static constexpr uint64_t kMotors = 0xf0001000;
  // Renode 1.17.0's control server grows its receive-thread stack per message
  // (fixed upstream in renode 57f6325b); reconnecting starts a fresh thread.
  static constexpr unsigned kMessagesPerConnection = 10000;

  void call(renode_error_t *error) {
    check(error);
    if (++messages_ >= kMessagesPerConnection) {
      close();
      connect();
    }
  }
  void connect() {
    messages_ = 0;
    // After a disconnect the server needs a moment to listen again.
    for (int attempt = 0;; ++attempt) {
      // The pinned C client can terminate on a failed connect instead of returning an error.
      if (!listening(std::stoul(port_))) {
        if (attempt == 100) throw std::runtime_error("Renode control-server reconnect timeout");
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        continue;
      }
      renode_error_t *error = renode_connect(port_.c_str(), &connection_);
      if (!error) break;
      if (attempt == 100) check(error);
      renode_free_error(error);
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    try {
      check(renode_get_machine(connection_, "type-c", &machine_));
      check(renode_get_sysbus(machine_, &bus_));
    } catch (...) { close(); throw; }
  }
  void close() {
    if (connection_) renode_disconnect(&connection_);
    std::free(bus_);
    std::free(machine_);
    connection_ = nullptr;
    bus_ = nullptr;
    machine_ = nullptr;
  }
  std::string port_;
  renode_t *connection_ = nullptr;
  renode_machine_t *machine_ = nullptr;
  renode_bus_context_t *bus_ = nullptr;
  unsigned messages_ = 0;
  uint64_t frames_ = 0;
};
}  // namespace sim::type_c
#endif  // SIM__RENODE_BOARD_HPP_
