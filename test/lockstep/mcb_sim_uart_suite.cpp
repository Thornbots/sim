// Copyright 2026 Thornbots
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "sim/mcb_sim_uart.hpp"

#include <gtest/gtest.h>

#include <chrono>
#include <cerrno>
#include <cstdlib>
#include <exception>
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include <poll.h>
#include <unistd.h>

namespace
{
constexpr int64_t kBaud = 115200;

std::string socket_path()
{
  std::string path = "/tmp/mcb_sim_uart_XXXXXX";
  const int fd = ::mkstemp(path.data());
  if (fd < 0) {
    throw std::runtime_error("could not create unique simulator UART path");
  }
  ::close(fd);
  ::unlink(path.c_str());
  return path;
}

int64_t wire_deadline(int64_t start_ns, std::size_t byte_count)
{
  const auto bits_ns = static_cast<int64_t>(byte_count) * 10000000000LL;
  return start_ns + (bits_ns + kBaud - 1) / kBaud;
}

bool readable(int fd, int timeout_ms)
{
  pollfd descriptor{fd, POLLIN, 0};
  int result;
  do {
    result = ::poll(&descriptor, 1, timeout_ms);
  } while (result < 0 && errno == EINTR);
  return result > 0 && (descriptor.revents & POLLIN) != 0;
}

struct Fd
{
  explicit Fd(int value) : value(value) {}
  ~Fd() {if (value >= 0) {::close(value);}}
  int value;
};

std::vector<uint8_t> pose_frame()
{
  return sim::mcb_protocol::frame(sim::mcb_protocol::Pose{
    1.25F, -2.5F, 0.5F, -0.25F, 0.1F, 1.2F, sim::mcb_protocol::kOdomPods});
}
}  // namespace

TEST(McbSimUart, ReceiveWaitsForSerializedFrameAndPreservesStamp) {
  namespace wire = dji_serial_bridge::sim_uart;
  constexpr int64_t capture_ns = 5000000;
  const auto bytes = pose_frame();
  const auto ready_ns = wire_deadline(capture_ns, bytes.size());
  const auto path = socket_path();
  sim::McbSimUart uart(path, kBaud);
  Fd client(wire::connect_socket(path));
  uart.before_tick(0);

  ASSERT_EQ(::write(uart.firmware_fd(), bytes.data(), bytes.size()),
            static_cast<ssize_t>(bytes.size()));
  uart.after_tick(capture_ns);
  uart.before_tick(ready_ns - 1);
  EXPECT_FALSE(readable(client.value, 0));

  std::exception_ptr tick_error;
  std::thread tick([&] {
    try {
      uart.before_tick(ready_ns);
    } catch (...) {
      tick_error = std::current_exception();
    }
  });
  const bool packet_ready = readable(client.value, 2500);
  std::optional<wire::Packet> packet;
  std::exception_ptr receive_error;
  if (packet_ready) {
    try {
      packet = wire::receive_packet(client.value);
      if (packet) {
        wire::send_packet(client.value, {packet->sequence, packet->stamp_ns, {}});
      }
    } catch (...) {
      receive_error = std::current_exception();
    }
  }
  tick.join();

  EXPECT_FALSE(tick_error);
  EXPECT_FALSE(receive_error);
  ASSERT_TRUE(packet_ready);
  ASSERT_TRUE(packet);
  EXPECT_EQ(packet->sequence, 1u);
  EXPECT_EQ(packet->stamp_ns, capture_ns);
  EXPECT_EQ(packet->bytes, bytes);
}

TEST(McbSimUart, TransmitWritesFirmwareBytesAtSerializationDeadline) {
  namespace wire = dji_serial_bridge::sim_uart;
  constexpr int64_t capture_ns = 9000000;
  const auto bytes = pose_frame();
  const auto first_byte_ns = wire_deadline(capture_ns, 1);
  const auto ready_ns = wire_deadline(capture_ns, bytes.size());
  const auto path = socket_path();
  sim::McbSimUart uart(path, kBaud);
  Fd client(wire::connect_socket(path));
  uart.before_tick(0);

  wire::send_packet(client.value, {23, capture_ns, bytes});
  uart.before_tick(capture_ns);
  uart.before_tick(first_byte_ns - 1);
  EXPECT_FALSE(readable(uart.firmware_fd(), 0));

  uart.before_tick(first_byte_ns);
  ASSERT_TRUE(readable(uart.firmware_fd(), 250));
  uint8_t first_byte = 0;
  ASSERT_EQ(::read(uart.firmware_fd(), &first_byte, 1), 1);
  EXPECT_EQ(first_byte, bytes.front());

  uart.before_tick(ready_ns);
  ASSERT_TRUE(readable(uart.firmware_fd(), 250));
  std::vector<uint8_t> received(bytes.size() - 1);
  ASSERT_EQ(::read(uart.firmware_fd(), received.data(), received.size()),
            static_cast<ssize_t>(received.size()));
  EXPECT_EQ(received, (std::vector<uint8_t>(bytes.begin() + 1, bytes.end())));
}
