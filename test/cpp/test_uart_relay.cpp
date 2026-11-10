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
#include <fcntl.h>
#include <pty.h>
#include <termios.h>
#include <unistd.h>
#include <gtest/gtest.h>
#include "sim/uart_relay.hpp"
#include "sim/mcb_firmware.hpp"

TEST(UartRelay, RelaysBothDirectionsAtTheirOwnWireDeadlines) {
  int master, slave;
  ASSERT_EQ(openpty(&master, &slave, nullptr, nullptr, nullptr), 0);
  termios settings{};
  ASSERT_EQ(tcgetattr(slave, &settings), 0);
  cfmakeraw(&settings);
  ASSERT_EQ(tcsetattr(slave, TCSANOW, &settings), 0);
  ASSERT_EQ(fcntl(master, F_SETFL, O_NONBLOCK), 0);
  ASSERT_EQ(fcntl(slave, F_SETFL, O_NONBLOCK), 0);
  sim::UartRelay relay(master, 10000, 10000, 500000);
  ASSERT_EQ(write(relay.child_fd(), "abc", 3), 3);
  ASSERT_EQ(write(slave, "XY", 2), 2);
  relay.pump(0);
  char data[8]{};
  EXPECT_EQ(read(slave, data, sizeof(data)), -1);
  relay.pump(1'000'000);
  ASSERT_EQ(read(slave, data, sizeof(data)), 1);
  EXPECT_EQ(data[0], 'a');
  EXPECT_EQ(read(relay.child_fd(), data, sizeof(data)), -1);
  relay.pump(3'000'000);
  ASSERT_EQ(read(slave, data, sizeof(data)), 2);
  EXPECT_EQ(std::string(data, 2), "bc");
  ASSERT_EQ(read(relay.child_fd(), data, sizeof(data)), 2);
  EXPECT_EQ(std::string(data, 2), "XY");
  close(master);
  close(slave);
}

TEST(UartRelay, RetainsBacklogWhenDestinationIsFull) {
  int pair[2];
  ASSERT_EQ(socketpair(AF_UNIX, SOCK_STREAM, 0, pair), 0);
  for (int fd : pair) ASSERT_EQ(fcntl(fd, F_SETFL, O_NONBLOCK), 0);
  int size = 1024;
  ASSERT_EQ(setsockopt(pair[0], SOL_SOCKET, SO_SNDBUF, &size, sizeof(size)), 0);
  sim::UartRelay relay(pair[0], 1'000'000, 1'000'000);
  const std::string input(32000, 'q');
  ASSERT_EQ(write(relay.child_fd(), input.data(), input.size()), static_cast<ssize_t>(input.size()));
  relay.pump(0);
  relay.pump(1'000'000'000);
  std::string output;
  char buffer[4096];
  for (int i = 0; i < 100 && output.size() < input.size(); ++i) {
    for (ssize_t n; (n = read(pair[1], buffer, sizeof(buffer))) > 0;)
      output.append(buffer, static_cast<size_t>(n));
    relay.pump(1'000'000'000);
  }
  EXPECT_EQ(output, input);
  close(pair[0]);
  close(pair[1]);
}

TEST(UartRelay, InputArrivingDuringFirmwareStepStartsAtNextObservation) {
  int master, slave;
  ASSERT_EQ(openpty(&master, &slave, nullptr, nullptr, nullptr), 0);
  termios settings{};
  ASSERT_EQ(tcgetattr(slave, &settings), 0);
  cfmakeraw(&settings);
  ASSERT_EQ(tcsetattr(slave, TCSANOW, &settings), 0);
  ASSERT_EQ(fcntl(master, F_SETFL, O_NONBLOCK), 0);
  sim::UartRelay relay(master, 10000, 10000, 0, true);
  relay.before_step(0);
  ASSERT_EQ(write(slave, "x", 1), 1);
  relay.after_step(0);
  relay.before_step(1'000'000);
  char byte;
  EXPECT_EQ(read(relay.child_fd(), &byte, 1), -1);
  relay.before_step(2'000'000);
  ASSERT_EQ(read(relay.child_fd(), &byte, 1), 1);
  EXPECT_EQ(byte, 'x');
  close(master);
  close(slave);
}

TEST(UartRelay, FirmwareProducesFramesThroughSocketAndPty) {
  const char *binary = std::getenv("T17_FIRMWARE_BINARY");
  if (!binary) GTEST_SKIP() << "set T17_FIRMWARE_BINARY to exercise a firmware backend";
  const char *adapter = std::getenv("T17_FIRMWARE_ADAPTER");
  sim::mcb_firmware::PtyLink pty("/tmp/t17_relay_" + std::to_string(getpid()));
  ASSERT_EQ(fcntl(pty.slave(), F_SETFL, O_NONBLOCK), 0);
  sim::UartRelay relay(pty.master(), 115200, 115200);
  sim::mcb_firmware::Firmware firmware(relay.child_fd(), binary, "stop", false,
                                        adapter ? adapter : "");
  relay.close_child();
  sim::mcb_firmware::Referee referee;
  sim::mcb_protocol::DJISerial parser;
  size_t frames = 0;
  for (int tick = 0; tick < 300; ++tick) {
    const int64_t time = tick * 1'000'000LL;
    relay.pump(time);
    firmware.step(1, {}, referee);
    relay.pump(time);
    uint8_t bytes[4096];
    for (ssize_t n; (n = read(pty.slave(), bytes, sizeof(bytes))) > 0;)
      frames += parser.feed(bytes, static_cast<size_t>(n)).size();
  }
  EXPECT_GT(frames, 0u);
}

TEST(UartRelay, ReportsFrameOnlyAfterItsSerializedLastByte) {
  int master, slave;
  ASSERT_EQ(openpty(&master, &slave, nullptr, nullptr, nullptr), 0);
  termios settings{};
  ASSERT_EQ(tcgetattr(slave, &settings), 0);
  cfmakeraw(&settings);
  ASSERT_EQ(tcsetattr(slave, TCSANOW, &settings), 0);
  ASSERT_EQ(fcntl(master, F_SETFL, O_NONBLOCK), 0);
  sim::UartRelay relay(master, 10000, 10000, 0, true);
  const auto frame = sim::mcb_protocol::encode_frame(sim::mcb_protocol::kPing, {42}, 255);
  relay.before_step(0);
  ASSERT_EQ(write(relay.child_fd(), frame.data(), 3), 3);
  relay.after_step(0);
  relay.before_step(1'000'000);
  ASSERT_EQ(write(relay.child_fd(), frame.data() + 3, frame.size() - 3),
            static_cast<ssize_t>(frame.size() - 3));
  relay.after_step(1'000'000);
  relay.before_step(9'000'000);
  EXPECT_TRUE(relay.take_delivered().empty());
  relay.before_step(10'000'000);
  const auto delivered = relay.take_delivered();
  ASSERT_EQ(delivered.size(), 1u);
  EXPECT_EQ(delivered[0].capture_ns, 0);
  EXPECT_EQ(delivered[0].bytes, frame);
  close(master);
  close(slave);
}

TEST(UartRelay, CoordinatedBackpressurePreservesBytesFramesAndCaptureTimes) {
  int pair[2];
  ASSERT_EQ(socketpair(AF_UNIX, SOCK_STREAM, 0, pair), 0);
  for (int fd : pair) ASSERT_EQ(fcntl(fd, F_SETFL, O_NONBLOCK), 0);
  int size = 1024;
  ASSERT_EQ(setsockopt(pair[0], SOL_SOCKET, SO_SNDBUF, &size, sizeof(size)), 0);
  sim::UartRelay relay(pair[0], 1'000'000, 1'000'000, 0, true);
  const auto frame = sim::mcb_protocol::encode_frame(sim::mcb_protocol::kPing, {42});
  constexpr size_t count = 3000;
  std::vector<uint8_t> input;
  for (size_t i = 0; i < count; ++i) input.insert(input.end(), frame.begin(), frame.end());
  ASSERT_EQ(write(relay.child_fd(), input.data(), input.size()), static_cast<ssize_t>(input.size()));
  relay.after_step(10);
  relay.before_step(1'000'000'000);
  auto delivered = relay.take_delivered();
  EXPECT_LT(delivered.size(), count);
  std::vector<uint8_t> output;
  uint8_t buffer[4096];
  for (int i = 0; i < 100 && (output.size() < input.size() || delivered.size() < count); ++i) {
    for (ssize_t n; (n = read(pair[1], buffer, sizeof(buffer))) > 0;)
      output.insert(output.end(), buffer, buffer + n);
    relay.before_step(1'000'000'000);
    auto next = relay.take_delivered();
    delivered.insert(delivered.end(), next.begin(), next.end());
  }
  EXPECT_EQ(output, input);
  ASSERT_EQ(delivered.size(), count);
  for (size_t i = 0; i < count; ++i) {
    EXPECT_EQ(delivered[i].id, i + 1);
    EXPECT_EQ(delivered[i].capture_ns, 10);
    EXPECT_EQ(delivered[i].bytes, frame);
  }
  close(pair[0]);
  close(pair[1]);
}

TEST(UartRelay, IncompleteFrameIsNotReportedOnDisconnect) {
  int pair[2];
  ASSERT_EQ(socketpair(AF_UNIX, SOCK_STREAM, 0, pair), 0);
  ASSERT_EQ(fcntl(pair[0], F_SETFL, O_NONBLOCK), 0);
  sim::UartRelay relay(pair[0], 10000, 10000, 0, true);
  const auto frame = sim::mcb_protocol::encode_frame(sim::mcb_protocol::kPing, {42});
  ASSERT_EQ(write(relay.child_fd(), frame.data(), 3), 3);
  relay.after_step(0);
  EXPECT_TRUE(relay.take_delivered().empty());
  relay.close_child();
  EXPECT_THROW(relay.after_step(1), std::runtime_error);
  EXPECT_TRUE(relay.take_delivered().empty());
  close(pair[0]);
  close(pair[1]);
}
