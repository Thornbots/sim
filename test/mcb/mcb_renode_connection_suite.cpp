// Copyright 2026 Thornbots. SPDX-License-Identifier: Apache-2.0
#include <fstream>
#include <netinet/in.h>
#include <sys/socket.h>
#include <gtest/gtest.h>
#include "sim/process.hpp"
#include "sim/renode_board.hpp"

TEST(RenodeConnection, ReconnectsPreservePeripheralState) {
  const int socket = ::socket(AF_INET, SOCK_STREAM | SOCK_CLOEXEC, 0);
  ASSERT_GE(socket, 0);
  sockaddr_in address{};
  address.sin_family = AF_INET;
  address.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
  socklen_t size = sizeof(address);
  ASSERT_EQ(bind(socket, reinterpret_cast<sockaddr *>(&address), size), 0);
  ASSERT_EQ(getsockname(socket, reinterpret_cast<sockaddr *>(&address), &size), 0);
  const auto port = std::to_string(ntohs(address.sin_port));
  close(socket);
  const auto base = "/tmp/renode_connection_" + std::to_string(getpid());
  std::ofstream(base + ".repl") <<
    "can1: CAN.TypeCCan @ sysbus <0x40006400, +0x400>\n"
    "can2: CAN.TypeCCan @ sysbus <0x40006800, +0x400>\n    master: can1\n"
    "motors1: CAN.TypeCMotors @ sysbus 0xF0001000\n"
    "motors2: CAN.TypeCMotors @ sysbus 0xF0001100\n";
  std::ofstream(base + ".resc") << "include @" SIM_SOURCE_DIR "/renode/TypeC.cs\n"
    "mach create \"type-c\"\nmachine LoadPlatformDescription @" << base << ".repl\n"
    "sysbus.can1 WriteDoubleWord 0x200 0xe01\n"
    "sysbus.can1 WriteDoubleWord 0x21c 1\n"
    "sysbus.can2 WriteDoubleWord 0x0 0x8000\n"
    "emulation CreateExternalControlServer \"server\" " << port << '\n';
  sim::LaunchTree server("renode_connection",
    {SIM_RENODE_ROOT "/renode", "--disable-gui", "--plain", "--port", "-1", base + ".resc"},
    base + ".log");
  {
    sim::type_c::Board board(port);
    EXPECT_EQ(board.read_register(0x4000661c), 1U) << "CAN2 reset must preserve CAN1 filters";
    const std::array<uint8_t, 8> expected{1, 2, 3, 4, 5, 6, 7, 8};
    board.write(0xf0001010, expected.data(), expected.size());
    for (int request = 0; request < 24000; ++request) {
      std::array<uint8_t, 8> actual{};
      board.read(0xf0001010, actual.data(), actual.size());
      ASSERT_EQ(actual, expected) << "request " << request;
    }
  }
  server.stop();
  EXPECT_EQ(server.read_log().find("Stack overflow"), std::string::npos);
  std::filesystem::remove(base + ".resc");
  std::filesystem::remove(base + ".repl");
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
