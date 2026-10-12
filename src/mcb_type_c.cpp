// Copyright 2026 Thornbots. SPDX-License-Identifier: Apache-2.0
#include <fstream>
#include <iostream>
#include <memory>
#include <sstream>
#include <netinet/in.h>
#include <sys/socket.h>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include "sim/process.hpp"
#include "sim/renode_board.hpp"

namespace {
namespace fs = std::filesystem;
std::string read_file(const fs::path &path) {
  std::ifstream input(path);
  if (!input) throw std::runtime_error("Cannot read " + path.string());
  return {std::istreambuf_iterator<char>(input), {}};
}
void write_file(const fs::path &path, const std::string &data) {
  std::ofstream output(path);
  output << data;
  if (!output) throw std::runtime_error("Cannot write " + path.string());
}
std::string setting(const char *name, const std::string &fallback = {}) {
  const char *value = std::getenv(name);
  return value && *value ? value : fallback;
}
unsigned available_port() {
  const int fd = socket(AF_INET, SOCK_STREAM | SOCK_CLOEXEC, 0);
  if (fd < 0) sim::mcb_firmware::system_error("socket");
  sockaddr_in address{};
  address.sin_family = AF_INET;
  address.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
  socklen_t size = sizeof(address);
  const bool ok = bind(fd, reinterpret_cast<sockaddr *>(&address), size) == 0 &&
      getsockname(fd, reinterpret_cast<sockaddr *>(&address), &size) == 0;
  close(fd);
  if (!ok) sim::mcb_firmware::system_error("reserve Renode port");
  return ntohs(address.sin_port);
}
struct Input {
  uint32_t cycles;
  float readings[10];
  uint32_t referee_bytes, mode;
  sim::mcb_motor::Feedback feedback;
  float external[3];
};
struct Output {
  float yaw = 0, pitch = 0, right = 0, forward = 0, spin = 0;
  uint32_t shots = 0, time_ms = 0;
  sim::mcb_motor::Commands motors{};
};
static_assert(sizeof(Input) == 144 && sizeof(Output) == 48);
class Emulator {
public:
  explicit Emulator(const std::string &elf) {
    const fs::path root = setting("MCB_RENODE_ROOT", SIM_RENODE_ROOT);
    const fs::path models = setting("MCB_RENODE_MODELS",
        ament_index_cpp::get_package_share_directory("sim") + "/renode");
    char directory[] = "/tmp/mcb-type-c-XXXXXX";
    const char *created = mkdtemp(directory);
    if (!created) sim::mcb_firmware::system_error("mkdtemp");
    directory_ = created;
    auto platform = read_file(root / "platforms/cpus/stm32f4.repl");
    const std::string original = "i2c2: I2C.STM32F1_I2C";
    const auto i2c = platform.find(original);
    if (i2c == std::string::npos) throw std::runtime_error("Unexpected Renode STM32 platform");
    platform.replace(i2c, original.size(), "i2c2: I2C.TypeCI2C");
    for (size_t can = platform.find("CAN.STMCAN"); can != std::string::npos;
         can = platform.find("CAN.STMCAN", can))
      platform.replace(can, std::string("CAN.STMCAN").size(), "CAN.TypeCCan");
    auto devices = read_file(models / "TypeC.repl");
    const auto include = devices.find("using \"");
    devices.erase(include, devices.find('\n', include) - include + 1);
    write_file(directory_ / "board.repl", platform + "\n" + devices);
    sim::capture({"arm-none-eabi-objcopy", "-O", "binary", elf,
                  (directory_ / "flash.bin").string()});
    if (!fs::exists(directory_ / "flash.bin")) throw std::runtime_error("ELF extraction failed");
    const auto port = std::to_string(available_port());
    const auto script = directory_ / "board.resc";
    write_file(script,
        "logLevel 2\ninclude @" + (models / "TypeC.cs").string() +
        "\ninclude @" + (models / "TypeCI2C.cs").string() +
        "\nmach create \"type-c\"\nmachine LoadPlatformDescription @" +
        (directory_ / "board.repl").string() + "\nsysbus LoadSymbolsFrom @" + elf +
        " true\nsysbus LoadBinary @" + (directory_ / "flash.bin").string() +
        " 0x08000000\nemulation CreateCANHub \"bus1\"\n"
        "emulation CreateCANHub \"bus2\"\nconnector Connect sysbus.can1 bus1\n"
        "connector Connect sysbus.motors1 bus1\nconnector Connect sysbus.can2 bus2\n"
        "connector Connect sysbus.motors2 bus2\nsysbus.adc1 FeedSample 3584 6 1000000\n"
        "emulation CreateExternalControlServer \"server\" " + port + "\n");
    process_ = std::make_unique<sim::LaunchTree>("renode",
        std::vector<std::string>{(root / "renode").string(), "--disable-gui", "--plain",
                                  "--port", "-1", script.string()},
        (directory_ / "renode.log").string());
    for (int attempt = 0; attempt < 100; ++attempt) {
      if (!process_->alive()) throw std::runtime_error(process_->read_log());
      if (sim::type_c::listening(std::stoul(port))) {
        board_ = std::make_unique<sim::type_c::Board>(port);
        break;
      }
      if (attempt == 99) throw std::runtime_error("Renode control-server startup timeout");
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
  }
  sim::type_c::Board &board() { return *board_; }
  const fs::path &directory() const { return directory_; }
private:
  fs::path directory_;
  std::unique_ptr<sim::LaunchTree> process_;
  std::unique_ptr<sim::type_c::Board> board_;
};
}

int main() {
  // LaunchTree diagnostics must not enter the binary hardware pipe.
  std::cout.rdbuf(std::cerr.rdbuf());
  try {
    const auto elf = setting("MCB_BOARD_ELF");
    const auto uart_fd = setting("MCB_UART_FD");
    if (elf.empty() || uart_fd.empty()) throw std::runtime_error("MCB_BOARD_ELF and MCB_UART_FD are required");
    const int uart = std::stoi(uart_fd);
    if (fcntl(uart, F_SETFL, O_NONBLOCK) < 0) sim::mcb_firmware::system_error("UART fcntl");
    Emulator emulator(elf);
    auto &board = emulator.board();
    // The board deliberately requires an absent remote during its power-on guard.
    board.advance(3500000);
    sim::mcb_motor::Plant plant;
    std::array<double, 10> readings{};
    board.sensors(readings);
    board.feedback(plant.feedback(readings, 0, 0, 0, 0));
    board.remote(false);
    sim::mcb_firmware::Referee referee;
    referee.game_stage = 2;
    sim::mcb_protocol::DJISerial parser;
    std::optional<sim::mcb_protocol::Pose> initial;
    // Stationary calibration: inputs are constant, so advance 10 ms at a time.
    for (unsigned step = 0; step < 600 && !initial; ++step) {
      if (step % 10 == 0) board.referee(sim::mcb_firmware::referee_frames(referee));
      board.advance(10000);
      for (const auto &frame : parser.feed(board.uart())) {
        if (frame.msg_type == sim::mcb_protocol::kPose)
          initial = sim::mcb_protocol::unpack<sim::mcb_protocol::Pose>(frame.payload);
      }
    }
    board.motors();
    if (!initial || board.frames() == 0 || parser.errors().crc8 || parser.errors().crc16)
      throw std::runtime_error("Board failed calibration/UART/CAN bring-up; see " + emulator.directory().string());
    std::cerr << "Type C calibrated; CAN frames=" << board.frames()
              << " start=" << initial->x << ',' << initial->y
              << " logs=" << emulator.directory() << '\n';
    std::fwrite("MCB2", 1, 4, stdout);
    const float start[] = {initial->x, initial->y};
    std::fwrite(start, sizeof(float), 2, stdout);
    std::fflush(stdout);
    Output output;
    sim::mcb_motor::ProjectileExit exit;
    Input input;
    uint32_t remote_mode = 0;
    while (std::fread(&input, sizeof(input), 1, stdin) == 1) {
      if (!input.cycles || input.cycles > 100 || input.referee_bytes > 4096 || input.mode > 5 ||
          (input.mode & 3) > 1)
        throw std::runtime_error("Type C supports stop and simple auto-drive modes only");
      std::vector<uint8_t> frames(input.referee_bytes);
      if (!frames.empty() && std::fread(frames.data(), 1, frames.size(), stdin) != frames.size())
        throw std::runtime_error("Truncated referee input");
      board.referee(frames);
      std::copy(std::begin(input.readings), std::end(input.readings), readings.begin());
      board.sensors(readings);
      board.feedback(input.feedback);
      if (input.mode != remote_mode) {
        board.remote((input.mode & 3) == 1, !(input.mode & 4));
        remote_mode = input.mode;
      }
      std::array<uint8_t, 1024> bytes{};
      for (ssize_t count; (count = ::read(uart, bytes.data(), bytes.size())) > 0;)
        board.jetson(bytes.data(), count);
      board.advance(1000 * input.cycles);
      const auto outgoing = board.uart();
      size_t offset = 0;
      while (offset < outgoing.size()) {
        const auto sent = ::write(uart, outgoing.data() + offset, outgoing.size() - offset);
        if (sent < 0 && errno == EINTR) continue;
        if (sent <= 0) throw std::runtime_error("Jetson PTY stopped accepting board traffic");
        offset += sent;
      }
      // Feedback is constant within a batch, so its exits belong to the batch's first tick.
      const auto exits = exit.update(input.feedback[2]);
      std::vector<uint32_t> shots(exits, output.time_ms + 1);
      output.time_ms += input.cycles;
      output.motors = board.motors();
      output.shots = shots.size();
      std::fwrite(&output, sizeof(output), 1, stdout);
      if (!shots.empty()) std::fwrite(shots.data(), sizeof(uint32_t), shots.size(), stdout);
      std::fflush(stdout);
    }
    return std::feof(stdin) ? 0 : 2;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 2;
  }
}
