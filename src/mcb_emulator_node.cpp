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

#include <algorithm>
#include <array>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <fcntl.h>
#include <poll.h>
#include <pty.h>
#include <signal.h>
#include <spawn.h>
#include <sys/stat.h>
#include <sys/wait.h>
#include <termios.h>
#include <unistd.h>

#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "sim/mcb_protocol.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/header.hpp"

extern char ** environ;

namespace sim
{
namespace
{
constexpr double kPi = 3.14159265358979323846;
constexpr double kHeadpitchLimit = 0.6;
constexpr int64_t kMaxCatchUpMs = 100;

[[noreturn]] void system_error(const std::string & operation)
{
  throw std::runtime_error(operation + ": " + std::strerror(errno));
}

void close_fd(int & fd)
{
  if (fd >= 0) {
    ::close(fd);
    fd = -1;
  }
}

double wrap(double angle)
{
  return std::atan2(std::sin(angle), std::cos(angle));
}

std::array<double, 2> rotate(double x, double y, double angle)
{
  const double c = std::cos(angle), s = std::sin(angle);
  return {c * x - s * y, s * x + c * y};
}

void append_u16(std::vector<uint8_t> & bytes, uint16_t value)
{
  bytes.push_back(value & 0xff);
  bytes.push_back(value >> 8);
}

void append_u32(std::vector<uint8_t> & bytes, uint32_t value)
{
  for (int shift = 0; shift < 32; shift += 8) {
    bytes.push_back((value >> shift) & 0xff);
  }
}

void append_float(std::vector<uint8_t> & bytes, double value)
{
  const float f = static_cast<float>(value);
  uint32_t bits;
  static_assert(sizeof(bits) == sizeof(f));
  std::memcpy(&bits, &f, sizeof(bits));
  append_u32(bytes, bits);
}

uint32_t read_u32(const uint8_t * data)
{
  return static_cast<uint32_t>(data[0]) | static_cast<uint32_t>(data[1]) << 8 |
         static_cast<uint32_t>(data[2]) << 16 | static_cast<uint32_t>(data[3]) << 24;
}

float read_float(const uint8_t * data)
{
  const uint32_t bits = read_u32(data);
  float value;
  std::memcpy(&value, &bits, sizeof(value));
  return value;
}

uint16_t referee_u16(int64_t value)
{
  if (value < 0 || value > 65535) {
    throw std::out_of_range("referee value outside uint16 range");
  }
  return static_cast<uint16_t>(value);
}

uint8_t referee_u8(int64_t value)
{
  if (value < 0 || value > 255) {
    throw std::out_of_range("referee value outside uint8 range");
  }
  return static_cast<uint8_t>(value);
}

struct Referee
{
  int64_t game_type = 4, game_stage = 4, stage_time_remaining = 300, robot_id = 107;
  int64_t current_hp = 400, max_hp = 400, hurt_armor_id = 0;
  bool restoration_zone = false, exchange_zone = false, central_buff_zone = false;
  bool shooter_power = true;

  void apply(const rclcpp::Parameter & p)
  {
    const auto & n = p.get_name();
    if (n == "game_type") {game_type = p.as_int();}
    else if (n == "game_stage") {game_stage = p.as_int();}
    else if (n == "stage_time_remaining") {stage_time_remaining = p.as_int();}
    else if (n == "robot_id") {robot_id = p.as_int();}
    else if (n == "current_hp") {current_hp = p.as_int();}
    else if (n == "max_hp") {max_hp = p.as_int();}
    else if (n == "hurt_armor_id") {hurt_armor_id = p.as_int();}
    else if (n == "restoration_zone") {restoration_zone = p.as_bool();}
    else if (n == "exchange_zone") {exchange_zone = p.as_bool();}
    else if (n == "central_buff_zone") {central_buff_zone = p.as_bool();}
    else if (n == "shooter_power") {shooter_power = p.as_bool();}
  }
};

std::vector<uint8_t> referee_frames(const Referee & ref)
{
  std::vector<uint8_t> result;
  const auto frame = [&result](uint16_t kind, const std::vector<uint8_t> & payload) {
      const auto packet = mcb_protocol::encode_frame(kind, payload);
      result.insert(result.end(), packet.begin(), packet.end());
    };
  std::vector<uint8_t> game{referee_u8(ref.game_type | (ref.game_stage << 4))};
  append_u16(game, referee_u16(ref.stage_time_remaining));
  game.insert(game.end(), 8, 0);
  frame(0x0001, game);
  frame(0x0206, {static_cast<uint8_t>(ref.hurt_armor_id & 3)});
  std::vector<uint8_t> robot{referee_u8(ref.robot_id), 1};
  for (const auto value : {ref.current_hp, ref.max_hp, int64_t{100}, int64_t{1000}, int64_t{240}}) {
    append_u16(robot, referee_u16(value));
  }
  robot.push_back(3 | static_cast<uint8_t>(ref.shooter_power) << 2);
  frame(0x0201, robot);
  std::vector<uint8_t> power;
  append_u16(power, 24000);
  append_u16(power, 0);
  append_float(power, 0.0);
  for (const uint16_t value : {60, 0, 0, 0}) {append_u16(power, value);}
  frame(0x0202, power);
  std::vector<uint8_t> zones;
  append_u32(zones, static_cast<uint32_t>(ref.restoration_zone) << 19 |
    static_cast<uint32_t>(ref.exchange_zone) << 20 |
    static_cast<uint32_t>(ref.central_buff_zone) << 23);
  frame(0x0209, zones);
  return result;
}

class PtyLink
{
public:
  explicit PtyLink(const std::string & link) : link_(link)
  {
    if (openpty(&master_, &slave_, nullptr, nullptr, nullptr) < 0) {system_error("openpty");}
    try {
      termios settings{};
      if (tcgetattr(slave_, &settings) < 0) {system_error("tcgetattr");}
      cfmakeraw(&settings);
      if (tcsetattr(slave_, TCSAFLUSH, &settings) < 0) {system_error("tcsetattr");}
      if (fcntl(master_, F_SETFL, fcntl(master_, F_GETFL) | O_NONBLOCK) < 0 ||
        fcntl(master_, F_SETFD, FD_CLOEXEC) < 0 || fcntl(slave_, F_SETFD, FD_CLOEXEC) < 0)
      {
        system_error("fcntl pty");
      }
      const char * name = ttyname(slave_);
      if (name == nullptr) {system_error("ttyname");}
      slave_name_ = name;
      struct stat info{};
      if (lstat(link_.c_str(), &info) == 0 && unlink(link_.c_str()) < 0) {
        system_error("unlink pty link");
      }
      if (symlink(slave_name_.c_str(), link_.c_str()) < 0) {system_error("symlink pty");}
    } catch (...) {
      close_fd(master_);
      close_fd(slave_);
      throw;
    }
  }

  ~PtyLink()
  {
    struct stat info{};
    if (lstat(link_.c_str(), &info) == 0 && S_ISLNK(info.st_mode)) {unlink(link_.c_str());}
    close_fd(master_);
    close_fd(slave_);
  }

  int master() const {return master_;}
  const std::string & link() const {return link_;}
  const std::string & slave_name() const {return slave_name_;}

private:
  int master_ = -1, slave_ = -1;
  std::string link_, slave_name_;
};

struct GzHardware
{
  bool ready = false, seen_odom = false, seen_joints = false;
  double chassis_yaw = 0.0, chassis_rate = 0.0;
  std::array<double, 2> pos{}, vel{}, yaw_joint{}, pitch_joint{};
  std::optional<std::array<double, 3>> boot;

  double turret_world_yaw() const {return chassis_yaw + yaw_joint[0];}

  void maybe_boot()
  {
    if (!boot && seen_odom && seen_joints) {
      boot = std::array<double, 3>{pos[0], pos[1], turret_world_yaw()};
      ready = true;
    }
  }

  void on_odom(const nav_msgs::msg::Odometry & msg)
  {
    const auto & q = msg.pose.pose.orientation;
    chassis_yaw = std::atan2(2.0 * (q.w * q.z + q.x * q.y),
      1.0 - 2.0 * (q.y * q.y + q.z * q.z));
    chassis_rate = msg.twist.twist.angular.z;
    pos = {msg.pose.pose.position.x, msg.pose.pose.position.y};
    vel = rotate(msg.twist.twist.linear.x, msg.twist.twist.linear.y, chassis_yaw);
    seen_odom = true;
    maybe_boot();
  }

  void on_joints(const sensor_msgs::msg::JointState & msg)
  {
    for (size_t i = 0; i < msg.name.size(); ++i) {
      if (msg.name[i] == "headlink" || msg.name[i] == "headpitch") {
        auto & joint = msg.name[i] == "headlink" ? yaw_joint : pitch_joint;
        joint = {msg.position.at(i), i < msg.velocity.size() ? msg.velocity[i] : 0.0};
      }
    }
    seen_joints = true;
    maybe_boot();
  }
};

class Firmware
{
public:
  Firmware(int master_fd, const std::string & requested_binary, const std::string & drive,
    bool auto_fire)
  {
    binary_ = requested_binary;
    if (binary_.empty()) {
      const char * override_path = std::getenv("MCB_FIRMWARE_BINARY");
      if (override_path != nullptr && *override_path != '\0') {binary_ = override_path;}
      else {
        binary_ = (std::filesystem::path(__FILE__).parent_path().parent_path().parent_path() /
          "firmware/MCBV3/MCB-project/build/sim/scons-release/MCB-project.elf").string();
      }
    }
    if (drive == "stop") {mode_ = 0;}
    else if (drive == "simple") {mode_ = 1;}
    else if (drive == "auto") {mode_ = 2;}
    else {throw std::invalid_argument("unknown MCB drive mode: " + drive);}
    if (!auto_fire) {mode_ |= 4;}
    if (access(binary_.c_str(), X_OK) != 0) {
      throw std::runtime_error("MCB firmware binary not built: " + binary_ +
        ". Run src/sim/tools/build_mcb_firmware.sh first, or set "
        "firmware_binary / MCB_FIRMWARE_BINARY.");
    }
    try {
      spawn(master_fd);
      if (read(4) != std::vector<uint8_t>{'M', 'C', 'B', '1'}) {
        throw std::runtime_error("MCB firmware hardware interface version mismatch");
      }
      const auto start = read(8);
      start_x_ = read_float(start.data());
      start_y_ = read_float(start.data() + 4);
    } catch (...) {
      close();
      throw;
    }
  }

  ~Firmware() {close();}
  const std::string & binary() const {return binary_;}
  uint32_t time_ms() const {return time_ms_;}

  std::array<double, 10> gz_readings(const GzHardware & hw, const Referee & ref) const
  {
    const double heading = ref.robot_id > 100 ? kPi : 0.0;
    const double sx = ref.robot_id > 100 ? start_x_ : -start_x_;
    const double c = std::cos(heading), s = std::sin(heading);
    const double dx = hw.pos[0] - sx, dy = hw.pos[1] - start_y_;
    const double forward = c * dx + s * dy, left = -s * dx + c * dy;
    const double vf = c * hw.vel[0] + s * hw.vel[1], vl = -s * hw.vel[0] + c * hw.vel[1];
    return {-left, forward, -vl, vf, hw.turret_world_yaw() - heading,
      hw.chassis_rate + hw.yaw_joint[1], hw.yaw_joint[0], hw.yaw_joint[1],
      hw.pitch_joint[0], hw.pitch_joint[1]};
  }

  struct Output
  {
    double yaw, pitch, right, forward, spin;
    std::vector<uint32_t> shots;
  };

  Output step(int64_t cycles, const std::array<double, 10> & readings, const Referee & ref)
  {
    if (cycles < 1 || cycles > 100) {
      throw std::invalid_argument("MCB hardware ticks must contain 1..100 cycles");
    }
    const auto packets = time_ms_ == 0 || time_ms_ / 100 != (time_ms_ + cycles) / 100 ?
      referee_frames(ref) : std::vector<uint8_t>{};
    std::vector<uint8_t> input;
    append_u32(input, static_cast<uint32_t>(cycles));
    for (double value : readings) {append_float(input, value);}
    append_u32(input, static_cast<uint32_t>(packets.size()));
    append_u32(input, mode_);
    input.insert(input.end(), packets.begin(), packets.end());
    size_t offset = 0;
    while (offset < input.size()) {
      const auto sent = ::write(input_fd_, input.data() + offset, input.size() - offset);
      if (sent < 0) {
        if (errno == EINTR) {continue;}
        system_error("write firmware hardware tick");
      }
      offset += static_cast<size_t>(sent);
    }
    const auto output = read(28);
    Output result{read_float(output.data()), read_float(output.data() + 4),
      read_float(output.data() + 8), read_float(output.data() + 12),
      read_float(output.data() + 16), {}};
    const uint32_t count = read_u32(output.data() + 20);
    time_ms_ = read_u32(output.data() + 24);
    const auto shots = read(static_cast<size_t>(count) * 4);
    for (size_t i = 0; i < count; ++i) {result.shots.push_back(read_u32(shots.data() + i * 4));}
    return result;
  }

private:
  void spawn(int master_fd)
  {
    int in[2], out[2];
    if (pipe2(in, O_CLOEXEC) < 0) {system_error("pipe firmware input");}
    if (pipe2(out, O_CLOEXEC) < 0) {
      ::close(in[0]);
      ::close(in[1]);
      system_error("pipe firmware output");
    }
    posix_spawn_file_actions_t actions;
    posix_spawn_file_actions_init(&actions);
    posix_spawn_file_actions_adddup2(&actions, in[0], STDIN_FILENO);
    posix_spawn_file_actions_adddup2(&actions, out[1], STDOUT_FILENO);
    posix_spawn_file_actions_addclose(&actions, in[0]);
    posix_spawn_file_actions_addclose(&actions, in[1]);
    posix_spawn_file_actions_addclose(&actions, out[0]);
    posix_spawn_file_actions_addclose(&actions, out[1]);
    // dup2 to the same fd clears CLOEXEC in the spawned child's file actions.
    posix_spawn_file_actions_adddup2(&actions, master_fd, master_fd);
    std::vector<std::string> environment;
    for (char ** p = environ; *p != nullptr; ++p) {
      if (std::strncmp(*p, "MCB_UART_FD=", 12) != 0) {environment.emplace_back(*p);}
    }
    environment.push_back("MCB_UART_FD=" + std::to_string(master_fd));
    std::vector<char *> envp;
    for (auto & entry : environment) {envp.push_back(entry.data());}
    envp.push_back(nullptr);
    char * argv[] = {binary_.data(), nullptr};
    posix_spawnattr_t attributes;
    posix_spawnattr_init(&attributes);
    sigset_t defaults;
    sigemptyset(&defaults);
    sigaddset(&defaults, SIGPIPE);
    posix_spawnattr_setsigdefault(&attributes, &defaults);
    posix_spawnattr_setflags(&attributes, POSIX_SPAWN_SETSIGDEF);
    const int error = posix_spawn(
      &pid_, binary_.c_str(), &actions, &attributes, argv, envp.data());
    posix_spawnattr_destroy(&attributes);
    posix_spawn_file_actions_destroy(&actions);
    ::close(in[0]);
    ::close(out[1]);
    if (error != 0) {
      ::close(in[1]);
      ::close(out[0]);
      pid_ = -1;
      throw std::runtime_error("spawn firmware: " + std::string(std::strerror(error)));
    }
    input_fd_ = in[1];
    output_fd_ = out[0];
  }

  std::vector<uint8_t> read(size_t count)
  {
    std::vector<uint8_t> data(count);
    size_t offset = 0;
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
    while (offset < count) {
      const auto remaining = deadline - std::chrono::steady_clock::now();
      const auto timeout = std::chrono::duration_cast<std::chrono::milliseconds>(remaining).count();
      pollfd descriptor{output_fd_, POLLIN, 0};
      if (timeout <= 0) {
        throw std::runtime_error("MCB firmware stopped responding to hardware ticks");
      }
      const int ready = poll(&descriptor, 1, static_cast<int>(timeout));
      if (ready < 0 && errno == EINTR) {continue;}
      if (ready < 0) {system_error("poll firmware");}
      if (ready == 0) {
        throw std::runtime_error("MCB firmware stopped responding to hardware ticks");
      }
      const auto received = ::read(output_fd_, data.data() + offset, count - offset);
      if (received < 0 && errno == EINTR) {continue;}
      if (received < 0) {system_error("read firmware");}
      if (received == 0) {
        int status = 0;
        const auto exited = waitpid(pid_, &status, WNOHANG);
        std::string code = "None";
        if (exited == pid_) {
          pid_ = -1;
          code = std::to_string(WIFEXITED(status) ? WEXITSTATUS(status) : -WTERMSIG(status));
        }
        throw std::runtime_error("MCB firmware exited (" + code + ")");
      }
      offset += static_cast<size_t>(received);
    }
    return data;
  }

  void close()
  {
    close_fd(input_fd_);
    if (pid_ > 0) {
      const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
      int status = 0;
      bool done = false;
      while (std::chrono::steady_clock::now() < deadline) {
        const auto result = waitpid(pid_, &status, WNOHANG);
        if (result == pid_ || (result < 0 && errno != EINTR)) {done = true; break;}
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
      }
      if (!done) {
        kill(pid_, SIGKILL);
        while (waitpid(pid_, &status, 0) < 0 && errno == EINTR) {}
      }
      pid_ = -1;
    }
    close_fd(output_fd_);
  }

  std::string binary_;
  uint32_t mode_ = 0, time_ms_ = 0;
  double start_x_ = 0.0, start_y_ = 0.0;
  pid_t pid_ = -1;
  int input_fd_ = -1, output_fd_ = -1;
};
}  // namespace

class McbEmulator : public rclcpp::Node
{
public:
  explicit McbEmulator(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("mcb_emulator", options)
  {
    const auto device_link = declare_parameter<std::string>("device_link", "/tmp/mcb_emulator_pty");
    const auto drive = declare_parameter<std::string>("drive", "stop");
    const bool auto_fire = declare_parameter<bool>("auto_fire", true);
    const auto binary = declare_parameter<std::string>("firmware_binary", "");
    const int64_t batch_ms = declare_parameter<int64_t>("batch_ms", 5);
    const double stats_period = declare_parameter<double>("stats_period_s", 5.0);
    for (const auto & entry : std::vector<std::pair<std::string, int64_t>>{
        {"game_type", 4}, {"game_stage", 4}, {"stage_time_remaining", 300}, {"robot_id", 107},
        {"current_hp", 400}, {"max_hp", 400}, {"hurt_armor_id", 0}})
    {
      declare_parameter<int64_t>(entry.first, entry.second);
      ref_.apply(get_parameter(entry.first));
    }
    for (const auto & entry : std::vector<std::pair<std::string, bool>>{
        {"restoration_zone", false}, {"exchange_zone", false}, {"central_buff_zone", false},
        {"shooter_power", true}})
    {
      declare_parameter<bool>(entry.first, entry.second);
      ref_.apply(get_parameter(entry.first));
    }
    params_callback_ = add_on_set_parameters_callback(
      [this](const std::vector<rclcpp::Parameter> & params) {
        for (const auto & p : params) {ref_.apply(p);}
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        return result;
      });
    pty_ = std::make_unique<PtyLink>(device_link);
    firmware_ = std::make_unique<Firmware>(pty_->master(), binary, drive, auto_fire);
    pan_pub_ = create_publisher<std_msgs::msg::Float64>("/head_pan_cmd", 10);
    pitch_pub_ = create_publisher<std_msgs::msg::Float64>("/head_pitch_cmd", 10);
    cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    shot_pub_ = create_publisher<std_msgs::msg::Header>("~/shot", 10);
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>("/sim/raw_odom", 10,
      [this](nav_msgs::msg::Odometry::ConstSharedPtr msg) {hw_.on_odom(*msg);});
    joints_sub_ = create_subscription<sensor_msgs::msg::JointState>("/sim/raw_joint_states", 10,
      [this](sensor_msgs::msg::JointState::ConstSharedPtr msg) {hw_.on_joints(*msg);});
    tick_timer_ = create_timer(std::chrono::duration<double>(batch_ms / 1000.0),
      [this] {tick();});
    stats_timer_ = create_timer(std::chrono::duration<double>(stats_period), [this] {
        RCLCPP_INFO(get_logger(), "MCB firmware running: %u ms", firmware_->time_ms());
      });
    RCLCPP_INFO(get_logger(), "MCB firmware %s on %s -> %s, drive=%s, auto_fire=%s",
      firmware_->binary().c_str(), pty_->link().c_str(), pty_->slave_name().c_str(),
      drive.c_str(), auto_fire ? "True" : "False");
  }

private:
  void tick()
  {
    const int64_t now_ms = get_clock()->now().nanoseconds() / 1000000;
    if (!hw_.ready) {last_ms_ = now_ms; return;}
    const int64_t cycles = last_ms_ ? std::min(now_ms - *last_ms_, kMaxCatchUpMs) : 1;
    last_ms_ = now_ms;
    if (cycles <= 0) {return;}
    const int64_t to_sim_ms = now_ms - cycles - firmware_->time_ms();
    const auto output = firmware_->step(cycles, firmware_->gz_readings(hw_, ref_), ref_);
    const double heading = ref_.robot_id > 100 ? kPi : 0.0;
    const double world_yaw = output.yaw + heading - (*hw_.boot)[2];
    const double desired = world_yaw + (*hw_.boot)[2] - hw_.chassis_yaw;
    std_msgs::msg::Float64 pan, pitch;
    pan.data = hw_.yaw_joint[0] + wrap(desired - hw_.yaw_joint[0]);
    pitch.data = std::clamp(output.pitch, -kHeadpitchLimit, kHeadpitchLimit);
    for (const auto t_ms : output.shots) {
      std_msgs::msg::Header header;
      header.stamp = rclcpp::Time((static_cast<int64_t>(t_ms) + to_sim_ms) * 1000000);
      header.frame_id = "muzzle";
      shot_pub_->publish(header);
    }
    pan_pub_->publish(pan);
    pitch_pub_->publish(pitch);
    geometry_msgs::msg::Twist twist;
    twist.linear.x = output.forward;
    twist.linear.y = -output.right;
    twist.angular.z = output.spin;
    cmd_vel_pub_->publish(twist);
  }

  Referee ref_;
  GzHardware hw_;
  std::optional<int64_t> last_ms_;
  std::unique_ptr<PtyLink> pty_;
  std::unique_ptr<Firmware> firmware_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr params_callback_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pan_pub_, pitch_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Publisher<std_msgs::msg::Header>::SharedPtr shot_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joints_sub_;
  rclcpp::TimerBase::SharedPtr tick_timer_, stats_timer_;
};
}  // namespace sim

int main(int argc, char ** argv)
{
  signal(SIGPIPE, SIG_IGN);
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<sim::McbEmulator>());
  } catch (const std::exception & error) {
    RCLCPP_ERROR(rclcpp::get_logger("mcb_emulator"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
