// Copyright 2026 Thornbots
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at http://www.apache.org/licenses/LICENSE-2.0
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and limitations under the License.
#include <sys/socket.h>
#include <chrono>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <map>
#include <memory>
#include <numeric>
#include <thread>
#include <gtest/gtest.h>
#include <nlohmann/json.hpp>
#include "hosted/mapping_control.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "dji_serial_bridge/msg/ping.hpp"
#include "dji_serial_bridge/msg/robot_pose.hpp"
#include "dji_serial_bridge/msg/ref_sys_status.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sim/mcb_firmware.hpp"
#include "sim/process.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace fs = std::filesystem;
namespace wire = sim::mcb_protocol;
namespace fixture = sim::mcb_firmware;
using Clock = std::chrono::steady_clock;
using Json = nlohmann::json;

namespace {
double seconds() { return std::chrono::duration<double>(Clock::now().time_since_epoch()).count(); }

class MmmSuite : public ::testing::Test {
protected:
  fs::path directory;
  std::unique_ptr<fixture::PtyLink> mcb, jetson;
  std::unique_ptr<fixture::Firmware> firmware;
  std::unique_ptr<sim::LaunchTree> simulator, robot, bridge;
  int control[2] = {-1, -1};
  hosted::MappingStatus status{};
  fixture::Firmware::Output output{};
  fixture::Referee referee;
  fixture::GzHardware hardware;
  std::shared_ptr<rclcpp::Node> node;
  std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> executor;
  std::unique_ptr<tf2_ros::Buffer> buffer;
  std::unique_ptr<tf2_ros::TransformListener> listener;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr drive;
  std::vector<rclcpp::SubscriptionBase::SharedPtr> subscriptions;
  wire::DJISerial parsers[2];
  std::map<int, int> counts[2], drops[2], corruptions[2];
  size_t telemetryFrames[2] = {};
  std::map<int, double> pending[2];
  std::map<int, std::vector<double>> latencies[2];
  std::map<int, double> rosPending;
  std::map<int, std::vector<double>> rosLatencies;
  std::vector<std::pair<int, double>> firmwareAcks;
  std::vector<double> loop[2];
  std::vector<double> periods[2];
  size_t poseCount = 0, refCount = 0;
  bool stall = false, driveSim = true;
  int phase = 0;
  Json report = Json::object();
  std::ofstream trace;

  void SetUp() override {
    const char *root = std::getenv("MMM_EVIDENCE_DIR");
    ASSERT_NE(root, nullptr) << "set MMM_EVIDENCE_DIR to a persistent directory";
    directory = fs::path(root) / ::testing::UnitTest::GetInstance()->current_test_info()->name();
    ASSERT_TRUE(fs::create_directories(directory)) << "use a fresh evidence directory";
    trace.open(directory / "frames.jsonl");
    mcb = std::make_unique<fixture::PtyLink>((directory / "mcb").string());
    jetson = std::make_unique<fixture::PtyLink>((directory / "jetson").string());
    ASSERT_EQ(fcntl(mcb->slave(), F_SETFL, O_NONBLOCK), 0);
    ASSERT_EQ(socketpair(AF_UNIX, SOCK_SEQPACKET | SOCK_NONBLOCK, 0, control), 0);
    node = std::make_shared<rclcpp::Node>("mmm_uart_test",
      rclcpp::NodeOptions().parameter_overrides({rclcpp::Parameter("use_sim_time", true)}));
    executor = std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
    executor->add_node(node);
    drive = node->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    subscriptions.push_back(node->create_subscription<nav_msgs::msg::Odometry>(
      "/sim/raw_odom", rclcpp::SensorDataQoS(), [&](nav_msgs::msg::Odometry::ConstSharedPtr m) {
        hardware.on_odom(*m);
      }));
    subscriptions.push_back(node->create_subscription<sensor_msgs::msg::JointState>(
      "/sim/raw_joint_states", rclcpp::SensorDataQoS(), [&](sensor_msgs::msg::JointState::ConstSharedPtr m) {
        hardware.on_joints(*m);
      }));
    subscriptions.push_back(node->create_subscription<dji_serial_bridge::msg::RobotPose>(
      "/dji_serial_bridge/pose", rclcpp::SensorDataQoS(), [&](dji_serial_bridge::msg::RobotPose::ConstSharedPtr) {
        ++poseCount;
      }));
    subscriptions.push_back(node->create_subscription<dji_serial_bridge::msg::RefSysStatus>(
      "/dji_serial_bridge/ref_sys", rclcpp::SensorDataQoS(), [&](dji_serial_bridge::msg::RefSysStatus::ConstSharedPtr) {
        ++refCount;
      }));
    subscriptions.push_back(node->create_subscription<dji_serial_bridge::msg::Ping>(
      "/dji_serial_bridge/ping_to_mcb", 10, [&](dji_serial_bridge::msg::Ping::ConstSharedPtr m) {
        if (m->number % 2 == 0) rosPending[m->number] = seconds();
      }));
    subscriptions.push_back(node->create_subscription<dji_serial_bridge::msg::Ping>(
      "/dji_serial_bridge/ping_from_mcb", 10, [&](dji_serial_bridge::msg::Ping::ConstSharedPtr m) {
        if (rosPending.count(m->number)) {
          rosLatencies[m->number].push_back(seconds() - rosPending[m->number]);
          rosPending.erase(m->number);
        }
      }));
    buffer = std::make_unique<tf2_ros::Buffer>(node->get_clock());
    listener = std::make_unique<tf2_ros::TransformListener>(*buffer);
    restartFirmware();
    restartBridge();
  }

  void TearDown() override {
    summaries();
    if (drive) { drive->publish(geometry_msgs::msg::Twist{}); }
    robot.reset();
    simulator.reset();
    bridge.reset();
    firmware.reset();
    std::vector<std::string> checker{"ros2", "run", "sim", "check_bench_log"};
    for (const auto &entry : fs::directory_iterator(directory))
      if (entry.path().extension() == ".log") checker.push_back(entry.path().string());
    if (checker.size() > 4) {
      EXPECT_NO_THROW(std::ofstream(directory / "log-check.txt") << sim::capture(checker, 20));
    }
    if (control[0] >= 0) close(control[0]);
    if (control[1] >= 0) close(control[1]);
    if (!directory.empty()) {
      std::ofstream(directory / "measurements.json") << report.dump(2) << '\n';
      std::cout << "MMM MEASUREMENTS " << report.dump() << '\n';
    }
  }

  void restartFirmware() {
    firmware.reset();
    char data[256];
    while (recv(control[0], data, sizeof(data), MSG_DONTWAIT) > 0) {}
    setenv("MCB_MMM_CONTROL_FD", std::to_string(control[1]).c_str(), 1);
    firmware = std::make_unique<fixture::Firmware>(mcb->master(), "", "stop", true);
    unsetenv("MCB_MMM_CONTROL_FD");
    status = {};
  }

  void restartBridge() {
    bridge.reset();
    bridge = std::make_unique<sim::LaunchTree>("bridge", std::vector<std::string>{
      "ros2", "run", "dji_serial_bridge", "dji_serial_bridge_node", "--ros-args",
      "-p", "device:=" + jetson->link(), "-p", "debug_log:=false", "-p", "read_poll_ms:=1",
      "-p", "use_sim_time:=true"},
      (directory / ("bridge-" + std::to_string(++phase) + ".log")).string());
  }

  void command(uint32_t action, float right = 0, float forward = 0, float spin = 0) {
    const hosted::MappingInput input{action, right, forward, spin};
    ASSERT_EQ(send(control[0], &input, sizeof(input), MSG_NOSIGNAL), sizeof(input));
  }

  void relay(int direction) {
    if (direction == 0 && stall) return;
    std::vector<uint8_t> bytes;
    if (direction == 0) {
      uint8_t data[4096];
      ssize_t count;
      while ((count = read(mcb->slave(), data, sizeof(data))) > 0) bytes.insert(bytes.end(), data, data + count);
    } else { bytes = jetson->read_data(); }
    for (const auto &frame : parsers[direction].feed(bytes)) {
      if (frame.msg_type == wire::kPose || frame.msg_type == wire::kRefSys) ++telemetryFrames[direction];
      auto packet = wire::encode_frame(frame.msg_type, frame.payload);
      if (frame.msg_type == wire::kPing) {
        const int number = frame.payload.at(0);
        ++counts[direction][number];
        const std::string fault = drops[direction][number] > 0 ? "drop" :
          (corruptions[direction][number] > 0 ? "crc" : "none");
        trace << Json{{"wall_s", seconds()}, {"direction", direction}, {"number", number},
          {"fault", fault}, {"state", status.state}}.dump() << '\n';
        trace.flush();
        if (fault == "drop") { --drops[direction][number]; continue; }
        if (fault == "crc") { --corruptions[direction][number]; packet.back() ^= 0x80; }
        if (fault == "none") {
          const int origin = number == 1 || number == 3 ? 0 : 1;
          if (direction == origin) { pending[direction][number] = seconds(); }
          else if (pending[origin].count(number)) {
            if (origin == 0) firmwareAcks.emplace_back(number, pending[origin][number]);
            else latencies[origin][number].push_back(seconds() - pending[origin][number]);
            pending[origin].erase(number);
          }
        }
      }
      if (direction == 0) { jetson->write_data(packet); }
      else { ASSERT_EQ(write(mcb->slave(), packet.data(), packet.size()), static_cast<ssize_t>(packet.size())); }
    }
  }

  void tick(int cycles = 5, bool paced = true) {
    const auto start = Clock::now();
    executor->spin_some();
    relay(1);
    auto readings = hardware.ready ? firmware->gz_readings(hardware, referee) : std::array<double, 10>{};
    output = firmware->step(cycles, readings, referee);
    ASSERT_EQ(recv(control[0], &status, sizeof(status), 0), sizeof(status));
    for (const auto &[number, sent] : firmwareAcks) latencies[0][number].push_back(seconds() - sent);
    firmwareAcks.clear();
    loop[status.state != 0].push_back(status.loopUs);
    periods[status.state != 0].push_back(status.periodUs);
    relay(0);
    if (driveSim) {
      geometry_msgs::msg::Twist twist;
      twist.linear.x = output.forward;
      twist.linear.y = -output.right;
      twist.angular.z = output.spin;
      drive->publish(twist);
    }
    if (status.state != 0) {
      EXPECT_EQ(output.motors[2], 0);
      EXPECT_EQ(output.motors[4], 0);
      EXPECT_EQ(output.motors[5], 0);
      EXPECT_TRUE(output.shots.empty());
      EXPECT_NEAR(output.pitch, 0, 1e-6);
      EXPECT_NEAR(status.headForwardError, 0, 1e-4);
      EXPECT_LE(std::hypot(output.right, output.forward), 0.400001);
      if (status.state == 1 || status.state == 4) { EXPECT_NEAR(std::hypot(output.right, output.forward), 0, 1e-6); }
    }
    if (paced) std::this_thread::sleep_until(start + std::chrono::milliseconds(cycles));
  }

  bool wait(const std::function<bool()> &predicate, double limit = 30) {
    const auto end = seconds() + limit;
    while (seconds() < end) {
      tick();
      if (::testing::Test::HasFatalFailure()) return false;
      if (predicate()) return true;
    }
    ADD_FAILURE() << "deadline state=" << status.state << "\n" << (robot ? robot->read_log() : "")
      << "\n" << bridge->read_log();
    return false;
  }

  void dwell(double duration) {
    const auto end = seconds() + duration;
    ASSERT_TRUE(wait([&] { return seconds() >= end; }, duration + 2));
  }

  void launchStack(bool failSetup = false) {
    simulator = std::make_unique<sim::LaunchTree>("sim", std::vector<std::string>{
      "ros2", "launch", "sim", "sim.launch.py", "gui:=false", "rviz:=false", "foxglove:=false",
      "pose_emulator:=false", "model:=sentry_v2", "x:=4.625", "yaw:=1.5707963267948966",
      "real_time_factor:=1.0", "odom_noise_enabled:=false"}, (directory / "sim.log").string());
    robot = std::make_unique<sim::LaunchTree>("robot", std::vector<std::string>{
      "ros2", "launch", "thornbots_pkg", "auto.launch.py", "real_hardware:=false",
      std::string("localization_mode:=") + (failSetup ? "none" : "mapping"),
      "load_map:=false", "autosave_map:=false", "use_rf2o:=false",
      "mmm_room:=true", "map_save_dir:=" + (directory / "maps").string(),
      "mmm_timeout_s:=45.0", "enable_target_selector:=false", "enable_target_tracker:=false",
      "enable_cv_target_bridge:=false", "dds_transport:=udp_only"}, (directory / "robot.log").string());
    ASSERT_TRUE(wait([&] { return hardware.ready && poseCount > 10 && counts[1][0] > 0; }, 60));
    dwell(5);
    ASSERT_TRUE(hardware.boot.has_value());
    EXPECT_NEAR((*hardware.boot)[2], fixture::kStartYaw, 0.05);
    ASSERT_TRUE(wait([&] { return buffer->canTransform("odom", "root", tf2::TimePointZero); }));
    const auto root = buffer->lookupTransform("odom", "root", tf2::TimePointZero);
    EXPECT_NEAR(root.transform.rotation.z, 0.0, 1e-8);
    EXPECT_NEAR(root.transform.rotation.w, 1.0, 1e-8);
    const auto publishers = node->get_publishers_info_by_topic("/dji_serial_bridge/ping_to_mcb");
    ASSERT_EQ(publishers.size(), 1U);
    EXPECT_EQ(publishers.front().node_name(), "mmm");
    report["ping_publishers"] = publishers.size();
  }

  size_t sessions() const {
    size_t count = 0;
    if (fs::exists(directory / "maps")) for (const auto &entry : fs::directory_iterator(directory / "maps"))
      if (fs::exists(entry.path() / "boot.json")) ++count;
    return count;
  }

  void summaries() {
    for (int direction = 0; direction < 2; ++direction) {
      for (const auto &[number, count] : counts[direction]) {
        auto &cell = report[direction == 0 ? "mcb_to_jetson" : "jetson_to_mcb"][std::to_string(number)];
        cell["frames"] = count;
        const auto &samples = latencies[direction][number];
        if (!samples.empty()) {
          cell["round_trip_max_ms"] = 1000 * *std::max_element(samples.begin(), samples.end());
          cell["round_trips"] = samples.size();
          EXPECT_LT(cell["round_trip_max_ms"].get<double>(), 100.0);
        }
      }
    }
    for (int active = 0; active < 2; ++active) {
      if (loop[active].empty()) continue;
      const auto mean = std::accumulate(loop[active].begin(), loop[active].end(), 0.0) / loop[active].size();
      report[active ? "active_loop_us" : "inactive_loop_us"] = mean;
      const auto period = std::accumulate(periods[active].begin(), periods[active].end(), 0.0) / periods[active].size();
      report[active ? "active_period_us" : "inactive_period_us"] = period;
    }
    report["telemetry_pose"] = poseCount;
    report["telemetry_ref_sys"] = refCount;
    report["state_transitions_since_firmware_boot"] = status.transitions;
    report["simulated_control_period_ms"] = 1;
    for (const auto &[number, samples] : rosLatencies) {
      if (samples.empty()) continue;
      const auto maximum = 1000 * *std::max_element(samples.begin(), samples.end());
      report["ros_round_trip_max_ms"][std::to_string(number)] = maximum;
      EXPECT_LT(maximum, 100.0);
    }
  }
};

TEST_F(MmmSuite, FullFlowLossRestartAndSave) {
  launchStack();
  const auto p0 = poseCount, r0 = refCount;
  const auto t0 = seconds();
  dwell(3);
  report["pose_hz"] = (poseCount - p0) / (seconds() - t0);
  report["ref_sys_hz"] = (refCount - r0) / (seconds() - t0);
  EXPECT_GT(report["pose_hz"].get<double>(), 75);
  EXPECT_GT(report["ref_sys_hz"].get<double>(), 8);
  for (int direction = 0; direction < 2; ++direction)
    for (int number : {1, 2, 3, 4}) { drops[direction][number] = 1; corruptions[direction][number] = 1; }
  command(1, 1, 1);
  ASSERT_TRUE(wait([&] { return status.state == 1; }));
  EXPECT_EQ(std::hypot(output.right, output.forward), 0);
  command(0);
  ASSERT_TRUE(wait([&] { return status.state == 2; }, 60));
  const auto firstSessions = sessions();
  ASSERT_EQ(firstSessions, 1U);
  command(0, 1, 1);
  dwell(0.2);
  report["ready_command_speed_mps"] = std::hypot(output.right, output.forward);
  report["ready_observed_speed_mps"] = std::hypot(hardware.vel[0], hardware.vel[1]);
  EXPECT_LE(report["ready_observed_speed_mps"].get<double>(), 0.43);
  EXPECT_NEAR(report["ready_command_speed_mps"].get<double>(), 0.4, 1e-5);
  command(0);
  tick();
  std::vector<uint32_t> beepStarts;
  bool previousBeep = status.readyBeep;
  const auto beepEnd = firmware->time_ms() + 2200;
  while (firmware->time_ms() < beepEnd) {
    tick();
    if (status.readyBeep && !previousBeep) beepStarts.push_back(status.timeMs);
    previousBeep = status.readyBeep;
  }
  ASSERT_GE(beepStarts.size(), 2U);
  EXPECT_NEAR(beepStarts[1] - beepStarts[0], 1000, 5);
  report["ready_beep_period_ms"] = beepStarts[1] - beepStarts[0];
  report["led_observation"] = "Ready state only; GPIO LED output is not observable";
  bridge.reset();
  command(0, 0, 1);
  dwell(1.2);
  restartBridge();
  ASSERT_TRUE(wait([&] { return status.state == 3; }, 20));
  command(0);
  dwell(2);
  report["restart_ready"] = true;
  dwell(6);
  restartFirmware();
  dwell(6);
  command(1);
  ASSERT_TRUE(wait([&] { return sessions() == firstSessions + 1 && status.state == 2; }, 60));
  EXPECT_EQ(status.transitions, 2U);
  report["firmware_restart_new_session"] = true;
  const auto departure = hardware.pos;
  command(0, 1, 1);
  ASSERT_TRUE(wait([&] { return status.state == 3 && std::hypot(
    hardware.pos[0] - departure[0], hardware.pos[1] - departure[1]) > 0.4; }, 30));
  report["mapping_command_speed_mps"] = std::hypot(output.right, output.forward);
  report["observed_speed_mps"] = std::hypot(hardware.vel[0], hardware.vel[1]);
  EXPECT_LE(report["observed_speed_mps"].get<double>(), 0.43);
  EXPECT_NEAR(report["mapping_command_speed_mps"].get<double>(), 0.4, 1e-5);
  command(0);
  dwell(2);
  bridge.reset();
  command(3);
  dwell(1.2);
  restartBridge();
  ASSERT_TRUE(wait([&] { return robot->read_log().find("MMM SAVE COMPLETE:") != std::string::npos; }, 30));
  dwell(6);
  EXPECT_EQ(status.state, 3U);
  report["restart_mapping"] = true;
  fs::path bootPath;
  for (const auto &entry : fs::directory_iterator(directory / "maps"))
    if (fs::exists(entry.path() / "boot.json") && (bootPath.empty() || entry.path() > bootPath)) bootPath = entry.path();
  Json boot;
  std::ifstream(bootPath / "boot.json") >> boot;
  const auto end = seconds() + 45;
  while (status.state != 2 && seconds() < end) {
    try {
      const auto transform = buffer->lookupTransform("map", "root", tf2::TimePointZero);
      const auto &p = transform.transform.translation;
      const auto &q = transform.transform.rotation;
      const double yaw = hardware.chassis_yaw + std::atan2(
        2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z));
      const double dx = boot["root"][0].get<double>() - p.x;
      const double dy = boot["root"][1].get<double>() - p.y;
      const double distance = std::hypot(dx, dy);
      const double speed = distance > 0.015 ? std::min(0.35, distance) / distance / 0.4 : 0;
      command(0, speed * (std::sin(yaw) * dx - std::cos(yaw) * dy),
        speed * (std::cos(yaw) * dx + std::sin(yaw) * dy));
    } catch (const tf2::TransformException &) { command(0); }
    tick();
  }
  command(0);
  ASSERT_EQ(status.state, 2U) << robot->read_log();
  dwell(6);
  // Creating the expected final directory forces the real finalization to fail.
  fs::create_directory(bootPath / "snapshot");
  command(3);
  ASSERT_TRUE(wait([&] { return status.state == 4; }));
  ASSERT_TRUE(wait([&] { return status.state == 3; }, 60));
  report["failed_ready_save_done_to_mapping"] = true;
  fs::remove(bootPath / "snapshot");
  ASSERT_TRUE(wait([&] { return status.state == 2; }, 30));
  dwell(6);
  command(3);
  ASSERT_TRUE(wait([&] { return fs::exists(bootPath / "snapshot/save.json"); }, 60));
  ASSERT_EQ(status.state, 4U);
  Json saved;
  std::ifstream(bootPath / "snapshot/save.json") >> saved;
  EXPECT_TRUE(saved.at("validated").get<bool>());
  for (const auto &name : {"map.posegraph", "map.data", "map.pgm", "map.yaml", "boot.json"})
    EXPECT_GT(fs::file_size(bootPath / "snapshot" / name), 0U);
  report["validated_bundle"] = (bootPath / "snapshot").string();
  EXPECT_EQ(status.transitions, 8U);
  dwell(2);
  const auto stable = counts[0][3] + counts[1][3];
  dwell(3);
  EXPECT_EQ(counts[0][3] + counts[1][3], stable);
  EXPECT_EQ(sessions(), 2U);
  for (int direction = 0; direction < 2; ++direction) {
    for (int number : {1, 2, 3, 4}) {
      EXPECT_EQ(drops[direction][number], 0);
      EXPECT_EQ(corruptions[direction][number], 0);
      EXPECT_LE(counts[direction][number], 24);
    }
  }
  report["injected_drop_frames"] = 8;
  report["injected_crc_frames"] = 8;
  summaries();
  if (!HasFailure()) {
    std::cout << "MMM REAL FIRMWARE E2E PASS: start -> ready -> mapping -> ready -> failed save -> mapping -> ready -> save -> Done\n";
  }
}

TEST_F(MmmSuite, SetupFailureAndLoss) {
  launchStack(true);
  for (int direction = 0; direction < 2; ++direction) {
    drops[direction][1] = drops[direction][6] = 1;
    corruptions[direction][1] = corruptions[direction][6] = 1;
  }
  command(1, 1, 1);
  ASSERT_TRUE(wait([&] { return status.state == 4; }, 30));
  dwell(3);
  const auto before = counts[0][6] + counts[1][6];
  dwell(3);
  EXPECT_EQ(counts[0][6] + counts[1][6], before);
  EXPECT_EQ(status.transitions, 2U);
  EXPECT_LE(counts[0][1], 12);
  EXPECT_LE(counts[1][6], 12);
  for (int direction = 0; direction < 2; ++direction) for (int number : {1, 6}) {
    EXPECT_EQ(drops[direction][number], 0);
    EXPECT_EQ(corruptions[direction][number], 0);
  }
  report["injected_drop_frames"] = 4;
  report["injected_crc_frames"] = 4;
  report["setup_failure_done"] = true;
  summaries();
}

TEST_F(MmmSuite, FullTransmitQueue) {
  driveSim = false;
  bridge.reset();
  stall = true;
  command(1);
  for (int i = 0; i < 500; ++i) tick(100, false);
  EXPECT_GT(status.failedWrites, 0U);
  const auto ping = wire::frame(wire::Ping{2});
  ASSERT_EQ(write(mcb->slave(), ping.data(), ping.size()), static_cast<ssize_t>(ping.size()));
  tick(5, false);
  EXPECT_EQ(status.state, 2U);
  report["failed_uart_writes"] = status.failedWrites;
  const auto before = counts[0][1];
  stall = false;
  relay(0);
  for (int i = 0; i < 200; ++i) tick(5, false);
  EXPECT_GT(counts[0][1], before);
  EXPECT_EQ(status.state, 2U);
  report["ready_echo_lost_at_stall_boundary"] = counts[0][2] == 0;
  if (counts[0][2] == 0) {
    ASSERT_EQ(write(mcb->slave(), ping.data(), ping.size()), static_cast<ssize_t>(ping.size()));
    for (int i = 0; i < 10; ++i) tick(5, false);
  }
  EXPECT_GT(counts[0][2], 0);
  EXPECT_GT(counts[0][1], 1);
  report["queued_start_delivered_after_drain"] = true;
  report["firmware_time_ms"] = status.timeMs;
  report["telemetry_frames_after_stall"] = telemetryFrames[0];
  report["stall_parser_crc16_errors"] = parsers[0].errors().crc16;
  EXPECT_LT(telemetryFrames[0], 2000U);
  summaries();
}
}  // namespace

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
