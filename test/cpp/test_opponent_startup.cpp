// Copyright 2026 Thornbots
// Licensed under the Apache License, Version 2.0.
//
// Real Gazebo (the paused ARCC field, no sim progression) plus the real
// opponent_driver executable. Failure cases must exit and must not publish ready.

#include <atomic>
#include <chrono>
#include <cstdlib>
#include <filesystem>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <unistd.h>

#include <gz/msgs/boolean.pb.h>
#include <gz/msgs/empty.pb.h>
#include <gz/msgs/entity_factory.pb.h>
#include <gz/msgs/scene.pb.h>
#include <gz/msgs/world_stats.pb.h>
#include <gz/transport/Node.hh>
#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/header.hpp>

#include "sim/process.hpp"

namespace
{
using namespace std::chrono_literals;
using Clock = std::chrono::steady_clock;

const std::string kWorld = "/world/ARCC_Field_2026/";
const std::string kWorldFile = std::string(SIM_SOURCE_DIR) + "/world/ARCC_Field_2026.sdf";
const std::string kModel = "opponent_0";
const double kZ = -0.0115;

struct Stats
{
  std::mutex mutex;
  bool seen = false;
  bool paused = false;
  int64_t sim_ns = -1;
};

class OpponentStartupTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    // gz partitions are identifiers; hyphens made the server and test not see each other.
    partition_ = "opp_startup_" + std::to_string(getpid());
    setenv("GZ_PARTITION", partition_.c_str(), 1);
    node_ = std::make_unique<gz::transport::Node>();
    rclcpp::init(0, nullptr);
    observer_ = std::make_shared<rclcpp::Node>("opponent_startup_test");
    ready_sub_ = make_ready_sub();
    stats_ok_ = node_->Subscribe<gz::msgs::WorldStatistics>(kWorld + "stats",
      [this](const gz::msgs::WorldStatistics & msg) {
        std::lock_guard<std::mutex> lock(stats_.mutex);
        stats_.seen = true;
        stats_.paused = msg.paused();
        stats_.sim_ns = static_cast<int64_t>(msg.sim_time().sec()) * 1000000000LL +
          msg.sim_time().nsec();
      });
  }

  void TearDown() override
  {
    driver_.reset();
    gz_.reset();
    node_.reset();
    ready_sub_.reset();
    observer_.reset();
    rclcpp::shutdown();
  }

  rclcpp::Subscription<std_msgs::msg::Header>::SharedPtr make_ready_sub()
  {
    return observer_->create_subscription<std_msgs::msg::Header>(
      "/opponent_driver/ready", rclcpp::QoS(1).reliable().transient_local(),
      [this](std_msgs::msg::Header::ConstSharedPtr msg) {
        ++ready_count_;
        ready_frame_ = msg->frame_id;
      });
  }

  std::string logfile(const std::string & tag) const
  {
    return "/tmp/opp_startup_" + std::to_string(getpid()) + "_" + tag + ".log";
  }

  bool wait_until(const std::function<bool()> & done, std::chrono::duration<double> timeout)
  {
    const auto end = Clock::now() + timeout;
    while (Clock::now() < end) {
      rclcpp::spin_some(observer_);
      if (done()) {return true;}
      std::this_thread::sleep_for(20ms);
    }
    rclcpp::spin_some(observer_);
    return done();
  }

  void start_gz(bool running)
  {
    // The world's model://sim/... meshes resolve under the parent of the package dir.
    const std::string resources = std::filesystem::path(SIM_SOURCE_DIR).parent_path().string() +
      ":" + SIM_SOURCE_DIR;
    std::vector<std::string> cmd = {"env", "GZ_PARTITION=" + partition_,
      "GZ_SIM_RESOURCE_PATH=" + resources, "gz", "sim", "-s", "-v", "0"};
    if (running) {cmd.push_back("-r");}
    cmd.push_back(kWorldFile);
    gz_ = std::make_unique<sim::LaunchTree>("gz_server", cmd, logfile("gz"));
  }

  std::optional<gz::msgs::Scene> scene_info()
  {
    gz::msgs::Empty request;
    gz::msgs::Scene reply;
    bool result = false;
    if (!node_->Request(kWorld + "scene/info", request, 1000u, reply, result) || !result) {
      return std::nullopt;
    }
    return reply;
  }

  int model_count()
  {
    const auto scene = scene_info();
    if (!scene) {return -1;}
    int count = 0;
    for (const auto & model : scene->model()) {count += model.name() == kModel;}
    return count;
  }

  // Returns the model's position when exactly one model has the name.
  std::optional<std::pair<double, double>> model_xy()
  {
    const auto scene = scene_info();
    if (!scene) {return std::nullopt;}
    std::optional<std::pair<double, double>> found;
    int count = 0;
    for (const auto & model : scene->model()) {
      if (model.name() != kModel) {continue;}
      ++count;
      found = std::make_pair(model.pose().position().x(), model.pose().position().y());
    }
    return count == 1 ? found : std::nullopt;
  }

  // Test-side create, used to plant a colliding model before the driver starts.
  bool spawn_model(const std::string & name, double x, double y)
  {
    gz::msgs::EntityFactory request;
    request.set_sdf("<sdf version=\"1.9\"><model name=\"" + name +
      "\"><static>true</static><link name=\"link\"/></model></sdf>");
    request.set_name(name);
    request.set_allow_renaming(false);
    request.mutable_pose()->mutable_position()->set_x(x);
    request.mutable_pose()->mutable_position()->set_y(y);
    request.mutable_pose()->mutable_position()->set_z(kZ);
    request.mutable_pose()->mutable_orientation()->set_w(1.0);
    gz::msgs::Boolean reply;
    bool result = false;
    return node_->Request(kWorld + "create", request, 3000u, reply, result) && result;
  }

  bool wait_for_world(std::chrono::duration<double> timeout)
  {
    return wait_until([this] {return scene_info().has_value();}, timeout);
  }

  void start_driver(const std::vector<std::string> & params)
  {
    std::vector<std::string> cmd = {"env", "GZ_PARTITION=" + partition_,
      OPPONENT_DRIVER_EXE, "--ros-args", "-p", "use_sim_time:=true"};
    cmd.insert(cmd.end(), params.begin(), params.end());
    driver_ = std::make_unique<sim::LaunchTree>("opponent_driver", cmd, logfile("driver"));
  }

  std::string driver_log() {return driver_ ? driver_->read_log() : std::string();}

  bool driver_exited_within(std::chrono::duration<double> timeout)
  {
    return wait_until([this] {return !driver_->alive();}, timeout);
  }

  std::string partition_;
  // Created in SetUp, after GZ_PARTITION is set; a member Node would join the default partition.
  std::unique_ptr<gz::transport::Node> node_;
  bool stats_ok_ = false;
  Stats stats_;
  std::shared_ptr<rclcpp::Node> observer_;
  rclcpp::Subscription<std_msgs::msg::Header>::SharedPtr ready_sub_;
  int ready_count_ = 0;  // Written only from this test's spin thread.
  std::string ready_frame_;
  std::unique_ptr<sim::LaunchTree> gz_;
  std::unique_ptr<sim::LaunchTree> driver_;
};

// Expects the first stats sample at zero, paused, with no progress since.
bool epoch_zero_paused(Stats & stats)
{
  std::lock_guard<std::mutex> lock(stats.mutex);
  return stats.seen && stats.paused && stats.sim_ns == 0;
}
}  // namespace

TEST_F(OpponentStartupTest, CorrectModelReadyOnPausedWorld)
{
  start_gz(false);
  ASSERT_TRUE(wait_for_world(120s)) << gz_->read_log();
  ASSERT_TRUE(wait_until([this] {return epoch_zero_paused(stats_);}, 30s));
  start_driver({"-p", "initial_position:=[1.0,2.0]", "-p", "startup_timeout_s:=90.0"});
  ASSERT_TRUE(wait_until([this] {return ready_count_ >= 1;}, 120s)) << driver_log();
  EXPECT_EQ(ready_frame_, kModel);
  EXPECT_EQ(model_count(), 1);
  const auto xy = model_xy();
  ASSERT_TRUE(xy.has_value());
  EXPECT_NEAR(xy->first, 1.0, 1e-3);
  EXPECT_NEAR(xy->second, 2.0, 1e-3);
  EXPECT_TRUE(epoch_zero_paused(stats_));
  EXPECT_TRUE(driver_->alive());
}

TEST_F(OpponentStartupTest, LateSubscriberReceivesLatchedReady)
{
  start_gz(false);
  ASSERT_TRUE(wait_for_world(120s)) << gz_->read_log();
  // No ready subscriber exists while the driver publishes.
  ready_sub_.reset();
  start_driver({"-p", "initial_position:=[1.0,2.0]", "-p", "startup_timeout_s:=90.0"});
  ASSERT_TRUE(wait_until([this] {return driver_log().find("ready at epoch zero") !=
    std::string::npos;}, 120s)) << driver_log();
  // A subscriber created after the publish must still receive the latched ready.
  std::this_thread::sleep_for(3s);
  ready_sub_ = make_ready_sub();
  EXPECT_TRUE(wait_until([this] {return ready_count_ >= 1;}, 30s));
  EXPECT_EQ(ready_frame_, kModel);
  EXPECT_EQ(model_count(), 1);
}

TEST_F(OpponentStartupTest, NonzeroEpochFailsClosed)
{
  start_gz(true);  // Running world: sim time moves past zero.
  ASSERT_TRUE(wait_for_world(120s)) << gz_->read_log();
  start_driver({"-p", "initial_position:=[1.0,2.0]", "-p", "startup_timeout_s:=90.0"});
  ASSERT_TRUE(driver_exited_within(120s)) << driver_log();
  const std::string log = driver_log();
  EXPECT_TRUE(log.find("epoch zero missed") != std::string::npos ||
    log.find("not paused at epoch zero") != std::string::npos) << log;
  EXPECT_EQ(ready_count_, 0);
  EXPECT_EQ(log.find("ready at epoch zero"), std::string::npos);
}

TEST_F(OpponentStartupTest, NameCollisionFailsClosed)
{
  start_gz(false);
  ASSERT_TRUE(wait_for_world(120s)) << gz_->read_log();
  ASSERT_TRUE(spawn_model(kModel, 9.0, 9.0));
  start_driver({"-p", "initial_position:=[1.0,2.0]", "-p", "startup_timeout_s:=20.0"});
  ASSERT_TRUE(driver_exited_within(60s)) << driver_log();
  EXPECT_NE(driver_log().find("model name collision or incorrect pose"), std::string::npos);
  EXPECT_EQ(ready_count_, 0);
  EXPECT_EQ(model_count(), 1);
}

TEST_F(OpponentStartupTest, MissingSceneFailsBounded)
{
  // No gz server: the scene request can never succeed.
  start_driver({"-p", "initial_position:=[1.0,2.0]", "-p", "startup_timeout_s:=3.0"});
  ASSERT_TRUE(driver_exited_within(30s)) << driver_log();
  EXPECT_NE(driver_log().find("opponent startup failed"), std::string::npos);
  EXPECT_EQ(ready_count_, 0);
}

TEST_F(OpponentStartupTest, InvalidInitialPositionExitsAtConstruction)
{
  start_driver({"-p", "initial_position:=[1.0]"});
  ASSERT_TRUE(driver_exited_within(30s)) << driver_log();
  EXPECT_NE(driver_log().find("initial_position requires"), std::string::npos);
  EXPECT_EQ(ready_count_, 0);
}

TEST_F(OpponentStartupTest, LegacyDefaultWaitsForOdometryAndSpawnsNothing)
{
  start_gz(false);
  ASSERT_TRUE(wait_for_world(120s)) << gz_->read_log();
  start_driver({});
  std::this_thread::sleep_for(5s);
  EXPECT_TRUE(driver_->alive()) << driver_log();
  EXPECT_EQ(ready_count_, 0);
  EXPECT_EQ(model_count(), 0);
}

TEST_F(OpponentStartupTest, StopDuringStartupJoinsCleanly)
{
  start_gz(false);
  ASSERT_TRUE(wait_for_world(120s)) << gz_->read_log();
  start_driver({"-p", "initial_position:=[1.0,2.0]", "-p", "startup_timeout_s:=90.0"});
  std::this_thread::sleep_for(500ms);
  driver_->stop(30.0);
  EXPECT_FALSE(driver_->alive());
  EXPECT_EQ(ready_count_, 0);
}
