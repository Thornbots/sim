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

#include <chrono>
#include <functional>
#include <thread>
#include <optional>
#include <vector>

#include <class_loader/class_loader.hpp>
#include <dji_serial_bridge/msg/panel_detection_array.hpp>
#include <gz/msgs/pose_v.pb.h>
#include <gz/transport/Node.hh>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/node_factory.hpp>
#include <std_msgs/msg/header.hpp>

#include <gtest/gtest.h>
#include <rosgraph_msgs/msg/clock.hpp>


namespace
{
using namespace std::chrono_literals;
const char * kRobot = R"(<robot name="test">
<link name="root"/><link name="head_pitch"/><link name="camera"/>
<link name="armor_front"><visual><geometry><box size="0.01 0.2 0.1"/></geometry></visual></link>
<joint name="head" type="fixed"><parent link="root"/><child link="head_pitch"/></joint>
<joint name="camera_joint" type="fixed"><parent link="head_pitch"/><child link="camera"/></joint>
<joint name="armor_joint" type="fixed"><parent link="root"/><child link="armor_front"/></joint>
</robot>)";

class DetectorTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    rclcpp::init(0, nullptr);
    executor = std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
    loader = std::make_unique<class_loader::ClassLoader>(DETECTOR_LIBRARY);
    observer = std::make_shared<rclcpp::Node>("detector_test_observer");
    clock = observer->create_publisher<rosgraph_msgs::msg::Clock>("/clock", rclcpp::ClockQoS());
    ready_sub = observer->create_subscription<std_msgs::msg::Header>(
      "/detector_standin/ready", rclcpp::QoS(1).reliable().transient_local(),
      [this](std_msgs::msg::Header::ConstSharedPtr) {++ready;});
    ack_sub = observer->create_subscription<std_msgs::msg::Header>(
      "/detector_standin/consumed", 10,
      [this](std_msgs::msg::Header::ConstSharedPtr m) {acks.push_back(m->stamp.nanosec);});
    out_sub = observer->create_subscription<dji_serial_bridge::msg::PanelDetectionArray>(
      "/cv/panel_detections", 10,
      [this](dji_serial_bridge::msg::PanelDetectionArray::ConstSharedPtr m) {
        outputs.push_back(m->header.stamp.nanosec);
        detection_counts.push_back(m->detections.size());
      });
    ours = gz.Advertise<gz::msgs::Pose_V>("/model/sentry/pose");
    theirs = gz.Advertise<gz::msgs::Pose_V>("/model/opponent_0/pose");
    executor->add_node(observer);
  }
  void TearDown() override
  {
    if (detector) {executor->remove_node(detector->get_node_base_interface()); detector.reset();}
    factory.reset();
    loader.reset();
    executor->remove_node(observer);
    observer.reset();
    executor.reset();
    rclcpp::shutdown();
  }
  void start(bool deterministic)
  {
    rclcpp::NodeOptions options;
    options.parameter_overrides({rclcpp::Parameter("use_sim_time", true),
      rclcpp::Parameter("robot_description", std::string(kRobot)),
      rclcpp::Parameter("deterministic_epoch_zero", deterministic)});
    auto classes = loader->getAvailableClasses<rclcpp_components::NodeFactory>();
    ASSERT_EQ(classes.size(), 1u);
    factory = loader->createInstance<rclcpp_components::NodeFactory>(classes.front());
    detector.emplace(factory->create_node_instance(options));
    executor->add_node(detector->get_node_base_interface());
    ASSERT_TRUE(wait([this, deterministic] {return clock->get_subscription_count() >= (deterministic ? 2u : 1u);}, 3s));
  }
  bool wait(const std::function<bool()> & condition, std::chrono::milliseconds timeout = 1500ms)
  {
    const auto end = std::chrono::steady_clock::now() + timeout;
    do {
      executor->spin_some();
      if (condition()) {return true;}
      std::this_thread::sleep_for(2ms);
    } while (std::chrono::steady_clock::now() < end);
    return false;
  }
  void time(uint32_t ns)
  {
    rosgraph_msgs::msg::Clock m;
    m.clock.nanosec = ns;
    clock->publish(m);
  }
  void pose(gz::transport::Node::Publisher & pub, const std::string & name, int ns)
  {
    gz::msgs::Pose_V msg;
    for (const auto & part : {name, name + "::root", name + "::head_pitch"}) {
      auto * p = msg.add_pose();
      p->set_name(part);
      p->mutable_orientation()->set_w(1.0);
      p->mutable_header()->mutable_stamp()->set_nsec(ns);
    }
    pub.Publish(msg);
  }
  std::shared_ptr<rclcpp::Node> observer;
  std::unique_ptr<class_loader::ClassLoader> loader;
  std::shared_ptr<rclcpp_components::NodeFactory> factory;
  std::optional<rclcpp_components::NodeInstanceWrapper> detector;
  std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> executor;
  rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr clock;
  rclcpp::Subscription<std_msgs::msg::Header>::SharedPtr ready_sub, ack_sub;
  rclcpp::Subscription<dji_serial_bridge::msg::PanelDetectionArray>::SharedPtr out_sub;
  gz::transport::Node gz;
  gz::transport::Node::Publisher ours, theirs;
  int ready = 0;
  std::vector<uint32_t> acks, outputs;
  std::vector<size_t> detection_counts;
};

TEST_F(DetectorTest, EpochZeroWaitsForTruthAndAcknowledgesScheduledTick)
{
  start(true);
  time(0);
  ASSERT_TRUE(wait([this] {return ready == 1;}));
  time(17000000);
  EXPECT_FALSE(wait([this] {return !acks.empty();}, 200ms));
  ASSERT_TRUE(wait([this] {return ours.HasConnections() && theirs.HasConnections();}));
  pose(ours, "sentry", 0);
  pose(theirs, "opponent_0", 0);
  pose(ours, "sentry", 17000000);
  pose(theirs, "opponent_0", 17000000);
  ASSERT_TRUE(wait([this] {return !acks.empty() && !outputs.empty();}));
  EXPECT_EQ(acks.front(), 17000000u);
  EXPECT_EQ(outputs.front(), 17000000u);
  EXPECT_EQ(detection_counts.front(), 0u);
  pose(ours, "sentry", 34000000);
  pose(theirs, "opponent_0", 34000000);
  time(33000000);
  EXPECT_FALSE(wait([this] {return acks.size() > 1;}, 100ms));
  time(34000000);
  ASSERT_TRUE(wait([this] {return acks.size() == 2 && outputs.size() == 2;}));
  EXPECT_EQ(acks.back(), 34000000u);
  EXPECT_EQ(outputs.back(), 34000000u);
  pose(ours, "sentry", 50000000);
  pose(theirs, "opponent_0", 50000000);
  time(50000000);
  ASSERT_TRUE(wait([this] {return acks.size() == 3 && outputs.size() == 3;}));
  EXPECT_EQ(acks.back(), 50000000u);
  EXPECT_EQ(outputs.back(), 50000000u);
}

TEST_F(DetectorTest, DelayedStartAndLateReadySubscriber)
{
  start(true);
  std::this_thread::sleep_for(200ms);
  time(0);
  ASSERT_TRUE(wait([this] {return ready == 1;}));
  int late_ready = 0;
  auto late = observer->create_subscription<std_msgs::msg::Header>(
    "/detector_standin/ready", rclcpp::QoS(1).reliable().transient_local(),
    [&late_ready](std_msgs::msg::Header::ConstSharedPtr) {++late_ready;});
  ASSERT_TRUE(wait([&late_ready] {return late_ready == 1;}));
  ASSERT_TRUE(wait([this] {return ours.HasConnections() && theirs.HasConnections();}));
  pose(ours, "sentry", 0);
  pose(theirs, "opponent_0", 0);
  pose(ours, "sentry", 17000000);
  pose(theirs, "opponent_0", 17000000);
  time(17000000);
  ASSERT_TRUE(wait([this] {return !acks.empty() && !outputs.empty();}));
  EXPECT_EQ(acks.front(), 17000000u);
  EXPECT_EQ(outputs.front(), 17000000u);
  EXPECT_EQ(detection_counts.front(), 0u);
}

TEST_F(DetectorTest, MissedEpochDoesNotAcknowledge)
{
  start(true);
  time(1000000);
  EXPECT_FALSE(wait([this] {return ready != 0 || !acks.empty();}, 300ms));
  time(0);
  EXPECT_FALSE(wait([this] {return ready != 0 || !acks.empty();}, 300ms));
}

TEST_F(DetectorTest, OverflowDoesNotAcknowledgeLateTruth)
{
  start(true);
  time(0);
  ASSERT_TRUE(wait([this] {return ready == 1;}));
  time(200000000);
  EXPECT_FALSE(wait([this] {return !acks.empty();}, 200ms));
  ASSERT_TRUE(wait([this] {return ours.HasConnections() && theirs.HasConnections();}));
  pose(ours, "sentry", 0);
  pose(theirs, "opponent_0", 0);
  pose(ours, "sentry", 200000000);
  pose(theirs, "opponent_0", 200000000);
  EXPECT_FALSE(wait([this] {return !acks.empty() || !outputs.empty();}, 300ms));
}

TEST_F(DetectorTest, LegacyPublishesWithClockAndTruth)
{
  start(false);
  ASSERT_TRUE(wait([this] {return ours.HasConnections() && theirs.HasConnections();}));
  pose(ours, "sentry", 0);
  pose(theirs, "opponent_0", 0);
  pose(ours, "sentry", 20000000);
  pose(theirs, "opponent_0", 20000000);
  time(17000000);
  ASSERT_TRUE(wait([this] {return !outputs.empty();}));
  EXPECT_TRUE(acks.empty());
}
}  // namespace
