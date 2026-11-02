// Copyright 2026 Thornbots
// SPDX-License-Identifier: Apache-2.0

#include "sim/e2e_harness.hpp"
#include "sim/tool_io.hpp"
#include <gtest/gtest.h>
#include <cstdlib>

namespace sim::e2e {
class InputAuditTest : public ::testing::Test {
protected:
  void SetUp() override {
    rclcpp::init(0, nullptr);
    auto pattern = (std::filesystem::temp_directory_path() / "input-audit-XXXXXX").string();
    const auto directory = mkdtemp(pattern.data());
    ASSERT_NE(directory, nullptr);
    directory_ = directory;
    scorer_ = std::make_shared<Scorer>("mcb_parked", (directory_ / "shots.jsonl").string());
    publisher_ = std::make_shared<rclcpp::Node>("input_audit_publisher");
  }
  void TearDown() override {
    scorer_.reset();
    publisher_.reset();
    rclcpp::shutdown();
    if (!directory_.empty()) std::filesystem::remove_all(directory_);
  }
  auto rows() { return sim::tools::json_lines(directory_ / "inputs.jsonl"); }
  std::filesystem::path directory_;
  std::shared_ptr<Scorer> scorer_;
  rclcpp::Node::SharedPtr publisher_;
};

TEST_F(InputAuditTest, KeepsEmptyFramesGeometryInvalidStatesAndExactStampsBeforeTeardown) {
  auto detections = publisher_->create_publisher<dji_serial_bridge::msg::PanelDetectionArray>(
      "/cv/panel_detections", rclcpp::QoS(10));
  auto states = publisher_->create_publisher<dji_serial_bridge::msg::TargetState>(
      "/cv/target_state", rclcpp::QoS(10));
  ASSERT_TRUE(scorer_->wait_until([&] {
    return detections->get_subscription_count() > 0 && states->get_subscription_count() > 0;
  }, 5, "audit subscriptions"));

  dji_serial_bridge::msg::PanelDetectionArray frame;
  frame.header.stamp.sec = 12;
  frame.header.stamp.nanosec = 123456789;
  frame.header.frame_id = "camera";
  detections->publish(frame);
  dji_serial_bridge::msg::PanelDetection panel;
  panel.header = frame.header;
  panel.robot_track_id = 7;
  panel.confidence = .75;
  panel.class_id = 2;
  panel.depth_m = 3;
  panel.center.x = 3;
  panel.corners[0].y = .125;
  frame.detections.push_back(panel);
  // Preserve regressing input stamps for diagnosis instead of dropping them.
  frame.header.stamp.sec = 11;
  detections->publish(frame);
  dji_serial_bridge::msg::TargetState state;
  state.header = panel.header;
  state.detection_stamp = frame.header.stamp;
  state.robot_track_id = 7;
  state.valid = true;
  states->publish(state);
  state.valid = false;
  states->publish(state);

  ASSERT_TRUE(scorer_->wait_until([&] { return rows().size() == 4; }, 5, "audit records"));
  const auto recorded = rows();
  size_t detection_count = 0, state_count = 0;
  for (const auto &row : recorded) {
    EXPECT_EQ(row.at("frame_id"), "camera");
    EXPECT_TRUE(row.at("received_ns").is_number_integer());
    if (row.at("topic") == "/cv/panel_detections") {
      if (detection_count++ == 0) {
        EXPECT_EQ(row.at("stamp_ns"), 12123456789LL);
        EXPECT_EQ(row.at("count"), 0);
        EXPECT_TRUE(row.at("detections").empty());
      } else {
        EXPECT_EQ(row.at("stamp_ns"), 11123456789LL);
        ASSERT_EQ(row.at("detections").size(), 1U);
        const auto &saved = row.at("detections").at(0);
        EXPECT_EQ(saved.at("stamp_ns"), 12123456789LL);
        EXPECT_EQ(saved.at("track_id"), 7);
        EXPECT_EQ(saved.at("center").at(0), 3);
        EXPECT_EQ(saved.at("corners").at(0).at(1), .125);
        EXPECT_EQ(saved.at("confidence"), .75);
      }
    } else if (row.at("topic") == "/cv/target_state") {
      EXPECT_EQ(row.at("valid"), state_count++ == 0);
      EXPECT_EQ(row.at("detection_stamp_ns"), 11123456789LL);
    } else {
      FAIL() << "unexpected audit topic " << row.at("topic");
    }
  }
  EXPECT_EQ(detection_count, 2U);
  EXPECT_EQ(state_count, 2U);
}
}  // namespace sim::e2e
