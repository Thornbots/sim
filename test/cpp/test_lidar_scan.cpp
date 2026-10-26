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

#include "sim/lidar_scan.hpp"
#include <filesystem>
#include <gtest/gtest.h>
#include <tinyxml2.h>
#include <string>

TEST(LidarScan, PreservesCaptureAndValidReturnsNormalizesInvalidReturns) {
  sensor_msgs::msg::LaserScan scan;
  scan.header.frame_id = "lidar";
  scan.header.stamp.sec = 17;
  scan.range_min = 0.05f;
  scan.range_max = 12.0f;
  scan.ranges = {0.05f, 12.0f, 1.0f, 0.0f, 12.1f, -INFINITY, INFINITY, NAN};
  scan.intensities.assign(scan.ranges.size(), 42.0f);
  const auto intensities = scan.intensities;
  sim::LidarScanFormat format;
  format.apply(scan);
  EXPECT_EQ(scan.header.stamp.sec, 17);
  EXPECT_EQ(scan.header.frame_id, "lidar");
  EXPECT_EQ(scan.intensities, intensities);
  EXPECT_FLOAT_EQ(scan.ranges[0], 0.05f);
  EXPECT_FLOAT_EQ(scan.ranges[1], 12.0f);
  EXPECT_FLOAT_EQ(scan.ranges[2], 1.0f);
  for (size_t i = 3; i < scan.ranges.size(); ++i)
    EXPECT_EQ(scan.ranges[i], INFINITY);
  EXPECT_NEAR(scan.scan_time, 1.0 / 11.2, 1e-8);
}

TEST(LidarScan, MeasuresCadenceAndHandlesResetAndMissingFrames) {
  sensor_msgs::msg::LaserScan scan;
  scan.ranges.assign(720, 1.0f);
  scan.range_max = 12.0f;
  scan.header.stamp.sec = 10;
  sim::LidarScanFormat format;
  format.apply(scan);
  scan.header.stamp.nanosec = 90000000;
  format.apply(scan);
  EXPECT_NEAR(scan.scan_time, 0.09, 1e-8);
  EXPECT_NEAR(scan.time_increment, 0.09 / 719, 1e-10);
  scan.header.stamp.sec = 1;
  format.apply(scan);
  EXPECT_NEAR(scan.scan_time, 1.0 / 11.2, 1e-8);
  scan.header.stamp.sec = 2;
  format.apply(scan);
  EXPECT_NEAR(scan.scan_time, 1.0 / 11.2, 1e-8);
}

TEST(LidarScan, BothSensorDefinitionsMatchRealGrid) {
  for (const auto *model : {"sentry.urdf.xacro", "sentry_v2.urdf.xacro"}) {
    tinyxml2::XMLDocument doc;
    const auto path = std::filesystem::path(SIM_SOURCE_DIR) / "urdf" / model;
    ASSERT_EQ(doc.LoadFile(path.c_str()), tinyxml2::XML_SUCCESS);
    const auto *root = doc.RootElement();
    const tinyxml2::XMLElement *sensor = nullptr;
    for (auto *child = root->FirstChildElement(); child; child = child->NextSiblingElement()) {
      auto *gazebo = child;
      if (std::string(child->Name()) == "xacro:unless")
        gazebo = child->FirstChildElement("gazebo");
      if (gazebo && gazebo->Attribute("reference", "lidar")) {
        sensor = gazebo->FirstChildElement("sensor");
        if (sensor)
          break;
      }
    }
    ASSERT_NE(sensor, nullptr) << model;
    EXPECT_DOUBLE_EQ(sensor->FirstChildElement("update_rate")->DoubleText(), 11.2);
    const auto *lidar = sensor->FirstChildElement("lidar");
    const auto *grid = lidar->FirstChildElement("scan")->FirstChildElement("horizontal");
    EXPECT_EQ(grid->FirstChildElement("samples")->IntText(), 720);
    const auto min_angle = grid->FirstChildElement("min_angle")->DoubleText();
    const auto max_angle = grid->FirstChildElement("max_angle")->DoubleText();
    EXPECT_DOUBLE_EQ(min_angle, -M_PI);
    EXPECT_DOUBLE_EQ(max_angle, M_PI);
    EXPECT_NEAR((max_angle - min_angle) / 719 * 180 / M_PI, 0.5006954, 1e-7);
    const auto *range = lidar->FirstChildElement("range");
    EXPECT_DOUBLE_EQ(range->FirstChildElement("min")->DoubleText(), 0.05);
    EXPECT_DOUBLE_EQ(range->FirstChildElement("max")->DoubleText(), 12.0);
  }
}
