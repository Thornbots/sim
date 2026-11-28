// Copyright 2026 Thornbots
// Licensed under the Apache License, Version 2.0.

#include <array>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <deque>
#include <memory>
#include <optional>
#include <string>
#include <utility>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav2_map_server/map_io.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <visualization_msgs/msg/marker_array.hpp>

#include "sim/field.hpp"

namespace {
using Odometry = nav_msgs::msg::Odometry;
using Map = nav_msgs::msg::OccupancyGrid;
using Marker = visualization_msgs::msg::Marker;
using MarkerArray = visualization_msgs::msg::MarkerArray;
using Point = geometry_msgs::msg::Point;
using Offset = std::array<double, 2>;

constexpr char kFrame[] = "localization_view";
constexpr char kPrefix[] = "/sim/localization/";
constexpr std::size_t kTrailLimit = 80;

double distance(const Point &a, const Point &b) {
  return std::hypot(a.x - b.x, a.y - b.y);
}

struct Track {
  const char *input;
  const char *name;
  std::array<float, 3> color;
  double width;
  double height;
  bool robot;
  Offset offset;
  Odometry::ConstSharedPtr latest;
  std::optional<rclcpp::Time> last_stamp;
  std::deque<Point> trail;
  rclcpp::Publisher<MarkerArray>::SharedPtr pub;
  rclcpp::Subscription<Odometry>::SharedPtr sub;

  Track(const char *input_topic, const char *label, std::array<float, 3> rgb,
        double line_width, double z, bool robot_marker = false, Offset translation = {})
      : input(input_topic), name(label), color(rgb), width(line_width), height(z),
        robot(robot_marker), offset(translation) {}

  void draw(rclcpp::Time now) {
    if (!latest) return;
    const rclcpp::Time stamp(latest->header.stamp, now.get_clock_type());
    if ((last_stamp && stamp <= *last_stamp) || (now - stamp).seconds() > 0.8) return;
    Point point = latest->pose.pose.position;
    // The map pose publisher uses nonfinite positions while a fix is unavailable.
    if (!std::isfinite(point.x) || !std::isfinite(point.y)) return;
    if (last_stamp && (stamp - *last_stamp).seconds() > 0.8) trail.clear();
    last_stamp = stamp;
    point.x += offset[0];
    point.y += offset[1];
    point.z = height;
    if (trail.empty() || distance(trail.back(), point) >= 0.015) {
      trail.push_back(point);
      if (trail.size() > kTrailLimit) trail.pop_front();
    }

    Marker line;
    line.header.frame_id = kFrame;
    line.header.stamp = stamp;
    line.ns = name;
    line.type = Marker::LINE_STRIP;
    line.pose.orientation.w = 1.0;
    line.scale.x = width;
    line.color.r = color[0];
    line.color.g = color[1];
    line.color.b = color[2];
    line.color.a = 1.0;
    line.lifetime = rclcpp::Duration::from_seconds(0.8);

    Marker pose = line;
    pose.id = 1;
    pose.type = Marker::SPHERE;
    pose.pose.position = point;
    pose.scale.x = pose.scale.y = 2.6 * width;
    pose.scale.z = 0.01;
    line.points.assign(trail.begin(), trail.end());
    if (line.points.size() < 2) line.action = Marker::DELETE;

    MarkerArray markers;
    markers.markers.reserve(4);
    markers.markers.push_back(std::move(line));
    if (robot) {
      // The robot: a dark-rimmed, body-sized disc with a heading arrow, under the trails.
      pose.type = Marker::CYLINDER;
      pose.pose.position.z = height - 0.1;
      pose.scale.x = pose.scale.y = 0.55;
      Marker heading = pose;
      heading.id = 2;
      heading.type = Marker::ARROW;
      heading.pose.position.z = height + 0.2;
      heading.pose.orientation = latest->pose.pose.orientation;
      heading.scale.x = 0.34;
      heading.scale.y = heading.scale.z = 0.09;
      heading.color.r = heading.color.g = heading.color.b = 0.08f;
      markers.markers.push_back(std::move(heading));
      Marker outline = pose;
      outline.id = 3;
      outline.pose.position.z = height - 0.15;
      outline.scale.x = outline.scale.y = 0.67;
      outline.color.r = outline.color.g = outline.color.b = 0.08f;
      markers.markers.push_back(std::move(outline));
    }
    markers.markers.push_back(std::move(pose));
    pub->publish(markers);
  }
};

class LocalizationView final : public rclcpp::Node {
public:
  LocalizationView() : Node("localization_view"), frame_(this) {
    const auto backend = declare_parameter("backend", std::string("amcl"));
    const auto map_path = declare_parameter("reference_map", std::string());
    const bool amcl = backend == "amcl", mapping = backend == "mapping";
    const auto spawn = amcl ? sim::field::kBlueStart : sim::field::kLoopCentre;
    tracks_[1].offset = spawn;
    if (mapping) {
      // Blank-map SLAM starts at the spawn, while navigation seeds at the team start.
      tracks_[2].offset = {spawn[0] - sim::field::kBlueStart[0],
                           spawn[1] - sim::field::kBlueStart[1]};
      tracks_[3].offset = spawn;
    }

    geometry_msgs::msg::TransformStamped frame;
    frame.header.frame_id = backend == "none" ? "nav_odom" : "map";
    frame.child_frame_id = kFrame;
    frame.transform.rotation.w = 1.0;
    if (mapping) {
      frame.transform.translation.x = -spawn[0];
      frame.transform.translation.y = -spawn[1];
    }
    frame_.sendTransform(frame);

    const auto map_qos = rclcpp::QoS(1).reliable().transient_local();
    map_pub_ = create_publisher<Map>(std::string(kPrefix) + "map", map_qos);
    if (map_path.empty()) {
      map_sub_ = create_subscription<Map>("/map", map_qos,
          [this](Map::ConstSharedPtr map) { map_pub_->publish(*map); });
    } else {
      Map map;
      if (nav2_map_server::loadMapFromYaml(map_path, map) ==
          nav2_map_server::LOAD_MAP_SUCCESS) {
        map.header.frame_id = kFrame;
        map_pub_->publish(map);
      } else {
        RCLCPP_WARN(get_logger(), "Could not load the RViz reference map %s", map_path.c_str());
      }
    }

    for (auto &track : tracks_) {
      track.pub = create_publisher<MarkerArray>(std::string(kPrefix) + track.name, 1);
      track.sub = create_subscription<Odometry>(track.input, rclcpp::SensorDataQoS().keep_last(1),
          [&track](Odometry::ConstSharedPtr msg) { track.latest = msg; });
    }
    timer_ = create_wall_timer(std::chrono::milliseconds(100), [this]() {
      const auto &truth = tracks_.front().latest;
      if (truth) {
        const auto &point = truth->pose.pose.position;
        if (previous_truth_ && distance(*previous_truth_, point) > 1.0) {
          for (auto &track : tracks_) track.trail.clear();
        }
        previous_truth_ = point;
      }
      const auto stamp = now();
      for (auto &track : tracks_) track.draw(stamp);
    });
  }

private:
  std::array<Track, 4> tracks_{{
      // Okabe-Ito colours (plus white), widest first so overlaps show as stripes.
      {"/sim/raw_odom", "truth", {1.0f, 1.0f, 1.0f}, 0.20, 0.30, true},
      {"/odom", "raw_odom", {0.90f, 0.62f, 0.0f}, 0.13, 0.35},
      {"/localization/odom", "ekf", {0.34f, 0.71f, 0.91f}, 0.085, 0.40},
      {"/localization/map_odom", "map_pose", {0.80f, 0.47f, 0.65f}, 0.045, 0.45},
  }};
  tf2_ros::StaticTransformBroadcaster frame_;
  std::optional<Point> previous_truth_;
  rclcpp::Publisher<Map>::SharedPtr map_pub_;
  rclcpp::Subscription<Map>::SharedPtr map_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};
}  // namespace

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LocalizationView>());
  rclcpp::shutdown();
  return 0;
}
