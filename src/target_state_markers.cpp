// Copyright 2026 Thornbots
// Licensed under the Apache License, Version 2.0.

#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <utility>

#include <dji_serial_bridge/msg/target_state.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

namespace
{

constexpr double kPi = 3.14159265358979323846;
constexpr double kPanelWidth = 0.135;
constexpr double kPanelHeight = 0.125;
constexpr double kPanelPitch = -kPi / 12.0;  // S122 panel normal points 15 degrees up
constexpr uint32_t kLifetimeNs = 500000000;

using TargetState = dji_serial_bridge::msg::TargetState;
using Marker = visualization_msgs::msg::Marker;
using Color = std::array<float, 3>;

Marker marker(const TargetState & state, const std::string & ns, int32_t type,
  const Color & rgb, float alpha = 1.0F)
{
  Marker out;
  out.header.stamp = state.header.stamp;
  out.header.frame_id = state.header.frame_id.empty() ? "odom" : state.header.frame_id;
  out.ns = ns;
  out.id = 0;
  out.type = type;
  out.action = Marker::ADD;
  out.pose.orientation.w = 1.0;
  out.color.r = rgb[0];
  out.color.g = rgb[1];
  out.color.b = rgb[2];
  out.color.a = alpha;
  out.lifetime.nanosec = kLifetimeNs;
  return out;
}

Marker arrow(const TargetState & state, const std::string & ns,
  const geometry_msgs::msg::Vector3 & vec, double scale, const Color & rgb)
{
  auto out = marker(state, ns, Marker::ARROW, rgb);
  const auto & c = state.center;
  geometry_msgs::msg::Point tip;
  tip.x = c.x + vec.x * scale;
  tip.y = c.y + vec.y * scale;
  tip.z = c.z + vec.z * scale;
  out.points = {c, tip};
  out.scale.x = 0.02;
  out.scale.y = 0.05;
  out.scale.z = 0.0;
  if (std::hypot(vec.x, vec.y, vec.z) * scale < 1e-3) {
    out.action = Marker::DELETE;
  }
  return out;
}

}  // namespace

class TargetStateMarkers : public rclcpp::Node
{
public:
  TargetStateMarkers()
  : Node("target_state_markers")
  {
    const auto input_topic = declare_parameter("input_topic", std::string("/cv/target_state"));
    const auto output_topic = declare_parameter("output_topic", std::string("/cv/target_state_markers"));
    const double rate = declare_parameter("max_rate_hz", 30.0);
    velocity_scale_s_ = declare_parameter("velocity_scale_s", 1.0);
    accel_scale_s2_ = declare_parameter("accel_scale_s2", 0.25);
    pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(output_topic, 10);
    sub_ = create_subscription<TargetState>(
      input_topic, 10, std::bind(&TargetStateMarkers::on_state, this, std::placeholders::_1));
    timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / rate),
      std::bind(&TargetStateMarkers::on_timer, this));
  }

private:
  void on_state(TargetState::ConstSharedPtr msg)
  {
    latest_ = std::move(msg);
  }

  void on_timer()
  {
    if (!latest_) {
      return;
    }
    auto state = std::move(latest_);
    visualization_msgs::msg::MarkerArray out;
    const Color panel_rgb = state->valid ? Color{1.0F, 0.5F, 0.0F} :
      Color{0.6F, 0.6F, 0.6F};
    auto center = marker(*state, "state_center", Marker::SPHERE, panel_rgb, 0.8F);
    center.pose.position = state->center;
    center.scale.x = center.scale.y = center.scale.z = 0.08;
    out.markers.push_back(std::move(center));
    for (int k = 0; k < 4; ++k) {
      const double yaw = state->yaw + k * kPi / 2.0;
      const double r = state->radius[k % 2];
      const double dz = state->z_offset[k % 2];
      auto panel = marker(*state, "state_panels", Marker::CUBE, panel_rgb,
        k == 0 ? 0.9F : 0.5F);
      panel.id = k;
      panel.pose.position.x = state->center.x + r * std::cos(yaw);
      panel.pose.position.y = state->center.y + r * std::sin(yaw);
      panel.pose.position.z = state->center.z + dz;
      const double cy = std::cos(yaw / 2.0);
      const double sy = std::sin(yaw / 2.0);
      const double cp = std::cos(kPanelPitch / 2.0);
      const double sp = std::sin(kPanelPitch / 2.0);
      panel.pose.orientation.x = -sy * sp;
      panel.pose.orientation.y = cy * sp;
      panel.pose.orientation.z = sy * cp;
      panel.pose.orientation.w = cy * cp;
      panel.scale.x = 0.02;
      panel.scale.y = kPanelWidth;
      panel.scale.z = kPanelHeight;
      out.markers.push_back(std::move(panel));
    }
    out.markers.push_back(arrow(*state, "state_velocity", state->velocity,
      velocity_scale_s_, {1.0F, 0.0F, 1.0F}));
    out.markers.push_back(arrow(*state, "state_accel", state->acceleration,
      accel_scale_s2_, {1.0F, 0.1F, 0.1F}));
    pub_->publish(out);
  }

  double velocity_scale_s_{1.0};
  double accel_scale_s2_{0.25};
  TargetState::ConstSharedPtr latest_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_;
  rclcpp::Subscription<TargetState>::SharedPtr sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TargetStateMarkers>());
  rclcpp::shutdown();
  return 0;
}
