// Copyright 2026 Thornbots
// Licensed under the Apache License, Version 2.0.

#include <chrono>
#include <memory>

#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/srv/get_map.hpp>
#include <rclcpp/rclcpp.hpp>

class E2EMapView final : public rclcpp::Node {
public:
  E2EMapView() : Node("e2e_map_view") {
    const auto qos = rclcpp::QoS(1).reliable().transient_local();
    pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>("/sim/e2e/map", qos);
    sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
        "/map", qos, [this](nav_msgs::msg::OccupancyGrid::ConstSharedPtr map) { cached_ = map; });
    client_ = create_client<nav_msgs::srv::GetMap>("/map_server/map");
    timer_ = create_wall_timer(std::chrono::seconds(1), [this]() {
      if (cached_) {
        pub_->publish(*cached_);
        return;
      }
      if (pending_ || !client_->service_is_ready()) return;
      pending_ = true;
      client_->async_send_request(std::make_shared<nav_msgs::srv::GetMap::Request>(),
          [this](rclcpp::Client<nav_msgs::srv::GetMap>::SharedFuture response) {
            pending_ = false;
            const auto &map = response.get()->map;
            if (map.info.width) {
              cached_ = std::make_shared<nav_msgs::msg::OccupancyGrid>(map);
            }
          });
    });
  }

private:
  bool pending_{false};
  nav_msgs::msg::OccupancyGrid::ConstSharedPtr cached_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr pub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr sub_;
  rclcpp::Client<nav_msgs::srv::GetMap>::SharedPtr client_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<E2EMapView>());
  rclcpp::shutdown();
  return 0;
}
