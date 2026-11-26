// Copyright 2026 Thornbots
// Licensed under the Apache License, Version 2.0.

#include <chrono>
#include <memory>
#include <utility>

#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/srv/get_map.hpp>
#include <rclcpp/rclcpp.hpp>

class E2EMapView final : public rclcpp::Node {
  using Map = nav_msgs::msg::OccupancyGrid;

public:
  E2EMapView() : Node("e2e_map_view") {
    const auto qos = rclcpp::QoS(1).reliable().transient_local();
    pub_ = create_publisher<Map>("/sim/e2e/map", qos);
    sub_ = create_subscription<Map>(
        "/map", qos, [this](Map::ConstSharedPtr map) { publish_map(std::move(map)); });
    client_ = create_client<nav_msgs::srv::GetMap>("/map_server/map");
    timer_ = create_wall_timer(std::chrono::seconds(1), [this]() {
      if (cached_ || pending_ || !client_->service_is_ready()) return;
      pending_ = true;
      client_->async_send_request(std::make_shared<nav_msgs::srv::GetMap::Request>(),
          [this](rclcpp::Client<nav_msgs::srv::GetMap>::SharedFuture response) {
            pending_ = false;
            if (!cached_) publish_map(std::make_shared<Map>(response.get()->map));
          });
    });
  }

private:
  void publish_map(Map::ConstSharedPtr map) {
    if (map->data.empty() ||
        (cached_ && map->header.frame_id == cached_->header.frame_id &&
         map->info == cached_->info && map->data == cached_->data)) {
      return;
    }
    pub_->publish(*map);
    cached_ = std::move(map);
    timer_->cancel();
  }

  bool pending_{false};
  Map::ConstSharedPtr cached_;
  rclcpp::Publisher<Map>::SharedPtr pub_;
  rclcpp::Subscription<Map>::SharedPtr sub_;
  rclcpp::Client<nav_msgs::srv::GetMap>::SharedPtr client_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<E2EMapView>());
  rclcpp::shutdown();
  return 0;
}
