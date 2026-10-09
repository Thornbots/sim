// Copyright 2026 Thornbots
// Licensed under the Apache License, Version 2.0.

#include <cstdio>
#include <memory>
#include <stdexcept>
#include <termios.h>
#include <unistd.h>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("wasd_teleop");
  auto pub = node->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
  termios settings{};
  if (tcgetattr(STDIN_FILENO, &settings) != 0) {
    throw std::runtime_error("wasd_teleop requires a terminal");
  }
  std::puts("WASD teleop: w/s forward, a/d strafe, space stop, x quit");
  try {
    while (rclcpp::ok()) {
      termios raw = settings;
      cfmakeraw(&raw);
      tcsetattr(STDIN_FILENO, TCSAFLUSH, &raw);
      const int key = std::getchar();
      tcsetattr(STDIN_FILENO, TCSADRAIN, &settings);
      if (key == 'x' || key == 3) {
        break;
      }
      geometry_msgs::msg::Twist twist;
      constexpr double speed = 0.3;
      if (key == 'w') twist.linear.x = speed;
      if (key == 's') twist.linear.x = -speed;
      if (key == 'a') twist.linear.y = speed;
      if (key == 'd') twist.linear.y = -speed;
      pub->publish(twist);
    }
  } catch (...) {
    pub->publish(geometry_msgs::msg::Twist{});
    tcsetattr(STDIN_FILENO, TCSADRAIN, &settings);
    node.reset();
    rclcpp::shutdown();
    throw;
  }
  pub->publish(geometry_msgs::msg::Twist{});
  tcsetattr(STDIN_FILENO, TCSADRAIN, &settings);
  rclcpp::shutdown();
  return 0;
}
