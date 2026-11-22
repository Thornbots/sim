// Copyright 2026 Thornbots
// Licensed under the Apache License, Version 2.0.

#include <algorithm>
#include <csignal>
#include <memory>
#include <string>
#include <vector>

#include <QApplication>
#include <QSignalBlocker>
#include <QTimer>
#include <rclcpp/rclcpp.hpp>
#include <rviz_common/display_group.hpp>
#include <rviz_common/logging.hpp>
#include <rviz_common/ros_integration/ros_client_abstraction_iface.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction.hpp>
#include <rviz_common/visualization_frame.hpp>
#include <rviz_common/visualization_manager.hpp>
#include <rviz_common/visualizer_app.hpp>

namespace {

volatile std::sig_atomic_t stop_requested = 0;

void request_stop(int) {
  stop_requested = 1;
}

class RosClient final : public rviz_common::ros_integration::RosClientAbstractionIface {
  using RosNode = rviz_common::ros_integration::RosNodeAbstraction;

public:
  rviz_common::ros_integration::RosNodeAbstractionIface::WeakPtr init(
      int argc, char **argv, const std::string &name, bool) override {
    rclcpp::init(argc, argv, rclcpp::InitOptions{}, rclcpp::SignalHandlerOptions::None);
    node_ = std::make_shared<RosNode>(name);
    return node_;
  }

  bool ok() override { return rclcpp::ok(); }
  void shutdown() override { rclcpp::shutdown(); }

private:
  std::shared_ptr<RosNode> node_;
};

void release_displays() {
  for (auto *widget : QApplication::topLevelWidgets()) {
    auto *frame = qobject_cast<rviz_common::VisualizationFrame *>(widget);
    if (!frame) continue;
    auto *manager = frame->getManager();
    manager->stopUpdate();
    // Camera displays can otherwise mutate the property model during removal.
    const QSignalBlocker block(manager->getRootDisplayGroup());
    manager->removeAllDisplays();
  }
}

}  // namespace

int main(int argc, char **argv) {
  std::signal(SIGINT, request_stop);
  std::signal(SIGTERM, request_stop);
  auto qt_args = rclcpp::remove_ros_arguments(argc, argv);
  if (qgetenv("XDG_SESSION_TYPE") == "wayland" && !qEnvironmentVariableIsSet("QT_QPA_PLATFORM") &&
      std::find(qt_args.begin(), qt_args.end(), "-platform") == qt_args.end()) {
    qputenv("QT_QPA_PLATFORM", "xcb");
  }
  std::vector<char *> qt_argv;
  for (auto &arg : qt_args) qt_argv.push_back(arg.data());
  qt_argv.push_back(nullptr);
  int qt_argc = static_cast<int>(qt_args.size());
  QApplication app(qt_argc, qt_argv.data());

  const auto logger = rclcpp::get_logger("rviz2");
  rviz_common::set_logging_handlers(
      [logger](const std::string &msg, const std::string &, size_t) {
        RCLCPP_DEBUG(logger, "%s", msg.c_str());
      },
      [logger](const std::string &msg, const std::string &, size_t) {
        RCLCPP_INFO(logger, "%s", msg.c_str());
      },
      [logger](const std::string &msg, const std::string &, size_t) {
        RCLCPP_WARN(logger, "%s", msg.c_str());
      },
      [logger](const std::string &msg, const std::string &, size_t) {
        RCLCPP_ERROR(logger, "%s", msg.c_str());
      });

  rviz_common::VisualizerApp viewer(std::make_unique<RosClient>());
  viewer.setApp(&app);
  if (!viewer.init(argc, argv)) return 1;

  QTimer stop_timer;
  QObject::connect(&stop_timer, &QTimer::timeout, &app, [&app] {
    if (stop_requested) app.quit();
  });
  QObject::connect(&app, &QCoreApplication::aboutToQuit, &app, [] {
    std::signal(SIGINT, SIG_DFL);
    std::signal(SIGTERM, SIG_DFL);
    release_displays();
  });
  stop_timer.start(25);
  return app.exec();
}
