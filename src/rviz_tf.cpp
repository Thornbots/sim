// Copyright 2026 Thornbots
// Licensed under the Apache License, Version 2.0.

#include <chrono>
#include <memory>
#include <utility>

#include <QTimer>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <rviz_default_plugins/transformation/tf_frame_transformer.hpp>
#include <tf2_ros/transform_listener.hpp>

namespace sim {
class RvizTf final : public rviz_default_plugins::transformation::TFFrameTransformer {
  using TFWrapper = rviz_default_plugins::transformation::TFWrapper;

public:
  RvizTf() : RvizTf(std::make_shared<TFWrapper>()) {}

  void initialize(rviz_common::ros_integration::RosNodeAbstractionIface::WeakPtr node,
                  rclcpp::Clock::SharedPtr clock) override {
    const auto raw_node = node.lock()->get_raw_node();
    wrapper_->initializeBuffer(std::move(clock), raw_node, false);
    tf_group_ = raw_node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false);
    rclcpp::SubscriptionOptions options;
    options.callback_group = tf_group_;
    listener_ = std::make_unique<tf2_ros::TransformListener>(
        *wrapper_->getBuffer(), raw_node, false, tf2_ros::DynamicListenerQoS(8),
        tf2_ros::StaticListenerQoS(), options, options);
    tf_executor_.add_callback_group(tf_group_, raw_node->get_node_base_interface());

    // Drain TF in batches on the GUI thread between RViz display updates.
    QObject::connect(&tf_timer_, &QTimer::timeout, &tf_timer_, [this] {
      if (rclcpp::ok()) tf_executor_.spin_all(std::chrono::milliseconds(2));
    });
    tf_timer_.setTimerType(Qt::PreciseTimer);
    tf_timer_.start(10);
  }

private:
  explicit RvizTf(std::shared_ptr<TFWrapper> wrapper)
      : TFFrameTransformer(wrapper), wrapper_(std::move(wrapper)) {}

  std::shared_ptr<TFWrapper> wrapper_;
  std::unique_ptr<tf2_ros::TransformListener> listener_;
  rclcpp::CallbackGroup::SharedPtr tf_group_;
  rclcpp::executors::SingleThreadedExecutor tf_executor_;
  QTimer tf_timer_;
};
}  // namespace sim

PLUGINLIB_EXPORT_CLASS(sim::RvizTf, rviz_common::transformation::FrameTransformer)
