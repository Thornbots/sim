// Copyright 2026 Thornbots
// Licensed under the Apache License, Version 2.0.

// Lockstep gate between rf2o and the EKF. rf2o answers a lidar scan at a wall-timed
// moment, so the stock EKF either had the result at its next tick or not. This node holds
// each result back and releases it at a lockstep hold chosen to precede an EKF tick: the
// first hold (`phase_s` + n * `period_s`, the tick minus one step) at least `ready_lag_s`
// after the scan stamp. At each hold at or after `start_s` it waits (wall time) for the
// quality diagnostic of every scan due and, when rf2o reports an increment, its
// Odometry; republishes those in stamp order; then publishes the hold time on `mark_topic`
// for the coordinator's chain once the navigation adapter's output (`forward_topic`, the
// EKF's input) carries every stamp released. Scans are the lidar grid `first_scan_s` + n * `scan_period_s`.
// It reads only stamps and the node clock, which the chain keeps at the hold.

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <map>
#include <set>
#include <stdexcept>
#include <string>

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/header.hpp>

namespace
{
int64_t to_ns(double s) {return static_cast<int64_t>(std::llround(s * 1e9));}

class ScanOdomGate : public rclcpp::Node
{
public:
  ScanOdomGate()
  : Node("scan_odom_gate", rclcpp::NodeOptions().parameter_overrides(
      {rclcpp::Parameter("use_sim_time", true)}))
  {
    period_ns_ = to_ns(declare_parameter("period_s", 0.034));
    const int64_t phase_ns = to_ns(declare_parameter("phase_s", 0.0));
    const int64_t start_ns = to_ns(declare_parameter("start_s", 0.1));
    scan_period_ns_ = to_ns(declare_parameter("scan_period_s", 0.089));
    next_scan_ns_ = to_ns(declare_parameter("first_scan_s", 0.179));
    lag_ns_ = to_ns(declare_parameter("ready_lag_s", 0.03));
    if (period_ns_ <= 0 || scan_period_ns_ <= 0 || lag_ns_ < 0) {
      throw std::invalid_argument("scan_odom_gate needs positive periods");
    }
    next_hold_ns_ = phase_ns;
    while (next_hold_ns_ < start_ns) {next_hold_ns_ += period_ns_;}
    const auto in = declare_parameter("odom_topic", std::string("/scan_odom"));
    const auto quality = declare_parameter("quality_topic", std::string("/scan_odom/quality"));
    out_ = create_publisher<nav_msgs::msg::Odometry>(
      declare_parameter("out_topic", std::string("/scan_odom/gated")), rclcpp::QoS(10));
    mark_ = create_publisher<std_msgs::msg::Header>(
      declare_parameter("mark_topic", std::string("/sim/e2e/scan_released")),
      rclcpp::QoS(100).reliable());
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      in, rclcpp::QoS(100).reliable(), [this](nav_msgs::msg::Odometry::ConstSharedPtr m) {
        odoms_[rclcpp::Time(m->header.stamp).nanoseconds()] = *m;
      });
    quality_sub_ = create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
      quality, rclcpp::QoS(100).reliable(),
      [this](diagnostic_msgs::msg::DiagnosticArray::ConstSharedPtr m) {
        bool increment = false;
        for (const auto & status : m->status) {
          for (const auto & kv : status.values) {
            if (kv.key == "increment") {increment = kv.value != "none";}
          }
        }
        qualities_[rclcpp::Time(m->header.stamp).nanoseconds()] = increment;
      });
    forward_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      declare_parameter("forward_topic", std::string("/localization/scan_odom")),
      rclcpp::QoS(100).reliable(), [this](nav_msgs::msg::Odometry::ConstSharedPtr m) {
        forwarded_.insert(rclcpp::Time(m->header.stamp).nanoseconds());
      });
    timer_ = create_wall_timer(std::chrono::milliseconds(1), [this] {poll();});
  }

private:
  void poll()
  {
    for (;;) {
      if (releasing_) {
        for (const int64_t stamp : awaited_) {
          if (forwarded_.count(stamp) == 0) {return;}
        }
        awaited_.clear();
        std_msgs::msg::Header h;
        h.stamp = rclcpp::Time(next_hold_ns_, RCL_ROS_TIME);
        mark_->publish(h);
        next_hold_ns_ += period_ns_;
        releasing_ = false;
      }
      if (now().nanoseconds() < next_hold_ns_) {return;}
      // Every scan whose result is due at this hold must have answered.
      std::map<int64_t, bool> due;
      for (int64_t s = next_scan_ns_; s + lag_ns_ <= next_hold_ns_; s += scan_period_ns_) {
        const auto q = qualities_.find(s);
        if (q == qualities_.end() || (q->second && odoms_.find(s) == odoms_.end())) {return;}
        due[s] = q->second;
      }
      for (const auto & entry : due) {
        if (entry.second) {
          out_->publish(odoms_.at(entry.first));
          awaited_.insert(entry.first);
        }
        odoms_.erase(entry.first);
        qualities_.erase(entry.first);
        next_scan_ns_ = entry.first + scan_period_ns_;
      }
      releasing_ = true;
    }
  }

  int64_t period_ns_, scan_period_ns_, next_scan_ns_, lag_ns_, next_hold_ns_;
  std::map<int64_t, nav_msgs::msg::Odometry> odoms_;
  std::map<int64_t, bool> qualities_;
  std::set<int64_t> awaited_, forwarded_;
  bool releasing_ = false;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr out_;
  rclcpp::Publisher<std_msgs::msg::Header>::SharedPtr mark_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr quality_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr forward_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};
}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ScanOdomGate>());
  rclcpp::shutdown();
  return 0;
}
