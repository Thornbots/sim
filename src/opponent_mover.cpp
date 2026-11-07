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

// A gz system that moves its model along a path every physics step: it
// holds the newest gz.msgs.Odometry on <topic> (target_driver's path,
// bridged from ROS) and sets the model's world pose to it carried forward
// at its velocity and yaw rate to the step's sim time, at height <z>.
// Past <stale_s> without a sample the model holds still. Teleporting from
// With <path_hz> it waits (up to 5 s wall) for the path sample of the latest tick before this step, so
// which sample carries it forward never depends on message timing.
// Python moved it in 4-36 cm jumps; VelocityControl moved only the first
// robot spawned (2026-09-29). see README.md for design rationale

#include <gz/msgs/odometry.pb.h>

#include <algorithm>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <cmath>
#include <limits>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <string>

#include <gz/math/Pose3.hh>
#include <gz/plugin/Register.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/components/JointPositionReset.hh>
#include <gz/sim/components/JointVelocityReset.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Util.hh>
#include <gz/transport/Node.hh>

namespace sim
{

class OpponentMover
  : public gz::sim::System,
  public gz::sim::ISystemConfigure,
  public gz::sim::ISystemPreUpdate
{
public:
  void Configure(
    const gz::sim::Entity & entity, const std::shared_ptr<const sdf::Element> & sdf,
    gz::sim::EntityComponentManager & ecm, gz::sim::EventManager &) override
  {
    model_ = gz::sim::Model(entity);
    const std::string topic = sdf->Get<std::string>(
      "topic", "/model/" + model_.Name(ecm) + "/path").first;
    z_ = sdf->Get<double>("z", 0.0).first;
    stale_s_ = sdf->Get<double>("stale_s", 0.1).first;
    path_hz_ = sdf->Get<double>("path_hz", 0.0).first;
    node_.Subscribe(topic, &OpponentMover::OnPath, this);
  }

  void PreUpdate(const gz::sim::UpdateInfo & info, gz::sim::EntityComponentManager & ecm) override
  {
    if (info.paused) {
      return;
    }
    std::optional<gz::msgs::Odometry> path;
    {
      std::unique_lock<std::mutex> lock(mutex_);
      if (path_hz_ > 0.0 && !path_ &&
        info.simTime.count() - info.dt.count() >=
        static_cast<int64_t>(std::ceil(1000.0 / path_hz_)) * 1000000)
      {
        // The first sample has been published: the step it starts on must not depend on timing.
        cv_.wait_for(lock, std::chrono::seconds(5), [&] {return path_.has_value();});
      }
      if (path_hz_ > 0.0 && path_) {
        // The newest tick the clock has published: ticks are ceil(k / hz) ms, and /clock
        // reaches the step before this one.
        const int64_t before = info.simTime.count() - info.dt.count();
        int64_t k = static_cast<int64_t>(std::floor(before * 1e-9 * path_hz_)) + 1;
        int64_t tick = 0;
        do {
          --k;
          tick = static_cast<int64_t>(std::ceil(k * 1000.0 / path_hz_)) * 1000000;
        } while (tick > before && k > 0);
        const auto stamp_ns = [](const gz::msgs::Odometry & m) {
            return static_cast<int64_t>(m.header().stamp().sec()) * 1000000000 +
                   m.header().stamp().nsec();
          };
        const double age = (before - stamp_ns(*path_)) * 1e-9;
        // Wait only for a tick the producer makes: its first sample carries its first tick
        // (17 ms at 60 Hz), and a long board boot lets that sample arrive while the clock is
        // still at zero, so no sample for the earlier ticks will ever come.
        if (age <= stale_s_ && tick >= first_stamp_) {
          cv_.wait_for(lock, std::chrono::seconds(5), [&] {return samples_.count(tick) > 0;});
        }
        // That tick's own sample, even when a newer one has already arrived: which of the
        // two a step used must not depend on message timing. Before the first tick there is
        // none, and the model holds still (never the newest sample: it is ahead of the clock).
        auto it = samples_.upper_bound(tick);
        if (it != samples_.begin()) {
          path = std::prev(it)->second;
        }
      }
      if (!path && path_hz_ <= 0.0) {
        path = path_;
      }
    }
    if (!path) {
      return;
    }
    const double t = std::chrono::duration<double>(info.simTime).count();
    const double dt = t - (path->header().stamp().sec() + path->header().stamp().nsec() * 1e-9);
    if (dt < -stale_s_ || dt > stale_s_) {
      return;
    }
    const auto & p = path->pose();
    const auto & v = path->twist();
    const auto & q = p.orientation();
    const double yaw = std::atan2(
      2.0 * (q.w() * q.z() + q.x() * q.y()), 1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z()));
    const auto world_velocity = gz::math::Quaterniond(
      q.w(), q.x(), q.y(), q.z()).RotateVector(
      gz::math::Vector3d(v.linear().x(), v.linear().y(), v.linear().z()));
    if (path_hz_ > 0.0) {
      // Head angles ride in the path's twist (angular x yaw, y pitch), set directly so the
      // head's PID and its reaction on the chassis cannot depend on message timing.
      const double heads[2] = {v.angular().x(), v.angular().y()};
      const char * names[2] = {"headlink", "headpitch"};
      for (int i = 0; i < 2; ++i) {
        const auto joint = model_.JointByName(ecm, names[i]);
        if (joint == gz::sim::kNullEntity) {continue;}
        auto * reset = ecm.Component<gz::sim::components::JointPositionReset>(joint);
        if (reset) {
          *reset = gz::sim::components::JointPositionReset({heads[i]});
        } else {
          ecm.CreateComponent(joint, gz::sim::components::JointPositionReset({heads[i]}));
        }
        // Kinematic: no velocity left to integrate between steps.
        auto * still = ecm.Component<gz::sim::components::JointVelocityReset>(joint);
        if (still) {
          *still = gz::sim::components::JointVelocityReset({0.0});
        } else {
          ecm.CreateComponent(joint, gz::sim::components::JointVelocityReset({0.0}));
        }
      }
    }
    model_.SetWorldPoseCmd(
      ecm, gz::math::Pose3d(
        p.position().x() + world_velocity.X() * dt,
        p.position().y() + world_velocity.Y() * dt, z_,
        0.0, 0.0, yaw + v.angular().z() * dt));
  }

private:
  void OnPath(const gz::msgs::Odometry & msg)
  {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      path_ = msg;
      const int64_t stamp = static_cast<int64_t>(msg.header().stamp().sec()) * 1000000000 +
        msg.header().stamp().nsec();
      samples_[stamp] = msg;
      first_stamp_ = std::min(first_stamp_, stamp);
      while (samples_.size() > 32) {samples_.erase(samples_.begin());}
    }
    cv_.notify_all();
  }

  gz::sim::Model model_{gz::sim::kNullEntity};
  double z_ = 0.0, stale_s_ = 0.1, path_hz_ = 0.0;
  std::condition_variable cv_;
  gz::transport::Node node_;
  std::mutex mutex_;  // gz-transport calls OnPath on its own thread
  std::optional<gz::msgs::Odometry> path_;
  std::map<int64_t, gz::msgs::Odometry> samples_;  // by stamp, the newest few
  int64_t first_stamp_ = std::numeric_limits<int64_t>::max();  // earliest stamp ever received
};

}  // namespace sim

GZ_ADD_PLUGIN(
  ::sim::OpponentMover, gz::sim::System, gz::sim::ISystemConfigure, gz::sim::ISystemPreUpdate)
