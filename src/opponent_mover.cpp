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
// Python moved it in 4-36 cm jumps; VelocityControl moved only the first
// robot spawned (2026-09-29). see README.md for design rationale

#include <gz/msgs/odometry.pb.h>

#include <chrono>
#include <cmath>
#include <memory>
#include <mutex>
#include <optional>
#include <string>

#include <gz/math/Pose3.hh>
#include <gz/plugin/Register.hh>
#include <gz/sim/Model.hh>
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
    node_.Subscribe(topic, &OpponentMover::OnPath, this);
  }

  void PreUpdate(const gz::sim::UpdateInfo & info, gz::sim::EntityComponentManager & ecm) override
  {
    if (info.paused) {
      return;
    }
    std::optional<gz::msgs::Odometry> path;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      path = path_;
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
    model_.SetWorldPoseCmd(
      ecm, gz::math::Pose3d(
        p.position().x() + v.linear().x() * dt, p.position().y() + v.linear().y() * dt, z_,
        0.0, 0.0, yaw + v.angular().z() * dt));
  }

private:
  void OnPath(const gz::msgs::Odometry & msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    path_ = msg;
  }

  gz::sim::Model model_{gz::sim::kNullEntity};
  double z_ = 0.0, stale_s_ = 0.1;
  gz::transport::Node node_;
  std::mutex mutex_;  // gz-transport calls OnPath on its own thread
  std::optional<gz::msgs::Odometry> path_;
};

}  // namespace sim

GZ_ADD_PLUGIN(
  ::sim::OpponentMover, gz::sim::System, gz::sim::ISystemConfigure, gz::sim::ISystemPreUpdate)
