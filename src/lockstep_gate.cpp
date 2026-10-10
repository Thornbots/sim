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

// A world system that holds physics at the time lockstep_coordinator last
// released it to (gz.msgs.Time on <release_topic>), so a slow machine waits
// instead of computing something else. It holds in PostUpdate: gz publishes
// /clock before a step's PreUpdate, so the clock nodes see while held is the
// held state's time. Each hold is announced on <held_topic>. A wait past
// <max_wait_s> wall is a lockstep timeout: logged, sent on <timeout_topic>,
// and physics runs free until the next release. see README.md for design rationale

#include <gz/msgs/time.pb.h>

#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <limits>
#include <memory>
#include <mutex>
#include <string>

#include <gz/common/Console.hh>
#include <gz/plugin/Register.hh>
#include <gz/sim/System.hh>
#include <gz/transport/Node.hh>

namespace sim
{

namespace
{
constexpr int64_t kFree = std::numeric_limits<int64_t>::max();

gz::msgs::Time to_msg(int64_t ns)
{
  gz::msgs::Time msg;
  msg.set_sec(ns / 1000000000);
  msg.set_nsec(ns % 1000000000);
  return msg;
}
}  // namespace

class LockstepGate
  : public gz::sim::System,
  public gz::sim::ISystemConfigure,
  public gz::sim::ISystemPostUpdate
{
public:
  void Configure(
    const gz::sim::Entity &, const std::shared_ptr<const sdf::Element> & sdf,
    gz::sim::EntityComponentManager &, gz::sim::EventManager &) override
  {
    const auto release = sdf->Get<std::string>("release_topic", "/sim/lockstep/release").first;
    held_pub_ = node_.Advertise<gz::msgs::Time>(
      sdf->Get<std::string>("held_topic", "/sim/lockstep/held").first);
    timeout_pub_ = node_.Advertise<gz::msgs::Time>(
      sdf->Get<std::string>("timeout_topic", "/sim/lockstep/timeout").first);
    max_wait_ = std::chrono::duration<double>(sdf->Get<double>("max_wait_s", 2.0).first);
    // Held at 0 until the first release, or free until then.
    const bool hold_at_start = sdf->Get<bool>("hold_at_start", false).first;
    released_ns_ = hold_at_start ? 0 : kFree;
    node_.Subscribe(release, &LockstepGate::OnRelease, this);
    gzmsg << "LockstepGate: releases on " << release << ", "
          << (hold_at_start ? "held" : "free") << " until the first" << std::endl;
  }

  void PostUpdate(const gz::sim::UpdateInfo & info, const gz::sim::EntityComponentManager &) override
  {
    if (info.paused) {
      return;
    }
    const int64_t t = std::chrono::duration_cast<std::chrono::nanoseconds>(info.simTime).count();
    std::unique_lock<std::mutex> lock(mutex_);
    if (t < released_ns_) {
      return;
    }
    held_pub_.Publish(to_msg(t));
    if (!cv_.wait_for(lock, max_wait_, [&] {return released_ns_ > t;})) {
      ++timeouts_;
      gzerr << "LockstepGate: lockstep timeout " << timeouts_ << " held at " << t * 1e-9
            << " s; running free until the next release" << std::endl;
      timeout_pub_.Publish(to_msg(t));
      released_ns_ = kFree;
    }
  }

private:
  void OnRelease(const gz::msgs::Time & msg)
  {
    const int64_t ns = msg.sec() * 1000000000LL + msg.nsec();
    {
      std::lock_guard<std::mutex> lock(mutex_);
      released_ns_ = ns;  // one in-order publisher; a release behind gz holds at the next step
    }
    cv_.notify_all();
  }

  gz::transport::Node::Publisher held_pub_, timeout_pub_;
  std::chrono::duration<double> max_wait_{2.0};
  std::mutex mutex_;  // gz-transport calls OnRelease on its own thread
  std::condition_variable cv_;
  int64_t released_ns_ = kFree;
  int timeouts_ = 0;
  // Destroy transport callbacks before the mutex and release state.
  gz::transport::Node node_;
};

}  // namespace sim

GZ_ADD_PLUGIN(
  ::sim::LockstepGate, gz::sim::System, gz::sim::ISystemConfigure, gz::sim::ISystemPostUpdate)
