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
// held state's time. Each hold is announced on <held_topic>. The coordinator
// repeats its release while it waits on slow producers, so only <max_wait_s>
// wall with no release at all (a dead coordinator) is a lockstep timeout:
// logged, sent on <timeout_topic>, and physics runs free until the next
// release. see README.md for design rationale

#include <gz/msgs/time.pb.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <limits>
#include <memory>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>

#include <gz/common/Console.hh>
#include <gz/plugin/Register.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Events.hh>
#include <gz/common/SignalHandler.hh>
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
    gz::sim::EntityComponentManager &, gz::sim::EventManager & events) override
  {
    events_ = &events;
    // The coordinator exits with the rest of the stack, so a gate holding for
    // its release would sit out max_wait_s (30 s in epoch-zero mode) and gz
    // would outlive launch's SIGINT and SIGTERM windows.
    signals_.AddCallback([stopping = stopping_](int) {*stopping = true;});
    const auto release = sdf->Get<std::string>("release_topic", "/sim/lockstep/release").first;
    held_pub_ = node_.Advertise<gz::msgs::Time>(
      sdf->Get<std::string>("held_topic", "/sim/lockstep/held").first);
    timeout_pub_ = node_.Advertise<gz::msgs::Time>(
      sdf->Get<std::string>("timeout_topic", "/sim/lockstep/timeout").first);
    max_wait_ = std::chrono::duration<double>(sdf->Get<double>("max_wait_s", 2.0).first);
    // Held at 0 until the first release, or free until then.
    strict_startup_ = sdf->Get<bool>("strict_startup", false).first;
    epoch_zero_pending_ = sdf->Get<bool>("epoch_zero_startup", false).first;
    if (epoch_zero_pending_ && !strict_startup_) {
      throw std::invalid_argument("epoch_zero_startup requires strict_startup");
    }
    const bool hold_at_start = strict_startup_ || sdf->Get<bool>("hold_at_start", false).first;
    released_ns_ = hold_at_start ? 0 : kFree;
    node_.Subscribe(release, &LockstepGate::OnRelease, this);
    gzmsg << "LockstepGate: releases on " << release << ", "
          << (hold_at_start ? "held" : "free") << " until the first" << std::endl;
  }

  void PostUpdate(const gz::sim::UpdateInfo & info, const gz::sim::EntityComponentManager &) override
  {
    const int64_t t = std::chrono::duration_cast<std::chrono::nanoseconds>(info.simTime).count();
    std::unique_lock<std::mutex> lock(mutex_);
    // Paused updates must return so scene/create services can finish startup.
    if (epoch_zero_pending_) {
      if (!info.paused || t != 0) {
        gzerr << "LockstepGate: epoch zero startup requires a paused world at zero; stopping"
              << std::endl;
        timeout_pub_.Publish(to_msg(t));
        events_->Emit<gz::sim::events::Stop>();
        return;
      }
      const auto now = std::chrono::steady_clock::now();
      if (!startup_deadline_) {
        startup_deadline_ = now +
          std::chrono::duration_cast<std::chrono::steady_clock::duration>(max_wait_);
      }
      if (now >= *startup_deadline_) {
        gzerr << "LockstepGate: lockstep timeout held at epoch zero; stopping strict session"
              << std::endl;
        timeout_pub_.Publish(to_msg(0));
        events_->Emit<gz::sim::events::Stop>();
        return;
      }
      if (released_ns_ > 0) {
        epoch_zero_pending_ = false;
        events_->Emit<gz::sim::events::Pause>(false);
      } else {
        held_pub_.Publish(to_msg(0));
      }
      return;
    }
    if (info.paused) {return;}
    if (t < released_ns_) {
      return;
    }
    held_pub_.Publish(to_msg(t));
    const auto wait = std::chrono::duration_cast<std::chrono::steady_clock::duration>(max_wait_);
    // Silence is measured from the later of this hold and the last release (a repeat counts).
    heard_ = std::max(heard_, std::chrono::steady_clock::now());
    // Wait in short slices so a stop request is noticed within 10 ms.
    while (released_ns_ <= t && !*stopping_ && std::chrono::steady_clock::now() < heard_ + wait) {
      cv_.wait_until(lock, std::min(heard_ + wait, std::chrono::steady_clock::now() +
        std::chrono::milliseconds(10)));
      if (strict_startup_ && released_ns_ <= t) {held_pub_.Publish(to_msg(t));}
    }
    if (*stopping_) {
      events_->Emit<gz::sim::events::Stop>();
      return;
    }
    const bool released = released_ns_ > t;
    if (!released) {
      ++timeouts_;
      if (strict_startup_) {
        gzerr << "LockstepGate: lockstep timeout held at " << t * 1e-9
              << " s; stopping strict session" << std::endl;
        timeout_pub_.Publish(to_msg(t));
        events_->Emit<gz::sim::events::Stop>();
        return;
      }
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
      heard_ = std::chrono::steady_clock::now();
    }
    cv_.notify_all();
  }

  gz::transport::Node::Publisher held_pub_, timeout_pub_;
  std::chrono::duration<double> max_wait_{2.0};
  std::chrono::steady_clock::time_point heard_{};  // last release, or this hold's start
  std::mutex mutex_;  // gz-transport calls OnRelease on its own thread
  std::condition_variable cv_;
  int64_t released_ns_ = kFree;
  int timeouts_ = 0;
  bool strict_startup_ = false;
  bool epoch_zero_pending_ = false;
  std::optional<std::chrono::steady_clock::time_point> startup_deadline_;
  gz::sim::EventManager * events_ = nullptr;
  gz::common::SignalHandler signals_;
  std::shared_ptr<std::atomic<bool>> stopping_ = std::make_shared<std::atomic<bool>>(false);
  // Destroy transport callbacks before the mutex and release state.
  gz::transport::Node node_;
};

}  // namespace sim

GZ_ADD_PLUGIN(
  ::sim::LockstepGate, gz::sim::System, gz::sim::ISystemConfigure, gz::sim::ISystemPostUpdate)
