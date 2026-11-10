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

#include "sim/mcb_aim.hpp"

#include <algorithm>

namespace sim::mcb_aim
{

namespace
{
// Age of a frame received at received_ms, as the firmware adds time since receipt.
aim::TargetInput aged(const aim::TargetInput & held, uint32_t received_ms, uint32_t now_ms)
{
  const float since =
    std::max<int32_t>(static_cast<int32_t>(now_ms - received_ms), 0) * 0.001f;
  aim::TargetInput in = held;
  in.state_age_s += since;
  in.detection_age_s += since;
  return in;
}
}  // namespace

Standin::Standin(const Config & config)
: config_(config), planner_(config.tuning)
{
}

void Standin::on_state(uint32_t now_ms, const aim::TargetInput & at_receipt)
{
  held_ = at_receipt;
  received_ms_ = now_ms;
  have_state_ = true;
  idle_.reset();  // a held state outranks the idle point
  planner_.observe(now_ms, at_receipt);
}

void Standin::on_idle(uint32_t now_ms, const aim::Vec3 & point)
{
  idle_ = point;
  idle_ms_ = now_ms;
}

Output Standin::tick(uint32_t now_ms, const Pose & pose)
{
  Output out;
  const auto & tuning = planner_.tuning();
  if (have_state_ && aged(held_, received_ms_, now_ms).state_age_s >= tuning.target_timeout_s) {
    have_state_ = false;
    planner_.reset();
  }
  // A request comes due once; it is released whether or not it can fire.
  if (pending_ && static_cast<int32_t>(now_ms - due_ms_) >= 0) {
    pending_ = false;
    if (config_.fire_enabled) {
      const double latency = tuning.firmware_latency_s;
      // Stamped at the ideal request time, not the tick that noticed it.
      out.fire = Fire{due_ms_ * 0.001, latency, (due_ms_ - plan_ms_) * 0.001 + latency};
    }
  }
  if (have_state_ && !pending_) {
    const aim::TargetInput in = aged(held_, received_ms_, now_ms);
    // The planner measures time from the state's stamp, so the shooter starts where it was then.
    const aim::Vec3 vel{static_cast<float>(pose.vel_x), static_cast<float>(pose.vel_y), 0.0f};
    const aim::Vec3 pos = aim::Vec3{static_cast<float>(pose.x), static_cast<float>(pose.y),
        static_cast<float>(config_.shooter_z)} - vel * in.state_age_s;
    const aim::Plan plan = planner_.plan(now_ms, in, pos, vel);
    held_aim_ = plan.aim;
    out.replanned = true;
    if (plan.fire && config_.fire_enabled) {
      pending_ = true;
      plan_ms_ = now_ms;
      due_ms_ = now_ms + static_cast<uint32_t>(plan.request_delay_ms);
    }
  }
  if (have_state_ || pending_) {
    out.aim = held_aim_;
  } else if (idle_ && static_cast<int32_t>(now_ms - idle_ms_) <
    static_cast<int32_t>(config_.idle_valid_s * 1000.0 + 0.5))
  {
    out.aim = idle_;
  }
  return out;
}

}  // namespace sim::mcb_aim
