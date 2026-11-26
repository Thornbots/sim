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

#include "sim/referee_launcher.hpp"

namespace sim::mcb_aim
{

Standin::Standin(const Config & config)
: config_(config), control_(config.tuning)
{
}

void Standin::on_state(uint32_t now_ms, const aim::TargetInput & in)
{
  control_.observe(now_ms, in);
}

void Standin::on_idle(uint32_t now_ms, const aim::Vec3 & point)
{
  idle_ = point;
  idle_ms_ = now_ms;
}

void Standin::set_firing_window(uint32_t start_ms, uint32_t stop_ms)
{
  fire_start_ms_ = start_ms;
  fire_duration_ms_ = stop_ms - start_ms;
  last_request_ms_.reset();
  heat_.reset(start_ms - start_ms % RefereeLauncher::kTickMs);
}

Output Standin::tick(uint32_t now_ms, const Pose & pose,
  std::optional<aim::GimbalMotion> measured)
{
  Output out;
  const aim::Vec3 vel{static_cast<float>(pose.vel_x), static_cast<float>(pose.vel_y), 0};
  const aim::Vec3 pivot{static_cast<float>(pose.x), static_cast<float>(pose.y),
    static_cast<float>(config_.shooter_z)};
  const auto plan = control_.tick(now_ms, pivot, vel);
  out.replanned = control_.replanned();
  if (holding_state()) {
    if (plan.valid) {
      out.aim = plan.aim;
      const auto actual = measured.value_or(
        aim::GimbalMotion{plan.yaw, plan.pitch, plan.yaw_rate, plan.pitch_rate});
      using namespace subsystems;
      const bool enabled = config_.fire_enabled &&
        (!fire_start_ms_ || now_ms - *fire_start_ms_ < fire_duration_ms_);
      if (enabled) {heat_.cool(now_ms, RefereeLauncher::kCoolingPerSecond);}
      if (enabled &&
        heat_.canCharge(RefereeLauncher::kHeatLimit, kHeatPer17mm * kSentryEstimatedRounds) &&
        (!last_request_ms_ || now_ms - *last_request_ms_ >= kSentryShotIntervalMs) &&
        control_.can_fire(now_ms, pivot, vel, actual))
      {
        heat_.charge(kHeatPer17mm);
        last_request_ms_ = now_ms;
        out.fire = Fire{now_ms*0.001, config_.tuning.firmware_latency_s,
          config_.tuning.firmware_latency_s};
      }
    }
  } else if (idle_ && now_ms-idle_ms_ <
    static_cast<uint32_t>(config_.idle_valid_s*1000+0.5))
  {
    out.aim = idle_;
  }
  return out;
}

}  // namespace sim::mcb_aim
