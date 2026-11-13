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

#ifndef SIM__MCB_AIM_HPP_
#define SIM__MCB_AIM_HPP_

#include <cstdint>
#include <optional>

// The firmware's pure planner, compiled from the checked-out firmware/MCBV3
// (subsystems/jetson); never copied into sim. See README "MCB aim stand-in".
#include "AimPlanner.hpp"
#include "sim/mcb_aim_topics.hpp"

namespace sim::mcb_aim
{

// Newest odometry, raw aim odom frame, as POSE carries it.
struct Pose
{
  double x = 0.0, y = 0.0, vel_x = 0.0, vel_y = 0.0;
};

struct Config
{
  aim::AimTuning tuning;     // AimTuning's defaults are the sentry firmware's values
  double shooter_z = 0.0;    // pitch-pivot height the planner shoots from, odom z
  bool fire_enabled = true;  // false: aim only, as the estimation bench
  double idle_valid_s = 0.2; // AutoAimAndFireCommand::TARGET_VALID_TIME
};

// An indexer request. The shot leaves latency_s after request_s.
struct Fire
{
  double request_s;      // sim time of the request: plan time + request delay
  double latency_s;      // request to chamber exit (FIRING_LATENCY_TIME)
  double planned_delay_s;  // plan time to chamber exit
};

struct Output
{
  std::optional<aim::Vec3> aim;  // the point the gimbal is commanded through
  std::optional<Fire> fire;      // a request that came due this tick
  bool replanned = false;
};

// AutoAimAndFireCommand's target-state path in sim time, advanced once per
// replan period (the bench grid): a held state, a replan unless a shot is
// pending, the pending shot holding its aim, state timeout, then the idle
// look point. Differences from the firmware, which free-runs a 1 ms control
// cycle: a replan waits for the next period boundary after a request comes
// due, and the shooter's pose comes from the newest POSE sample.
class Standin
{
public:
  explicit Standin(const Config & config = {});
  const Config & config() const {return config_;}
  // A TargetState arrived; ages are those at receipt.
  void on_state(uint32_t now_ms, const aim::TargetInput & at_receipt);
  // An idle CV_TARGET look point arrived.
  void on_idle(uint32_t now_ms, const aim::Vec3 & point);
  Output tick(uint32_t now_ms, const Pose & pose);
  bool holding_state() const {return have_state_;}

private:
  Config config_;
  aim::AimPlanner planner_;
  aim::TargetInput held_;
  uint32_t received_ms_ = 0;
  bool have_state_ = false;
  aim::Vec3 held_aim_;
  bool pending_ = false;
  uint32_t plan_ms_ = 0, due_ms_ = 0;
  std::optional<aim::Vec3> idle_;
  uint32_t idle_ms_ = 0;
};

}  // namespace sim::mcb_aim

#endif  // SIM__MCB_AIM_HPP_
