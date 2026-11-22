// Copyright 2026 Thornbots
// SPDX-License-Identifier: Apache-2.0
#ifndef SIM__REFEREE_LAUNCHER_HPP_
#define SIM__REFEREE_LAUNCHER_HPP_

#include <algorithm>
#include <cstdint>
#include <limits>

namespace sim
{

// Referee accounting for the sentry's 17 mm launcher.
class RefereeLauncher
{
public:
  static constexpr uint16_t kHeatLimit = 260;
  static constexpr uint16_t kGameLockHeat = 360;
  static constexpr uint16_t kCoolingPerSecond = 30;
  static constexpr uint16_t kHeatPerShot = 10;
  static constexpr uint32_t kTickMs = 100;

  void cool(uint32_t now_ms)
  {
    const uint32_t ticks = (now_ms - tick_ms_) / kTickMs;
    tick_ms_ += ticks * kTickMs;
    heat_ -= std::min<uint32_t>(heat_, ticks * (kCoolingPerSecond / 10));
    if (heat_ == 0) {recovering_ = false;}
  }

  bool record_exit()
  {
    const bool had_power = powered();
    heat_ = std::min<unsigned>(heat_ + kHeatPerShot, std::numeric_limits<uint16_t>::max());
    recovering_ |= heat_ > kHeatLimit;
    game_lock_ |= heat_ > kGameLockHeat;
    return had_power;
  }

  void on_defeat()
  {
    heat_ = 0;
    recovering_ = false;
  }

  bool powered() const {return !recovering_ && !game_lock_;}
  uint16_t heat() const {return heat_;}

private:
  uint32_t tick_ms_ = 0;
  uint16_t heat_ = 0;
  bool recovering_ = false;
  bool game_lock_ = false;
};

}  // namespace sim

#endif  // SIM__REFEREE_LAUNCHER_HPP_
