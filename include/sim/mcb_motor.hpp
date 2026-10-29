// Copyright 2026 Thornbots. SPDX-License-Identifier: Apache-2.0
#ifndef SIM__MCB_MOTOR_HPP_
#define SIM__MCB_MOTOR_HPP_
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <optional>

namespace sim::mcb_motor {
constexpr double kPi = 3.14159265358979323846;
constexpr double kDriveRatio = 3591.0 / 187.0;
constexpr double kWheelRadius = 0.094902;
constexpr double kTrack = 0.50038;
constexpr double kYawRatio = 32319.0 / 748.0;
constexpr double kPitchRatio = 3.0;
using Feedback = std::array<std::array<uint8_t, 8>, 10>;
using Commands = std::array<int16_t, 10>;

struct ProjectileExit {
  // Sentry's estimated 36:1 indexer gearbox and eight chambers.
  static constexpr double kShaftTurnsPerBall = 36.0 / 8;
  // A ball leaves kExitTravel into a feed that starts from the firmware's
  // pre-indexed rest (n + kRestAt balls, INITIAL_INDEX_OFFSET): ~22 ms after
  // the request, near the robot's timed 20 ms (2026-10-28). One per feed, so
  // overshoot doesn't count twice; the plant's acceleration is an estimate.
  static constexpr double kRestAt = 0.20, kRestTolerance = 0.1, kExitTravel = 0.045;
  static constexpr int kStillCounts = 2, kStillUpdates = 10;  // encoder counts, updates
  int previous = 0, still = 0;
  double turns = 0;
  std::optional<double> rest;  // armed: where the shaft last settled, in balls
  uint32_t update(const std::array<uint8_t, 8> &feedback) {
    const int encoder = static_cast<int>(feedback[0]) << 8 | feedback[1];
    const int delta = (encoder - previous + 12288) % 8192 - 4096;
    previous = encoder;
    // The index motor is inverted; reverse/unjam movement does not add shots.
    turns -= delta / 8192.0;
    const double balls = turns / kShaftTurnsPerBall;
    still = std::abs(delta) <= kStillCounts ? still + 1 : 0;
    if (still >= kStillUpdates) {
      const double phase = balls - std::floor(balls);
      rest = std::abs(phase - kRestAt) <= kRestTolerance ? std::optional<double>(balls)
                                                          : std::nullopt;
    }
    if (rest && balls - *rest >= kExitTravel) {
      rest.reset();
      return 1;
    }
    return 0;
  }
};

inline std::array<uint8_t, 8> packet(double angle, double speed, double current) {
  const double turn = angle / (2 * kPi);
  const auto encoder = static_cast<uint16_t>((turn - std::floor(turn)) * 8192);
  const auto rpm = static_cast<int16_t>(std::clamp(std::round(speed * 30 / kPi), -32768.0, 32767.0));
  const auto amps = static_cast<int16_t>(std::clamp(std::round(current * 819.2), -32768.0, 32767.0));
  const auto word = [](int16_t value) { return static_cast<uint16_t>(value); };
  return {static_cast<uint8_t>(encoder >> 8), static_cast<uint8_t>(encoder),
          static_cast<uint8_t>(word(rpm) >> 8), static_cast<uint8_t>(word(rpm)),
          static_cast<uint8_t>(word(amps) >> 8), static_cast<uint8_t>(word(amps)), 25, 0};
}

struct Plant {
  Commands commands{};
  std::array<double, 10> angle{}, speed{}, current{};
  std::array<double, 4> wheel_speed{};
  // Effective roller/bearing resistance; estimated, not a measured tire coefficient.
  double rolling_resistance = 0.03;

  // readings use the firmware's pod axes; chassis twist is in ROS body axes.
  Feedback feedback(const std::array<double, 10> &readings,
                    double forward, double left, double spin, double dt) {
    angle[0] = readings[6] * kYawRatio;
    speed[0] = readings[7] * kYawRatio;
    angle[1] = -(readings[8] - 1.62) * kPitchRatio;
    speed[1] = -readings[9] * kPitchRatio;
    const double right = -left;
    const double scale = kDriveRatio / kWheelRadius;
    const std::array<double, 4> wheel = {
        (right + forward) / std::sqrt(2.0), (-right + forward) / std::sqrt(2.0),
        (-right - forward) / std::sqrt(2.0), (right - forward) / std::sqrt(2.0)};
    for (size_t i = 0; i < 4; ++i) {
      wheel_speed[i] = wheel[i] - kTrack * spin / 2;
      speed[i + 6] = scale * wheel_speed[i];
      angle[i + 6] += speed[i + 6] * dt;
    }
    for (size_t i = 0; i < 10; ++i) {
      if (i == 1) {
        current[i] = (commands[i] / 1000.0 - 0.716 * speed[i]) / 8.705;
      } else {
        current[i] = commands[i] / 819.2;
      }
      if (i >= 2 && i < 6) {
        // Unmeshed rotors: M3508 torque, estimated inertia and viscous load.
        speed[i] += (0.02299 * current[i] - 0.0001 * speed[i]) * dt / 0.00005;
        angle[i] += speed[i] * dt;
        // A loaded cartridge stops reverse indexing at home; current remains
        // present so the firmware can detect the stall through CAN feedback.
        if (i == 2 && angle[i] > 0) {
          angle[i] = 0;
          speed[i] = 0;
        }
      }
    }
    Feedback result{};
    for (size_t i = 0; i < 10; ++i) result[i] = packet(angle[i], speed[i], current[i]);
    return result;
  }

  std::array<double, 2> gimbal_effort(double yaw_rate, double pitch_rate) const {
    const double yaw = 0.02299 * kYawRatio * commands[0] / 819.2 - 0.12 * yaw_rate;
    const double pitch_current = (-commands[1] / 1000.0 - 0.716 * kPitchRatio * pitch_rate) / 8.705;
    return {yaw, 0.741 * kPitchRatio * pitch_current};
  }

  std::array<double, 3> chassis_wrench(double mass = 15.68) const {
    std::array<double, 4> force{};
    for (size_t i = 0; i < 4; ++i) {
      force[i] = 0.02299 * kDriveRatio / kWheelRadius * commands[i + 6] / 819.2;
      // Dissipate rolling motion even with zero motor current; no position constraint.
      force[i] -= rolling_resistance * mass * 9.81 / 4 *
                  std::tanh(wheel_speed[i] / 0.001);
      // Estimated tire traction, shared evenly; contacts still belong to gz.
      force[i] = std::clamp(force[i], -mass * 9.81 * 0.9 / 4, mass * 9.81 * 0.9 / 4);
    }
    return {(force[0] + force[1] - force[2] - force[3]) / std::sqrt(2.0),
            -(force[0] - force[1] - force[2] + force[3]) / std::sqrt(2.0),
            -kTrack / 2 * (force[0] + force[1] + force[2] + force[3])};
  }
};
}  // namespace sim::mcb_motor
#endif  // SIM__MCB_MOTOR_HPP_
