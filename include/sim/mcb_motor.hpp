// Copyright 2026 Thornbots. SPDX-License-Identifier: Apache-2.0
#ifndef SIM__MCB_MOTOR_HPP_
#define SIM__MCB_MOTOR_HPP_
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>

namespace sim::mcb_motor {
constexpr double kPi = 3.14159265358979323846;
constexpr double kDriveRatio = 3591.0 / 187.0;
constexpr double kWheelRadius = 0.094902;
constexpr double kTrack = 0.50038;
constexpr double kYawRatio = 32319.0 / 748.0;
constexpr double kPitchRatio = 3.0;
// Sentry's encoder offset in GimbalSubsystemConstants.hpp; verified through POSE.
constexpr double kPitchOffset = 0.18;
using Feedback = std::array<std::array<uint8_t, 8>, 10>;
using Commands = std::array<int16_t, 10>;

struct SentryIndexer {
  // Output-shaft SI constants from SENTRY IndexerControllerConstants.hpp;
  // C610/IndexerUnit commands and measured current use 1000 counts per amp.
  static constexpr double kRatio = 36, kChambers = 8, kCountsPerAmp = 1000;
  static constexpr double kKt = 0.3125, kInertia = 0.004, kViscous = 0, kFriction = 0;
  static constexpr double kBackEmf = 0.4324, kResistance = 0.906;
  static constexpr double kVoltageLimit = 24, kCurrentLimit = 10;

  // Quasistatic current drive with voltage saturation and regenerative braking;
  // winding inductance, switching and protection transients are not modeled.
  static double current(int16_t command, double output_speed) {
    const double requested = std::clamp(command / kCountsPerAmp, -kCurrentLimit, kCurrentLimit);
    const double emf = kBackEmf * output_speed;
    const double voltage = std::clamp(kResistance * requested + emf, -kVoltageLimit, kVoltageLimit);
    return std::clamp((voltage - emf) / kResistance, -kCurrentLimit, kCurrentLimit);
  }
};

struct ProjectileExit {
  static constexpr double kShaftTurnsPerBall = SentryIndexer::kRatio / SentryIndexer::kChambers;
  // Infer the release plane from the robot's measured 20 ms delay (2026-10-28)
  // and the declared initial full-current acceleration, not the old M3508 plant.
  // This is estimated geometry: ~0.199 chambers from the pre-indexed rest.
  // Every further forward crossing is one chamber apart, including overshoot.
  static constexpr double kMeasuredExitDelay = 0.020;
  static constexpr double kRestAt = 0.20, kRestTolerance = 0.1;
  static constexpr double kExitTravel = 0.5 * SentryIndexer::kKt * SentryIndexer::kCurrentLimit /
      SentryIndexer::kInertia * kMeasuredExitDelay * kMeasuredExitDelay /
      (2 * kPi / SentryIndexer::kChambers);
  int previous = 0;
  double turns = 0;
  // The plant starts at the loaded cartridge's mechanical home (shaft zero).
  // Fixed geometry does not require a stopped motor to establish its first plane.
  double next_exit = kRestAt + kExitTravel;
  uint32_t update(const std::array<uint8_t, 8> &feedback) {
    const int encoder = static_cast<int>(feedback[0]) << 8 | feedback[1];
    const int delta = (encoder - previous + 12288) % 8192 - 4096;
    previous = encoder;
    // The index motor is inverted; reverse/unjam movement does not add shots.
    turns -= delta / 8192.0;
    const double balls = turns / kShaftTurnsPerBall;
    uint32_t exits = 0;
    while (balls >= next_exit) {
      ++exits;
      next_exit += 1.0;
    }
    return exits;
  }
};

inline std::array<uint8_t, 8> packet(double angle, double speed, double current,
                                    double counts_per_amp = 819.2) {
  const double turn = angle / (2 * kPi);
  const auto encoder = static_cast<uint16_t>((turn - std::floor(turn)) * 8192);
  const auto rpm = static_cast<int16_t>(std::clamp(std::round(speed * 30 / kPi), -32768.0, 32767.0));
  const auto amps = static_cast<int16_t>(std::clamp(std::round(current * counts_per_amp), -32768.0, 32767.0));
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
    angle[1] = -(readings[8] + kPitchOffset) * kPitchRatio;
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
      } else if (i == 2) {
        current[i] = SentryIndexer::current(commands[i], speed[i] / SentryIndexer::kRatio);
      } else {
        current[i] = commands[i] / 819.2;
      }
      if (i >= 2 && i < 6) {
        if (i == 2) {
          // Integrate at the output shaft, then return rotor coordinates for CAN.
          const double output_speed = speed[i] / SentryIndexer::kRatio;
          const double friction = SentryIndexer::kViscous * output_speed +
              (output_speed == 0 ? 0 : std::copysign(SentryIndexer::kFriction, output_speed));
          speed[i] += SentryIndexer::kRatio *
              (SentryIndexer::kKt * current[i] - friction) * dt / SentryIndexer::kInertia;
        } else {
          // Other unmeshed rotors retain estimated M3508 inertia and viscous load.
          speed[i] += (0.02299 * current[i] - 0.0001 * speed[i]) * dt / 0.00005;
        }
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
    for (size_t i = 0; i < 10; ++i)
      result[i] = packet(angle[i], speed[i], current[i], i == 2 ? SentryIndexer::kCountsPerAmp : 819.2);
    return result;
  }

  // Motor-side electromagnetic torque, paired with the diagnostic rotor angle.
  double rotor_torque(size_t i) const {
    const double kt = i == 2 ? SentryIndexer::kKt / SentryIndexer::kRatio :
        (i == 1 ? 0.741 : 0.02299);
    return kt * current[i];
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
