// Copyright 2026 Thornbots
// Licensed under the Apache License, Version 2.0.

// Framing and smoothing for the FollowCamera gz GUI plugin, free of Qt and
// rendering so it can be unit tested. The camera keeps a fixed world yaw and
// pitch, aims at the midpoint of our robot and its framed target, and backs
// off until both fit; a critically damped spring with velocity feed-forward
// moves it in sim time. see README.md#follow-camera for design rationale

#ifndef SIM__FOLLOW_CAMERA_HPP_
#define SIM__FOLLOW_CAMERA_HPP_

#include <algorithm>
#include <cmath>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include <gz/math/Vector3.hh>

namespace sim::follow_camera
{

using gz::math::Vector3d;

struct Settings
{
  double yaw = 2.35;          // camera heading, rad (world)
  double hfov = 0.8;          // rad; narrower than gz's default flattens perspective
  double pitch = 0.72;        // downward tilt, rad
  double look_height = 0.25;  // m, aim height above the floor
  double pad = 0.6;           // m around each framed robot
  double fill = 0.8;          // framed extent / field of view
  double min_width = 4.5;     // m, narrowest view width at the aim point
  double max_width = 10.0;    // m, widest view width at the aim point
  double max_span = 7.0;      // m, targets farther than this are not framed
  double switch_ratio = 0.75;  // new target must be this much nearer to switch
  double settle_s = 0.45;     // spring time constant for the aim point
  double zoom_settle_s = 0.9;  // spring time constant for the distance
  double velocity_s = 0.25;   // low-pass time constant for robot velocities
};

struct Lens
{
  double tan_half_h;
  double tan_half_v;
};

inline Vector3d forward(const Settings & s)
{
  return {std::cos(s.pitch) * std::cos(s.yaw), std::cos(s.pitch) * std::sin(s.yaw),
          -std::sin(s.pitch)};
}

// Distance from the aim point that fits every point (padded) in view.
inline double fit_distance(const Settings & s, const Lens & lens, const Vector3d & aim,
                           const std::vector<Vector3d> & points)
{
  const Vector3d f = forward(s);
  const Vector3d right = f.Cross(Vector3d::UnitZ).Normalized();
  const Vector3d up = right.Cross(f);
  const double min_d = s.min_width / (2 * lens.tan_half_h);
  const double max_d = s.max_width / (2 * lens.tan_half_h);
  double d = min_d;
  for (const auto & p : points) {
    const Vector3d v = p - aim;
    const double depth = v.Dot(f);
    d = std::max(d, (std::abs(v.Dot(right)) + s.pad) / (s.fill * lens.tan_half_h) - depth);
    d = std::max(d, (std::abs(v.Dot(up)) + s.pad) / (s.fill * lens.tan_half_v) - depth);
  }
  return std::clamp(d, min_d, max_d);
}

// One critically damped spring axis tracking a goal that reaches `goal` at
// the end of the step moving at goal_v. Exact solution: stable for any dt and
// no lag behind constant velocity.
struct Spring
{
  double x = 0, v = 0;
  void step(double goal, double goal_v, double tau, double dt)
  {
    const double w = 1 / tau;
    const double e = x - (goal - goal_v * dt), de = v - goal_v;
    const double k = de + w * e, decay = std::exp(-w * dt);
    x = goal + (e + k * dt) * decay;
    v = goal_v + (de - w * k * dt) * decay;
  }
};

// Smoothed robot velocity from successive sim-stamped positions.
struct Track
{
  Vector3d position, velocity;
  bool valid = false;
  void update(const Vector3d & p, double dt, double tau)
  {
    if (valid && dt > 0) {
      const double a = 1 - std::exp(-dt / tau);
      velocity += a * ((p - position) / dt - velocity);
    } else if (!valid) {
      velocity = Vector3d::Zero;
    }
    position = p;
    valid = true;
  }
};

class Framer
{
public:
  explicit Framer(Settings settings = {}) : s_(settings) {}

  const Settings & settings() const { return s_; }
  const std::optional<std::string> & target() const { return target_; }

  // Advance by dt sim seconds. Targets are (name, position) of the robots we
  // may frame; returns the camera position. reset snaps to the goal.
  Vector3d update(const Vector3d & robot, const std::vector<std::pair<std::string, Vector3d>> &
                  targets, const Lens & lens, double dt, bool reset)
  {
    if (reset) {
      robot_.valid = false;
      tracks_.clear();
      target_.reset();
    }
    robot_.update(robot, dt, s_.velocity_s);
    choose(robot, targets);
    Vector3d goal = robot, goal_v = robot_.velocity;
    std::vector<Vector3d> framed{robot};
    std::vector<std::pair<std::string, Track>> kept;
    for (const auto & [name, p] : targets) {
      Track track = find(name);
      track.update(p, dt, s_.velocity_s);
      kept.emplace_back(name, track);
      if (target_ && name == *target_) {
        goal = 0.5 * (robot + p);
        goal_v = 0.5 * (robot_.velocity + track.velocity);
        framed.push_back(p);
      }
    }
    tracks_ = std::move(kept);
    goal.Z(s_.look_height);
    goal_v.Z(0);
    const double distance = fit_distance(s_, lens, goal, framed);
    if (reset || !started_) {
      for (int i = 0; i < 3; ++i) aim_[i] = {goal[i], goal_v[i]};
      distance_ = {distance, 0};
      started_ = true;
    } else if (dt > 0) {
      for (int i = 0; i < 3; ++i) aim_[i].step(goal[i], goal_v[i], s_.settle_s, dt);
      distance_.step(distance, 0, s_.zoom_settle_s, dt);
    }
    return aim() - distance_.x * forward(s_);
  }

  Vector3d aim() const { return {aim_[0].x, aim_[1].x, aim_[2].x}; }

private:
  void choose(const Vector3d & robot, const std::vector<std::pair<std::string, Vector3d>> & targets)
  {
    std::optional<std::string> nearest;
    double best = s_.max_span, current = INFINITY;
    for (const auto & [name, p] : targets) {
      const double d = (p - robot).Length();
      if (target_ && name == *target_) current = d;
      if (d < best) best = d, nearest = name;
    }
    if (current > s_.max_span || (nearest && best < s_.switch_ratio * current)) {
      target_ = nearest;
    }
  }

  Track find(const std::string & name) const
  {
    for (const auto & [n, t] : tracks_) if (n == name) return t;
    return {};
  }

  Settings s_;
  Track robot_;
  std::vector<std::pair<std::string, Track>> tracks_;
  std::optional<std::string> target_;
  Spring aim_[3], distance_;
  bool started_ = false;
};

}  // namespace sim::follow_camera

#endif  // SIM__FOLLOW_CAMERA_HPP_
