// Copyright 2026 Thornbots
// Licensed under the Apache License, Version 2.0.
#pragma once

#include <cmath>
#include <cstdint>
#include <string_view>

namespace sim::keyed_noise
{

inline uint64_t splitmix64(uint64_t value)
{
  value += 0x9e3779b97f4a7c15ULL;
  value = (value ^ (value >> 30)) * 0xbf58476d1ce4e5b9ULL;
  value = (value ^ (value >> 27)) * 0x94d049bb133111ebULL;
  return value ^ (value >> 31);
}

inline uint64_t fnv1a64(std::string_view text)
{
  uint64_t hash = 14695981039346656037ULL;
  for (const unsigned char byte : text) {
    hash ^= byte;
    hash *= 1099511628211ULL;
  }
  return hash;
}

inline uint64_t key(uint64_t seed, std::string_view robot, int64_t stamp_ns,
                   uint64_t component)
{
  uint64_t value = splitmix64(seed ^ fnv1a64(robot));
  value = splitmix64(value ^ static_cast<uint64_t>(stamp_ns));
  return splitmix64(value ^ component);
}

inline double uniform_open(uint64_t value, uint64_t draw)
{
  const uint64_t bits = splitmix64(value ^ (draw * 0x9e3779b97f4a7c15ULL)) >> 11;
  const double uniform = (static_cast<double>(bits) + 0.5) * 0x1.0p-53;
  return uniform < 1.0 ? uniform : std::nextafter(1.0, 0.0);
}

inline double normal(uint64_t seed, std::string_view robot, int64_t stamp_ns,
                     uint64_t component)
{
  const uint64_t value = key(seed, robot, stamp_ns, component);
  const double radius = std::sqrt(-2.0 * std::log(uniform_open(value, 0)));
  const double angle = 2.0 * 3.141592653589793238462643383279502884 *
                       uniform_open(value, 1);
  return radius * std::cos(angle);
}

}  // namespace sim::keyed_noise
