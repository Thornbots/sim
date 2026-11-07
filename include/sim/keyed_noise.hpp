// Copyright 2026 Thornbots
// Licensed under the Apache License, Version 2.0.
#ifndef SIM__KEYED_NOISE_HPP_
#define SIM__KEYED_NOISE_HPP_

#include <cstdint>

namespace sim
{

// splitmix64: a well-mixed 64-bit value from any 64-bit input.
inline uint64_t splitmix64(uint64_t x)
{
  x += 0x9e3779b97f4a7c15ULL;
  x = (x ^ (x >> 30)) * 0xbf58476d1ce4e5b9ULL;
  x = (x ^ (x >> 27)) * 0x94d049bb133111ebULL;
  return x ^ (x >> 31);
}

// A stream key for one sample of one source: noise is drawn from a generator
// seeded with it, so a draw depends on (seed, robot, stamp, item) only, never
// on how many other draws came first or in which order they arrived. `robot` is
// the source's index, `stamp_ns` the sample's sim time, `item` a frame, panel
// or shot index within it.
inline uint64_t noise_key(uint64_t seed, uint64_t robot, int64_t stamp_ns, uint64_t item = 0)
{
  return splitmix64(
    splitmix64(splitmix64(splitmix64(seed) ^ robot) ^ static_cast<uint64_t>(stamp_ns)) ^ item);
}

}  // namespace sim

#endif  // SIM__KEYED_NOISE_HPP_
