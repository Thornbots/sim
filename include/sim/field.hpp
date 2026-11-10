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

#pragma once

#include <array>
#include <filesystem>
#include <string>

// The default field, mirrored by sim/field.py. world/model.config names the
// default SDF; every field world is named kWorld so gz paths never change.
namespace sim::field
{
inline constexpr char kWorld[] = "field";
// Team starts: centre of each Starting Zone, booting facing +y (README "The world frame").
inline constexpr std::array<double, 2> kRedStart{-5.25, -3.0}, kBlueStart{5.25, -3.0};
// Localization drive loop centre: clear floor between the Central Wall and
// blue's Side High Ground (README "Geometry and helpers").
inline constexpr std::array<double, 2> kLoopCentre{1.55, -1.0};

// "/world/field/<suffix>" for gz transport services and topics.
inline std::string world_path(const std::string & suffix)
{
  return std::string("/world/") + kWorld + "/" + suffix;
}

// The default field SDF named by <share>/world/model.config (combat library).
std::filesystem::path default_world(const std::filesystem::path & share);
}  // namespace sim::field
