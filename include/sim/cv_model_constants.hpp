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

#ifndef SIM__CV_MODEL_CONSTANTS_HPP_
#define SIM__CV_MODEL_CONSTANTS_HPP_
#include <array>
namespace sim::cv_model {
inline constexpr std::array<double, 3> kFastenedRotation{0, 0, 0};
inline constexpr std::array<double, 3> kHeadlinkRotation{0, 0, 0};
inline constexpr std::array<double, 3> kHeadlinkTranslation{
    -.000171242, 9.52126e-05, .248293};
inline constexpr std::array<double, 3> kHeadlinkAxis{0, 0, 1};
inline constexpr std::array<double, 3> kHeadpitchRotation{0, 0, 0};
inline constexpr std::array<double, 3> kHeadpitchTranslation{-.00760542,
                                                             -.100122, .14235};
inline constexpr std::array<double, 3> kHeadpitchAxis{0, 1, 0};
inline constexpr std::array<double, 3> kCameraTranslation{.0920381, .0948673,
                                                          .0566588};
inline constexpr double kPanelRadiusX = .252, kPanelRadiusY = .252,
                        kPanelWidth = .135, kPanelHeight = .125,
                        kPanelNormalAngle = 1.3089969389957472;
} // namespace sim::cv_model
#endif
