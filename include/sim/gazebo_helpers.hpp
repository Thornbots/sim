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

#ifndef SIM__GAZEBO_HELPERS_HPP_
#define SIM__GAZEBO_HELPERS_HPP_
#include <array>
#include <cmath>
#include <cstdlib>
#include <gz/msgs/boolean.pb.h>
#include <gz/msgs/empty.pb.h>
#include <gz/msgs/entity.pb.h>
#include <gz/msgs/entity_factory.pb.h>
#include <gz/msgs/pose.pb.h>
#include <gz/msgs/scene.pb.h>
#include <gz/msgs/world_control.pb.h>
#include <gz/transport/Node.hh>
#include <memory>
#include <optional>
#include <string>
#include <vector>
namespace sim::gazebo {
constexpr double kSpawnYaw = -1.57079632679489661923;
constexpr double kZ = 0.03;
inline gz::transport::Node &node() {
  static std::unique_ptr<gz::transport::Node> instance = [] {
    if (!std::getenv("GZ_IP")) {
      setenv("GZ_IP", "127.0.0.1", 0);
    }
    return std::make_unique<gz::transport::Node>();
  }();
  return *instance;
}
template <typename Request>
bool request(const std::string &service, const Request &request) {
  gz::msgs::Boolean reply;
  bool result = false;
  return node().Request("/world/ARCC_Field_2026/" + service, request, 2000u,
                        reply, result) &&
         result && reply.data();
}
inline bool reset_joints() {
  gz::msgs::WorldControl req;
  req.mutable_reset()->set_model_only(true);
  return request("control", req);
}
inline bool set_model_pose(const std::string &name, double x, double y,
                           double z, double yaw = 0.0) {
  gz::msgs::Pose req;
  req.set_name(name);
  req.mutable_position()->set_x(x);
  req.mutable_position()->set_y(y);
  req.mutable_position()->set_z(z);
  req.mutable_orientation()->set_z(std::sin(yaw / 2.0));
  req.mutable_orientation()->set_w(std::cos(yaw / 2.0));
  return request("set_pose", req);
}
inline bool teleport(double x, double y, double z = kZ, double yaw = 0.0) {
  reset_joints();
  const bool ok = set_model_pose("sentry", x, y, z, yaw);
  reset_joints();
  return ok;
}
inline bool spawn_model(const std::string &name, const std::string &sdf,
                        double x, double y, double z) {
  gz::msgs::EntityFactory req;
  req.set_sdf(sdf);
  req.set_name(name);
  req.set_allow_renaming(false);
  req.mutable_pose()->mutable_position()->set_x(x);
  req.mutable_pose()->mutable_position()->set_y(y);
  req.mutable_pose()->mutable_position()->set_z(z);
  return request("create", req);
}
inline bool remove_model(const std::string &name) {
  gz::msgs::Entity req;
  req.set_name(name);
  req.set_type(gz::msgs::Entity::MODEL);
  return request("remove", req);
}
inline std::optional<std::vector<std::string>> model_names() {
  gz::msgs::Scene reply;
  bool result = false;
  if (!node().Request("/world/ARCC_Field_2026/scene/info", gz::msgs::Empty{},
                      2000u, reply, result) ||
      !result) {
    return std::nullopt;
  }
  std::vector<std::string> names;
  for (const auto &model : reply.model()) {
    names.push_back(model.name());
  }
  return names;
}
inline std::vector<std::array<double, 2>> build_grid() {
  std::vector<std::array<double, 2>> points;
  bool forward = true;
  for (double y = -3.5; y <= 3.5 + 1e-9; y += 0.5) {
    for (int i = 0; i <= 20; ++i) {
      points.push_back({forward ? -5.0 + i * 0.5 : 5.0 - i * 0.5, y});
    }
    forward = !forward;
  }
  return points;
}
} // namespace sim::gazebo
#endif // SIM__GAZEBO_HELPERS_HPP_
