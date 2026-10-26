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

#include "sim/combat.hpp"

#include <sstream>
#include <stdexcept>
#include <tinyxml2.h>

namespace sim::combat
{
namespace
{
Eigen::Matrix4d pose(const tinyxml2::XMLElement * element)
{
  const auto * value = element->FirstChildElement("pose");
  if (!value) {
    return Eigen::Matrix4d::Identity();
  }
  if (value->Attribute("relative_to") || !value->GetText()) {
    throw std::runtime_error("field poses must be relative to their parent");
  }
  Eigen::Vector3d xyz, rpy;
  std::istringstream input(value->GetText());
  if (!(input >> xyz.x() >> xyz.y() >> xyz.z() >> rpy.x() >> rpy.y() >> rpy.z())) {
    throw std::runtime_error("invalid field pose");
  }
  return armor_offset(xyz, rpy);
}
}  // namespace

Mesh load_field(const std::filesystem::path & share)
{
  tinyxml2::XMLDocument xml;
  if (xml.LoadFile((share / "world/ARCC_Field_2026.sdf").c_str()) != tinyxml2::XML_SUCCESS) {
    throw std::runtime_error("cannot read field SDF");
  }
  const auto * root = xml.FirstChildElement("sdf");
  const auto * world = root ? root->FirstChildElement("world") : nullptr;
  const auto * model = world ? world->FirstChildElement("model") : nullptr;
  if (!model || !model->Attribute("name", "ARCC_Field_2026")) {
    throw std::runtime_error("missing field model");
  }
  Mesh result;
  for (auto * link = model->FirstChildElement("link"); link;
    link = link->NextSiblingElement("link"))
  {
    for (auto * collision = link->FirstChildElement("collision"); collision;
      collision = collision->NextSiblingElement("collision"))
    {
      const auto * geometry = collision->FirstChildElement("geometry");
      const auto * mesh = geometry ? geometry->FirstChildElement("mesh") : nullptr;
      const auto * uri = mesh ? mesh->FirstChildElement("uri") : nullptr;
      const std::string prefix = "model://sim/";
      if (!uri || !uri->GetText() || std::string(uri->GetText()).find(prefix) != 0) {
        throw std::runtime_error("field collision must use a sim mesh URI");
      }
      Eigen::Vector3d scale = Eigen::Vector3d::Ones();
      if (const auto * value = mesh->FirstChildElement("scale")) {
        std::istringstream input(value->GetText() ? value->GetText() : "");
        if (!(input >> scale.x() >> scale.y() >> scale.z())) {
          throw std::runtime_error("invalid field mesh scale");
        }
      }
      auto triangles = load_stl(share / std::string(uri->GetText()).substr(prefix.size()));
      for (auto & triangle : triangles) {
        for (auto & vertex : triangle) {
          vertex = vertex.cwiseProduct(scale);
        }
      }
      triangles = transform_mesh(triangles, pose(model) * pose(link) * pose(collision));
      result.insert(result.end(), triangles.begin(), triangles.end());
    }
  }
  if (result.empty()) {
    throw std::runtime_error("field has no collision triangles");
  }
  return result;
}
}  // namespace sim::combat
