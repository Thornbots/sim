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

#include <Eigen/Dense>
#include <assimp/Importer.hpp>
#include <assimp/postprocess.h>
#include <assimp/scene.h>
#include <libqhull_r/qhull_ra.h>
#include <tinyxml2.h>
#include <yaml-cpp/yaml.h>
#include <algorithm>
#include <array>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <regex>
#include <set>
#include <sstream>
#include "Simplify.h"

namespace {
using V = Eigen::Vector3d;
using M = Eigen::Matrix3d;
using T = Eigen::Matrix4d;
using Element = tinyxml2::XMLElement;
struct Mesh { std::vector<V> vertices; std::vector<std::array<int, 3>> faces; };
struct Inertia { double mass = 0; V com = V::Zero(); M tensor = M::Zero(); };
V vector(const std::string &text) {
  V value;
  std::istringstream input(text);
  input >> value.x() >> value.y() >> value.z();
  if (!input) throw std::runtime_error("invalid vector " + text);
  return value;
}
V vector(const YAML::Node &node) { return {node[0].as<double>(), node[1].as<double>(), node[2].as<double>()}; }
std::string attr(Element *e, const char *key, const char *fallback = "") {
  return e && e->Attribute(key) ? e->Attribute(key) : fallback;
}
M rotation(const V &rpy) {
  return (Eigen::AngleAxisd(rpy.z(), V::UnitZ()) * Eigen::AngleAxisd(rpy.y(), V::UnitY()) *
          Eigen::AngleAxisd(rpy.x(), V::UnitX())).toRotationMatrix();
}
T origin(Element *e) {
  T tf = T::Identity();
  tf.block<3, 3>(0, 0) = rotation(vector(attr(e, "rpy", "0 0 0")));
  tf.block<3, 1>(0, 3) = vector(attr(e, "xyz", "0 0 0"));
  return tf;
}
V point(const T &tf, const V &v) { return tf.block<3, 3>(0, 0) * v + tf.block<3, 1>(0, 3); }
void transform(Mesh &mesh, const T &tf, const V &offset = V::Zero()) {
  for (auto &v : mesh.vertices) v = point(tf, v) - offset;
}
std::pair<V, V> bounds(const Mesh &mesh) {
  if (mesh.vertices.empty()) throw std::runtime_error("empty mesh");
  V lo = mesh.vertices.front(), hi = lo;
  for (const auto &p : mesh.vertices) { lo = lo.cwiseMin(p); hi = hi.cwiseMax(p); }
  return {lo, hi};
}
V centroid(const Mesh &mesh) {
  double area = 0;
  V sum = V::Zero();
  for (const auto &face : mesh.faces) {
    const V &a = mesh.vertices[face[0]], &b = mesh.vertices[face[1]], &c = mesh.vertices[face[2]];
    double weight = (b - a).cross(c - a).norm() / 2;
    sum += weight * (a + b + c) / 3;
    area += weight;
  }
  return sum / area;
}
void merge_positions(Mesh &mesh) {
  // STL face normals make Assimp retain duplicate positions. Trimesh's
  // default processing merges positions rounded to eight decimal places.
  std::map<std::array<std::int64_t, 3>, int> unique;
  std::vector<V> merged;
  std::vector<int> remap;
  remap.reserve(mesh.vertices.size());
  for (const auto &v : mesh.vertices) {
    const std::array<std::int64_t, 3> key = {
      static_cast<std::int64_t>(std::nearbyint(v.x() * 1e8)),
      static_cast<std::int64_t>(std::nearbyint(v.y() * 1e8)),
      static_cast<std::int64_t>(std::nearbyint(v.z() * 1e8))};
    const auto inserted = unique.emplace(key, static_cast<int>(merged.size()));
    if (inserted.second) merged.push_back(v);
    remap.push_back(inserted.first->second);
  }
  for (auto &face : mesh.faces) for (auto &id : face) id = remap.at(id);
  mesh.vertices = std::move(merged);
}
Mesh load_mesh(const std::filesystem::path &path) {
  Assimp::Importer importer;
  const auto scene = importer.ReadFile(path.string(), aiProcess_Triangulate | aiProcess_JoinIdenticalVertices);
  if (!scene || !scene->HasMeshes()) throw std::runtime_error("cannot load " + path.string() + ": " + importer.GetErrorString());
  Mesh mesh;
  for (unsigned k = 0; k < scene->mNumMeshes; ++k) {
    const auto *part = scene->mMeshes[k];
    const int offset = static_cast<int>(mesh.vertices.size());
    for (unsigned i = 0; i < part->mNumVertices; ++i) {
      const auto &v = part->mVertices[i];
      mesh.vertices.emplace_back(v.x, v.y, v.z);
    }
    for (unsigned i = 0; i < part->mNumFaces; ++i) {
      const auto &f = part->mFaces[i];
      if (f.mNumIndices == 3) mesh.faces.push_back({offset + static_cast<int>(f.mIndices[0]),
        offset + static_cast<int>(f.mIndices[1]), offset + static_cast<int>(f.mIndices[2])});
    }
  }
  merge_positions(mesh);
  return mesh;
}
Mesh hull(const std::vector<V> &points) {
  if (points.size() < 4) throw std::runtime_error("convex hull needs four vertices");
  std::vector<double> coords;
  coords.reserve(points.size() * 3);
  for (const auto &p : points) for (int i = 0; i < 3; ++i) coords.push_back(p[i]);
  qhT context;
  qhT *qh = &context;
  qh_zero(qh, stderr);
  char options[] = "qhull Qt QbB Pp";
  const int status = qh_new_qhull(qh, 3, static_cast<int>(points.size()), coords.data(), false, options, nullptr, stderr);
  Mesh mesh;
  mesh.vertices = points;
  V centre = V::Zero();
  for (const auto &p : points) centre += p;
  centre /= points.size();
  if (!status) {
    facetT *facet;
    vertexT *vertex, **vertexp;
    FORALLfacets {
      std::array<int, 3> face{};
      int index = 0;
      FOREACHvertex_(facet->vertices) {
        if (index < 3) face[index++] = qh_pointid(qh, vertex->point);
      }
      if (index != 3) continue;
      const V &a = points[face[0]], &b = points[face[1]], &c = points[face[2]];
      if ((b - a).cross(c - a).norm() <= 1e-13) continue;
      if ((b - a).cross(c - a).dot(a - centre) < 0) std::swap(face[0], face[2]);
      mesh.faces.push_back(face);
    }
  }
  qh_freeqhull(qh, !qh_ALL);
  int remaining, bytes;
  qh_memfreeshort(qh, &remaining, &bytes);
  if (status) throw std::runtime_error("convex hull failed");
  // Keep only hull vertices, as trimesh's convex_hull does before decimation.
  std::map<int, int> remap;
  std::vector<V> vertices;
  for (const auto &f : mesh.faces) for (int id : f) remap.emplace(id, 0);
  for (auto &v : remap) { v.second = static_cast<int>(vertices.size()); vertices.push_back(points[v.first]); }
  for (auto &f : mesh.faces) for (auto &id : f) id = remap.at(id);
  mesh.vertices = std::move(vertices);
  merge_positions(mesh);
  return mesh;
}
void decimate(Mesh &mesh, int target) {
  if (static_cast<int>(mesh.faces.size()) <= target) return;
  Simplify::vertices.clear(); Simplify::triangles.clear(); Simplify::refs.clear();
  Simplify::vertices.reserve(mesh.vertices.size());
  for (const auto &p : mesh.vertices) { Simplify::Vertex v{}; v.p = vec3f(p.x(), p.y(), p.z()); Simplify::vertices.push_back(v); }
  for (const auto &f : mesh.faces) { Simplify::Triangle t{}; for (int i = 0; i < 3; ++i) t.v[i] = f[i]; Simplify::triangles.push_back(t); }
  Simplify::simplify_mesh(target, 7, false);
  mesh.vertices.clear(); mesh.faces.clear();
  for (const auto &v : Simplify::vertices) mesh.vertices.emplace_back(v.p.x, v.p.y, v.p.z);
  for (const auto &t : Simplify::triangles) mesh.faces.push_back({t.v[0], t.v[1], t.v[2]});
  merge_positions(mesh);
}
void append(Mesh &to, const Mesh &part) {
  const auto offset = static_cast<int>(to.vertices.size());
  to.vertices.insert(to.vertices.end(), part.vertices.begin(), part.vertices.end());
  for (const auto &f : part.faces) to.faces.push_back({f[0] + offset, f[1] + offset, f[2] + offset});
}
void save_stl(const std::filesystem::path &path, const Mesh &mesh) {
  std::ofstream out(path, std::ios::binary);
  if (!out) throw std::runtime_error("cannot write " + path.string());
  std::array<char, 80> header{};
  out.write(header.data(), header.size());
  const auto count = static_cast<std::uint32_t>(mesh.faces.size());
  out.write(reinterpret_cast<const char *>(&count), sizeof(count));
  for (const auto &f : mesh.faces) {
    const V normal = (mesh.vertices[f[1]] - mesh.vertices[f[0]]).cross(mesh.vertices[f[2]] - mesh.vertices[f[0]]).normalized();
    std::array<float, 12> data{};
    for (int i = 0; i < 3; ++i) { data[i] = static_cast<float>(normal[i]);
      for (int j = 0; j < 3; ++j) data[3 + j * 3 + i] = static_cast<float>(mesh.vertices[f[j]][i]); }
    const std::uint16_t attr = 0;
    out.write(reinterpret_cast<const char *>(data.data()), sizeof(data));
    out.write(reinterpret_cast<const char *>(&attr), sizeof(attr));
  }
}
class Export {
 public:
  explicit Export(const std::filesystem::path &directory) : mesh_dir_(directory / "meshes") {
    std::filesystem::path urdf;
    for (const auto &entry : std::filesystem::directory_iterator(directory / "urdf"))
      if (entry.path().extension() == ".urdf") { urdf = entry.path(); break; }
    if (document_.LoadFile(urdf.c_str()) != tinyxml2::XML_SUCCESS) throw std::runtime_error("cannot read export URDF");
    auto root = document_.RootElement();
    for (auto e = root->FirstChildElement("link"); e; e = e->NextSiblingElement("link")) {
      links.emplace(attr(e, "name"), e); order.push_back(attr(e, "name"));
    }
    std::set<std::string> has_parent;
    for (auto e = root->FirstChildElement("joint"); e; e = e->NextSiblingElement("joint")) {
      joints.emplace(attr(e, "name"), e);
      children[attr(e->FirstChildElement("parent"), "link")].push_back(e);
      has_parent.insert(attr(e->FirstChildElement("child"), "link"));
    }
    std::string base;
    for (const auto &name : order) if (!has_parent.count(name)) { base = name; break; }
    world[base] = T::Identity();
    std::vector<std::string> stack{base};
    while (!stack.empty()) {
      const auto parent = stack.back(); stack.pop_back();
      for (auto j : children[parent]) {
        const auto child = attr(j->FirstChildElement("child"), "link");
        world[child] = world.at(parent) * origin(j->FirstChildElement("origin"));
        stack.push_back(child);
      }
    }
  }
  Element *visual_mesh(const std::string &name) const {
    auto visual = links.at(name)->FirstChildElement("visual");
    auto geometry = visual ? visual->FirstChildElement("geometry") : nullptr;
    return geometry ? geometry->FirstChildElement("mesh") : nullptr;
  }
  Mesh mesh(const std::string &name) {
    auto visual = links.at(name)->FirstChildElement("visual");
    const auto filename = std::filesystem::path(attr(visual_mesh(name), "filename")).filename().string();
    if (!cache_.count(filename)) cache_[filename] = load_mesh(mesh_dir_ / filename);
    auto result = cache_.at(filename);
    transform(result, world.at(name) * origin(visual->FirstChildElement("origin")));
    return result;
  }
  std::set<std::string> subtree(const std::string &joint) {
    std::vector<std::string> stack{attr(joints.at(joint)->FirstChildElement("child"), "link")};
    std::set<std::string> found;
    while (!stack.empty()) {
      const auto name = stack.back(); stack.pop_back(); found.insert(name);
      for (auto j : children[name]) stack.push_back(attr(j->FirstChildElement("child"), "link"));
    }
    return found;
  }
  std::pair<V, V> joint_world(const std::string &name) const {
    auto j = joints.at(name);
    const auto &tf = world.at(attr(j->FirstChildElement("child"), "link"));
    return {tf.block<3, 1>(0, 3), tf.block<3, 3>(0, 0) * vector(attr(j->FirstChildElement("axis"), "xyz")).normalized()};
  }
  Inertia inertial(const std::string &name) const {
    auto e = links.at(name)->FirstChildElement("inertial");
    Inertia result;
    if (!e || !e->FirstChildElement("mass")) { result.com = world.at(name).block<3, 1>(0, 3); return result; }
    const T tf = world.at(name) * origin(e->FirstChildElement("origin"));
    auto i = e->FirstChildElement("inertia");
    const auto number = [&](const char *key) { return std::stod(attr(i, key)); };
    M tensor;
    tensor << number("ixx"), number("ixy"), number("ixz"), number("ixy"), number("iyy"), number("iyz"),
      number("ixz"), number("iyz"), number("izz");
    result.mass = std::stod(attr(e->FirstChildElement("mass"), "value"));
    result.com = tf.block<3, 1>(0, 3);
    result.tensor = tf.block<3, 3>(0, 0) * tensor * tf.block<3, 3>(0, 0).transpose();
    return result;
  }
  std::map<std::string, Element *> links, joints;
  std::map<std::string, std::vector<Element *>> children;
  std::map<std::string, T> world;
  std::vector<std::string> order;
 private:
  tinyxml2::XMLDocument document_;
  std::filesystem::path mesh_dir_;
  std::map<std::string, Mesh> cache_;
};
Inertia combine(const std::vector<Inertia> &parts) {
  Inertia result;
  for (const auto &p : parts) { result.mass += p.mass; result.com += p.mass * p.com; }
  if (result.mass <= 0) return {};
  result.com /= result.mass;
  for (const auto &p : parts) {
    const V delta = p.com - result.com;
    result.tensor += p.tensor + p.mass * (delta.squaredNorm() * M::Identity() - delta * delta.transpose());
  }
  return result;
}
std::map<std::string, std::string> assign(Export &exported, const YAML::Node &cfg) {
  const std::regex drop(cfg["drop"].as<std::string>()), carrier(cfg["carrier_parts"].as<std::string>());
  std::map<std::string, std::string> owner;
  for (const auto &p : exported.order) if (exported.visual_mesh(p) && !std::regex_search(p, drop)) owner[p] = "root";
  for (const auto &body : cfg["subtree_bodies"]) {
    const auto inside = exported.subtree(body["joint"].as<std::string>());
    for (auto &p : owner) if (inside.count(p.first)) p.second = body["name"].as<std::string>();
  }
  for (std::size_t k = 0; k < cfg["wheels"].size(); ++k) {
    const V centre = vector(cfg["wheels"][k]["centre"]), axis = vector(cfg["wheels"][k]["axis"]).normalized();
    for (auto &p : owner) if (p.second == "root") {
      double rmax = 0, amin = std::numeric_limits<double>::infinity(), amax = -amin;
      for (const auto &v : exported.mesh(p.first).vertices) {
        const V d = v - centre;
        const double a = d.dot(axis);
        rmax = std::max(rmax, (d - a * axis).norm()); amin = std::min(amin, a); amax = std::max(amax, a);
      }
      if (rmax <= cfg["wheel_radius"].as<double>() + 0.005 && std::max(std::abs(amin), std::abs(amax)) <= cfg["wheel_half_width"].as<double>())
        p.second = "wheel_" + std::to_string(k);
      else if (rmax <= cfg["carrier_radius"].as<double>() && amax <= 0 && amin >= -cfg["carrier_depth"].as<double>())
        p.second = "carrier_" + std::to_string(k);
    }
  }
  for (auto &p : owner) if (p.second == "root" && std::regex_search(p.first, carrier)) {
    const V c = centroid(exported.mesh(p.first));
    double distance = std::numeric_limits<double>::infinity(); std::size_t nearest = 0;
    for (std::size_t k = 0; k < cfg["wheels"].size(); ++k) {
      const double d = (c - vector(cfg["wheels"][k]["centre"])).head<2>().norm();
      if (d < distance) { nearest = k; distance = d; }
    }
    p.second = "carrier_" + std::to_string(nearest);
  }
  return owner;
}
std::string fmt(const V &value) { std::ostringstream out; out << std::setprecision(6) << value.x() << ' ' << value.y() << ' ' << value.z(); return out.str(); }
struct Armor { V xyz, rpy, size; };
std::vector<Armor> armor_panels(Export &exported, const YAML::Node &cfg, const T &to_out) {
  const std::regex face(cfg["face_mesh"].as<std::string>());
  std::vector<Armor> panels;
  for (const auto &name : exported.order) {
    auto mesh = exported.visual_mesh(name);
    if (!mesh || !std::regex_search(std::filesystem::path(attr(mesh, "filename")).filename().string(), face)) continue;
    auto m = exported.mesh(name); transform(m, to_out);
    V mean = V::Zero(); for (const auto &v : m.vertices) mean += v; mean /= m.vertices.size();
    Eigen::MatrixXd centered(m.vertices.size(), 3);
    for (std::size_t i = 0; i < m.vertices.size(); ++i) centered.row(i) = (m.vertices[i] - mean).transpose();
    V n = Eigen::JacobiSVD<Eigen::MatrixXd>(centered, Eigen::ComputeThinV).matrixV().col(2);
    if (n.head<2>().dot(mean.head<2>()) < 0) n = -n;
    const V y = V::UnitZ().cross(n).normalized(), z = n.cross(y);
    M basis; basis.col(0) = n; basis.col(1) = y; basis.col(2) = z;
    Mesh local = m; for (auto &v : local.vertices) v = basis.transpose() * v;
    const auto [lo, hi] = bounds(local);
    panels.push_back({basis * ((lo + hi) / 2), {0, -std::asin(n.z()), std::atan2(n.y(), n.x())}, hi - lo});
  }
  if (panels.size() != 4) throw std::runtime_error("armor.face_mesh matched " + std::to_string(panels.size()) + " parts, want 4");
  const auto azimuth = [](double a) { constexpr double tau = 6.28318530717958647692; return a < 0 ? a + tau : a; };
  std::sort(panels.begin(), panels.end(), [&](const auto &a, const auto &b) { return azimuth(a.rpy.z()) < azimuth(b.rpy.z()); });
  return panels;
}
std::string urdf(const YAML::Node &cfg, const std::map<std::string, Inertia> &links, const std::map<std::string, V> &origins,
                 const std::map<std::string, std::pair<std::string, V>> &sensors, const std::vector<Armor> &armor, const V &yaw) {
  std::ostringstream out; out << std::setprecision(6);
  const auto pkg = cfg["mesh_uri"].as<std::string>();
  out << "<?xml version=\"1.0\"?>\n<!-- Generated by sim/tools/simplify_urdf from an Onshape export; do not edit. -->\n<robot name=\"" << cfg["robot_name"].as<std::string>() << "\">\n";
  const auto link = [&](const std::string &name, const std::string &collision) {
    const auto &d = links.at(name); const auto &i = d.tensor;
    out << "  <link name=\"" << name << "\">\n    <inertial><origin xyz=\"" << fmt(d.com) << "\"/><mass value=\"" << d.mass
        << "\"/><inertia ixx=\"" << i(0, 0) << "\" ixy=\"" << i(0, 1) << "\" ixz=\"" << i(0, 2)
        << "\" iyy=\"" << i(1, 1) << "\" iyz=\"" << i(1, 2) << "\" izz=\"" << i(2, 2) << "\"/></inertial>\n"
        << "    <visual><geometry><mesh filename=\"" << pkg << '/' << name << ".stl\"/></geometry></visual>\n";
    if (!collision.empty()) out << "    <collision>" << collision << "</collision>\n";
    out << "  </link>\n";
  };
  const auto joint = [&](const std::string &name, const std::string &kind, const std::string &parent, const std::string &child,
                         const V &xyz, const std::string &axis = "", const std::string &extra = "") {
    out << "  <joint name=\"" << name << "\" type=\"" << kind << "\"><parent link=\"" << parent << "\"/><child link=\"" << child
        << "\"/><origin xyz=\"" << fmt(xyz) << "\"/>";
    if (!axis.empty()) out << "<axis xyz=\"" << axis << "\"/>";
    out << extra << "</joint>\n";
  };
  const auto collision = [&](const std::string &name) { return "<geometry><mesh filename=\"" + pkg + "/collision/" + name + ".stl\"/></geometry>"; };
  link("root", collision("root")); out << "  <link name=\"body\"/>\n";
  joint("fastened_2", "fixed", "root", "body", V::Zero()); link("head", collision("head"));
  joint("headlink", "continuous", "body", "head", origins.at("head"), fmt(V(0, 0, (yaw.z() < 0 ? -1 : yaw.z() > 0 ? 1 : 0) * cfg["yaw_sign"].as<double>())));
  link("head_pitch", collision("head_pitch"));
  const auto number = [](double v) { std::ostringstream text; text << v; return text.str(); };
  joint("headpitch", "revolute", "head", "head_pitch", origins.at("head_pitch") - origins.at("head"), "0 1 0",
    "<limit lower=\"" + number(cfg["pitch_limits"][0].as<double>()) + "\" upper=\"" + number(cfg["pitch_limits"][1].as<double>()) + "\" effort=\"200\" velocity=\"10\"/>");
  const auto s = cfg["suspension"];
  for (std::size_t k = 0; k < cfg["wheels"].size(); ++k) {
    const auto id = std::to_string(k), carrier = "carrier_" + id, wheel = "wheel_" + id;
    link(carrier, ""); joint("suspension_" + id, "prismatic", "body", carrier, origins.at(carrier), "0 0 1",
      "<limit lower=\"" + number(-s["travel_down"].as<double>()) + "\" upper=\"" + number(s["travel_up"].as<double>()) +
      "\" effort=\"1000\" velocity=\"5\"/><dynamics damping=\"" + number(s["damping"].as<double>()) + "\"/>");
    link(wheel, "<geometry><sphere radius=\"" + number(cfg["wheel_radius"].as<double>()) + "\"/></geometry>");
    const Eigen::Vector2d radial = origins.at(wheel).head<2>().normalized();
    joint(wheel + "_spin", "continuous", carrier, wheel, V::Zero(), fmt({radial.x(), radial.y(), 0}));
  }
  for (std::size_t k = 0; k < armor.size(); ++k) {
    const auto name = "armor_" + std::to_string(k), box = "<geometry><box size=\"" + fmt(armor[k].size) + "\"/></geometry>";
    out << "  <link name=\"" << name << "\"><visual>" << box << "</visual><collision>" << box << "</collision></link>\n"
        << "  <joint name=\"" << name << "link\" type=\"fixed\"><parent link=\"root\"/><child link=\"" << name
        << "\"/><origin xyz=\"" << fmt(armor[k].xyz) << "\" rpy=\"" << fmt(armor[k].rpy) << "\"/></joint>\n";
  }
  for (const auto &sensor : sensors) {
    out << "  <link name=\"" << sensor.first << "\"/>\n";
    joint(sensor.first + "link", "fixed", sensor.second.first, sensor.first, sensor.second.second - origins.at(sensor.second.first));
  }
  out << "</robot>\n"; return out.str();
}
void run(const std::filesystem::path &directory, const std::filesystem::path &config, const std::filesystem::path &output) {
  const auto cfg = YAML::LoadFile(config.string());
  Export exported(directory); const auto owner = assign(exported, cfg);
  T frame = T::Identity(); frame.block<3, 3>(0, 0) = rotation({0, 0, cfg["frame"]["yaw"].as<double>()});
  frame.block<3, 1>(0, 3) = vector(cfg["frame"]["origin"]); const T to_out = frame.inverse();
  const auto [yaw_pt, yaw_ax] = exported.joint_world(cfg["yaw_joint"].as<std::string>());
  const auto [pitch_pt, pitch_ax] = exported.joint_world(cfg["pitch_joint"].as<std::string>());
  std::map<std::string, V> origins{{"root", V::Zero()}, {"head", point(to_out, yaw_pt)}, {"head_pitch", point(to_out, pitch_pt)}};
  for (std::size_t k = 0; k < cfg["wheels"].size(); ++k)
    origins["carrier_" + std::to_string(k)] = origins["wheel_" + std::to_string(k)] = point(to_out, vector(cfg["wheels"][k]["centre"]));
  std::map<std::string, std::vector<std::string>> bodies;
  for (const auto &name : exported.order) if (owner.count(name)) bodies[owner.at(name)].push_back(name);
  std::filesystem::create_directories(output / "meshes" / "collision");
  std::map<std::string, Inertia> links;
  for (const auto &body : bodies) {
    std::vector<Inertia> parts; for (const auto &p : body.second) parts.push_back(exported.inertial(p));
    auto d = combine(parts); const double mass = d.mass;
    Mesh mesh; std::vector<V> solid;
    for (const auto &p : body.second) {
      const auto m = exported.mesh(p); const auto [lo, hi] = bounds(m);
      if (point(to_out, lo).z() >= cfg["collision_min_z"].as<double>()) solid.insert(solid.end(), m.vertices.begin(), m.vertices.end());
      if ((hi - lo).norm() >= cfg["min_part_size"].as<double>() && m.vertices.size() >= 4) {
        auto h = hull(m.vertices); decimate(h, cfg["max_hull_faces"].as<int>()); append(mesh, h);
      }
    }
    transform(mesh, to_out, origins.at(body.first));
    const auto prefix = body.first.substr(0, body.first.find('_'));
    const auto faces = cfg["faces"][prefix] ? cfg["faces"][prefix] : cfg["faces"]["default"];
    decimate(mesh, faces.as<int>()); save_stl(output / "meshes" / (body.first + ".stl"), mesh);
    auto collision = hull(solid.empty() ? mesh.vertices : solid);
    transform(collision, to_out, origins.at(body.first)); save_stl(output / "meshes" / "collision" / (body.first + ".stl"), collision);
    d.mass = std::max(d.mass, 1e-4); d.com = point(to_out, d.com) - origins.at(body.first);
    d.tensor = to_out.block<3, 3>(0, 0) * d.tensor * to_out.block<3, 3>(0, 0).transpose(); links[body.first] = d;
    std::cout << std::left << std::setw(12) << body.first << std::right << ' ' << std::setw(5) << body.second.size()
              << " parts " << std::fixed << std::setprecision(3) << std::setw(7) << mass << " kg " << std::setw(6) << mesh.faces.size() << " faces\n";
  }
  std::map<std::string, std::pair<std::string, V>> sensors;
  for (const auto &s : cfg["sensors"]) {
    const auto name = s.first.as<std::string>(), part = s.second.as<std::string>(); const auto [lo, hi] = bounds(exported.mesh(part));
    sensors[name] = {owner.at(part), point(to_out, (lo + hi) / 2)};
  }
  const V yaw = to_out.block<3, 3>(0, 0) * yaw_ax, pitch = to_out.block<3, 3>(0, 0) * pitch_ax;
  std::ofstream out(output / (cfg["robot_name"].as<std::string>() + ".urdf"));
  out << urdf(cfg, links, origins, sensors, armor_panels(exported, cfg["armor"], to_out), yaw);
  if (!out) throw std::runtime_error("cannot write URDF");
  double total = 0; for (const auto &p : links) total += p.second.mass;
  std::cout << "total " << total << " kg; yaw axis [" << yaw.transpose() << "], pitch axis [" << pitch.transpose() << "]\n";
}
}  // namespace
int main(int argc, char **argv) {
  if (argc != 4) { std::cerr << "usage: simplify_urdf EXPORT_DIR CONFIG.yaml OUT_DIR\n"; return 2; }
  try { run(argv[1], argv[2], argv[3]); return 0; }
  catch (const std::exception &e) { std::cerr << e.what() << '\n'; return 1; }
}
