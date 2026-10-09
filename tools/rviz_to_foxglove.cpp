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

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <yaml-cpp/yaml.h>
#include <iomanip>
#include <iostream>
#include "sim/tool_io.hpp"

namespace {
using sim::tools::Json;
double rounded(double value, int places) {
  const double scale = std::pow(10.0, places);
  return std::nearbyint(value * scale) / scale;
}
std::string color(const YAML::Node &rgb, double alpha) {
  std::istringstream input(rgb.as<std::string>());
  int r, g, b;
  char separator;
  input >> r >> separator >> g >> separator >> b;
  if (!input) throw std::runtime_error("invalid RViz color");
  std::ostringstream out;
  out << '#' << std::hex << std::setfill('0') << std::setw(2) << r << std::setw(2) << g << std::setw(2) << b
      << std::setw(2) << static_cast<int>(std::nearbyint(alpha * 255));
  return out.str();
}
std::string topic(const YAML::Node &display) {
  const auto t = display["Topic"] && !display["Topic"].IsNull() ? display["Topic"] : display["Description Topic"];
  return t.IsMap() ? t["Value"].as<std::string>() : t.as<std::string>();
}
Json camera(const YAML::Node &view) {
  constexpr double degrees = 180.0 / 3.14159265358979323846;
  const double pitch = view["Pitch"].as<double>(), yaw = view["Yaw"].as<double>();
  Json target = Json::array();
  for (const auto key : {"X", "Y", "Z"}) target.push_back(rounded(view["Focal Point"][key].as<double>(0), 3));
  return {{"perspective", true}, {"distance", rounded(view["Distance"].as<double>(), 3)},
    {"phi", rounded(90 - pitch * degrees, 2)},
    {"thetaOffset", rounded(std::atan2(-std::cos(yaw), -std::sin(yaw)) * degrees, 2)},
    {"target", target}, {"targetOffset", {0, 0, 0}}, {"targetOrientation", {0, 0, 0, 1}},
    {"fovy", 45}, {"near", 0.01}, {"far", 5000}};
}
Json convert(const std::filesystem::path &path) {
  const auto manager = YAML::LoadFile(path.string())["Visualization Manager"];
  const auto frame = manager["Global Options"]["Fixed Frame"].as<std::string>(), name = path.stem().string();
  Json three_d{{"cameraState", camera(manager["Views"]["Current"])}, {"followMode", "follow-pose"},
    {"followTf", frame}, {"scene", {{"meshUpAxis", "z_up"}}}, {"transforms", Json::object()},
    {"topics", Json::object()}, {"layers", Json::object()}, {"publish", {{"type", "point"}}}, {"imageMode", Json::object()}};
  std::string image_topic;
  for (const auto &d : manager["Displays"]) {
    if (!d["Enabled"].as<bool>(false)) continue;
    const auto full = d["Class"].as<std::string>(), cls = full.substr(full.rfind('/') + 1);
    const double alpha = d["Alpha"].as<double>(1);
    if (cls == "Grid") three_d["layers"]["grid"] = {{"layerId", "foxglove.Grid"}, {"visible", true},
      {"frameId", frame}, {"size", 20}, {"divisions", 20}, {"color", color(d["Color"], d["Alpha"].as<double>(0.5))}};
    else if (cls == "Image") image_topic = topic(d);
    else if (cls == "LaserScan") three_d["topics"][topic(d)] = {{"visible", true},
      {"pointSize", d["Size (Pixels)"].as<int>(3)}, {"colorMode", "flat"}, {"flatColor", color(d["Color"], alpha)}};
    else if (cls == "Odometry") {
      const auto shape = d["Shape"];
      three_d["topics"][topic(d)] = {{"visible", true}, {"type", "arrow"},
        {"color", color(shape && shape["Color"] ? shape["Color"] : YAML::Node("255; 25; 0"),
                         shape ? shape["Alpha"].as<double>(1) : 1)}};
    } else if (cls == "Polygon") three_d["topics"][topic(d)] = {{"visible", true}, {"color", color(d["Color"], alpha)}};
    else if (cls == "RobotModel" || cls == "Map" || cls == "MarkerArray" || cls == "Marker")
      three_d["topics"][topic(d)] = {{"visible", true}};
    else std::cerr << path.filename().string() << ": skipping " << cls << " display \"" << d["Name"].as<std::string>("None") << "\"\n";
  }
  Json config{{"3D!" + name, three_d}}, layout = "3D!" + name;
  if (!image_topic.empty()) {
    config["Image!" + name] = {{"imageMode", {{"imageTopic", image_topic}}}};
    layout = {{"first", "3D!" + name}, {"second", "Image!" + name}, {"direction", "row"}, {"splitPercentage", 70}};
  }
  return {{"configById", config}, {"globalVariables", Json::object()}, {"userNodes", Json::object()},
    {"playbackConfig", {{"speed", 1}}}, {"layout", layout}};
}
}  // namespace
int main(int argc, char **argv) {
  try {
    bool check = false;
    std::filesystem::path package = ament_index_cpp::get_package_share_directory("sim");
    for (int i = 1; i < argc; ++i) {
      const std::string arg = argv[i];
      if (arg == "--check") check = true;
      else if (arg == "--package-dir" && i + 1 < argc) package = argv[++i];
      else { std::cerr << "usage: rviz_to_foxglove [--check] [--package-dir DIR]\n"; return 2; }
    }
    std::vector<std::filesystem::path> files;
    for (const auto &entry : std::filesystem::directory_iterator(package / "rviz"))
      if (entry.path().extension() == ".rviz") files.push_back(entry.path());
    std::sort(files.begin(), files.end());
    std::vector<std::string> stale;
    for (const auto &rviz : files) {
      const auto out = package / "foxglove" / (rviz.stem().string() + ".json");
      const auto value = convert(rviz);
      const auto text = value.dump(2, ' ', true) + '\n';
      if (check) {
        if (!std::filesystem::exists(out) || sim::tools::read(out) != text) stale.push_back(out.filename().string());
      } else {
        std::filesystem::create_directories(out.parent_path());
        sim::tools::write(out, text);
        std::cout << "wrote foxglove/" << out.filename().string() << '\n';
      }
    }
    if (!stale.empty()) {
      std::cerr << "stale, rerun ros2 run sim rviz_to_foxglove: ";
      for (std::size_t i = 0; i < stale.size(); ++i) std::cerr << (i ? ", " : "") << stale[i];
      std::cerr << '\n'; return 1;
    }
    return 0;
  } catch (const std::exception &e) { std::cerr << e.what() << '\n'; return 1; }
}
