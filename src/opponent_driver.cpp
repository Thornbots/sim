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

#include <cerrno>
#include <chrono>
#include <cstdlib>
#include <cstring>
#include <functional>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <sys/types.h>
#include <sys/wait.h>
#include <unistd.h>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "gz/msgs/boolean.pb.h"
#include "gz/msgs/empty.pb.h"
#include "gz/msgs/entity_factory.pb.h"
#include "gz/msgs/scene.pb.h"
#include "gz/transport/Node.hh"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tinyxml2.h"

namespace
{
constexpr const char * kWorld = "/world/ARCC_Field_2026/";

std::string xacro_opponent(const std::string & name)
{
  const std::string file = ament_index_cpp::get_package_share_directory("sim") +
    "/urdf/sentry_v2.urdf.xacro";
  const std::string name_arg = "name:=" + name;
  int output[2];
  if (pipe(output) < 0) {throw std::runtime_error("could not open xacro output pipe");}
  const pid_t pid = fork();
  if (pid < 0) {
    close(output[0]);
    close(output[1]);
    throw std::runtime_error("could not start xacro");
  }
  if (pid == 0) {
    close(output[0]);
    if (dup2(output[1], STDOUT_FILENO) < 0) {_exit(127);}
    close(output[1]);
    execlp("xacro", "xacro", file.c_str(), "opponent:=true", name_arg.c_str(),
      static_cast<char *>(nullptr));
    _exit(127);
  }
  close(output[1]);
  std::string result;
  char buffer[4096];
  ssize_t count;
  do {
    count = read(output[0], buffer, sizeof(buffer));
    if (count > 0) {result.append(buffer, static_cast<std::size_t>(count));}
  } while (count > 0 || (count < 0 && errno == EINTR));
  close(output[0]);
  int status = 0;
  pid_t waited;
  do {waited = waitpid(pid, &status, 0);} while (waited < 0 && errno == EINTR);
  if (count < 0 || waited < 0 || !WIFEXITED(status) || WEXITSTATUS(status) != 0) {
    throw std::runtime_error("xacro opponent generation failed");
  }
  return result;
}

void visit_elements(tinyxml2::XMLElement * element,
  const std::function<void(tinyxml2::XMLElement *)> & visitor)
{
  visitor(element);
  for (auto * child = element->FirstChildElement(); child; child = child->NextSiblingElement()) {
    visit_elements(child, visitor);
  }
}

std::string opponent_urdf(const std::string & name)
{
  tinyxml2::XMLDocument document;
  const std::string xml = xacro_opponent(name);
  if (document.Parse(xml.c_str()) != tinyxml2::XML_SUCCESS || !document.RootElement()) {
    throw std::runtime_error("could not parse opponent URDF");
  }
  // Match opponents.py: render collision shapes, then fix all chassis joints.
  visit_elements(document.RootElement(), [&document](tinyxml2::XMLElement * element) {
    if (std::strcmp(element->Name(), "link") != 0 || !element->FirstChildElement("collision")) {
      return;
    }
    while (auto * visual = element->FirstChildElement("visual")) {element->DeleteChild(visual);}
    for (auto * collision = element->FirstChildElement("collision"); collision;
      collision = collision->NextSiblingElement("collision"))
    {
      auto * visual = document.NewElement("visual");
      for (auto * child = collision->FirstChildElement(); child;
        child = child->NextSiblingElement())
      {
        visual->InsertEndChild(child->DeepClone(&document));
      }
      const auto * link_name = element->Attribute("name");
      if (link_name && std::strcmp(link_name, "root") == 0) {
        auto * geometry = visual->FirstChildElement("geometry");
        if (auto * mesh = geometry ? geometry->FirstChildElement("mesh") : nullptr) {
          mesh->SetAttribute("scale", "0.75 0.75 1");
        }
      }
      element->InsertEndChild(visual);
    }
  });
  visit_elements(document.RootElement(), [](tinyxml2::XMLElement * element) {
    if (std::strcmp(element->Name(), "joint") != 0) {return;}
    const char * joint_name = element->Attribute("name");
    if (joint_name && (std::strcmp(joint_name, "headlink") == 0 ||
      std::strcmp(joint_name, "headpitch") == 0)) {return;}
    element->SetAttribute("type", "fixed");
    for (const char * tag : {"limit", "axis", "dynamics"}) {
      while (auto * child = element->FirstChildElement(tag)) {element->DeleteChild(child);}
    }
  });
  tinyxml2::XMLPrinter printer;
  document.Print(&printer);
  return printer.CStr();
}

class OpponentDriver : public rclcpp::Node
{
public:
  OpponentDriver() : Node("opponent_driver")
  {
    name_ = declare_parameter<std::string>("name", "opponent_0");
    z_ = declare_parameter("z", -0.0115);
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>("/target/ground_truth_odom", 10,
      [this](nav_msgs::msg::Odometry::ConstSharedPtr msg) {
        if (!start_) {start_.emplace(msg->pose.pose.position.x, msg->pose.pose.position.y);}
      });
    timer_ = create_timer(std::chrono::duration<double>(0.5), [this]() {spawn();});
  }

private:
  std::optional<std::vector<std::string>> model_names()
  {
    gz::msgs::Empty request;
    gz::msgs::Scene reply;
    bool result = false;
    if (!gz_node_->Request(std::string(kWorld) + "scene/info", request, 2000u, reply, result) ||
      !result) {return std::nullopt;}
    std::vector<std::string> names;
    for (const auto & model : reply.model()) {names.push_back(model.name());}
    return names;
  }

  bool present(const std::optional<std::vector<std::string>> & names) const
  {
    if (!names) {return false;}
    for (const auto & name : *names) {if (name == name_) {return true;}}
    return false;
  }

  void spawn()
  {
    if (!start_) {return;}
    if (!gz_node_) {
      if (!std::getenv("GZ_IP")) {setenv("GZ_IP", "127.0.0.1", 0);}
      gz_node_ = std::make_unique<gz::transport::Node>();
    }
    auto names = model_names();
    if (names && !present(names)) {
      gz::msgs::EntityFactory request;
      request.set_sdf(opponent_urdf(name_));
      request.set_name(name_);
      request.set_allow_renaming(false);
      request.mutable_pose()->mutable_position()->set_x(start_->first);
      request.mutable_pose()->mutable_position()->set_y(start_->second);
      request.mutable_pose()->mutable_position()->set_z(z_);
      gz::msgs::Boolean reply;
      bool result = false;
      // A create reply may time out after spawning; scene/info decides completion.
      gz_node_->Request(std::string(kWorld) + "create", request, 2000u, reply, result);
      names = model_names();
    }
    if (present(names)) {
      timer_->cancel();
      RCLCPP_INFO(get_logger(), "%s spawned at (%.2f, %.2f)",
        name_.c_str(), start_->first, start_->second);
    }
  }

  std::string name_;
  double z_;
  std::optional<std::pair<double, double>> start_;
  std::unique_ptr<gz::transport::Node> gz_node_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};
}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OpponentDriver>());
  rclcpp::shutdown();
  return 0;
}
