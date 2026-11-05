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
#include <cstdio>
#include <atomic>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>
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
#include "gz/msgs/world_stats.pb.h"
#include "gz/transport/Node.hh"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/header.hpp"
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
    initial_yaw_ = declare_parameter("initial_yaw", 0.0);
    const auto initial = declare_parameter<std::vector<double>>(
      "initial_position", std::vector<double>{});
    const double timeout = declare_parameter("startup_timeout_s", 30.0);
    if (!initial.empty()) {
      if (initial.size() != 2 || !std::isfinite(initial[0]) || !std::isfinite(initial[1]) ||
        !std::isfinite(z_) || !std::isfinite(initial_yaw_) || !std::isfinite(timeout) || timeout <= 0.0 ||
        !get_parameter("use_sim_time").as_bool())
      {
        throw std::invalid_argument(
          "initial_position requires two finite coordinates, sim time and a positive timeout");
      }
      start_.emplace(initial[0], initial[1]);
      ready_ = create_publisher<std_msgs::msg::Header>(
        "~/ready", rclcpp::QoS(1).reliable().transient_local());
      worker_ = std::thread([this, timeout]() {startup(timeout);});
      return;
    }
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>("/target/ground_truth_odom", 10,
      [this](nav_msgs::msg::Odometry::ConstSharedPtr msg) {
        if (!start_) {start_.emplace(msg->pose.pose.position.x, msg->pose.pose.position.y);}
      });
    timer_ = create_timer(std::chrono::duration<double>(0.5), [this]() {spawn();});
  }

  ~OpponentDriver() override
  {
    stop_ = true;
    if (worker_.joinable()) {worker_.join();}
  }

  bool failed() const {return failed_;}

private:
  // Latest /world/<name>/stats sample. gz publishes it at 5 Hz, paused or not.
  struct WorldSamples
  {
    std::mutex mutex;
    uint64_t count = 0;
    bool paused = false;
    int64_t sim_ns = -1;

    uint64_t latest_count()
    {
      std::lock_guard<std::mutex> lock(mutex);
      return count;
    }
  };

  // Waits for a sample newer than `after`; throws unless the world is paused at sim time zero.
  // Sim time never decreases, so a zero sample taken after an action means zero throughout it.
  bool check_epoch_zero(WorldSamples & world, uint64_t after,
    std::chrono::steady_clock::time_point deadline)
  {
    while (!stop_ && rclcpp::ok() && std::chrono::steady_clock::now() < deadline) {
      bool fresh = false;
      bool paused = false;
      int64_t sim_ns = -1;
      {
        std::lock_guard<std::mutex> lock(world.mutex);
        fresh = world.count > after;
        paused = world.paused;
        sim_ns = world.sim_ns;
      }
      if (fresh) {
        if (sim_ns != 0) {
          throw std::runtime_error("epoch zero missed: sim time " + std::to_string(sim_ns) + " ns");
        }
        if (!paused) {throw std::runtime_error("world is not paused at epoch zero");}
        return true;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    return false;
  }

  void startup(double timeout)
  {
    try {
      WorldSamples world;
      gz::transport::Node node;
      const auto deadline = std::chrono::steady_clock::now() +
        std::chrono::duration_cast<std::chrono::steady_clock::duration>(
          std::chrono::duration<double>(timeout));
      const bool subscribed = node.Subscribe<gz::msgs::WorldStatistics>(
        std::string(kWorld) + "stats",
        [&world](const gz::msgs::WorldStatistics & msg) {
          std::lock_guard<std::mutex> lock(world.mutex);
          ++world.count;
          world.paused = msg.paused();
          world.sim_ns = static_cast<int64_t>(msg.sim_time().sec()) * 1000000000LL +
            msg.sim_time().nsec();
        });
      if (!subscribed) {throw std::runtime_error("could not subscribe to world stats");}
      std::string sdf;
      uint64_t after = 0;  // Require a stats sample newer than this before judging the world.
      while (!stop_ && rclcpp::ok() && std::chrono::steady_clock::now() < deadline) {
        if (!check_epoch_zero(world, after, deadline)) {break;}
        gz::msgs::Empty query;
        gz::msgs::Scene scene;
        bool result = false;
        if (!node.Request(std::string(kWorld) + "scene/info", query, 500u, scene, result) ||
          !result) {
          std::this_thread::sleep_for(std::chrono::milliseconds(50));
          continue;
        }
        int matches = 0;
        bool correct = false;
        for (const auto & model : scene.model()) {
          if (model.name() != name_) {continue;}
          ++matches;
          const auto & p = model.pose().position();
          const auto & q = model.pose().orientation();
          const double dot = q.z() * std::sin(initial_yaw_ / 2) +
            q.w() * std::cos(initial_yaw_ / 2);
          correct = std::abs(p.x() - start_->first) < 1e-4 &&
            std::abs(p.y() - start_->second) < 1e-4 && std::abs(p.z() - z_) < 1e-4 &&
            std::abs(q.x()) < 1e-4 && std::abs(q.y()) < 1e-4 &&
            std::abs(std::abs(dot) - 1.0) < 1e-4;
        }
        if (matches) {
          if (matches != 1 || !correct) {
            throw std::runtime_error("model name collision or incorrect pose");
          }
          // Ready only after a zero, paused sample taken after the scene confirmed the model.
          if (!check_epoch_zero(world, world.latest_count(), deadline)) {break;}
          std_msgs::msg::Header ready;
          ready.frame_id = name_;
          ready_->publish(ready);
          RCLCPP_INFO(get_logger(), "%s ready at epoch zero", name_.c_str());
          return;
        }
        if (sdf.empty()) {sdf = opponent_urdf(name_);}
        gz::msgs::EntityFactory request;
        request.set_sdf(sdf);
        request.set_name(name_);
        request.set_allow_renaming(false);
        request.mutable_pose()->mutable_position()->set_x(start_->first);
        request.mutable_pose()->mutable_position()->set_y(start_->second);
        request.mutable_pose()->mutable_position()->set_z(z_);
        request.mutable_pose()->mutable_orientation()->set_z(std::sin(initial_yaw_ / 2));
        request.mutable_pose()->mutable_orientation()->set_w(std::cos(initial_yaw_ / 2));
        gz::msgs::Boolean reply;
        // A timed-out reply may still have spawned; the next scene/info decides.
        node.Request(std::string(kWorld) + "create", request, 500u, reply, result);
        after = world.latest_count();
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
      }
      if (stop_ || !rclcpp::ok()) {return;}
      throw std::runtime_error("scene confirmation timed out");
    } catch (const std::exception & error) {
      RCLCPP_ERROR(get_logger(), "opponent startup failed: %s", error.what());
      failed_ = true;
      rclcpp::shutdown();
    }
  }

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
  double z_, initial_yaw_;
  std::optional<std::pair<double, double>> start_;
  std::unique_ptr<gz::transport::Node> gz_node_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::Header>::SharedPtr ready_;
  std::atomic<bool> stop_{false};
  std::atomic<bool> failed_{false};
  std::thread worker_;
};
}  // namespace

int main(int argc, char ** argv)
{
  // setenv is not thread-safe, so set the default before any thread exists.
  if (!std::getenv("GZ_IP")) {setenv("GZ_IP", "127.0.0.1", 0);}
  rclcpp::init(argc, argv);
  std::shared_ptr<OpponentDriver> driver;
  try {
    driver = std::make_shared<OpponentDriver>();
  } catch (const std::exception & error) {
    std::fprintf(stderr, "opponent_driver: %s\n", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::spin(driver);
  const bool failed = driver->failed();
  driver.reset();
  if (rclcpp::ok()) {rclcpp::shutdown();}
  return failed ? 1 : 0;
}
