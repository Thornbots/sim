// Copyright 2026 Thornbots
// Licensed under the Apache License, Version 2.0.

// gz GUI plugin that moves the 3D view's camera to follow our robot and keep
// the nearest listed target in frame, so recordings need no digital zoom.
// Off until /gui/follow_camera (gz.msgs.Boolean) enables it; framing lives in
// sim/follow_camera.hpp. see README.md#follow-camera for design rationale

#include <gz/msgs/boolean.pb.h>
#include <gz/msgs/clock.pb.h>

#include <cmath>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include <gz/common/Console.hh>
#include <gz/gui/Application.hh>
#include <gz/gui/GuiEvents.hh>
#include <gz/gui/MainWindow.hh>
#include <gz/gui/Plugin.hh>
#include <gz/math/Matrix3.hh>
#include <gz/plugin/Register.hh>
#include <gz/rendering/Camera.hh>
#include <gz/rendering/RenderingIface.hh>
#include <gz/rendering/Scene.hh>
#include <gz/transport/Node.hh>

#include "sim/follow_camera.hpp"

namespace sim
{

class FollowCamera : public gz::gui::Plugin
{
  Q_OBJECT

public:
  FollowCamera() = default;

  void LoadConfig(const tinyxml2::XMLElement * elem) override
  {
    if (title.empty()) title = "Follow camera";
    follow_camera::Settings s;
    std::string clock_topic = "/world/field/clock";
    if (elem) {
      const auto number = [elem](const char * key, double & value) {
          if (auto * child = elem->FirstChildElement(key)) child->QueryDoubleText(&value);
        };
      number("yaw", s.yaw);
      number("hfov", s.hfov);
      number("pitch", s.pitch);
      number("look_height", s.look_height);
      number("pad", s.pad);
      number("fill", s.fill);
      number("min_width", s.min_width);
      number("max_width", s.max_width);
      number("max_span", s.max_span);
      number("settle_s", s.settle_s);
      number("zoom_settle_s", s.zoom_settle_s);
      number("switch_ratio", s.switch_ratio);
      number("velocity_s", s.velocity_s);
      if (auto * child = elem->FirstChildElement("robot"); child && child->GetText()) {
        robot_ = child->GetText();
      }
      for (auto * child = elem->FirstChildElement("target"); child;
           child = child->NextSiblingElement("target")) {
        if (child->GetText()) targets_.emplace_back(child->GetText());
      }
      if (auto * child = elem->FirstChildElement("clock_topic"); child && child->GetText()) {
        clock_topic = child->GetText();
      }
      if (auto * child = elem->FirstChildElement("enabled")) child->QueryBoolText(&enabled_);
    }
    framer_ = follow_camera::Framer(s);
    node_.Subscribe(clock_topic, &FollowCamera::OnClock, this);
    node_.Advertise("/gui/follow_camera", &FollowCamera::OnEnable, this);
    gz::gui::App()->findChild<gz::gui::MainWindow *>()->installEventFilter(this);
  }

protected:
  bool eventFilter(QObject * obj, QEvent * event) override
  {
    if (event->type() == gz::gui::events::Render::kType) OnRender();
    return QObject::eventFilter(obj, event);
  }

private:
  void OnClock(const gz::msgs::Clock & msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    sim_time_ = msg.sim().sec() + 1e-9 * msg.sim().nsec();
  }

  bool OnEnable(const gz::msgs::Boolean & req, gz::msgs::Boolean & rep)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (req.data() && !enabled_) reset_ = true;
    enabled_ = req.data();
    rep.set_data(true);
    return true;
  }

  void OnRender()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!enabled_ || !sim_time_) return;
    if (!scene_) scene_ = gz::rendering::sceneFromFirstRenderEngine();
    if (!scene_) return;
    if (!camera_) {
      for (unsigned int i = 0; i < scene_->NodeCount() && !camera_; ++i) {
        camera_ = std::dynamic_pointer_cast<gz::rendering::Camera>(scene_->NodeByIndex(i));
      }
      if (!camera_) return;
    }
    const auto robot = scene_->NodeByName(robot_);
    if (!robot) return;
    std::vector<std::pair<std::string, follow_camera::Vector3d>> targets;
    for (const auto & name : targets_) {
      if (const auto node = scene_->NodeByName(name)) {
        targets.emplace_back(name, node->WorldPosition());
      }
    }
    if (reset_) camera_->SetHFOV(framer_.settings().hfov);
    const double dt = last_time_ ? std::max(0.0, *sim_time_ - *last_time_) : 0.0;
    last_time_ = sim_time_;
    const double tan_h = std::tan(camera_->HFOV().Radian() / 2);
    const follow_camera::Lens lens{tan_h, tan_h / camera_->AspectRatio()};
    const auto eye = framer_.update(robot->WorldPosition(), targets, lens, dt, reset_);
    reset_ = false;
    const auto & s = framer_.settings();
    camera_->SetWorldPose(gz::math::Pose3d(eye, gz::math::Quaterniond(0, s.pitch, s.yaw)));
  }

  std::mutex mutex_;
  gz::transport::Node node_;
  gz::rendering::ScenePtr scene_;
  gz::rendering::CameraPtr camera_;
  follow_camera::Framer framer_;
  std::string robot_ = "sentry";
  std::vector<std::string> targets_;
  std::optional<double> sim_time_, last_time_;
  bool enabled_ = false, reset_ = true;
};

}  // namespace sim

GZ_ADD_PLUGIN(sim::FollowCamera, gz::gui::Plugin)

#include "FollowCamera.moc"
