// Copyright 2026 Thornbots. SPDX-License-Identifier: Apache-2.0
#include <array>
#include <chrono>
#include <memory>
#include <string>
#include <gz/plugin/Register.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/sim/components/JointForceCmd.hh>
#include <gz/sim/components/JointPosition.hh>
#include <gz/sim/components/JointVelocity.hh>
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/header.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include "sim/mcb_firmware.hpp"
#include "sim/mcb_sensors.hpp"

namespace sim {
class McbFirmwareSystem : public gz::sim::System,
                          public gz::sim::ISystemConfigure,
                          public gz::sim::ISystemPreUpdate {
public:
  void Configure(const gz::sim::Entity &entity,
                 const std::shared_ptr<const sdf::Element> &sdf,
                 gz::sim::EntityComponentManager &ecm, gz::sim::EventManager &) override {
    gz::sim::Model model(entity);
    body_ = gz::sim::Link(model.CanonicalLink(ecm));
    body_.EnableVelocityChecks(ecm);
    yaw_ = model.JointByName(ecm, "headlink");
    pitch_ = model.JointByName(ecm, "headpitch");
    for (auto joint : {yaw_, pitch_}) {
      if (joint == gz::sim::kNullEntity) throw std::runtime_error("MCB missing gimbal joint");
      ecm.CreateComponent(joint, gz::sim::components::JointPosition());
      ecm.CreateComponent(joint, gz::sim::components::JointVelocity());
    }
    context_ = std::make_shared<rclcpp::Context>();
    context_->init(0, nullptr);
    rclcpp::NodeOptions options;
    options.context(context_).parameter_overrides({rclcpp::Parameter("use_sim_time", true)});
    node_ = std::make_shared<rclcpp::Node>("mcb_emulator", options);
    for (const auto &entry : std::vector<std::pair<std::string, int64_t>>{
             {"game_type", 4}, {"game_stage", sdf->Get<int>("game_stage", 4).first},
             {"stage_time_remaining", 300}, {"robot_id", 107}, {"current_hp", 400},
             {"max_hp", 400}, {"hurt_armor_id", 0}}) {
      node_->declare_parameter<int64_t>(entry.first, entry.second);
      referee_.apply(node_->get_parameter(entry.first));
    }
    for (const auto &entry : std::vector<std::pair<std::string, bool>>{
             {"restoration_zone", false}, {"exchange_zone", false},
             {"central_buff_zone", false}, {"shooter_power", true}, {"chassis_power", true}}) {
      node_->declare_parameter<bool>(entry.first, entry.second);
      referee_.apply(node_->get_parameter(entry.first));
    }
    parameters_ = node_->add_on_set_parameters_callback([this](const std::vector<rclcpp::Parameter> &params) {
      for (const auto &p : params) referee_.apply(p);
      rcl_interfaces::msg::SetParametersResult result;
      result.successful = true;
      return result;
    });
    shots_ = node_->create_publisher<std_msgs::msg::Header>("~/shot", 100);
    ticks_ = node_->create_publisher<std_msgs::msg::Header>("~/tick", 100);
    motors_ = node_->create_publisher<sensor_msgs::msg::JointState>("~/motors", 10);
    setpoints_ = node_->create_publisher<sensor_msgs::msg::JointState>("~/gimbal_setpoint", 100);
    drive_ = sdf->Get<std::string>("drive", "stop").first;
    command_ = node_->create_subscription<geometry_msgs::msg::Twist>("/cmd_vel", 10,
      [this](geometry_msgs::msg::Twist::ConstSharedPtr msg) {
        external_ = {-msg->linear.y, msg->linear.x, msg->angular.z};
        command_time_ = current_time_;
      });
    rclcpp::ExecutorOptions executor_options;
    executor_options.context = context_;
    executor_ = std::make_unique<rclcpp::executors::SingleThreadedExecutor>(executor_options);
    executor_->add_node(node_);
    pty_ = std::make_unique<mcb_firmware::PtyLink>(sdf->Get<std::string>("device_link", "/tmp/mcb_emulator_pty").first);
    firmware_ = std::make_unique<mcb_firmware::Firmware>(pty_->master(),
        sdf->Get<std::string>("firmware_binary", "").first, drive_,
        sdf->Get<bool>("auto_fire", true).first);
    RCLCPP_INFO(node_->get_logger(), "MCB physical motor system: %s, drive=%s",
                firmware_->binary().c_str(), drive_.c_str());
  }

  ~McbFirmwareSystem() override {
    firmware_.reset();
    pty_.reset();
    executor_.reset();
    if (context_) context_->shutdown("MCB physics system destroyed");
  }

  void PreUpdate(const gz::sim::UpdateInfo &info, gz::sim::EntityComponentManager &ecm) override {
    if (info.paused) return;
    current_time_ = std::chrono::duration<double>(info.simTime).count();
    executor_->spin_some();
    const auto pos = body_.WorldPose(ecm);
    const auto vel = body_.WorldLinearVelocity(ecm);
    const auto rate = body_.WorldAngularVelocity(ecm);
    const auto yp = ecm.Component<gz::sim::components::JointPosition>(yaw_);
    const auto yv = ecm.Component<gz::sim::components::JointVelocity>(yaw_);
    const auto pp = ecm.Component<gz::sim::components::JointPosition>(pitch_);
    const auto pv = ecm.Component<gz::sim::components::JointVelocity>(pitch_);
    if (!pos || !vel || !rate || yp->Data().empty() || yv->Data().empty() ||
        pp->Data().empty() || pv->Data().empty()) return;
    const double dt = std::chrono::duration<double>(info.dt).count();
    if (std::abs(dt - 0.001) > 1e-9) throw std::runtime_error("MCB requires 1 ms physics steps");
    mcb_firmware::GzHardware hw;
    hw.pos = {pos->Pos().X(), pos->Pos().Y()};
    hw.vel = {vel->X(), vel->Y()};
    hw.chassis_yaw = pos->Rot().Yaw();
    hw.chassis_rate = rate->Z();
    hw.yaw_joint = {yp->Data()[0], yv->Data()[0]};
    hw.pitch_joint = {pp->Data()[0], pv->Data()[0]};
    auto readings = firmware_->gz_readings(hw, referee_);
    const auto imu = imu_.sample(readings[4], readings[5]);
    readings[4] = imu[0];
    readings[5] = imu[1];
    const auto body_velocity = pos->Rot().RotateVectorReverse(*vel);
    const auto feedback = plant_.feedback(readings, body_velocity.X(), body_velocity.Y(), rate->Z(), dt);
    if (current_time_ - command_time_ > 0.1) external_ = {};
    const auto output = firmware_->step(1, readings, referee_, feedback, external_);
    plant_.commands = output.motors;
    const auto effort = plant_.gimbal_effort(yv->Data()[0], pv->Data()[0]);
    for (const auto &pair : {std::make_pair(yaw_, effort[0]), std::make_pair(pitch_, effort[1])}) {
      auto force = ecm.Component<gz::sim::components::JointForceCmd>(pair.first);
      if (force) force->Data() = {pair.second};
      else ecm.CreateComponent(pair.first, gz::sim::components::JointForceCmd({pair.second}));
    }
    const auto wrench = plant_.chassis_wrench();
    body_.AddWorldWrench(ecm, pos->Rot().RotateVector({wrench[0], wrench[1], 0}), {0, 0, wrench[2]});
    std_msgs::msg::Header tick;
    tick.stamp = rclcpp::Time(std::chrono::duration_cast<std::chrono::nanoseconds>(info.simTime).count());
    ticks_->publish(tick);
    if (firmware_->time_ms() % 10 == 0) {
      sensor_msgs::msg::JointState setpoint;
      setpoint.header = tick;
      setpoint.header.frame_id = "odom";
      setpoint.name = {"desired_head_yaw_field", "desired_head_pitch"};
      setpoint.position = {output.yaw + (referee_.robot_id > 100 ? mcb_firmware::kPi : 0),
                           output.pitch};
      setpoints_->publish(setpoint);
      sensor_msgs::msg::JointState motors;
      motors.header = tick;
      motors.name = {"can1/207", "can2/205", "can2/206", "can1/205", "can2/203",
                     "can2/202", "can1/201", "can1/202", "can1/203", "can1/204"};
      for (size_t i = 0; i < 10; ++i) {
        motors.position.push_back(plant_.angle[i]);
        motors.velocity.push_back(plant_.speed[i]);
        motors.effort.push_back(plant_.current[i] * (i == 1 ? 0.741 : 0.02299));
      }
      motors_->publish(motors);
    }
    for ([[maybe_unused]] auto shot : output.shots) {
      tick.frame_id = "muzzle";
      shots_->publish(tick);
    }
  }

private:
  gz::sim::Link body_{gz::sim::kNullEntity};
  gz::sim::Entity yaw_ = gz::sim::kNullEntity, pitch_ = gz::sim::kNullEntity;
  mcb_firmware::Referee referee_;
  mcb_motor::Plant plant_;
  McbImuDelay imu_;
  std::array<double, 3> external_{};
  double current_time_ = 0, command_time_ = 0;
  std::string drive_;
  std::shared_ptr<rclcpp::Context> context_;
  rclcpp::Node::SharedPtr node_;
  std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameters_;
  rclcpp::Publisher<std_msgs::msg::Header>::SharedPtr shots_, ticks_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr motors_, setpoints_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr command_;
  std::unique_ptr<mcb_firmware::PtyLink> pty_;
  std::unique_ptr<mcb_firmware::Firmware> firmware_;
};
}  // namespace sim
GZ_ADD_PLUGIN(::sim::McbFirmwareSystem, gz::sim::System,
              gz::sim::ISystemConfigure, gz::sim::ISystemPreUpdate)
