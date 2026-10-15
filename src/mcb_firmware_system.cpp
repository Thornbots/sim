// Copyright 2026 Thornbots. SPDX-License-Identifier: Apache-2.0
#include <array>
#include <chrono>
#include <memory>
#include <map>
#include <set>
#include <atomic>
#include <gz/common/SignalHandler.hh>
#include <string>
#include <ament_index_cpp/get_package_prefix.hpp>
#include <gz/plugin/Register.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/sim/components/JointForceCmd.hh>
#include <gz/sim/components/Inertial.hh>
#include <gz/sim/components/JointPosition.hh>
#include <gz/sim/components/JointVelocity.hh>
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/header.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include "sim/mcb_firmware.hpp"
#include "sim/mcb_sensors.hpp"
#include "sim/mcb_sim_uart.hpp"

namespace sim {
class McbFirmwareSystem : public gz::sim::System,
                          public gz::sim::ISystemConfigure,
                          public gz::sim::ISystemPreUpdate,
                          public gz::sim::ISystemPostUpdate {
public:
  void Configure(const gz::sim::Entity &entity,
                 const std::shared_ptr<const sdf::Element> &sdf,
                 gz::sim::EntityComponentManager &ecm, gz::sim::EventManager &) override {
    gz::sim::Model model(entity);
    body_ = gz::sim::Link(model.CanonicalLink(ecm));
    body_.EnableVelocityChecks(ecm);
    head_ = gz::sim::Link(model.LinkByName(ecm, "head"));
    if (!head_.Valid(ecm)) throw std::runtime_error("MCB requires the yaw head link");
    head_.EnableVelocityChecks(ecm);
    head_.EnableAccelerationChecks(ecm);
    for (const auto link : model.Links(ecm)) {
      const auto inertial = ecm.Component<gz::sim::components::Inertial>(link);
      if (inertial) mass_ += inertial->Data().MassMatrix().Mass();
    }
    if (mass_ <= 0) throw std::runtime_error("MCB motor plant requires model mass");
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
    drive_ = sdf->Get<std::string>("drive", "stop").first;
    auto_fire_ = sdf->Get<bool>("auto_fire", true).first;
    backend_ = sdf->Get<std::string>("backend", "hosted").first;
    node_->declare_parameter("drive", drive_);
    node_->declare_parameter("auto_fire", auto_fire_);
    parameters_ = node_->add_on_set_parameters_callback([this](const std::vector<rclcpp::Parameter> &params) {
      rcl_interfaces::msg::SetParametersResult result;
      for (const auto &p : params) {
        if (p.get_name() == "drive") {
          try {
            const auto mode = mcb_firmware::control_mode(p.as_string(), true);
            if (backend_ == "renode" && mode > 1) throw std::invalid_argument("Board drive must be stop/simple");
          } catch (const std::exception &error) {
            result.reason = error.what();
            result.successful = false;
            return result;
          }
        }
      }
      for (const auto &p : params) {
        referee_.apply(p);
        if (p.get_name() == "drive") drive_ = p.as_string();
        if (p.get_name() == "auto_fire") auto_fire_ = p.as_bool();
      }
      if (firmware_) firmware_->set_mode(mcb_firmware::control_mode(drive_, auto_fire_));
      result.successful = true;
      return result;
    });
    shots_ = node_->create_publisher<std_msgs::msg::Header>("~/shot", 100);
    ticks_ = node_->create_publisher<std_msgs::msg::Header>("~/tick", 100);
    consumed_ticks_ = node_->create_publisher<std_msgs::msg::Header>("~/consumed_tick", 100);
    const auto aim_tick_topic = sdf->Get<std::string>("aim_tick_topic", "").first;
    if (!signals_.AddCallback([this](int) {stopping_ = true;}))
      throw std::runtime_error("MCB system could not register shutdown signal callback");
    if (!aim_tick_topic.empty()) {
      aim_period_ns_ = static_cast<int64_t>(sdf->Get<double>("aim_period_s", 0.025).first * 1e9);
      if (aim_period_ns_ <= 0 || aim_period_ns_ % 1000000 != 0)
        throw std::invalid_argument("aim hold period must align to 1 ms physics steps");
      aim_tick_sub_ = node_->create_subscription<std_msgs::msg::Header>(aim_tick_topic, 100,
        [this](std_msgs::msg::Header::ConstSharedPtr tick) {
          const auto stamp = rclcpp::Time(tick->stamp).nanoseconds();
          if (!aim_ticks_.emplace(stamp, !tick->frame_id.empty()).second)
            throw std::runtime_error("lockstep timeout: duplicate aimer tick");
        });
    }
    const auto raw_pose_ack_topic = sdf->Get<std::string>("raw_pose_ack_topic", "").first;
    if (!raw_pose_ack_topic.empty()) {
      raw_pose_ack_sub_ = node_->create_subscription<std_msgs::msg::Header>(raw_pose_ack_topic, 100,
        [this](std_msgs::msg::Header::ConstSharedPtr ack) {
          raw_pose_acks_.insert(rclcpp::Time(ack->stamp).nanoseconds());
        });
    }
    const auto scorer_pose_ack_topic = sdf->Get<std::string>("scorer_pose_ack_topic", "").first;
    const auto ekf_input_ack_topic = sdf->Get<std::string>("ekf_input_ack_topic", "").first;
    if (!ekf_input_ack_topic.empty()) {
      ekf_input_ack_sub_ = node_->create_subscription<std_msgs::msg::Header>(ekf_input_ack_topic, 100,
        [this](std_msgs::msg::Header::ConstSharedPtr ack) {
          ekf_input_acks_.insert(rclcpp::Time(ack->stamp).nanoseconds());
        });
    }
    if (!scorer_pose_ack_topic.empty()) {
      scorer_pose_ack_sub_ = node_->create_subscription<std_msgs::msg::Header>(scorer_pose_ack_topic, 100,
        [this](std_msgs::msg::Header::ConstSharedPtr ack) {
          scorer_pose_acks_.insert(rclcpp::Time(ack->stamp).nanoseconds());
        });
    }
    motors_ = node_->create_publisher<sensor_msgs::msg::JointState>("~/motors", 10);
    setpoints_ = node_->create_publisher<sensor_msgs::msg::JointState>("~/gimbal_setpoint", 100);
    command_ = node_->create_subscription<geometry_msgs::msg::Twist>("/cmd_vel", 10,
      [this](geometry_msgs::msg::Twist::ConstSharedPtr msg) {
        external_ = {-msg->linear.y, msg->linear.x, msg->angular.z};
        command_time_ = current_time_;
      });
    rclcpp::ExecutorOptions executor_options;
    executor_options.context = context_;
    executor_ = std::make_unique<rclcpp::executors::SingleThreadedExecutor>(executor_options);
    executor_->add_node(node_);
    const auto uart_path = sdf->Get<std::string>("uart_socket", "").first;
    if (uart_path.empty()) {
      pty_ = std::make_unique<mcb_firmware::PtyLink>(sdf->Get<std::string>("device_link", "/tmp/mcb_emulator_pty").first);
    } else {
      uart_ = std::make_unique<McbSimUart>(uart_path, sdf->Get<int>("baudrate", 115200).first);
    }
    const auto backend = sdf->Get<std::string>("backend", "hosted").first;
    if (backend != "hosted" && backend != "renode") throw std::runtime_error("Unknown MCB backend: " + backend);
    const auto adapter = backend == "renode" ?
        ament_index_cpp::get_package_prefix("sim") + "/lib/sim/mcb_type_c" : "";
    firmware_ = std::make_unique<mcb_firmware::Firmware>(uart_ ? uart_->firmware_fd() : pty_->master(),
        sdf->Get<std::string>("firmware_binary", "").first, drive_,
        sdf->Get<bool>("auto_fire", true).first, adapter);
    if (uart_) uart_->child_started();
    RCLCPP_INFO(node_->get_logger(), "MCB physical motor system: %s, drive=%s",
                firmware_->binary().c_str(), drive_.c_str());
  }

  ~McbFirmwareSystem() override {
    if (node_ && aim_tick_sub_) {
      RCLCPP_INFO(node_->get_logger(),
        "MCB consumption (final): raw_pose %zu holds %.6f s; aimer_uart %zu holds %.6f s",
        raw_pose_holds_, raw_pose_wait_s_, aim_holds_, aim_wait_s_);
    }
    firmware_.reset();
    uart_.reset();
    pty_.reset();
    executor_.reset();
    if (context_) context_->shutdown("MCB physics system destroyed");
  }

  void PreUpdate(const gz::sim::UpdateInfo &info, gz::sim::EntityComponentManager &ecm) override {
    if (info.paused || stopping_) return;
    current_time_ = std::chrono::duration<double>(info.simTime).count();
    executor_->spin_some();
    const int64_t stamp_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(info.simTime).count();
    if (uart_ && !uart_work([&] {uart_->before_tick(stamp_ns);})) return;
    if (uart_ && !raw_pose_ack_sub_) uart_->take_pose_stamps();
    const auto pos = body_.WorldPose(ecm);
    const auto vel = body_.WorldLinearVelocity(ecm);
    const auto rate = body_.WorldAngularVelocity(ecm);
    const auto head_pose = head_.WorldPose(ecm);
    const auto head_rate = head_.WorldAngularVelocity(ecm);
    const auto head_accel = head_.WorldLinearAcceleration(ecm);
    const auto yp = ecm.Component<gz::sim::components::JointPosition>(yaw_);
    const auto yv = ecm.Component<gz::sim::components::JointVelocity>(yaw_);
    const auto pp = ecm.Component<gz::sim::components::JointPosition>(pitch_);
    const auto pv = ecm.Component<gz::sim::components::JointVelocity>(pitch_);
    if (!pos || !vel || !rate || !head_pose || !head_rate || !head_accel ||
        yp->Data().empty() || yv->Data().empty() ||
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
    const auto gyro = head_pose->Rot().RotateVectorReverse(*head_rate);
    const double heading = referee_.robot_id > 100 ? mcb_motor::kPi : 0;
    const double yaw = head_pose->Rot().Yaw() - heading;
    readings[4] = std::atan2(std::sin(yaw), std::cos(yaw));
    readings[5] = gyro.Z();
    if (backend_ == "hosted") {
      const auto delayed = imu_.sample(readings[4], readings[5]);
      readings[4] = delayed[0];
      readings[5] = delayed[1];
    }
    const auto body_velocity = pos->Rot().RotateVectorReverse(*vel);
    const auto feedback = plant_.feedback(readings, body_velocity.X(), body_velocity.Y(), rate->Z(), dt);
    if (current_time_ - command_time_ > 0.1) external_ = {};
    const auto acceleration = head_pose->Rot().RotateVectorReverse(
        *head_accel + gz::math::Vector3d(0, 0, 9.80665)) / 9.80665;
    const std::array<double, 6> imu{gyro.X(), gyro.Y(), gyro.Z(),
                                  acceleration.X(), acceleration.Y(), acceleration.Z()};
    const auto output = firmware_->step(1, readings, referee_, feedback, external_, imu);
    if (uart_ && !uart_work([&] {uart_->after_tick(stamp_ns);})) return;
    plant_.commands = output.motors;
    const auto effort = plant_.gimbal_effort(yv->Data()[0], pv->Data()[0]);
    for (const auto &pair : {std::make_pair(yaw_, effort[0]), std::make_pair(pitch_, effort[1])}) {
      auto force = ecm.Component<gz::sim::components::JointForceCmd>(pair.first);
      if (force) force->Data() = {pair.second};
      else ecm.CreateComponent(pair.first, gz::sim::components::JointForceCmd({pair.second}));
    }
    const auto wrench = plant_.chassis_wrench(mass_);
    body_.AddWorldWrench(ecm, pos->Rot().RotateVector({wrench[0], wrench[1], 0}), {0, 0, wrench[2]});
    std_msgs::msg::Header tick;
    tick.stamp = rclcpp::Time(std::chrono::duration_cast<std::chrono::nanoseconds>(info.simTime).count());
    ticks_->publish(tick);
    if (firmware_->time_ms() % 10 == 0) {
      if (backend_ == "hosted") {
        sensor_msgs::msg::JointState setpoint;
        setpoint.header = tick;
        setpoint.header.frame_id = "odom";
        setpoint.name = {"desired_head_yaw_field", "desired_head_pitch"};
        setpoint.position = {output.yaw + (referee_.robot_id > 100 ? mcb_firmware::kPi : 0),
                             output.pitch};
        setpoints_->publish(setpoint);
      }
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
    const auto exits = projectiles_.update(feedback[2]);
    for (uint32_t shot = 0; shot < exits; ++shot) {
      tick.frame_id = "muzzle";
      shots_->publish(tick);
    }
  }

  void PostUpdate(const gz::sim::UpdateInfo &info,
                  const gz::sim::EntityComponentManager &) override {
    if (info.paused || stopping_ || !uart_ || !aim_tick_sub_) return;
    const int64_t stamp = std::chrono::duration_cast<std::chrono::nanoseconds>(info.simTime).count();
    if (raw_pose_ack_sub_) {
      for (const auto capture : uart_->take_pose_stamps()) {
        const auto started = std::chrono::steady_clock::now();
        const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
        while (!raw_pose_acks_.count(capture) ||
          (scorer_pose_ack_sub_ && !scorer_pose_acks_.count(capture)) ||
          (ekf_input_ack_sub_ && !ekf_input_acks_.count(capture))) {
          if (stopping_) return;
          executor_->spin_some();
          if (std::chrono::steady_clock::now() >= deadline)
            throw std::runtime_error("lockstep timeout: raw pose/articulated TF at " +
              std::to_string(capture));
          uart_->wait_input();
        }
        raw_pose_acks_.erase(capture);
        scorer_pose_acks_.erase(capture);
        ekf_input_acks_.erase(capture);
        ++raw_pose_holds_;
        raw_pose_wait_s_ += std::chrono::duration<double>(
          std::chrono::steady_clock::now() - started).count();
      }
    }
    std_msgs::msg::Header consumed;
    consumed.stamp = rclcpp::Time(stamp, RCL_ROS_TIME);
    if (stamp <= 0 || stamp % aim_period_ns_ != 0) {
      consumed_ticks_->publish(consumed);
      return;
    }
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
    const auto started = std::chrono::steady_clock::now();
    for (;;) {
      if (stopping_) return;
      executor_->spin_some();
      if (!uart_work([&] {uart_->poll_input(stamp);})) return;
      const auto tick = aim_ticks_.find(stamp);
      if (tick != aim_ticks_.end() && (!tick->second || uart_->received_cv(stamp))) break;
      if (std::chrono::steady_clock::now() >= deadline)
        throw std::runtime_error("lockstep timeout: aimer/relay/UART chain at " + std::to_string(stamp));
      uart_->wait_input();
    }
    aim_ticks_.erase(aim_ticks_.begin(), aim_ticks_.upper_bound(stamp));
    ++aim_holds_;
    aim_wait_s_ += std::chrono::duration<double>(std::chrono::steady_clock::now() - started).count();
    consumed_ticks_->publish(consumed);
  }

private:
  template<typename Work> bool uart_work(Work work) {
    try {work(); return !stopping_;} catch (...) {if (stopping_) return false; throw;}
  }
  gz::sim::Link body_{gz::sim::kNullEntity};
  gz::sim::Link head_{gz::sim::kNullEntity};
  gz::sim::Entity yaw_ = gz::sim::kNullEntity, pitch_ = gz::sim::kNullEntity;
  mcb_firmware::Referee referee_;
  mcb_motor::Plant plant_;
  mcb_motor::ProjectileExit projectiles_;
  McbImuDelay imu_;
  std::array<double, 3> external_{};
  double current_time_ = 0, command_time_ = 0;
  std::string drive_, backend_;
  bool auto_fire_ = true;
  double mass_ = 0;
  std::shared_ptr<rclcpp::Context> context_;
  rclcpp::Node::SharedPtr node_;
  std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameters_;
  rclcpp::Publisher<std_msgs::msg::Header>::SharedPtr shots_, ticks_;
  rclcpp::Publisher<std_msgs::msg::Header>::SharedPtr consumed_ticks_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr motors_, setpoints_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr command_;
  rclcpp::Subscription<std_msgs::msg::Header>::SharedPtr aim_tick_sub_;
  rclcpp::Subscription<std_msgs::msg::Header>::SharedPtr raw_pose_ack_sub_;
  rclcpp::Subscription<std_msgs::msg::Header>::SharedPtr scorer_pose_ack_sub_;
  rclcpp::Subscription<std_msgs::msg::Header>::SharedPtr ekf_input_ack_sub_;
  std::set<int64_t> raw_pose_acks_;
  std::set<int64_t> scorer_pose_acks_;
  std::set<int64_t> ekf_input_acks_;
  std::map<int64_t, bool> aim_ticks_;
  int64_t aim_period_ns_ = 25000000;
  size_t raw_pose_holds_ = 0, aim_holds_ = 0;
  double raw_pose_wait_s_ = 0, aim_wait_s_ = 0;
  std::unique_ptr<mcb_firmware::PtyLink> pty_;
  std::unique_ptr<mcb_firmware::Firmware> firmware_;
  std::unique_ptr<McbSimUart> uart_;
  std::atomic<bool> stopping_{false};
  gz::common::SignalHandler signals_;
};
}  // namespace sim
GZ_ADD_PLUGIN(::sim::McbFirmwareSystem, gz::sim::System,
              gz::sim::ISystemConfigure, gz::sim::ISystemPreUpdate, gz::sim::ISystemPostUpdate)
