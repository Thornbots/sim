// Copyright 2026 Thornbots. SPDX-License-Identifier: Apache-2.0
#include <array>
#include <chrono>
#include <deque>
#include <mutex>
#include <thread>
#include <memory>
#include <string>
#include <ament_index_cpp/get_package_prefix.hpp>
#include <gz/plugin/Register.hh>
#include <gz/msgs/clock.pb.h>
#include <gz/sim/Link.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/sim/components/JointForceCmd.hh>
#include <gz/sim/components/Inertial.hh>
#include <gz/sim/components/JointPosition.hh>
#include <gz/sim/components/JointVelocity.hh>
#include <gz/common/SignalHandler.hh>
#include <gz/sim/Events.hh>
#include <gz/sim/EventManager.hh>
#include <gz/transport/Node.hh>
#include <atomic>
#include <csignal>
#include <geometry_msgs/msg/twist.hpp>
#include <dji_serial_bridge/msg/sim_frame_metadata.hpp>
#include <dji_serial_bridge/msg/sim_frame_ack.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rosgraph_msgs/msg/clock.hpp>
#include <std_msgs/msg/header.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include "sim/mcb_firmware.hpp"
#include "sim/mcb_sensors.hpp"
#include "sim/uart_relay.hpp"

namespace sim {
class McbFirmwareSystem : public gz::sim::System,
                          public gz::sim::ISystemConfigure,
                          public gz::sim::ISystemPreUpdate,
                          public gz::sim::ISystemPostUpdate {
public:
  void Configure(const gz::sim::Entity &entity,
                 const std::shared_ptr<const sdf::Element> &sdf,
                 gz::sim::EntityComponentManager &ecm, gz::sim::EventManager &events) override {
    events_ = &events;
    // Launch stops the UART bridge and coordinator at the same moment as gz;
    // a missing acknowledgment then is shutdown, not a lockstep failure.
    signals_.AddCallback([stopping = stopping_](int) {
      *stopping = true;
      mcb_firmware::cancelled() = true;
    });
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
    // UART coordination is spun by the gz thread inside its hold loops; parameters and
    // /cmd_vel live on node_, which a thread of its own services even while gz is held.
    sync_node_ = std::make_shared<rclcpp::Node>("mcb_emulator_sync", options);
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
      // Only stage: the next PreUpdate applies them, so a change lands on a known step
      // however late this thread ran.
      std::lock_guard<std::mutex> lock(control_mutex_);
      staged_.insert(staged_.end(), params.begin(), params.end());
      result.successful = true;
      return result;
    });
    shots_ = node_->create_publisher<std_msgs::msg::Header>("~/shot", 100);
    ticks_ = node_->create_publisher<std_msgs::msg::Header>("~/tick", 100);
    motors_ = node_->create_publisher<sensor_msgs::msg::JointState>("~/motors", 10);
    setpoints_ = node_->create_publisher<sensor_msgs::msg::JointState>("~/gimbal_setpoint", 100);
    command_ = node_->create_subscription<geometry_msgs::msg::Twist>("/cmd_vel", 10,
      [this](geometry_msgs::msg::Twist::ConstSharedPtr msg) {
        std::lock_guard<std::mutex> lock(control_mutex_);
        staged_external_ = {-msg->linear.y, msg->linear.x, msg->angular.z};
        has_staged_external_ = true;
      });
    rclcpp::ExecutorOptions executor_options;
    executor_options.context = context_;
    executor_ = std::make_unique<rclcpp::executors::SingleThreadedExecutor>(executor_options);
    executor_->add_node(sync_node_);
    control_executor_ = std::make_unique<rclcpp::executors::SingleThreadedExecutor>(executor_options);
    control_executor_->add_node(node_);
    control_thread_ = std::thread([this]() {control_executor_->spin();});
    pty_ = std::make_unique<mcb_firmware::PtyLink>(sdf->Get<std::string>("device_link", "/tmp/mcb_emulator_pty").first);
    const auto backend = sdf->Get<std::string>("backend", "hosted").first;
    if (backend != "hosted" && backend != "renode") throw std::runtime_error("Unknown MCB backend: " + backend);
    const auto adapter = backend == "renode" ?
        ament_index_cpp::get_package_prefix("sim") + "/lib/sim/mcb_type_c" : "";
    coordinated_ = sdf->Get<bool>("uart_deterministic", false).first;
    epoch_zero_ = sdf->Get<bool>("uart_epoch_zero", false).first;
    if (epoch_zero_ && !coordinated_) throw std::runtime_error("UART epoch zero requires coordination");
    if (coordinated_) {
      clock_pub_ = gz_node_.Advertise<gz::msgs::Clock>("/sim/uart_clock");
      clock_sub_ = sync_node_->create_subscription<rosgraph_msgs::msg::Clock>(
          "/clock", rclcpp::ClockQoS(), [this](rosgraph_msgs::msg::Clock::ConstSharedPtr msg) {
            const auto &stamp = msg->clock;
            if (stamp.sec < 0 || stamp.nanosec >= 1'000'000'000u) {
              failure_ = "invalid ROS clock";
              return;
            }
            const auto value = static_cast<int64_t>(stamp.sec) * 1'000'000'000 + stamp.nanosec;
            if (epoch_zero_ && ((clock_seen_ && value < clock_ns_) || (!physics_started_ && value != 0)))
              failure_ = "UART epoch zero missed or ROS clock moved backwards";
            clock_ns_ = value;
            clock_seen_ = true;
          });
      ready_sub_ = sync_node_->create_subscription<std_msgs::msg::Header>(
          "/dji_serial_bridge/sim_clock_ready", rclcpp::QoS(1).reliable().transient_local(),
          [this](std_msgs::msg::Header::ConstSharedPtr msg) {
            const auto &stamp = msg->stamp;
            if (stamp.sec >= 0 && stamp.nanosec < 1'000'000'000u)
              ready_ns_ = static_cast<int64_t>(stamp.sec) * 1'000'000'000 + stamp.nanosec;
          });
      if (epoch_zero_) {
        bootstrap_pub_ = node_->create_publisher<std_msgs::msg::Header>(
            "~/epoch_zero_ready", rclcpp::QoS(1).reliable().transient_local());
        observed_sub_ = sync_node_->create_subscription<std_msgs::msg::Header>(
            "/dji_serial_bridge/sim_clock_observed", rclcpp::QoS(1).reliable().transient_local(),
            [this](std_msgs::msg::Header::ConstSharedPtr msg) {
              if (bootstrap_done_) return;
              if (msg->stamp.sec != 0 || msg->stamp.nanosec != 0)
                failure_ = "UART bridge missed epoch zero";
              else bridge_zero_seen_ = true;
            });
        detector_sub_ = sync_node_->create_subscription<std_msgs::msg::Header>(
            sdf->Get<std::string>("uart_epoch_zero_ack", "/detector_standin/ready").first,
            rclcpp::QoS(1).reliable().transient_local(),
            [this](std_msgs::msg::Header::ConstSharedPtr msg) {
              if (msg->stamp.sec != 0 || msg->stamp.nanosec != 0)
                failure_ = "UART epoch zero acknowledgment is nonzero";
              else detector_zero_seen_ = true;
            });
      }
      metadata_ = node_->create_publisher<dji_serial_bridge::msg::SimFrameMetadata>(
          "/dji_serial_bridge/sim_frame_metadata", rclcpp::QoS(32).reliable());
      ack_ = sync_node_->create_subscription<dji_serial_bridge::msg::SimFrameAck>(
          "/dji_serial_bridge/sim_frame_ack", rclcpp::QoS(32).reliable(),
          [this](dji_serial_bridge::msg::SimFrameAck::ConstSharedPtr msg) {
            const auto &stamp = msg->header.stamp;
            const int64_t publication_ns = static_cast<int64_t>(stamp.sec) * 1'000'000'000 + stamp.nanosec;
            if (!msg->published || pending_.empty() ||
                pending_.front().id != msg->frame_id || stamp.sec < 0 ||
                stamp.nanosec >= 1'000'000'000u || publication_ns != pending_.front().delivery_ns) {
              failure_ = "invalid UART acknowledgment: " + msg->reason;
              return;
            }
            pending_.pop_front();
          });
      // Optional: hold physics at a delivered POSE until the stack has consumed it (the
      // probe publishes its capture stamp), so the EKF and TF see it before the next tick.
      consumed_topic_ = sdf->Get<std::string>("uart_pose_consumed", "none").first;
      if (consumed_topic_ == "none") consumed_topic_.clear();
      if (!consumed_topic_.empty()) {
        consumed_sub_ = sync_node_->create_subscription<std_msgs::msg::Header>(
            consumed_topic_, rclcpp::QoS(100).reliable(),
            [this](std_msgs::msg::Header::ConstSharedPtr msg) {
              const int64_t ns = static_cast<int64_t>(msg->stamp.sec) * 1'000'000'000 + msg->stamp.nanosec;
              while (!consume_pending_.empty() && consume_pending_.front() <= ns)
                consume_pending_.pop_front();
            });
      }
    }
    if (coordinated_ || sdf->Get<bool>("uart_relay_fixture", false).first) {
      relay_ = std::make_unique<UartRelay>(pty_->master(),
          sdf->Get<unsigned int>("uart_baud", 115200).first,
          sdf->Get<unsigned int>("uart_baud", 115200).first,
          static_cast<int64_t>(sdf->Get<int>("mcb_read_delay_ms", 0).first) * 1'000'000,
          coordinated_);
      if (!coordinated_) RCLCPP_WARN(node_->get_logger(), "UART relay fixture: timestamps and publication are not lockstep");
    }
    try {
      firmware_ = std::make_unique<mcb_firmware::Firmware>(relay_ ? relay_->child_fd() : pty_->master(),
          sdf->Get<std::string>("firmware_binary", "").first, drive_,
          sdf->Get<bool>("auto_fire", true).first, adapter);
    } catch (const std::runtime_error &) {
      // Ctrl-C during the board's boot calibration: stop instead of failing.
      if (!mcb_firmware::cancelled()) throw;
      events.Emit<gz::sim::events::Stop>();
      return;
    }
    if (relay_) relay_->close_child();
    if (coordinated_) {
      const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
      while (metadata_->get_subscription_count() == 0 || ack_->get_publisher_count() == 0 ||
             ready_sub_->get_publisher_count() == 0 || clock_sub_->get_publisher_count() == 0 ||
             !clock_pub_.HasConnections() || ready_ns_ < 0) {
        if (std::chrono::steady_clock::now() >= deadline)
          throw std::runtime_error("UART bridge coordination discovery timeout");
        executor_->spin_once(std::chrono::milliseconds(10));
      }
    }
    RCLCPP_INFO(node_->get_logger(), "MCB physical motor system: %s, drive=%s",
                firmware_->binary().c_str(), drive_.c_str());
  }

  ~McbFirmwareSystem() override {
    if (control_executor_) control_executor_->cancel();
    if (control_thread_.joinable()) control_thread_.join();
    firmware_.reset();
    relay_.reset();
    pty_.reset();
    executor_.reset();
    if (context_) context_->shutdown("MCB physics system destroyed");
  }

  // Apply what the control thread staged since the last step.
  void commit_control() {
    std::vector<rclcpp::Parameter> params;
    {
      std::lock_guard<std::mutex> lock(control_mutex_);
      params.swap(staged_);
      if (has_staged_external_) {
        external_ = staged_external_;
        command_time_ = current_time_;
        has_staged_external_ = false;
      }
    }
    if (params.empty()) return;
    for (const auto &p : params) {
      referee_.apply(p);
      if (p.get_name() == "drive") drive_ = p.as_string();
      if (p.get_name() == "auto_fire") auto_fire_ = p.as_bool();
    }
    if (firmware_) firmware_->set_mode(mcb_firmware::control_mode(drive_, auto_fire_));
  }

  void PreUpdate(const gz::sim::UpdateInfo &info, gz::sim::EntityComponentManager &ecm) override {
    commit_control();
    if (!firmware_) return;  // Configure was cancelled by a stop request
    if (epoch_zero_ && !bootstrap_done_) {
      executor_->spin_some();
      if (!failure_.empty()) throw std::runtime_error(failure_);
      const auto held = std::chrono::duration_cast<std::chrono::nanoseconds>(info.simTime).count();
      if (!info.paused || held != 0) throw std::runtime_error("UART epoch zero missed");
      const auto now = std::chrono::steady_clock::now();
      if (!bootstrap_started_) {
        bootstrap_started_ = true;
        bootstrap_deadline_ = now + std::chrono::seconds(30);
      }
      if (now >= bootstrap_deadline_) throw std::runtime_error("UART epoch zero acknowledgment timeout");
      if (clock_seen_ && bridge_zero_seen_ && detector_zero_seen_) {
        bootstrap_done_ = true;
        clock_ready_ = true;
        bootstrap_pub_->publish(std_msgs::msg::Header{});
      } else if (now >= bootstrap_next_) {
        gz::msgs::Clock clock;
        clock.mutable_sim();
        if (!clock_pub_.Publish(clock)) throw std::runtime_error("UART epoch zero publication failed");
        bootstrap_next_ = now + std::chrono::milliseconds(50);
      }
      return;
    }
    if (info.paused) {
      if (epoch_zero_) {
        executor_->spin_some();
        if (!failure_.empty()) throw std::runtime_error(failure_);
      }
      return;
    }
    current_time_ = std::chrono::duration<double>(info.simTime).count();
    executor_->spin_some();
    if (!failure_.empty()) throw std::runtime_error(failure_);
    physics_started_ = true;
    if (coordinated_ && !pending_.empty())
      throw std::runtime_error("UART advanced with acknowledgment pending");
    if (coordinated_) {
      const auto held = std::chrono::duration_cast<std::chrono::nanoseconds>(info.simTime).count();
      if (held == 0) return;
      gz::msgs::Clock clock;
      clock.mutable_sim()->set_sec(held / 1'000'000'000);
      clock.mutable_sim()->set_nsec(held % 1'000'000'000);
      if (!clock_pub_.Publish(clock)) throw std::runtime_error("UART delivery clock publication failed");
      const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
      while (!clock_ready_ && (ready_ns_ != held || clock_ns_ != held)) {
        if (std::chrono::steady_clock::now() >= deadline)
          throw std::runtime_error("UART bridge clock discovery timeout");
        executor_->spin_once(std::chrono::milliseconds(10));
      }
      clock_ready_ = true;
    }
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
    if (!spawn_) spawn_ = std::array<double, 3>{hw.pos[0], hw.pos[1], hw.chassis_yaw};
    hw.boot = spawn_;
    auto readings = firmware_->gz_readings(hw, referee_);
    const auto gyro = head_pose->Rot().RotateVectorReverse(*head_rate);
    const double heading = mcb_firmware::kStartYaw;
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
    const auto uart_time = std::chrono::duration_cast<std::chrono::nanoseconds>(info.simTime).count();
    if (relay_) relay_->before_step(uart_time);
    mcb_firmware::Firmware::Output output;
    try {
      output = firmware_->step(1, readings, referee_, feedback, external_, imu);
    } catch (const std::runtime_error &) {
      // kill_launch.sh and Ctrl-C signal every process, so the firmware can die
      // of SIGINT/SIGTERM just before this process learns it is stopping.
      const int by = firmware_->exit_signal();
      if (*stopping_ || by == SIGINT || by == SIGTERM) {
        events_->Emit<gz::sim::events::Stop>();
        return;
      }
      throw;
    }
    if (relay_) relay_->after_step(uart_time);
    if (coordinated_) {
      for (const auto &frame : relay_->take_delivered()) {
        if (pending_.size() >= 32) throw std::runtime_error("UART acknowledgment queue overflow");
        pending_.push_back({frame.id, uart_time});
        if (!consumed_topic_.empty() && frame.bytes.size() > 6 &&
            (frame.bytes[5] | frame.bytes[6] << 8) == mcb_protocol::kPose)
          consume_pending_.push_back(frame.capture_ns);
        dji_serial_bridge::msg::SimFrameMetadata metadata;
        metadata.header.stamp = rclcpp::Time(frame.capture_ns, RCL_ROS_TIME);
        metadata.delivery_time = rclcpp::Time(uart_time, RCL_ROS_TIME);
        metadata.frame_id = frame.id;
        metadata.frame = frame.bytes;
        metadata_->publish(metadata);
      }
    }
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
        setpoint.position = {output.yaw + mcb_firmware::kStartYaw,
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
        motors.effort.push_back(plant_.rotor_torque(i));
      }
      motors_->publish(motors);
    }
    const auto exits = projectiles_.update(feedback[2]);
    for (uint32_t shot = 0; shot < exits; ++shot) {
      tick.frame_id = "muzzle";
      shots_->publish(tick);
    }
  }

  void PostUpdate(const gz::sim::UpdateInfo &info, const gz::sim::EntityComponentManager &) override {
    if (!firmware_ || !coordinated_ || info.paused) return;
    if (pending_.empty() && consume_pending_.empty() && failure_.empty()) return;
    const auto held = std::chrono::duration_cast<std::chrono::nanoseconds>(info.simTime).count();
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(1500);
    while (failure_.empty() && (!pending_.empty() || !consume_pending_.empty() || clock_ns_ < held)) {
      if (*stopping_) {
        events_->Emit<gz::sim::events::Stop>();
        return;
      }
      if (std::chrono::steady_clock::now() >= deadline)
        throw std::runtime_error("UART acknowledgment or ROS clock timeout: pending=" +
            std::to_string(pending_.size()) + " clock=" + std::to_string(clock_ns_) +
            " held=" + std::to_string(held) + " unconsumed_poses=" +
            std::to_string(consume_pending_.size()));
      executor_->spin_once(std::chrono::milliseconds(10));
    }
    if (!failure_.empty()) throw std::runtime_error(failure_);
    if (clock_ns_ != held) throw std::runtime_error("ROS clock overshot UART hold");
  }

private:
  gz::common::SignalHandler signals_;
  std::shared_ptr<std::atomic<bool>> stopping_ = std::make_shared<std::atomic<bool>>(false);
  gz::sim::EventManager *events_ = nullptr;
  gz::transport::Node gz_node_;
  gz::transport::Node::Publisher clock_pub_;
  rclcpp::Subscription<rosgraph_msgs::msg::Clock>::SharedPtr clock_sub_;
  std::string consumed_topic_;
  std::deque<int64_t> consume_pending_;
  rclcpp::Subscription<std_msgs::msg::Header>::SharedPtr consumed_sub_;
  rclcpp::Subscription<std_msgs::msg::Header>::SharedPtr ready_sub_;
  rclcpp::Subscription<std_msgs::msg::Header>::SharedPtr observed_sub_, detector_sub_;
  rclcpp::Publisher<std_msgs::msg::Header>::SharedPtr bootstrap_pub_;
  int64_t clock_ns_ = 0;
  int64_t ready_ns_ = -1;
  bool clock_ready_ = false;
  bool epoch_zero_ = false, bootstrap_started_ = false, bootstrap_done_ = false, physics_started_ = false;
  bool clock_seen_ = false, bridge_zero_seen_ = false, detector_zero_seen_ = false;
  std::chrono::steady_clock::time_point bootstrap_deadline_, bootstrap_next_{};
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
  bool auto_fire_ = true, coordinated_ = false;
  struct PendingAck { uint64_t id; int64_t delivery_ns; };
  std::deque<PendingAck> pending_;
  std::string failure_;
  rclcpp::Publisher<dji_serial_bridge::msg::SimFrameMetadata>::SharedPtr metadata_;
  rclcpp::Subscription<dji_serial_bridge::msg::SimFrameAck>::SharedPtr ack_;
  double mass_ = 0;
  std::shared_ptr<rclcpp::Context> context_;
  rclcpp::Node::SharedPtr node_, sync_node_;
  std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> executor_, control_executor_;
  std::thread control_thread_;
  std::mutex control_mutex_;  // staged_, staged_external_, has_staged_external_
  std::vector<rclcpp::Parameter> staged_;
  std::array<double, 3> staged_external_{};
  bool has_staged_external_ = false;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameters_;
  rclcpp::Publisher<std_msgs::msg::Header>::SharedPtr shots_, ticks_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr motors_, setpoints_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr command_;
  std::unique_ptr<mcb_firmware::PtyLink> pty_;
  std::unique_ptr<UartRelay> relay_;
  std::unique_ptr<mcb_firmware::Firmware> firmware_;
  std::optional<std::array<double, 3>> spawn_;
};
}  // namespace sim
GZ_ADD_PLUGIN(::sim::McbFirmwareSystem, gz::sim::System,
              gz::sim::ISystemConfigure, gz::sim::ISystemPreUpdate,
              gz::sim::ISystemPostUpdate)
