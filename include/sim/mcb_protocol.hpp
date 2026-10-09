// Copyright 2026 Thornbots
// Licensed under the Apache License, Version 2.0.

#ifndef SIM__MCB_PROTOCOL_HPP_
#define SIM__MCB_PROTOCOL_HPP_

#include <array>
#include <cstddef>
#include <cstdint>
#include <vector>

namespace sim::mcb_protocol
{

constexpr uint16_t kNavGoal = 0;
constexpr uint16_t kCvTarget = 1;
constexpr uint16_t kPose = 2;
constexpr uint16_t kRefSys = 3;
constexpr uint16_t kRelocalize = 4;
constexpr uint16_t kPing = 5;
constexpr uint8_t kOdomPods = 0;
constexpr uint8_t kHeadByte = 0xA5;
constexpr size_t kRxBufferSize = 1024;
constexpr uint8_t kCrc8Init = 0xFF;
constexpr uint16_t kCrc16Init = 0xFFFF;
constexpr uint8_t kCvTargetFlagFire = 0x01;
constexpr uint8_t kCvTargetFlagTypeCBasedPatrol = 0x02;
constexpr uint8_t kCvTargetFlagTurnToHit = 0x04;
constexpr uint8_t kCvTargetFlagsDefault =
  kCvTargetFlagTurnToHit | kCvTargetFlagTypeCBasedPatrol;

struct NavGoal
{
  static constexpr uint16_t kType = kNavGoal;
  static constexpr size_t kSize = 8;
  float x = 0.0F;
  float y = 0.0F;
};

struct CvTarget
{
  static constexpr uint16_t kType = kCvTarget;
  static constexpr size_t kSize = 15;
  float x = 0.0F;
  float y = 0.0F;
  float z = 0.0F;
  uint16_t delay_ms = 0;
  uint8_t flags = kCvTargetFlagsDefault;
};

struct Relocalize
{
  static constexpr uint16_t kType = kRelocalize;
  static constexpr size_t kSize = 8;
  float x = 0.0F;
  float y = 0.0F;
};

struct Pose
{
  static constexpr uint16_t kType = kPose;
  static constexpr size_t kSize = 25;
  float x = 0.0F;
  float y = 0.0F;
  float vel_x = 0.0F;
  float vel_y = 0.0F;
  float head_pitch = 0.0F;
  float head_yaw = 0.0F;
  uint8_t odom_status = kOdomPods;
};

struct RefSys
{
  static constexpr uint16_t kType = kRefSys;
  static constexpr size_t kSize = 11;
  uint8_t game_stage = 0;
  uint16_t stage_time_remaining = 0;
  uint16_t robot_hp = 0;
  uint8_t robot_id = 0;
  float delta_angle_got_hit_in = 0.0F;
  uint8_t booleans = 0;
};

struct Ping
{
  static constexpr uint16_t kType = kPing;
  static constexpr size_t kSize = 1;
  uint8_t number = 0;
};

const std::array<uint8_t, 256> & crc8_table();
const std::array<uint16_t, 256> & crc16_table();
uint8_t crc8(const uint8_t * data, size_t size, uint8_t init = kCrc8Init);
uint16_t crc16(const uint8_t * data, size_t size, uint16_t init = kCrc16Init);

std::vector<uint8_t> pack(const NavGoal & msg);
std::vector<uint8_t> pack(const CvTarget & msg);
std::vector<uint8_t> pack(const Relocalize & msg);
std::vector<uint8_t> pack(const Pose & msg);
std::vector<uint8_t> pack(const RefSys & msg);
std::vector<uint8_t> pack(const Ping & msg);

template<typename T>
T unpack(const std::vector<uint8_t> & data);
template<> NavGoal unpack<NavGoal>(const std::vector<uint8_t> & data);
template<> CvTarget unpack<CvTarget>(const std::vector<uint8_t> & data);
template<> Relocalize unpack<Relocalize>(const std::vector<uint8_t> & data);
template<> Pose unpack<Pose>(const std::vector<uint8_t> & data);
template<> RefSys unpack<RefSys>(const std::vector<uint8_t> & data);
template<> Ping unpack<Ping>(const std::vector<uint8_t> & data);

std::vector<uint8_t> encode_frame(
  uint16_t msg_type, const std::vector<uint8_t> & payload, uint8_t seq = 0);

template<typename T>
std::vector<uint8_t> frame(const T & msg)
{
  return encode_frame(T::kType, pack(msg));
}

struct Frame
{
  uint16_t msg_type;
  std::vector<uint8_t> payload;
};

struct ParserErrors
{
  size_t crc8 = 0;
  size_t crc16 = 0;
  size_t length = 0;
};

class DJISerial
{
public:
  explicit DJISerial(bool crc_enabled = true) : crc_enabled_(crc_enabled) {}
  std::vector<Frame> feed(const uint8_t * data, size_t size);
  std::vector<Frame> feed(const std::vector<uint8_t> & data)
  {
    return feed(data.data(), data.size());
  }
  const ParserErrors & errors() const {return errors_;}

private:
  enum class State {kSearch, kHeader, kData};
  bool crc_enabled_;
  State state_ = State::kSearch;
  std::vector<uint8_t> buffer_;
  ParserErrors errors_;
};

}  // namespace sim::mcb_protocol

#endif  // SIM__MCB_PROTOCOL_HPP_
