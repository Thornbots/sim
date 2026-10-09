// Copyright 2026 Thornbots
// Licensed under the Apache License, Version 2.0.

#include "sim/mcb_protocol.hpp"

#include <cstring>
#include <limits>
#include <stdexcept>

namespace sim::mcb_protocol
{
namespace
{

template<typename T, uint16_t Poly>
std::array<T, 256> make_table()
{
  std::array<T, 256> table{};
  for (size_t i = 0; i < table.size(); ++i) {
    T crc = static_cast<T>(i);
    for (int bit = 0; bit < 8; ++bit) {
      crc = (crc & 1U) ? static_cast<T>((crc >> 1) ^ Poly) : static_cast<T>(crc >> 1);
    }
    table[i] = crc;
  }
  return table;
}

void append_u16(std::vector<uint8_t> & out, uint16_t value)
{
  out.push_back(static_cast<uint8_t>(value));
  out.push_back(static_cast<uint8_t>(value >> 8));
}

void append_float(std::vector<uint8_t> & out, float value)
{
  static_assert(sizeof(float) == sizeof(uint32_t), "wire floats require 32 bits");
  uint32_t bits;
  std::memcpy(&bits, &value, sizeof(bits));
  for (int shift = 0; shift < 32; shift += 8) {
    out.push_back(static_cast<uint8_t>(bits >> shift));
  }
}

uint16_t read_u16(const uint8_t * data)
{
  return static_cast<uint16_t>(data[0] | (static_cast<uint16_t>(data[1]) << 8));
}

float read_float(const uint8_t * data)
{
  const uint32_t bits = static_cast<uint32_t>(data[0]) |
    (static_cast<uint32_t>(data[1]) << 8) |
    (static_cast<uint32_t>(data[2]) << 16) |
    (static_cast<uint32_t>(data[3]) << 24);
  float value;
  std::memcpy(&value, &bits, sizeof(value));
  return value;
}

void require_size(const std::vector<uint8_t> & data, size_t expected)
{
  if (data.size() != expected) {
    throw std::invalid_argument("MCB payload has incorrect size");
  }
}

}  // namespace

const std::array<uint8_t, 256> & crc8_table()
{
  static const auto table = make_table<uint8_t, 0x8C>();
  return table;
}

const std::array<uint16_t, 256> & crc16_table()
{
  static const auto table = make_table<uint16_t, 0x8408>();
  return table;
}

uint8_t crc8(const uint8_t * data, size_t size, uint8_t init)
{
  uint8_t crc = init;
  for (size_t i = 0; i < size; ++i) {
    crc = crc8_table()[crc ^ data[i]];
  }
  return crc;
}

uint16_t crc16(const uint8_t * data, size_t size, uint16_t init)
{
  uint16_t crc = init;
  for (size_t i = 0; i < size; ++i) {
    crc = static_cast<uint16_t>((crc >> 8) ^ crc16_table()[(crc ^ data[i]) & 0xFF]);
  }
  return crc;
}

std::vector<uint8_t> pack(const NavGoal & msg)
{
  std::vector<uint8_t> out;
  out.reserve(NavGoal::kSize);
  append_float(out, msg.x);
  append_float(out, msg.y);
  return out;
}

std::vector<uint8_t> pack(const CvTarget & msg)
{
  std::vector<uint8_t> out;
  out.reserve(CvTarget::kSize);
  append_float(out, msg.x);
  append_float(out, msg.y);
  append_float(out, msg.z);
  append_u16(out, msg.delay_ms);
  out.push_back(msg.flags);
  return out;
}

std::vector<uint8_t> pack(const Relocalize & msg)
{
  std::vector<uint8_t> out;
  out.reserve(Relocalize::kSize);
  append_float(out, msg.x);
  append_float(out, msg.y);
  return out;
}

std::vector<uint8_t> pack(const Pose & msg)
{
  std::vector<uint8_t> out;
  out.reserve(Pose::kSize);
  append_float(out, msg.x);
  append_float(out, msg.y);
  append_float(out, msg.vel_x);
  append_float(out, msg.vel_y);
  append_float(out, msg.head_pitch);
  append_float(out, msg.head_yaw);
  out.push_back(msg.odom_status);
  return out;
}

std::vector<uint8_t> pack(const RefSys & msg)
{
  std::vector<uint8_t> out;
  out.reserve(RefSys::kSize);
  out.push_back(msg.game_stage);
  append_u16(out, msg.stage_time_remaining);
  append_u16(out, msg.robot_hp);
  out.push_back(msg.robot_id);
  append_float(out, msg.delta_angle_got_hit_in);
  out.push_back(msg.booleans);
  return out;
}

std::vector<uint8_t> pack(const Ping & msg)
{
  return {msg.number};
}

template<> NavGoal unpack<NavGoal>(const std::vector<uint8_t> & data)
{
  require_size(data, NavGoal::kSize);
  return {read_float(data.data()), read_float(data.data() + 4)};
}

template<> CvTarget unpack<CvTarget>(const std::vector<uint8_t> & data)
{
  require_size(data, CvTarget::kSize);
  return {read_float(data.data()), read_float(data.data() + 4), read_float(data.data() + 8),
    read_u16(data.data() + 12), data[14]};
}

template<> Relocalize unpack<Relocalize>(const std::vector<uint8_t> & data)
{
  require_size(data, Relocalize::kSize);
  return {read_float(data.data()), read_float(data.data() + 4)};
}

template<> Pose unpack<Pose>(const std::vector<uint8_t> & data)
{
  require_size(data, Pose::kSize);
  return {read_float(data.data()), read_float(data.data() + 4), read_float(data.data() + 8),
    read_float(data.data() + 12), read_float(data.data() + 16), read_float(data.data() + 20),
    data[24]};
}

template<> RefSys unpack<RefSys>(const std::vector<uint8_t> & data)
{
  require_size(data, RefSys::kSize);
  return {data[0], read_u16(data.data() + 1), read_u16(data.data() + 3), data[5],
    read_float(data.data() + 6), data[10]};
}

template<> Ping unpack<Ping>(const std::vector<uint8_t> & data)
{
  require_size(data, Ping::kSize);
  return {data[0]};
}

std::vector<uint8_t> encode_frame(
  uint16_t msg_type, const std::vector<uint8_t> & payload, uint8_t seq)
{
  if (payload.size() > std::numeric_limits<uint16_t>::max()) {
    throw std::length_error("MCB frame payload exceeds uint16 length");
  }
  std::vector<uint8_t> out;
  out.reserve(payload.size() + 9);
  out.push_back(kHeadByte);
  append_u16(out, static_cast<uint16_t>(payload.size()));
  out.push_back(seq);
  out.push_back(crc8(out.data(), out.size()));
  append_u16(out, msg_type);
  out.insert(out.end(), payload.begin(), payload.end());
  append_u16(out, crc16(out.data(), out.size()));
  return out;
}

std::vector<Frame> DJISerial::feed(const uint8_t * data, size_t size)
{
  std::vector<Frame> frames;
  for (size_t i = 0; i < size; ++i) {
    const uint8_t byte = data[i];
    if (state_ == State::kSearch) {
      if (byte == kHeadByte) {
        buffer_ = {byte};
        state_ = State::kHeader;
      }
      continue;
    }
    buffer_.push_back(byte);
    if (state_ == State::kHeader) {
      if (buffer_.size() < 5) {
        continue;
      }
      if (crc_enabled_ && crc8(buffer_.data(), 4) != buffer_[4]) {
        ++errors_.crc8;
        state_ = State::kSearch;
        continue;
      }
      if (read_u16(buffer_.data() + 1) >= kRxBufferSize) {
        ++errors_.length;
        state_ = State::kSearch;
        continue;
      }
      state_ = State::kData;
      continue;
    }
    const size_t length = read_u16(buffer_.data() + 1);
    if (buffer_.size() < 7 + length + (crc_enabled_ ? 2 : 0)) {
      continue;
    }
    state_ = State::kSearch;
    if (crc_enabled_ && read_u16(buffer_.data() + 7 + length) !=
      crc16(buffer_.data(), 7 + length))
    {
      ++errors_.crc16;
      continue;
    }
    frames.push_back({read_u16(buffer_.data() + 5),
      std::vector<uint8_t>(buffer_.begin() + 7, buffer_.begin() + 7 + length)});
  }
  return frames;
}

}  // namespace sim::mcb_protocol
