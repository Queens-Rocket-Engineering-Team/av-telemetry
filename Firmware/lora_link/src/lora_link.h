#pragma once

#include <cstdint>
#include <cstring>
#include <aim_catalog.h>

namespace lora {

// 10-byte over-the-air packet: Comms encodes, GS decodes.
//   [0]     (class << 4) | (source & 0x0F)
//   [1]     subject
//   [2..5]  value   (int32, little-endian)
//   [6..9]  timestamp (uint32, little-endian)
static constexpr uint8_t kPacketSize = 10;
static constexpr uint32_t kNodeAliveTimeoutMs = 30000;

struct Packet {
  aim::Class  cls;
  aim::Source source;
  uint8_t     subject;
  int32_t     value;
  uint32_t    timestampMs;
};

inline void encode(const Packet& p, uint8_t buf[kPacketSize]) {
  buf[0] = (static_cast<uint8_t>(p.cls) << 4) | (static_cast<uint8_t>(p.source) & 0x0F);
  buf[1] = p.subject;
  memcpy(&buf[2], &p.value, 4);
  memcpy(&buf[6], &p.timestampMs, 4);
}

inline Packet decode(const uint8_t buf[kPacketSize]) {
  Packet p;
  p.cls       = static_cast<aim::Class>(buf[0] >> 4);
  p.source    = static_cast<aim::Source>(buf[0] & 0x0F);
  p.subject   = buf[1];
  memcpy(&p.value, &buf[2], 4);
  memcpy(&p.timestampMs, &buf[6], 4);
  return p;
}

// The 5 CAN nodes whose liveness both Comms and GS track.
static constexpr uint8_t kTrackedNodeCount = 5;

struct NodeEntry {
  aim::Source source;
  const char* name;
};

inline constexpr NodeEntry kTrackedNodes[kTrackedNodeCount] = {
  {aim::Source::Ucm,       "UCM"},
  {aim::Source::Lcm,       "LCM"},
  {aim::Source::Altimeter,  "ALT"},
  {aim::Source::Gps,        "GPS"},
  {aim::Source::Power,      "PWR"},
};

}  // namespace lora
