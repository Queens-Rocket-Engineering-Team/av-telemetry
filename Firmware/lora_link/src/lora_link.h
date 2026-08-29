#pragma once

#include <cstdint>
#include <cstring>
#include <aim_catalog.h>

namespace lora {

static constexpr uint8_t kFastPacketSize = 13;
static constexpr uint8_t kSlowPacketSize = 5;
static constexpr uint32_t kNodeAliveTimeoutMs = 10000;

#pragma pack(push, 1)

struct Header {
  uint8_t flight_state : 3;
  uint8_t seq_cnt      : 4;
  uint8_t frame_type   : 1;
};

struct FastFrame {
  Header   header;
  int16_t  alt_m;
  int16_t  accel_raw;
  int32_t  gps_lat;
  int32_t  gps_lon;

  // Wire-native setters: accept raw values directly.
  void setAltitudeFromWire(int32_t cm)       { alt_m = static_cast<int16_t>(cm / 100); }
  void setAccelFromWire(int32_t mm_s2)       { accel_raw = static_cast<int16_t>(mm_s2 * 100 / 9810); }  // 1G = 9810 mm/s² (magnitude x100)
  void setGpsPosition(int32_t lat1e7, int32_t lon1e7) {
    gps_lat = lat1e7;
    gps_lon = lon1e7;
  }

  // Engineering-unit getters for telemetry decoding.
  float  getAccelG()       const { return static_cast<float>(accel_raw) * 0.01f; }
};

struct SlowFrame {
  Header   header;
  uint8_t  gps_sats   : 4;
  uint8_t  gps_fix    : 1;
  uint8_t  reserved   : 3;
  uint8_t  vcc_raw;
  uint8_t  fet_status;
  uint8_t  liveness;

  bool   hasGpsFix()      const { return gps_fix != 0; }
  uint8_t getSatellites() const { return gps_sats; }

  float getBatteryVolts() const { return static_cast<float>(vcc_raw) * 0.05f; }

  void setGpsStatus(uint8_t sats, bool hasFix) {
    gps_sats = sats & 0x0F;
    gps_fix  = hasFix ? 1U : 0U;
  }

  void setBatteryVolts(float volts) {
    if (volts < 0.0f) volts = 0.0f;
    if (volts > 12.75f) volts = 12.75f;
    vcc_raw = static_cast<uint8_t>(volts / 0.05f + 0.5f);
  }

  void setSolenoidState(uint8_t channel, bool energized) {
    if (channel < 6) {
      if (energized) {
        fet_status |= (1U << channel);
      } else {
        fet_status &= ~(1U << channel);
      }
    }
  }

  void setLivenessMask(uint8_t mask) {
    liveness = mask & 0x1F;
  }
};

#pragma pack(pop)

static_assert(sizeof(FastFrame) == kFastPacketSize, "FastFrame size mismatch");
static_assert(sizeof(SlowFrame) == kSlowPacketSize, "SlowFrame size mismatch");

inline bool isFastFrame(const uint8_t* buf) { return (buf[0] & 0x80) == 0; }
inline bool isSlowFrame(const uint8_t* buf) { return (buf[0] & 0x80) != 0; }

inline void encodeFast(const FastFrame& f, uint8_t buf[kFastPacketSize]) {
  memcpy(buf, &f, kFastPacketSize);
}

inline FastFrame decodeFast(const uint8_t buf[kFastPacketSize]) {
  FastFrame f = {};
  memcpy(&f, buf, kFastPacketSize);
  return f;
}

inline void encodeSlow(const SlowFrame& f, uint8_t buf[kSlowPacketSize]) {
  memcpy(buf, &f, kSlowPacketSize);
}

inline SlowFrame decodeSlow(const uint8_t buf[kSlowPacketSize]) {
  SlowFrame f = {};
  memcpy(&f, buf, kSlowPacketSize);
  return f;
}

static constexpr uint8_t kTrackedNodeCount = 5;

inline const char* sourceName(aim::Source src) {
  switch (src) {
    case aim::Source::Ucm:       return "UCM";
    case aim::Source::Lcm:       return "LCM";
    case aim::Source::Altimeter: return "ALT";
    case aim::Source::Gps:       return "GPS";
    case aim::Source::Power:     return "PWR";
    case aim::Source::Comms:     return "COMMS";
    default:                     return "?";
  }
}

inline aim::Source trackedSource(uint8_t index) {
  switch (index) {
    case 0: return aim::Source::Ucm;
    case 1: return aim::Source::Lcm;
    case 2: return aim::Source::Altimeter;
    case 3: return aim::Source::Gps;
    case 4: return aim::Source::Power;
    default: return static_cast<aim::Source>(0);
  }
}

class LivenessTracker {
 public:
  void recordRx(aim::Source src, uint32_t nowMs) {
    switch (src) {
      case aim::Source::Ucm:       lastHeardMs_[0] = nowMs; everHeard_[0] = true; return;
      case aim::Source::Lcm:       lastHeardMs_[1] = nowMs; everHeard_[1] = true; return;
      case aim::Source::Altimeter: lastHeardMs_[2] = nowMs; everHeard_[2] = true; return;
      case aim::Source::Gps:       lastHeardMs_[3] = nowMs; everHeard_[3] = true; return;
      case aim::Source::Power:     lastHeardMs_[4] = nowMs; everHeard_[4] = true; return;
      default: return;
    }
  }

  uint8_t getMask(uint32_t nowMs) const {
    uint8_t mask = 0U;
    for (uint8_t i = 0U; i < kTrackedNodeCount; i++) {
      if (everHeard_[i] && (nowMs - lastHeardMs_[i] < kNodeAliveTimeoutMs)) {
        mask |= (1U << i);
      }
    }
    return mask;
  }

  bool isAlive(uint8_t i, uint32_t nowMs) const {
    return everHeard_[i] && (nowMs - lastHeardMs_[i] < kNodeAliveTimeoutMs);
  }

  bool everHeard(uint8_t i)     const { return everHeard_[i]; }
  uint32_t lastHeardMs(uint8_t i) const { return lastHeardMs_[i]; }

  // Receiver side: inspect liveness bitmask decoded from a SlowFrame.
  static bool isNodeAlive(uint8_t mask, uint8_t nodeIndex) {
    return (mask & (1U << nodeIndex)) != 0;
  }

 private:
  uint32_t lastHeardMs_[kTrackedNodeCount] = {};
  bool     everHeard_[kTrackedNodeCount]   = {};
};

}  // namespace lora
