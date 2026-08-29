#include "node.h"
#include <Adafruit_NeoPixel.h>
#include <aim_job.h>
#include <logger.h>
#include <SPI.h>
#include <RadioLib.h>
#include <lora_link.h>
#include <aim_flight_recorder.h>

static lora::LivenessTracker s_liveness;

// LoRa SPI & Radio Objects
static SPIClass s_loraSpi(pins::kRfMosi, pins::kRfMiso, pins::kRfSclk);
static Module s_radioModule(pins::kRfCs, pins::kRfDio1, pins::kRfReset, pins::kRfBusy, s_loraSpi);
static SX1262 s_radio(&s_radioModule);

static bool s_transmitting = false;
static volatile bool s_transmittedFlag = false;
static bool s_loraInitOk = false;
static bool s_lowPower = false;

static Adafruit_NeoPixel s_rgbLeds(1U, pins::kRgbData, NEO_GRB + NEO_KHZ800);

struct StateSnapshot {
  lora::FastFrame fast;
  lora::SlowFrame slow;
};

static StateSnapshot s_snapshot = {};
static uint8_t s_seqCnt = 0;

static aim::Job s_fastTxJob{100U};  // 10 Hz LoRa fast frame
static aim::Job s_slowTxJob{2000U}; // 0.5 Hz LoRa slow frame
static aim::Job s_flashLogJob{100U};  // 10 Hz flight log

// Last startTransmit() error already reported, so a persistent fault logs once
// rather than at the 20 Hz frame rate.
static int s_txLastLoggedErr = RADIOLIB_ERR_NONE;

#if defined(AIM_COMMS_SELFTEST) && defined(FLIGHT_BUILD)
#error "AIM_COMMS_SELFTEST is a bench aid and must not be built into flight firmware"
#endif

#ifdef AIM_COMMS_SELFTEST
static aim::Job s_selfTestJob{1000U};
static uint16_t s_selfTestCount = 0U;
static uint32_t s_txOkCount     = 0U;
static uint32_t s_txErrCount    = 0U;
#endif

// Hands a frame to the radio, reporting a failure the TX path used to swallow.
static bool startTx(const uint8_t* buf, uint8_t len) {
  const int state = s_radio.startTransmit(buf, len);
  if (state == RADIOLIB_ERR_NONE) {
    s_txLastLoggedErr = RADIOLIB_ERR_NONE;
#ifdef AIM_COMMS_SELFTEST
    s_txOkCount++;
#endif
    return true;
  }

#ifdef AIM_COMMS_SELFTEST
  s_txErrCount++;
#endif
  if (state != s_txLastLoggedErr) {
    s_txLastLoggedErr = state;
    LOG_ERROR("LoRa startTransmit failed, code %d", state);
  }
  return false;
}

#ifdef AIM_COMMS_SELFTEST
// Bench link check: stamp obviously-synthetic, *changing* values into the frames
// so the GS proves decode rather than just carrier, and report TX accounting on
// the console so a failure can be pinned to the Comms side or the RF path.
static void selfTestTick(uint32_t nowMs) {
  if (!s_selfTestJob.due(nowMs)) return;
  s_selfTestCount++;

  // 0..999 m sawtooth: the GS altitude readout should step once per second.
  s_snapshot.fast.alt_m     = static_cast<int16_t>(s_selfTestCount % 1000U);
  s_snapshot.fast.accel_raw = 100;   // 1.00 G magnitude — constant, but not zero
  s_snapshot.fast.setGpsPosition(451234567, -736543210);

  // Non-zero GPS status so the slow frame is distinguishable from an empty one too.
  s_snapshot.slow.setGpsStatus(9, true);

  // Battery: exercises the 8-bit vcc_raw field and the GS getBatteryVolts() path.
  s_snapshot.slow.setBatteryVolts(4.20f);

  LOG_INFO("SELFTEST n=%u alt=%d txOk=%lu txErr=%lu",
           static_cast<unsigned>(s_selfTestCount),
           static_cast<int>(s_snapshot.fast.alt_m),
           static_cast<unsigned long>(s_txOkCount),
           static_cast<unsigned long>(s_txErrCount));
}
#endif

static void setTxFlag(void) {
  s_transmittedFlag = true;
}

static void updateLed(aim::NodeState state) {
  static aim::NodeState s_lastState = static_cast<aim::NodeState>(0xFF);
  if (state == s_lastState) return;
  s_lastState = state;
  uint8_t r = 0, g = 0, b = 0;
  switch (state) {
    case aim::NodeState::Nominal: g = 255; break;
    case aim::NodeState::Fault:   r = 255; break;
    default:                      b = 255; break;
  }
  s_rgbLeds.setPixelColor(0, s_rgbLeds.Color(r, g, b));
  s_rgbLeds.show();
}

void nodeInit() {
  s_rgbLeds.begin();
  s_rgbLeds.setPixelColor(0, s_rgbLeds.Color(0, 0, 0));
  s_rgbLeds.show();

  s_loraSpi.begin();
  s_radio.reset();
  delay(100);
  int state = s_radio.begin();
  if (state == RADIOLIB_ERR_NONE) {
    s_loraInitOk = true;
    LOG_INFO("LoRa SX1262 init success");
  } else {
    LOG_ERROR("LoRa SX1262 init failed, code %d", state);
  }

  s_radio.setDio1Action(setTxFlag);

  // Configure Profile A: 250 kHz, SF8, 904.5 MHz, CR 4/5
  s_radio.setFrequency(904.5);
  s_radio.setBandwidth(250.0);
  s_radio.setSpreadingFactor(8);
  s_radio.setCodingRate(5); // CR 4/5
  s_radio.setOutputPower(20);
  s_radio.setSyncWord(0x12);
  s_radio.setPreambleLength(8);
  s_radio.setCRC(true);
  s_radio.explicitHeader();
}

void nodeUpdate(uint32_t nowMs) {
  updateLed(nodeCurrentState());

#ifdef AIM_COMMS_SELFTEST
  selfTestTick(nowMs);
#endif

  if (s_transmitting) {
    if (s_transmittedFlag) {
      s_transmittedFlag = false;
      s_radio.finishTransmit();
      s_transmitting = false;
    } else if (nowMs - s_slowTxJob.lastMs > 500U && nowMs - s_fastTxJob.lastMs > 500U) {
      s_radio.finishTransmit();
      s_transmitting = false;
      LOG_WARN("LoRa TX timeout recovered");
    } else {
      return;
    }
  }

  if (!s_transmitting && s_loraInitOk) {
    if (s_slowTxJob.due(nowMs)) {
      s_snapshot.slow.header.frame_type = 1;
      s_snapshot.slow.header.seq_cnt    = s_seqCnt;
      s_snapshot.slow.setLivenessMask(s_liveness.getMask(nowMs));

      uint8_t buf[lora::kSlowPacketSize];
      lora::encodeSlow(s_snapshot.slow, buf);

      if (startTx(buf, lora::kSlowPacketSize)) {
        s_transmitting = true;
        s_seqCnt = (s_seqCnt + 1U) % 16U;
      }
    } else if (s_fastTxJob.due(nowMs)) {
      s_snapshot.fast.header.frame_type = 0;
      s_snapshot.fast.header.seq_cnt    = s_seqCnt;

      uint8_t buf[lora::kFastPacketSize];
      lora::encodeFast(s_snapshot.fast, buf);

      if (startTx(buf, lora::kFastPacketSize)) {
        s_transmitting = true;
        s_seqCnt = (s_seqCnt + 1U) % 16U;
      }
    }
  }
}

void nodeServiceCanTx(uint32_t nowMs, AimNetwork& aim) {
  (void)nowMs;
  (void)aim;
}

void nodeOnRx(const aim::Msg& m, uint32_t nowMs) {
  s_liveness.recordRx(m.source, nowMs);

  if (m.cls == aim::Class::Event) {
    if (m.subject == aim::subject::LowPower) {
      s_lowPower = (m.b[0] == 1U);
      LOG_DEBUG("Comms low power state updated: %d", s_lowPower);
    } else if (m.subject == aim::subject::LaunchDetect) {
      if (m.b[0] == 1U) {
        s_snapshot.fast.header.flight_state = static_cast<uint8_t>(aim::FlightPhase::Boost);
        s_snapshot.slow.header.flight_state = static_cast<uint8_t>(aim::FlightPhase::Boost);
      }
    }
  } else if (m.cls == aim::Class::State) {
    if (m.subject == aim::subject::Av203) {
      s_snapshot.slow.setSolenoidState(0, m.b[0] == 1U);
    } else if (m.subject == aim::subject::Av205) {
      s_snapshot.slow.setSolenoidState(1, m.b[0] == 1U);
    } else if (m.subject == aim::subject::Av204) {
      s_snapshot.slow.setSolenoidState(2, m.b[0] == 1U);
    } else if (m.subject == aim::subject::PwrPtUcm) {
      s_snapshot.slow.setSolenoidState(3, m.b[0] == 1U);
    } else if (m.subject == aim::subject::PwrSolLcm) {
      s_snapshot.slow.setSolenoidState(4, m.b[0] == 1U);
    } else if (m.subject == aim::subject::PwrPtLcm) {
      s_snapshot.slow.setSolenoidState(5, m.b[0] == 1U);
    }

  } else if (m.cls == aim::Class::Sensor) {
    if (m.subject == aim::subject::Altitude) {
      s_snapshot.fast.setAltitudeFromWire(m.sensorValue());
    } else if (m.subject == aim::subject::Acceleration) {
      s_snapshot.fast.setAccelFromWire(m.sensorValue());
    } else if (m.subject == aim::subject::GpsPosition) {
      int32_t lat = 0, lon = 0;
      m.getGpsPosition(lon, lat);
      s_snapshot.fast.setGpsPosition(lat, lon);
      const bool hasFix = (lat != 0 || lon != 0);
      s_snapshot.slow.setGpsStatus(s_snapshot.slow.getSatellites(), hasFix);
    } else if (m.subject == aim::subject::GpsNumSats) {
      const uint8_t sats = static_cast<uint8_t>(m.sensorValue() & 0x0F);
      const bool hasFix = (s_snapshot.fast.gps_lat != 0 || s_snapshot.fast.gps_lon != 0);
      s_snapshot.slow.setGpsStatus(sats, hasFix);
    } else if (m.subject == aim::subject::BattVolt) {
      s_snapshot.slow.setBatteryVolts(static_cast<float>(m.sensorValue()) / 1000.0f);
    }
  }
}

aim::NodeState nodeCurrentState() {
  if (!s_loraInitOk) {
    return aim::NodeState::Fault;
  }
  return aim::NodeState::Nominal;
}

uint16_t nodeErrorBits() {
  return 0U;
}

void nodeServiceLog(uint32_t nowMs, AimFlightRecorder& recorder) {
  if (!s_flashLogJob.due(nowMs)) return;

  const uint32_t vals[3] = {
    nowMs,
    static_cast<uint32_t>(s_snapshot.fast.header.flight_state),
    static_cast<uint32_t>(s_seqCnt)
  };
  recorder.writeRow(vals, nowMs);
}

#ifndef FLIGHT_BUILD
static void hookCanSnapshot(Stream& out) {
  const uint32_t nowMs = millis();
  out.println("=== CAN Bus Snapshot ===");

  out.print("flight_state=");
  switch (static_cast<aim::FlightPhase>(s_snapshot.fast.header.flight_state)) {
    case aim::FlightPhase::Preflight: out.println("Preflight (0)"); break;
    case aim::FlightPhase::Boost:     out.println("Boost (1)"); break;
    case aim::FlightPhase::Coast:     out.println("Coast (2)"); break;
    case aim::FlightPhase::Decent:    out.println("Decent (3)"); break;
    case aim::FlightPhase::Landed:    out.println("Landed (4)"); break;
    default:
      out.println(static_cast<unsigned>(s_snapshot.fast.header.flight_state));
      break;
  }

  out.print("alt_m=");
  out.print(s_snapshot.fast.alt_m);
  out.println(" m");

  out.print("accel=");
  out.print(s_snapshot.fast.getAccelG(), 2);
  out.print(" G (raw=");
  out.print(s_snapshot.fast.accel_raw);
  out.println(")");

  out.print("gps_lat=");
  out.println(static_cast<long>(s_snapshot.fast.gps_lat));
  out.print("gps_lon=");
  out.println(static_cast<long>(s_snapshot.fast.gps_lon));
  out.print("gps_sats=");
  out.println(static_cast<unsigned>(s_snapshot.slow.getSatellites()));
  out.print("gps_fix=");
  out.println(s_snapshot.slow.hasGpsFix() ? "YES" : "NO");

  out.print("batt_vcc=");
  out.print(s_snapshot.slow.getBatteryVolts(), 2);
  out.println(" V");

  out.print("fet_status=0x");
  out.println(s_snapshot.slow.fet_status, HEX);
  out.print("  AV203 [0]: "); out.println((s_snapshot.slow.fet_status & (1U << 0)) ? "ENERGIZED" : "DE-ENERGIZED");
  out.print("  AV205 [1]: "); out.println((s_snapshot.slow.fet_status & (1U << 1)) ? "ENERGIZED" : "DE-ENERGIZED");
  out.print("  AV204 [2]: "); out.println((s_snapshot.slow.fet_status & (1U << 2)) ? "ENERGIZED" : "DE-ENERGIZED");
  out.print("  PWR_PT_UCM [3]: "); out.println((s_snapshot.slow.fet_status & (1U << 3)) ? "ON" : "OFF");
  out.print("  PWR_SOL_LCM [4]: "); out.println((s_snapshot.slow.fet_status & (1U << 4)) ? "ON" : "OFF");
  out.print("  PWR_PT_LCM [5]: "); out.println((s_snapshot.slow.fet_status & (1U << 5)) ? "ON" : "OFF");

  out.print("seq_cnt=");
  out.println(static_cast<unsigned>(s_seqCnt));
  out.print("low_power=");
  out.println(s_lowPower ? 1 : 0);
  out.print("liveness_mask=0x");
  out.println(s_liveness.getMask(nowMs), HEX);
}

static void hookLiveness(Stream& out) {
  const uint32_t nowMs = millis();
  out.println("Node liveness:");
  for (uint8_t i = 0U; i < lora::kTrackedNodeCount; i++) {
    out.print("  ");
    out.print(lora::sourceName(static_cast<aim::Source>(i + 1)));
    out.print(": ");
    if (!s_liveness.everHeard(i)) {
      out.println("never heard");
    } else {
      const uint32_t ageMs = nowMs - s_liveness.lastHeardMs(i);
      out.print(s_liveness.isAlive(i, nowMs) ? "ALIVE" : "DEAD");
      out.print(" ageMs=");
      out.println(static_cast<unsigned long>(ageMs));
    }
  }
}

static const AimConsoleHook s_consoleHooks[] = {
  {'p', "canbus snapshot", hookCanSnapshot},
  {'n', "node liveness", hookLiveness},
};

const AimConsoleHook* nodeConsoleHooks(uint8_t& count) {
  count = sizeof(s_consoleHooks) / sizeof(s_consoleHooks[0]);
  return s_consoleHooks;
}
#endif
