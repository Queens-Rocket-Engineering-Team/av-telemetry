#include "node.h"
#include <Adafruit_NeoPixel.h>
#include <aim_job.h>
#include <logger.h>
#include <SPI.h>
#include <RadioLib.h>
#include <lora_link.h>

static NodeLiveness s_liveness[lora::kTrackedNodeCount];

// LoRa SPI & Radio Objects
static SPIClass s_loraSpi(pins::kRfMosi, pins::kRfMiso, pins::kRfSclk);
static Module s_radioModule(pins::kRfCs, pins::kRfDio1, pins::kRfReset, pins::kRfBusy, s_loraSpi);
static SX1262 s_radio(&s_radioModule);

// TX Queue for non-blocking LoRa transmission
static constexpr uint8_t kQueueSize = 16U;
static aim::Msg s_txQueue[kQueueSize];
static volatile uint8_t s_queueHead = 0U;
static volatile uint8_t s_queueTail = 0U;
static bool s_transmitting = false;
static volatile bool s_transmittedFlag = false;
static bool s_loraInitOk = false;
static bool s_lowPower = false;

static Adafruit_NeoPixel s_rgbLeds(1U, pins::kRgbData, NEO_GRB + NEO_KHZ800);

static void setTxFlag(void) {
  s_transmittedFlag = true;
}

static bool enqueueTx(const aim::Msg& m) {
  uint8_t nextHead = (s_queueHead + 1U) % kQueueSize;
  if (nextHead == s_queueTail) {
    return false; // Queue full
  }
  s_txQueue[s_queueHead] = m;
  s_queueHead = nextHead;
  return true;
}

static bool dequeueTx(aim::Msg& m) {
  if (s_queueHead == s_queueTail) {
    return false; // Queue empty
  }
  m = s_txQueue[s_queueTail];
  s_queueTail = (s_queueTail + 1U) % kQueueSize;
  return true;
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

void nodeLivenessInit(uint32_t nowMs) {
  for (uint8_t i = 0U; i < lora::kTrackedNodeCount; i++) {
    s_liveness[i].source = lora::kTrackedNodes[i].source;
    s_liveness[i].lastHeardMs = nowMs;
    s_liveness[i].everHeard = false;
  }
}

void nodeLivenessOnRx(aim::Source source, uint32_t nowMs) {
  for (uint8_t i = 0U; i < lora::kTrackedNodeCount; i++) {
    if (s_liveness[i].source == source) {
      s_liveness[i].lastHeardMs = nowMs;
      s_liveness[i].everHeard = true;
      return;
    }
  }
}

const NodeLiveness* nodeLivenessTable(uint8_t* countOut) {
  if (countOut != nullptr) {
    *countOut = lora::kTrackedNodeCount;
  }
  return s_liveness;
}

void nodeInit() {
  nodeLivenessInit(millis());

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

  s_radio.setFrequency(905.4);
  s_radio.setBandwidth(62.5);
  s_radio.setSpreadingFactor(10);
  s_radio.setCodingRate(6);
  s_radio.setOutputPower(20);
  s_radio.setSyncWord(0x12);
  s_radio.setPreambleLength(8);
  s_radio.setCRC(true);
}

void nodeUpdate(uint32_t nowMs) {
  updateLed(nodeCurrentState());
  (void)nowMs;

  if (s_transmitting) {
    if (s_transmittedFlag) {
      s_transmittedFlag = false;
      s_radio.finishTransmit();
      s_transmitting = false;
    } else {
      return; // Still transmitting
    }
  }

  if (!s_transmitting) {
    aim::Msg m;
    if (dequeueTx(m)) {
      lora::Packet pkt;
      pkt.cls         = m.cls;
      pkt.source      = m.source;
      pkt.subject     = m.subject;
      memcpy(&pkt.value, m.b, 4);
      pkt.timestampMs = m.timestampMs;

      uint8_t buf[lora::kPacketSize];
      lora::encode(pkt, buf);

      int state = s_radio.startTransmit(buf, lora::kPacketSize);
      if (state == RADIOLIB_ERR_NONE) {
        s_transmitting = true;
      } else {
        LOG_ERROR("LoRa startTransmit failed, code %d", state);
      }
    }
  }
}

void nodeServiceCanTx(uint32_t nowMs, AimNetwork& aim) {
#ifdef AIM_COMMS_TIME_MASTER
  static aim::Job s_timeSyncJob{1000U, 0U};
  if (s_timeSyncJob.due(nowMs)) {
    aim::Msg m = {};
    m.cls = aim::Class::Time;
    m.subject = aim::subject::TimeSync;
    if (!aim.send(m)) {
      LOG_ERROR("TimeSync TX failed");
    }
  }
#else
  (void)nowMs;
  (void)aim;
#endif
}

void nodeOnRx(const aim::Msg& m, uint32_t nowMs) {
  nodeLivenessOnRx(m.source, nowMs);

  if (m.cls == aim::Class::Event && m.subject == aim::subject::LowPower) {
    s_lowPower = (m.b[0] == 1U);
    LOG_INFO("Comms low power state updated: %d", s_lowPower);
  }

  // Enqueue message for LoRa downlink forwarding
  enqueueTx(m);
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

#ifndef FLIGHT_BUILD
static const char* sourceName(aim::Source src) {
  switch (src) {
    case aim::Source::Comms:     return "COMMS";
    case aim::Source::Ucm:       return "UCM";
    case aim::Source::Lcm:       return "LCM";
    case aim::Source::Altimeter: return "ALT";
    case aim::Source::Gps:       return "GPS";
    case aim::Source::Power:     return "PWR";
    default:                     return "?";
  }
}

static void hookLiveness(Stream& out) {
  uint8_t count = 0U;
  const NodeLiveness* table = nodeLivenessTable(&count);
  const uint32_t nowMs = millis();

  out.println("Node liveness:");
  for (uint8_t i = 0U; i < count; i++) {
    const uint32_t ageMs = nowMs - table[i].lastHeardMs;
    const bool alive = table[i].everHeard && (ageMs < lora::kNodeAliveTimeoutMs);
    out.print("  ");
    out.print(sourceName(table[i].source));
    out.print(" (0x");
    out.print(static_cast<unsigned>(table[i].source), HEX);
    out.print("): ");
    if (!table[i].everHeard) {
      out.println("never heard");
    } else {
      out.print(alive ? "ALIVE" : "DEAD");
      out.print(" ageMs=");
      out.println(static_cast<unsigned long>(ageMs));
    }
  }
}

static const AimConsoleHook s_consoleHooks[] = {
  {'n', "node liveness", hookLiveness},
};

const AimConsoleHook* nodeConsoleHooks(uint8_t& count) {
  count = sizeof(s_consoleHooks) / sizeof(s_consoleHooks[0]);
  return s_consoleHooks;
}
#endif
