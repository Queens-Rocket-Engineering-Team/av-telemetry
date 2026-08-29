#include "qlcp_uplink.h"
#include "global.h"
#include "radio_control.h"
#include "power_sensors.h"
#include "gps.h"
#include <lora_link.h>
extern "C" {
#include "wifi_tools.h"
#include <qlcp_lib.h>
}
#include <WiFi.h>
#include <esp_netif.h>
#include <esp_timer.h>

// QLCP CONFIGURATION CONTRACT (GREG Ground Station Interface)
// -------------------------------------------------------------
// Protocol v3 (ctl-qlcp-lib @ 13682f9). Two v3 rules drive this file:
//
// 1. sensor_id is the zero-based ordinal of each entry below, traversed group
//    by group in serialized member order (spec 3.5). The kSen* bases in this
//    file MUST track this JSON exactly — a mismatch silently relabels every
//    reading after the first divergence, with no error anywhere.
// 2. Units live here and ONLY here. v3 removed the per-reading unit byte from
//    DATA packets, so the server takes units from this JSON. Every sensor needs
//    a non-empty unit string — the server rejects the whole CONFIG and drops the
//    connection on the first "" it finds, so genuinely dimensionless quantities
//    get a descriptive word ("count", "state") rather than being left blank.
//
// GREG is sensor-only: it relays rocket telemetry and reports its own health,
// but owns no controls, so there is no "controls" object. The rocket's FET
// states arrive over LoRa but are deliberately not published — they are the
// UCM/LCM's controls, not GREG's, and QLCP has no way to express relayed
// control ownership.
// --- QLCP Schema & Telemetry Integration Guide ---
// Available Data on AIM Network / LoRa link:
//   - FastFrame (100 Hz): Altitude (m), Accel (G via getAccelG()), Pt204 Chamber Pressure (PSI via getPressurePsi())
//   - SlowFrame (1 Hz):   GPS Lat/Lon/Sats, Node Liveness (5 tracked nodes), Solenoid States (fet_status bitmask for AV203, AV205, AV204)
//
// Recommended QLCP Schema Updates (kBoardQlcpConfigJson & GREG Ground UI):
//   1. rocket_position: Replace "Vel" ("m/s") with "Accel" ("G") and "ChamberPt" ("PSI") to match FastFrame.
//   2. rocket_nodes: Change "unit": "ms" to "unit": "" (readings send 1.0=alive, 0.0=dead, -1.0=no link).
//   3. Solenoids (Optional): Expose fet_status bitmask from SlowFrame as valve telemetry.
constexpr char kBoardQlcpConfigJson[] = R"json({
  "device_name": "GREG",
  "device_type": "Sensor Monitor",
  "sensors": {
    "rocket_position": {
      "Lat":   { "unit": "deg" },
      "Lon":   { "unit": "deg" },
      "Alt":   { "unit": "m" },
      "Accel": { "unit": "G" },
      "Sats":  { "unit": "count" }
    },
    "rocket_nodes": {
      "UCM": { "unit": "state" },
      "LCM": { "unit": "state" },
      "ALT": { "unit": "state" },
      "GPS": { "unit": "state" },
      "PWR": { "unit": "state" }
    },
    "rocket_link": {
      "RSSI":    { "unit": "dBm" },
      "SNR":     { "unit": "dB" },
      "FreqErr": { "unit": "Hz" },
      "Packets": { "unit": "count" },
      "LinkAge": { "unit": "ms" }
    },
    "rocket_radio_config": {
      "Freq": { "unit": "MHz" },
      "BW":   { "unit": "kHz" },
      "SF":   { "unit": "count" },
      "CR":   { "unit": "count" }
    },
    "ground_station": {
      "BattV":      { "unit": "V" },
      "SysCurrent": { "unit": "A" },
      "AmbTemp":    { "unit": "degC" }
    },
    "voltage_sense": {
      "RocketBatt": { "unit": "V" }
    }
  }
})json";

// sensor_id bases, one per group above, in JSON member order.
constexpr uint8_t kSenPositionBase = 0U;   // Lat, Lon, Alt, Accel, Sats
constexpr uint8_t kSenNodesBase    = 5U;   // UCM, LCM, ALT, GPS, PWR
constexpr uint8_t kSenLinkBase     = 10U;  // RSSI, SNR, FreqErr, Packets, LinkAge
constexpr uint8_t kSenRadioBase    = 15U;  // Freq, BW, SF, CR
constexpr uint8_t kSenGsBase       = 19U;  // BattV, SysCurrent, AmbTemp
constexpr uint8_t kSenRocketBatt   = 22U;  // RocketBatt
constexpr uint8_t kSensorCount     = 23U;

static_assert(kSenNodesBase == kSenPositionBase + 5U, "rocket_position holds 5 sensors");
static_assert(kSenLinkBase == kSenNodesBase + lora::kTrackedNodeCount, "rocket_nodes holds one sensor per tracked node");
static_assert(kSenRadioBase == kSenLinkBase + 5U, "rocket_link holds 5 sensors");
static_assert(kSenGsBase == kSenRadioBase + 4U, "rocket_radio_config holds 4 sensors");
static_assert(kSenRocketBatt == kSenGsBase + 3U, "ground_station holds 3 sensors");
static_assert(kSensorCount == kSenRocketBatt + 1U, "voltage_sense holds 1 sensor");

enum QlcpNetState : uint8_t {
  QLCP_NET_IDLE = 0U,
  QLCP_NET_DISCOVER,
  QLCP_NET_TCP_CONNECT,
  QLCP_NET_CONNECTED,
  QLCP_NET_BACKOFF
};

static QlcpNetState s_netState = QLCP_NET_IDLE;
static uint32_t s_stateEnteredMs = 0U;
static uint32_t s_lastRxMs = 0U;
static uint32_t s_backoffMs = 1000U;
static net_link_t s_netLink = {};

// CONFIG is the first packet on every new TCP connection (spec 11), and its ACK
// is what gates the first timesync cycle (spec 7.7.4.1), so the sequence it was
// sent under has to be remembered to match that ACK.
static bool s_configSent = false;
static bool s_configAcked = false;
static uint8_t s_configSeq = 0U;

static uint8_t s_sequence = 0U;          // wrapping 8-bit, per endpoint (spec 3.3)
static int64_t s_tsOffsetUs = 0;         // deviceTime_us - serverTime_us (spec 7.7.3)
static uint32_t s_lastTimesyncMs = 0U;
static uint16_t s_streamFrequencyHz = 0U;
static uint32_t s_lastStreamTxMs = 0U;

constexpr uint32_t kNetTcpConnectTimeoutMs = 5000U;
constexpr uint32_t kNetRxIdleTimeoutMs     = 15000U;
constexpr uint32_t kNetBackoffMinMs        = 1000U;
constexpr uint32_t kNetBackoffMaxMs        = 8000U;

// Spec 7.7.4.2 requires a resync at least every 60 s; 55 s keeps us inside that
// ceiling even with main-loop jitter, at a cost of one 17-byte packet.
constexpr uint32_t kTimesyncIntervalMs = 55000U;

// Spec 7.3: when the requested rate exceeds our capability, stream at the
// highest supported rate that does not exceed it. The main loop also drives the
// e-paper display and the LoRa radio, and the rocket telemetry underneath
// arrives at a few Hz at best, so faster only resends stale values.
constexpr uint16_t kMaxStreamHz = 20U;

// Before the first successful timesync the offset is zero, so this emits raw
// device time — which is exactly what spec 3.4.3 asks for.
static void fillHeader(qlcp_header& header) {
  header.sequence = s_sequence++;
  header.timestamp_us = static_cast<uint64_t>(esp_timer_get_time() - s_tsOffsetUs);
}

static void netTransition(QlcpNetState next, uint32_t nowMs) {
  Serial.printf("QLCP net: %u -> %u\n", s_netState, next);
  s_netState = next;
  s_stateEnteredMs = nowMs;
}

static void netFail(uint32_t nowMs) {
  net_link_close_all(&s_netLink);
  s_streamFrequencyHz = 0U;
  s_configSent = false;
  s_configAcked = false;
  netTransition(QLCP_NET_BACKOFF, nowMs);
}

// One reading from every sensor, in the id order fixed by the CONFIG JSON.
// Used both for the periodic stream and for GET_SINGLE (spec 10.4).
static void sendTelemetry() {
  qlcp_sensor_data readings[kSensorCount] = {};
  const uint32_t nowMs = millis();

  // rocket_position
  readings[kSenPositionBase + 0U].id = kSenPositionBase + 0U;
  readings[kSenPositionBase + 0U].value = static_cast<float>(getRocketLatDeg());
  readings[kSenPositionBase + 1U].id = kSenPositionBase + 1U;
  readings[kSenPositionBase + 1U].value = static_cast<float>(getRocketLonDeg());
  readings[kSenPositionBase + 2U].id = kSenPositionBase + 2U;
  readings[kSenPositionBase + 2U].value = static_cast<float>(getRocketAltitudeMeters());
  readings[kSenPositionBase + 3U].id = kSenPositionBase + 3U;
  readings[kSenPositionBase + 3U].value = getRocketAccelG();
  readings[kSenPositionBase + 4U].id = kSenPositionBase + 4U;
  readings[kSenPositionBase + 4U].value = static_cast<float>(getRocketGPSSats());

  // rocket_nodes: 1.0 = alive, 0.0 = dead, -1.0 = no link established yet
  const uint8_t mask = getRocketLivenessMask();
  const uint32_t lastRfMs = getRfmLastRFReceived();
  const bool loraLinkAlive = (lastRfMs > 0U) && ((nowMs - lastRfMs) < lora::kNodeAliveTimeoutMs);
  for (uint8_t i = 0U; i < lora::kTrackedNodeCount; i++) {
    readings[kSenNodesBase + i].id = kSenNodesBase + i;
    readings[kSenNodesBase + i].value = (lastRfMs == 0U)
        ? -1.0f
        : ((loraLinkAlive && lora::LivenessTracker::isNodeAlive(mask, i)) ? 1.0f : 0.0f);
  }

  // rocket_link
  readings[kSenLinkBase + 0U].id = kSenLinkBase + 0U;
  readings[kSenLinkBase + 0U].value = static_cast<float>(getRfmLastRSSI());
  readings[kSenLinkBase + 1U].id = kSenLinkBase + 1U;
  readings[kSenLinkBase + 1U].value = getRfmLastSNR();
  readings[kSenLinkBase + 2U].id = kSenLinkBase + 2U;
  readings[kSenLinkBase + 2U].value = static_cast<float>(getRfmLastFreqErr());
  readings[kSenLinkBase + 3U].id = kSenLinkBase + 3U;
  readings[kSenLinkBase + 3U].value = static_cast<float>(getRfmPacketCount());
  readings[kSenLinkBase + 4U].id = kSenLinkBase + 4U;
  readings[kSenLinkBase + 4U].value = static_cast<float>(nowMs - lastRfMs);

  // rocket_radio_config
  readings[kSenRadioBase + 0U].id = kSenRadioBase + 0U;
  readings[kSenRadioBase + 0U].value = static_cast<float>(getRadioFreq());
  readings[kSenRadioBase + 1U].id = kSenRadioBase + 1U;
  readings[kSenRadioBase + 1U].value = static_cast<float>(getRadioBandwidth());
  readings[kSenRadioBase + 2U].id = kSenRadioBase + 2U;
  readings[kSenRadioBase + 2U].value = static_cast<float>(getRadioSF());
  readings[kSenRadioBase + 3U].id = kSenRadioBase + 3U;
  readings[kSenRadioBase + 3U].value = static_cast<float>(getRadioCR());

  // ground_station
  readings[kSenGsBase + 0U].id = kSenGsBase + 0U;
  readings[kSenGsBase + 0U].value = getBatteryVoltage() / 1000.0f;
  readings[kSenGsBase + 1U].id = kSenGsBase + 1U;
  readings[kSenGsBase + 1U].value = getSystemCurrent() / 1000.0f;
  readings[kSenGsBase + 2U].id = kSenGsBase + 2U;
  readings[kSenGsBase + 2U].value = getAmbTemperature() / 100.0f;

  // voltage_sense
  readings[kSenRocketBatt].id = kSenRocketBatt;
  readings[kSenRocketBatt].value = getRocketBatteryVolts();

  qlcp_data_packet pkt = {};
  fillHeader(pkt.header);
  pkt.sensor_data = readings;
  pkt.sensor_count = kSensorCount;

  (void)udp_send_data(&s_netLink, &pkt);
}

static void qlcpTelemetryService(uint32_t nowMs) {
  if ((s_netState != QLCP_NET_CONNECTED) || (s_streamFrequencyHz == 0U)) {
    return;
  }
  const uint32_t periodMs = 1000U / s_streamFrequencyHz; // non-zero: rate is clamped to kMaxStreamHz
  if ((nowMs - s_lastStreamTxMs) < periodMs) {
    return;
  }
  s_lastStreamTxMs = nowMs;

  sendTelemetry();
}

static void sendAck(uint8_t ackType, uint8_t ackSeq) {
  qlcp_server_payload out = {};
  out.packet_type = QLCP_PT_ACK;
  fillHeader(out.payload_data.ack.header);
  out.payload_data.ack.ack_packet_type = ackType;
  out.payload_data.ack.ack_sequence = ackSeq;
  if (tcp_tx_payload(&s_netLink, &out) != 0) {
    Serial.println("[WARN] ACK dropped - TX busy");
  }
}

static void sendNack(uint8_t nackType, uint8_t nackSeq, uint8_t errCode) {
  qlcp_server_payload out = {};
  out.packet_type = QLCP_PT_NACK;
  fillHeader(out.payload_data.nack.header);
  out.payload_data.nack.nack_packet_type = nackType;
  out.payload_data.nack.nack_sequence = nackSeq;
  out.payload_data.nack.nack_error_code = errCode;
  if (tcp_tx_payload(&s_netLink, &out) != 0) {
    Serial.println("[WARN] NACK dropped - TX busy");
  }
}

// qlcp_encode_status() rejects a NULL control_data even when control_count is
// zero, so a device with no controls has to hand it a dummy the encoder will
// never read. Passing nullptr instead makes every STATUS fail to encode and be
// dropped silently — it compiles and links fine, and only shows up as missing
// responses against a live server.
static const qlcp_control_data kNoControls[1] = {};

// GREG owns no controls, so every STATUS carries count = 0. It still has to be
// sent: spec 10.1 makes STATUS the required response to CONTROL and
// STATUS_REQUEST, and spec 10.3 to ESTOP.
static void sendStatus(uint8_t ackType, uint8_t ackSeq) {
  qlcp_server_payload out = {};
  out.packet_type = QLCP_PT_STATUS;
  fillHeader(out.payload_data.status.header);
  out.payload_data.status.ack_packet_type = ackType;
  out.payload_data.status.ack_sequence = ackSeq;
  out.payload_data.status.control_data = kNoControls;
  out.payload_data.status.control_count = 0U;
  if (tcp_tx_payload(&s_netLink, &out) != 0) {
    Serial.println("[WARN] STATUS dropped - TX busy");
  }
}

// Device-initiated (spec 7.7.1). The server echoes our send time back as
// t1_echo_us alongside its own receipt clock in TIMESYNC_RESP.
static void sendTimesyncReq() {
  qlcp_server_payload out = {};
  out.packet_type = QLCP_PT_TIMESYNC_REQ;
  out.payload_data.header_only.packet_type = QLCP_PT_TIMESYNC_REQ;
  fillHeader(out.payload_data.header_only.header);
  if (tcp_tx_payload(&s_netLink, &out) != 0) {
    Serial.println("[WARN] TIMESYNC_REQ dropped - TX busy");
  }
}

static void qlcpHandlePacket(const qlcp_client_payload& in) {
  switch (in.packet_type) {
    case QLCP_PT_HEARTBEAT: {
      sendAck(QLCP_PT_HEARTBEAT, in.payload_data.header_only.header.sequence);
      break;
    }
    case QLCP_PT_TIMESYNC_RESP: {
      // Four-timestamp exchange, spec 7.7.3. T4 is sampled first and the
      // differences are signed — the offset is negative whenever the device
      // clock trails the server's.
      const int64_t t4 = esp_timer_get_time();
      const int64_t t1 = static_cast<int64_t>(in.payload_data.timesync_resp.t1_echo_us);       // our send time, echoed
      const int64_t t2 = static_cast<int64_t>(in.payload_data.timesync_resp.t2_us);            // server receipt
      const int64_t t3 = static_cast<int64_t>(in.payload_data.timesync_resp.header.timestamp_us); // server send
      s_tsOffsetUs = ((t1 - t2) + (t4 - t3)) / 2;
      Serial.printf("QLCP timesync completed. Offset: %lld us\n", static_cast<long long>(s_tsOffsetUs));
      sendAck(QLCP_PT_TIMESYNC_RESP, in.payload_data.timesync_resp.header.sequence);
      break;
    }
    case QLCP_PT_STREAM_START: {
      const uint16_t seq = in.payload_data.stream_start.header.sequence;
      const uint16_t freq = in.payload_data.stream_start.stream_frequency;
      if (freq == 0U) {
        // Spec 7.3 defines the valid range as 1-65535; 0 is not "stop".
        Serial.println("[WARN] STREAM_START with frequency 0 - rejected");
        sendNack(QLCP_PT_STREAM_START, seq, QLCP_ERR_INVALID_PARAM);
        break;
      }
      s_streamFrequencyHz = (freq > kMaxStreamHz) ? kMaxStreamHz : freq;
      s_lastStreamTxMs = millis();
      if (s_streamFrequencyHz != freq) {
        Serial.printf("QLCP Stream Start at %u Hz (clamped from %u Hz)\n", s_streamFrequencyHz, freq);
      } else {
        Serial.printf("QLCP Stream Start at %u Hz\n", s_streamFrequencyHz);
      }
      sendAck(QLCP_PT_STREAM_START, seq);
      break;
    }
    case QLCP_PT_STREAM_STOP: {
      s_streamFrequencyHz = 0U;
      Serial.println("QLCP Stream Stop");
      sendAck(QLCP_PT_STREAM_STOP, in.payload_data.header_only.header.sequence);
      break;
    }
    case QLCP_PT_GET_SINGLE: {
      // Spec 10.4: one reading from every sensor, in a single DATA packet.
      // DATA is the required response here — no ACK (spec 10.1).
      sendTelemetry();
      break;
    }
    case QLCP_PT_ESTOP: {
      // GREG drives nothing, so "all controls to default" is vacuous — but the
      // STATUS response is still mandatory (spec 10.3).
      Serial.println("[WARN] ESTOP received on Ground Station - no controls to safe");
      sendStatus(QLCP_PT_ESTOP, in.payload_data.header_only.header.sequence);
      break;
    }
    case QLCP_PT_STATUS_REQUEST: {
      sendStatus(QLCP_PT_STATUS_REQUEST, in.payload_data.header_only.header.sequence);
      break;
    }
    case QLCP_PT_CONTROL: {
      // GREG declares no controls, so every control_id is out of range.
      Serial.printf("[WARN] CONTROL for id %u - ground station has no controls\n",
                    in.payload_data.control.control_data.id);
      sendNack(QLCP_PT_CONTROL, in.payload_data.control.header.sequence, QLCP_ERR_INVALID_ID);
      break;
    }
    case QLCP_PT_ACK: {
      // Spec 7.7.4.1: the first timesync cycle starts on the CONFIG ACK. Until
      // the server has our config there is nothing for it to time-align.
      const uint8_t ackType = in.payload_data.ack.ack_packet_type;
      const uint8_t ackSeq = in.payload_data.ack.ack_sequence;
      if (!s_configAcked && (ackType == QLCP_PT_CONFIG) && (ackSeq == s_configSeq)) {
        s_configAcked = true;
        Serial.println("QLCP CONFIG acknowledged - starting timesync");
        sendTimesyncReq();
        s_lastTimesyncMs = millis();
      }
      break;
    }
    case QLCP_PT_NACK: {
      Serial.printf("[WARN] Server NACK: type %u seq %u err %u\n",
                    in.payload_data.nack.nack_packet_type,
                    in.payload_data.nack.nack_sequence,
                    in.payload_data.nack.nack_error_code);
      break;
    }
    default: {
      // Reached only for types the decoder accepted; genuinely unknown types
      // arrive via the rx == 2 path below (spec 10.7).
      sendNack(in.packet_type, in.payload_data.header_only.header.sequence, QLCP_ERR_UNKNOWN_TYPE);
      break;
    }
  }
}

void qlcpUplinkInit() {
  net_link_init(&s_netLink);
}

void qlcpUplinkService() {
  uint32_t nowMs = millis();

  if (WiFi.status() != WL_CONNECTED) {
    if (s_netState != QLCP_NET_IDLE && s_netState != QLCP_NET_BACKOFF) {
      netFail(nowMs);
    }
    if (s_netState == QLCP_NET_BACKOFF) {
      if (nowMs - s_stateEnteredMs >= s_backoffMs) {
        s_stateEnteredMs = nowMs;
      }
    }
    return;
  }

  switch (s_netState) {
    case QLCP_NET_IDLE: {
      s_netLink.netif_handle = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
      if (discovery_listen_begin(&s_netLink) == ESP_OK) {
        netTransition(QLCP_NET_DISCOVER, nowMs);
      } else {
        netFail(nowMs);
      }
      break;
    }
    case QLCP_NET_DISCOVER: {
      const int found = discovery_listen_service(&s_netLink);
      if (found == 1) {
        discovery_listen_end(&s_netLink);
        if (tcp_connect_begin(&s_netLink) == ESP_OK) {
          netTransition(QLCP_NET_TCP_CONNECT, nowMs);
        } else {
          netFail(nowMs);
        }
      } else if (found < 0) {
        netFail(nowMs);
      }
      break;
    }
    case QLCP_NET_TCP_CONNECT: {
      const int conn = tcp_connect_service(&s_netLink);
      if (conn == 1) {
        if (udp_create_socket(&s_netLink) != ESP_OK) {
          netFail(nowMs);
          break;
        }
        s_configSent = false;
        s_configAcked = false;
        s_lastRxMs = nowMs;
        s_backoffMs = kNetBackoffMinMs;
        netTransition(QLCP_NET_CONNECTED, nowMs);
      } else if ((conn < 0) || ((nowMs - s_stateEnteredMs) >= kNetTcpConnectTimeoutMs)) {
        netFail(nowMs);
      }
      break;
    }
    case QLCP_NET_CONNECTED: {
      // Spec 11: CONFIG is always the first packet on a new connection.
      if (!s_configSent) {
        qlcp_server_payload out = {};
        out.packet_type = QLCP_PT_CONFIG;
        fillHeader(out.payload_data.config.header);
        out.payload_data.config.config_data = reinterpret_cast<const uint8_t*>(kBoardQlcpConfigJson);
        out.payload_data.config.config_data_len = static_cast<uint16_t>(sizeof(kBoardQlcpConfigJson) - 1U);
        if (tcp_tx_payload(&s_netLink, &out) == 0) {
          s_configSent = true;
          s_configSeq = out.payload_data.config.header.sequence;
          Serial.println("Sent CONFIG packet to server");
        }
      }
      // Periodic resync. Only runs once the first cycle has been kicked off by
      // the CONFIG ACK.
      if (s_configAcked && ((nowMs - s_lastTimesyncMs) >= kTimesyncIntervalMs)) {
        sendTimesyncReq();
        s_lastTimesyncMs = nowMs;
      }
      if (tcp_tx_service(&s_netLink) < 0) {
        netFail(nowMs);
        break;
      }
      qlcp_client_payload in = {};
      const int rx = tcp_rx_service(&s_netLink, &in);
      if (rx < 0) {
        netFail(nowMs);
        break;
      }
      if (rx == 1) {
        s_lastRxMs = nowMs;
        qlcpHandlePacket(in);
      } else if (rx == 2) {
        // Framing was valid but the type is unrecognized (spec 10.7).
        s_lastRxMs = nowMs;
        Serial.printf("[WARN] Unknown packet type %u - NACKing\n", in.packet_type);
        sendNack(in.packet_type, in.payload_data.header_only.header.sequence, QLCP_ERR_UNKNOWN_TYPE);
      }
      if ((nowMs - s_lastRxMs) >= kNetRxIdleTimeoutMs) {
        Serial.println("[WARN] QLCP server silent - reconnecting");
        netFail(nowMs);
        break;
      }

      qlcpTelemetryService(nowMs);
      break;
    }
    case QLCP_NET_BACKOFF: {
      if ((nowMs - s_stateEnteredMs) >= s_backoffMs) {
        s_backoffMs = (s_backoffMs >= (kNetBackoffMaxMs / 2U)) ? kNetBackoffMaxMs : (s_backoffMs * 2U);
        if (discovery_listen_begin(&s_netLink) == ESP_OK) {
          netTransition(QLCP_NET_DISCOVER, nowMs);
        } else {
          netFail(nowMs);
        }
      }
      break;
    }
    default:
      break;
  }
}
