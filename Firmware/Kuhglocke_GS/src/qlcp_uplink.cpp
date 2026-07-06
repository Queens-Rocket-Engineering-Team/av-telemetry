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

constexpr char kBoardQlcpConfigJson[] = R"json({
  "device_name": "GREG",
  "device_type": "Sensor Monitor",
  "sensor_info": {
    "rocket_position": {
      "Lat":  { "unit": "deg" },
      "Lon":  { "unit": "deg" },
      "Alt":  { "unit": "m" },
      "Vel":  { "unit": "m/s" },
      "Sats": { "unit": "" }
    },
    "rocket_nodes": {
      "UCM": { "unit": "ms" },
      "LCM": { "unit": "ms" },
      "ALT": { "unit": "ms" },
      "GPS": { "unit": "ms" },
      "PWR": { "unit": "ms" }
    },
    "rocket_link": {
      "RSSI":    { "unit": "dBm" },
      "SNR":     { "unit": "dB" },
      "FreqErr": { "unit": "Hz" },
      "Packets": { "unit": "" },
      "LinkAge": { "unit": "ms" }
    },
    "rocket_radio_config": {
      "Freq": { "unit": "MHz" },
      "BW":   { "unit": "kHz" },
      "SF":   { "unit": "" },
      "CR":   { "unit": "" }
    },
    "ground_station": {
      "BattV":      { "unit": "V" },
      "SysCurrent": { "unit": "A" },
      "AmbTemp":    { "unit": "C" }
    },
    "voltage_sense": {
      "RocketBatt": {
        "unit": "V"
      }
    }
  }
})json";

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
static bool s_configSent = false;
static net_link_t s_netLink = {};

static uint16_t s_sequence = 0U;
static uint32_t s_tsOffset = 0U;
static uint16_t s_streamFrequencyHz = 0U;
static uint32_t s_lastStreamTxMs = 0U;

constexpr uint32_t kNetTcpConnectTimeoutMs = 5000U;
constexpr uint32_t kNetRxIdleTimeoutMs    = 15000U;
constexpr uint32_t kNetBackoffMinMs       = 1000U;
constexpr uint32_t kNetBackoffMaxMs       = 8000U;

static void fillHeader(qlcp_header& header) {
  header.sequence = static_cast<uint8_t>(s_sequence++);
  header.timestamp = s_tsOffset + millis();
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
  netTransition(QLCP_NET_BACKOFF, nowMs);
}

static void sendTelemetry() {
  static constexpr uint8_t kSensorCount = 22U;
  qlcp_sensor_data readings[kSensorCount] = {};
  uint32_t nowMs = millis();

  // Position (sensor_id 0-4)
  readings[0].sensor_id = 0;
  readings[0].unit = QLCP_UNIT_UNITLESS;
  readings[0].value = static_cast<float>(getRocketLatDeg());

  readings[1].sensor_id = 1;
  readings[1].unit = QLCP_UNIT_UNITLESS;
  readings[1].value = static_cast<float>(getRocketLonDeg());

  readings[2].sensor_id = 2;
  readings[2].unit = QLCP_UNIT_UNITLESS;
  readings[2].value = static_cast<float>(getRocketAltitudeMeters());

  readings[3].sensor_id = 3;
  readings[3].unit = QLCP_UNIT_UNITLESS;
  readings[3].value = static_cast<float>(getRocketVelocity());

  readings[4].sensor_id = 4;
  readings[4].unit = QLCP_UNIT_UNITLESS;
  readings[4].value = static_cast<float>(getRocketGPSSats());

  // Node liveness (sensor_id 5-9): age in ms, -1 = never heard
  const NodeStatus* nodes = getNodeStatusTable();
  for (uint8_t i = 0; i < lora::kTrackedNodeCount; i++) {
    readings[5 + i].sensor_id = 5 + i;
    readings[5 + i].unit = QLCP_UNIT_MILLISECONDS;
    readings[5 + i].value = nodes[i].everHeard
        ? static_cast<float>(nowMs - nodes[i].lastHeardMs)
        : -1.0f;
  }

  // Link quality (sensor_id 10-14)
  readings[10].sensor_id = 10;
  readings[10].unit = QLCP_UNIT_UNITLESS;
  readings[10].value = static_cast<float>(getRfmLastRSSI());

  readings[11].sensor_id = 11;
  readings[11].unit = QLCP_UNIT_UNITLESS;
  readings[11].value = static_cast<float>(getRfmLastSNR());

  readings[12].sensor_id = 12;
  readings[12].unit = QLCP_UNIT_HERTZ;
  readings[12].value = static_cast<float>(getRfmLastFreqErr());

  readings[13].sensor_id = 13;
  readings[13].unit = QLCP_UNIT_UNITLESS;
  readings[13].value = static_cast<float>(getRfmPacketCount());

  readings[14].sensor_id = 14;
  readings[14].unit = QLCP_UNIT_MILLISECONDS;
  readings[14].value = static_cast<float>(nowMs - getRfmLastRFReceived());

  // Radio config (sensor_id 15-18)
  readings[15].sensor_id = 15;
  readings[15].unit = QLCP_UNIT_UNITLESS;
  readings[15].value = static_cast<float>(getRadioFreq());

  readings[16].sensor_id = 16;
  readings[16].unit = QLCP_UNIT_UNITLESS;
  readings[16].value = static_cast<float>(getRadioBandwidth());

  readings[17].sensor_id = 17;
  readings[17].unit = QLCP_UNIT_UNITLESS;
  readings[17].value = static_cast<float>(getRadioSF());

  readings[18].sensor_id = 18;
  readings[18].unit = QLCP_UNIT_UNITLESS;
  readings[18].value = static_cast<float>(getRadioCR());

  // GS health (sensor_id 19-21)
  readings[19].sensor_id = 19;
  readings[19].unit = QLCP_UNIT_VOLTS;
  readings[19].value = getBatteryVoltage() / 1000.0f;

  readings[20].sensor_id = 20;
  readings[20].unit = QLCP_UNIT_AMPS;
  readings[20].value = getSystemCurrent() / 1000.0f;

  readings[21].sensor_id = 21;
  readings[21].unit = QLCP_UNIT_CELSIUS;
  readings[21].value = getAmbTemperature() / 100.0f;

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
  const uint32_t periodMs = 1000U / s_streamFrequencyHz;
  if ((nowMs - s_lastStreamTxMs) < periodMs) {
    return;
  }
  s_lastStreamTxMs = nowMs;

  sendTelemetry();
}

static void sendAck(uint8_t ackType, uint16_t ackSeq) {
  qlcp_server_payload out = {};
  out.packet_type = QLCP_PT_ACK;
  fillHeader(out.payload_data.ack.header);
  out.payload_data.ack.ack_packet_type = ackType;
  out.payload_data.ack.ack_sequence = ackSeq;
  if (tcp_tx_payload(&s_netLink, &out) != 0) {
    Serial.println("[WARN] ACK dropped - TX busy");
  }
}

static void sendNack(uint8_t nackType, uint16_t nackSeq, uint8_t errCode) {
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

static void qlcpHandlePacket(const qlcp_client_payload& in) {
  switch (in.packet_type) {
    case QLCP_PT_TIMESYNC: {
      const uint32_t serverTime = in.payload_data.header_only.timestamp;
      s_tsOffset = serverTime - millis();
      Serial.printf("QLCP timesync completed. Offset: %u ms\n", s_tsOffset);
      sendAck(QLCP_PT_TIMESYNC, in.payload_data.header_only.sequence);
      break;
    }
    case QLCP_PT_HEARTBEAT: {
      sendAck(QLCP_PT_HEARTBEAT, in.payload_data.header_only.sequence);
      break;
    }
    case QLCP_PT_STREAM_START: {
      const uint16_t freq = in.payload_data.stream_start.stream_frequency;
      if (freq > 0U) {
        s_streamFrequencyHz = freq;
        s_lastStreamTxMs = millis();
        Serial.printf("QLCP Stream Start at %u Hz\n", freq);
      }
      sendAck(QLCP_PT_STREAM_START, in.payload_data.header_only.sequence);
      break;
    }
    case QLCP_PT_STREAM_STOP: {
      s_streamFrequencyHz = 0U;
      Serial.println("QLCP Stream Stop");
      sendAck(QLCP_PT_STREAM_STOP, in.payload_data.header_only.sequence);
      break;
    }
    case QLCP_PT_GET_SINGLE: {
      sendTelemetry();
      break;
    }
    case QLCP_PT_ESTOP: {
      Serial.println("[WARN] ESTOP received on Ground Station!");
      sendAck(QLCP_PT_ESTOP, in.payload_data.header_only.sequence);
      break;
    }
    case QLCP_PT_STATUS_REQUEST: {
      qlcp_server_payload out = {};
      out.packet_type = QLCP_PT_STATUS;
      fillHeader(out.payload_data.status.header);
      out.payload_data.status.control_data = nullptr;
      out.payload_data.status.control_count = 0;
      out.payload_data.status.device_status = QLCP_DS_ACTIVE;
      if (tcp_tx_payload(&s_netLink, &out) != 0) {
        Serial.println("[WARN] STATUS dropped - TX busy");
      }
      break;
    }
    case QLCP_PT_CONTROL: {
      sendNack(QLCP_PT_CONTROL, in.payload_data.header_only.sequence, QLCP_ERR_UNKNOWN_TYPE);
      break;
    }
    default: {
      sendNack(in.packet_type, in.payload_data.header_only.sequence, QLCP_ERR_UNKNOWN_TYPE);
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
      if (ssdp_listen_begin(&s_netLink) == ESP_OK) {
        netTransition(QLCP_NET_DISCOVER, nowMs);
      } else {
        netFail(nowMs);
      }
      break;
    }
    case QLCP_NET_DISCOVER: {
      const int found = ssdp_listen_service(&s_netLink);
      if (found == 1) {
        ssdp_listen_end(&s_netLink);
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
        s_lastRxMs = nowMs;
        s_backoffMs = kNetBackoffMinMs;
        netTransition(QLCP_NET_CONNECTED, nowMs);
      } else if ((conn < 0) || ((nowMs - s_stateEnteredMs) >= kNetTcpConnectTimeoutMs)) {
        netFail(nowMs);
      }
      break;
    }
    case QLCP_NET_CONNECTED: {
      if (!s_configSent) {
        qlcp_server_payload out = {};
        out.packet_type = QLCP_PT_CONFIG;
        fillHeader(out.payload_data.config.header);
        out.payload_data.config.config_data = kBoardQlcpConfigJson;
        out.payload_data.config.config_data_len = sizeof(kBoardQlcpConfigJson) - 1U;
        if (tcp_tx_payload(&s_netLink, &out) == 0) {
          s_configSent = true;
          Serial.println("Sent CONFIG packet to server");
        }
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
        if (ssdp_listen_begin(&s_netLink) == ESP_OK) {
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
