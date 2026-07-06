#include "radio_control.h"
#include "pinouts.h"
#include "global.h"
#include "user_interface.h"
#include <SPI.h>
#include <RadioLib.h>
#include <aim_catalog.h>
#include <lora_link.h>

// Static state variables
static double s_freqOpts[] = {902.0, 905.4, 928.0};
static uint8_t s_freqSelected = 1;

static constexpr uint8_t s_numBandwidthOpts = 10;
static double s_bandwidthOpts[s_numBandwidthOpts] = {7.8, 10.4, 15.6, 20.8, 31.25, 41.7, 62.5, 125, 250, 500};
static uint8_t s_bandwidthSelected = 6;

static constexpr uint8_t s_numSpreadOpts = 6;
static int32_t s_spreadOpts[s_numSpreadOpts] = {7, 8, 9, 10, 11, 12};
static uint8_t s_spreadSelected = 3;

static constexpr uint8_t s_numCodingOpts = 4;
static int32_t s_codingOpts[s_numCodingOpts] = {5, 6, 7, 8};
static uint8_t s_codingSelected = 1;

static volatile uint32_t s_rfmLastRFReceived = 0;
static volatile int16_t s_rfmLastRSSI = 0;
static volatile float s_rfmLastSNR = 0;
static volatile int32_t s_rfmLastFreqErr = 0;
static volatile bool s_rfmLastPacketValid = false;

static volatile int32_t s_rocketGPSLat = 0;
static volatile int32_t s_rocketGPSLon = 0;
static volatile uint8_t s_rocketGPSSats = 0;
static volatile int32_t s_rocketAltitude = 0;
static volatile uint8_t s_rocketStatus = 0;
static volatile double s_rocketVelocity = 0;

static int32_t s_curFreqOffset = 0;
static volatile bool s_rfmReceivedFlag = false;

// Per-node liveness tracking — node list sourced from shared lora_link.
static NodeStatus s_nodeStatus[lora::kTrackedNodeCount] = {
  {lora::kTrackedNodes[0].name, 0, false},
  {lora::kTrackedNodes[1].name, 0, false},
  {lora::kTrackedNodes[2].name, 0, false},
  {lora::kTrackedNodes[3].name, 0, false},
  {lora::kTrackedNodes[4].name, 0, false},
};

static uint32_t s_packetCount = 0;

// Radio editing state
static bool    s_editActive = false;
static uint8_t s_editParam = 0;       // 0=freq, 1=BW, 2=SF, 3=CR
static uint8_t s_editFreqIdx;
static uint8_t s_editBwIdx;
static uint8_t s_editSfIdx;
static uint8_t s_editCrIdx;

// Mutex for cross-core synchronization
static portMUX_TYPE s_rocketMux = portMUX_INITIALIZER_UNLOCKED;

// SPI + Radio Objects
static SPIClass s_rfmSPI(HSPI);
static SPISettings s_rfmSPISettings(kRfmSpiClock, MSBFIRST, SPI_MODE0);
static Module s_radioModule(pins::kRfCs, pins::kRfDio0, pins::kRfReset, pins::kRfDio1, s_rfmSPI, s_rfmSPISettings);
static RFM95 s_radio(&s_radioModule);

static void byteArrayToHexStr(const byte* byteArray, int length, char* outBuf, size_t maxLen) {
  if (static_cast<size_t>(length * 2 + 1) > maxLen) {
    if (maxLen > 0) outBuf[0] = '\0';
    return;
  }
  static const char hex[] = "0123456789ABCDEF";
  for (int i = 0; i < length; i++) {
    uint8_t b = byteArray[i];
    outBuf[i * 2] = hex[(b >> 4) & 0x0F];
    outBuf[i * 2 + 1] = hex[b & 0x0F];
  }
  outBuf[length * 2] = '\0';
}

#if defined(ESP8266) || defined(ESP32)
  ICACHE_RAM_ATTR
#endif
void setFlag(void) {
  s_rfmReceivedFlag = true;
}

bool isRFMConnected() {
  portENTER_CRITICAL(&s_rocketMux);
  uint32_t lastRF = s_rfmLastRFReceived;
  portEXIT_CRITICAL(&s_rocketMux);
  return (millis() - lastRF < kRfmConnectedTimeout);
}

void rfmInit() {
  // Initialize SPI Bus
  s_rfmSPI.begin(pins::kRfSck, pins::kRfMiso, pins::kRfMosi, pins::kRfCs);
  pinMode(s_rfmSPI.pinSS(), OUTPUT);

  setRGB(2, 255, 0, 0, true); // set RF light to RED
  Serial.print(F("RFM95: Initializing ... "));
  s_radio.reset();
  delay(100);
  int state = s_radio.begin();
  if (state == RADIOLIB_ERR_NONE) {
    Serial.println(F("success!"));
  } else {
    Serial.print(F("failed, code "));
    Serial.println(state);
    setRGB(2, 255, 0, 0, true); // set RF light to RED
    return; // degraded mode: return instead of hanging
  }

  s_radio.setDio0Action(setFlag, RISING);

  double baseFreq = s_freqOpts[s_freqSelected];
  double freqOffset = (s_curFreqOffset / 1000000.0); // Hz -> MHz
  
  s_radio.setFrequency(baseFreq + freqOffset);
  s_radio.setBandwidth(s_bandwidthOpts[s_bandwidthSelected]);
  s_radio.setSpreadingFactor(s_spreadOpts[s_spreadSelected]);
  s_radio.setCodingRate(s_codingOpts[s_codingSelected]);
  s_radio.setSyncWord(0x12);
  s_radio.setPreambleLength(8);

  state = s_radio.startReceive();
  if (state == RADIOLIB_ERR_NONE) {
    Serial.println(F("Radio Listening Active"));
  } else {
    Serial.print(F("failed starting listening, code "));
    Serial.println(state);
  }
}

void onRFMReceive() {
  uint8_t rfmPayload[lora::kPacketSize];

  int state = s_radio.readData(rfmPayload, lora::kPacketSize);

  // Restart receiver immediately
  s_radio.startReceive();

  if (state != RADIOLIB_ERR_NONE) {
    if (state != RADIOLIB_ERR_CRC_MISMATCH) {
      Serial.print("Receive failed, code ");
      Serial.println(state);
    }
    return;
  }

  lora::Packet pkt = lora::decode(rfmPayload);

  // Update per-node liveness
  uint32_t nowMs = millis();
  for (uint8_t i = 0; i < lora::kTrackedNodeCount; i++) {
    if (lora::kTrackedNodes[i].source == pkt.source) {
      s_nodeStatus[i].lastHeardMs = nowMs;
      s_nodeStatus[i].everHeard = true;
      break;
    }
  }

  s_packetCount++;

  // Derive rocket status (seenGPS=bit0, seenAltimeter=bit1, seenSensors=bit2)
  uint8_t status = 0;
  if (nowMs - s_nodeStatus[3].lastHeardMs < 5000) status |= 1; // GPS
  if (nowMs - s_nodeStatus[2].lastHeardMs < 5000) status |= 2; // ALT
  if ((nowMs - s_nodeStatus[0].lastHeardMs < 5000) ||
      (nowMs - s_nodeStatus[1].lastHeardMs < 5000) ||
      (nowMs - s_nodeStatus[4].lastHeardMs < 5000)) status |= 4; // UCM/LCM/PWR

  // Protect writes to shared variables
  portENTER_CRITICAL(&s_rocketMux);
  s_rfmLastPacketValid = true;
  s_rocketStatus = status;

  if (pkt.cls == aim::Class::Sensor) {
    if (pkt.subject == aim::subject::GpsLat) {
      s_rocketGPSLat = pkt.value;
    } else if (pkt.subject == aim::subject::GpsLon) {
      s_rocketGPSLon = pkt.value;
    } else if (pkt.subject == aim::subject::GpsNumSats) {
      s_rocketGPSSats = pkt.value;
    } else if (pkt.subject == aim::subject::Altitude) {
      s_rocketAltitude = pkt.value;
    }
  }

  s_rfmLastRFReceived = nowMs;
  s_rfmLastRSSI = s_radio.getRSSI();
  s_rfmLastSNR = s_radio.getSNR();
  s_rfmLastFreqErr = s_radio.getFrequencyError();
  portEXIT_CRITICAL(&s_rocketMux);

  triggerRFFlash();

  char hexBuf[lora::kPacketSize * 2 + 1];
  byteArrayToHexStr(rfmPayload, lora::kPacketSize, hexBuf, sizeof(hexBuf));

  char logBuf[100];
  snprintf(logBuf, sizeof(logBuf), "RocketPacket:%u,%d,%.1f,%d,%s",
           nowMs,
           s_rfmLastRSSI,
           s_rfmLastSNR,
           s_rfmLastFreqErr,
           hexBuf);
  writeToSDLog(logBuf);

  char logBuf2[100];
  snprintf(logBuf2, sizeof(logBuf2), "RocketData:%u,%.6f,%.6f,%.2f,%.2f,%u",
           getRocketGPSSats(),
           getRocketLatDeg(),
           getRocketLonDeg(),
           getRocketAltitudeMeters(),
           getRocketVelocity(),
           getRocketStatus());
  writeToSDLog(logBuf2);
}

bool getRfmReceivedFlag() {
  return s_rfmReceivedFlag;
}

void clearRfmReceivedFlag() {
  s_rfmReceivedFlag = false;
}

double getRadioFreq() {
  return s_freqOpts[s_freqSelected];
}
// NOTE: Kept for future manual tuning / local menu support
double getRadioBandwidth() {
  return s_bandwidthOpts[s_bandwidthSelected];
}

// NOTE: Kept for future manual tuning / local menu support
int32_t getRadioSF() {
  return s_spreadOpts[s_spreadSelected];
}

// NOTE: Kept for future manual tuning / local menu support
int32_t getRadioCR() {
  return s_codingOpts[s_codingSelected];
}
uint32_t getRfmLastRFReceived() {
  portENTER_CRITICAL(&s_rocketMux);
  uint32_t val = s_rfmLastRFReceived;
  portEXIT_CRITICAL(&s_rocketMux);
  return val;
}

int16_t getRfmLastRSSI() {
  portENTER_CRITICAL(&s_rocketMux);
  int16_t val = s_rfmLastRSSI;
  portEXIT_CRITICAL(&s_rocketMux);
  return val;
}

float getRfmLastSNR() {
  portENTER_CRITICAL(&s_rocketMux);
  float val = s_rfmLastSNR;
  portEXIT_CRITICAL(&s_rocketMux);
  return val;
}



bool isRfmLastPacketValid() {
  portENTER_CRITICAL(&s_rocketMux);
  bool val = s_rfmLastPacketValid;
  portEXIT_CRITICAL(&s_rocketMux);
  return val;
}

int32_t getRocketGPSLat() {
  portENTER_CRITICAL(&s_rocketMux);
  int32_t val = s_rocketGPSLat;
  portEXIT_CRITICAL(&s_rocketMux);
  return val;
}

int32_t getRocketGPSLon() {
  portENTER_CRITICAL(&s_rocketMux);
  int32_t val = s_rocketGPSLon;
  portEXIT_CRITICAL(&s_rocketMux);
  return val;
}

uint8_t getRocketGPSSats() {
  portENTER_CRITICAL(&s_rocketMux);
  uint8_t val = s_rocketGPSSats;
  portEXIT_CRITICAL(&s_rocketMux);
  return val;
}

uint8_t getRocketStatus() {
  portENTER_CRITICAL(&s_rocketMux);
  uint8_t val = s_rocketStatus;
  portEXIT_CRITICAL(&s_rocketMux);
  return val;
}

double getRocketVelocity() {
  portENTER_CRITICAL(&s_rocketMux);
  double val = s_rocketVelocity;
  portEXIT_CRITICAL(&s_rocketMux);
  return val;
}

// Catalog wire scaling (aim_catalog.h): GPS degrees x10^7, altitude meters x100.
double getRocketLatDeg() {
  portENTER_CRITICAL(&s_rocketMux);
  int32_t val = s_rocketGPSLat;
  portEXIT_CRITICAL(&s_rocketMux);
  return val / 1.0e7;
}

double getRocketLonDeg() {
  portENTER_CRITICAL(&s_rocketMux);
  int32_t val = s_rocketGPSLon;
  portEXIT_CRITICAL(&s_rocketMux);
  return val / 1.0e7;
}

double getRocketAltitudeMeters() {
  portENTER_CRITICAL(&s_rocketMux);
  int32_t val = s_rocketAltitude;
  portEXIT_CRITICAL(&s_rocketMux);
  return val / 100.0;
}

void setRocketVelocity(double vel) {
  portENTER_CRITICAL(&s_rocketMux);
  s_rocketVelocity = vel;
  portEXIT_CRITICAL(&s_rocketMux);
}
// NOTE: Kept for future manual tuning / local menu support
int32_t getCurFreqOffset() {
  return s_curFreqOffset;
}
void changeFreqOffset(int32_t amount) {
  s_curFreqOffset += amount;
}

void setRadioConfig(const String& name, uint16_t value) {
  if (name == "bandwidth") {
    if (value < s_numBandwidthOpts) {
      s_bandwidthSelected = value;
    }
  } else if (name == "spreadingfactor") {
    if (value < s_numSpreadOpts) {
      s_spreadSelected = value;
    }
  } else if (name == "codingrate") {
    if (value < s_numCodingOpts) {
      s_codingSelected = value;
    }
  }
}

int32_t getRfmLastFreqErr() {
  portENTER_CRITICAL(&s_rocketMux);
  int32_t val = s_rfmLastFreqErr;
  portEXIT_CRITICAL(&s_rocketMux);
  return val;
}

uint32_t getRfmPacketCount() {
  return s_packetCount;
}

const NodeStatus* getNodeStatusTable() {
  return s_nodeStatus;
}

// --- Radio parameter editing (driven by menu buttons) ---

void radioEditBegin() {
  s_editActive = true;
  s_editParam = 0;
  s_editFreqIdx = s_freqSelected;
  s_editBwIdx = s_bandwidthSelected;
  s_editSfIdx = s_spreadSelected;
  s_editCrIdx = s_codingSelected;
}

void radioEditMove(int8_t dir) {
  switch (s_editParam) {
    case 0: // frequency
      s_editFreqIdx = (s_editFreqIdx + 3 + dir) % 3;
      break;
    case 1: // bandwidth
      s_editBwIdx = (s_editBwIdx + s_numBandwidthOpts + dir) % s_numBandwidthOpts;
      break;
    case 2: // spreading factor
      s_editSfIdx = (s_editSfIdx + s_numSpreadOpts + dir) % s_numSpreadOpts;
      break;
    case 3: // coding rate
      s_editCrIdx = (s_editCrIdx + s_numCodingOpts + dir) % s_numCodingOpts;
      break;
  }
}

void radioEditConfirm() {
  if (s_editParam < 3) {
    s_editParam++;
  } else {
    // Apply all changes
    s_freqSelected = s_editFreqIdx;
    s_bandwidthSelected = s_editBwIdx;
    s_spreadSelected = s_editSfIdx;
    s_codingSelected = s_editCrIdx;
    s_editActive = false;
    reapplyRadioConfig();
  }
}

void radioEditCancel() {
  s_editActive = false;
}

uint8_t radioEditParam() {
  return s_editParam;
}

bool radioEditActive() {
  return s_editActive;
}

double radioEditFreqPreview() {
  return s_freqOpts[s_editFreqIdx];
}

double radioEditBwPreview() {
  return s_bandwidthOpts[s_editBwIdx];
}

int32_t radioEditSfPreview() {
  return s_spreadOpts[s_editSfIdx];
}

int32_t radioEditCrPreview() {
  return s_codingOpts[s_editCrIdx];
}

void reapplyRadioConfig() {
  double baseFreq = s_freqOpts[s_freqSelected];
  double freqOffset = s_curFreqOffset / 1000000.0;

  s_radio.standby();
  s_radio.setFrequency(baseFreq + freqOffset);
  s_radio.setBandwidth(s_bandwidthOpts[s_bandwidthSelected]);
  s_radio.setSpreadingFactor(s_spreadOpts[s_spreadSelected]);
  s_radio.setCodingRate(s_codingOpts[s_codingSelected]);
  s_radio.startReceive();
}
