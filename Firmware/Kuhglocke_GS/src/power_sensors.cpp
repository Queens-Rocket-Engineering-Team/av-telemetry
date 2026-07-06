#include "power_sensors.h"
#include "pinouts.h"
#include "user_interface.h"
#include <Wire.h>
#include <NAU7802_2CH.h>
#include <MedianFilterLib.h>

// Global constants moved here or kept local
static constexpr double kBattVdiv = 3.003579098067;
static constexpr uint16_t kSensorSamplePeriod = 250;

// Local static state
static NAU7802 s_nau;
static MedianFilter<int32_t> s_battVoltFilter(8);
static MedianFilter<int32_t> s_sysCurrentFilter(8);
static MedianFilter<uint32_t> s_ambTempFilter(8);

static uint32_t s_lastSensorRead = 0;
static bool s_nauChannel = 0;

// Private helpers
static int16_t rawReadTempSensor() {
  Wire.requestFrom((int)pins::kP3t1755Addr, 2);
  if (Wire.available() != 2) {
    return -999;
  }
  uint8_t b1 = Wire.read();
  uint8_t b2 = Wire.read();
  int16_t rawData = (b1 << 8) | b2;
  rawData = rawData >> 4; // Shift first to right-align the 12-bit value
  if (rawData & 0x800) {
    rawData -= 0x1000; // Sign extend 12-bit to 16-bit signed
  }
  return (rawData * 6.25);
}

void initPowerSensors() {
  if (!s_nau.begin(Wire, true)) {
    Serial.println("[ERROR] NAU7802 initialization error");
    setRGB(0, 255, 0, 0); // Solid RED on power LED to indicate error
    return;
  }
  
  s_nau.setChannel(NAU7802_CHANNEL_1);
  s_nau.setPGACapEnable(false);
  s_nau.setBypassPGA(true);
  s_nau.setLDO(NAU7802_LDO_3V3); // ref=3.3V
  s_nau.setSampleRate(NAU7802_SPS_40);
  s_nau.calibrateAFE();

  Serial.println("NAU7802 setup & calibration complete");
}

void handleReadPowerSensors() {
  if (millis() - s_lastSensorRead > kSensorSamplePeriod) {
    s_lastSensorRead = millis();

    // === NAU7802 Reading ===
    if (s_nau.available()) {
      int32_t nauRaw = s_nau.getReading();
      // Range check for 24-bit signed integer (including negatives)
      if (nauRaw >= -8388608 && nauRaw <= 8388607) {
        if (s_nauChannel == 0) {
          s_nauChannel = 1;
          s_nau.setChannel(NAU7802_CHANNEL_2);
          s_battVoltFilter.AddValue(nauRaw);
        } else {
          s_nauChannel = 0;
          s_nau.setChannel(NAU7802_CHANNEL_1);
          s_sysCurrentFilter.AddValue(nauRaw);
        }
      }
    }

    // === Temperature Sensor Reading ===
    int16_t tempReading = rawReadTempSensor();
    if (tempReading != -999 && tempReading > -500 && tempReading < 1500) {
      s_ambTempFilter.AddValue(tempReading);
    }
  }
}

uint16_t getPSUVoltage() {
  return 3300; // Hardcoded PSU Voltage
}

uint16_t getBatteryVoltage() {
  uint16_t vRef = getPSUVoltage() / 2; // full-scale range is 0.5*VREF
  int32_t battCount = s_battVoltFilter.GetFiltered();
  return uint16_t(kBattVdiv * vRef * (battCount / 8388607.0));
}

uint16_t getSystemCurrent() {
  uint16_t vRef = getPSUVoltage() / 2; // full-scale range is 0.5*VREF
  int32_t currCount = s_sysCurrentFilter.GetFiltered();
  uint16_t currVolt = (vRef * (currCount / 8388607.0));
  return (currVolt - 250) * 1.25; // ACS70331 simplified formula
}

int16_t getAmbTemperature() {
  return s_ambTempFilter.GetFiltered();
}

uint8_t voltToPercent(uint16_t mv) {
  if (mv >= 4160) { return 100; }
  else if (mv >= 4110) { return 96; }
  else if (mv >= 4080) { return 92; }
  else if (mv >= 4050) { return 88; }
  else if (mv >= 4010) { return 84; }
  else if (mv >= 3970) { return 80; }
  else if (mv >= 3920) { return 76; }
  else if (mv >= 3880) { return 72; }
  else if (mv >= 3840) { return 68; }
  else if (mv >= 3800) { return 64; }
  else if (mv >= 3750) { return 60; }
  else if (mv >= 3720) { return 56; }
  else if (mv >= 3690) { return 52; }
  else if (mv >= 3660) { return 48; }
  else if (mv >= 3630) { return 44; }
  else if (mv >= 3620) { return 40; }
  else if (mv >= 3590) { return 36; }
  else if (mv >= 3570) { return 32; }
  else if (mv >= 3540) { return 28; }
  else if (mv >= 3500) { return 24; }
  else if (mv >= 3460) { return 20; }
  else if (mv >= 3430) { return 16; }
  else if (mv >= 3330) { return 12; }
  else if (mv >= 3220) { return 8; }
  else if (mv >= 3100) { return 4; }
  else { return 0; }
}

uint8_t getChargingStatus() {
  uint16_t chrgStat = analogRead(pins::kChrgStat);
  if (chrgStat < 200) {
    return 0; // Charging
  } else if (chrgStat > 3800) {
    return 2; // Disabled (on battery)
  }
  return 1; // Fully Charged
}
