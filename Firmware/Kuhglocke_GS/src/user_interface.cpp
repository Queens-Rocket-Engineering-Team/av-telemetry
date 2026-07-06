#include "user_interface.h"
#include "pinouts.h"
#include "global.h"
#include "power_sensors.h"
#include "radio_control.h"
#include <Adafruit_NeoPixel.h>
#include <FS.h>
#include <SD_MMC.h>


// Static state variables
static Adafruit_NeoPixel s_rgbLEDs(kNumRgbLeds, pins::kRgbData, NEO_GRB + NEO_KHZ800);
static bool s_usbDataOnly = false;
static File s_logFile;
static uint16_t s_logFileNumber = 0;
static bool s_indBattFlashState = true;
static uint32_t s_indBattLastToggle = 0;
static uint32_t s_indRFFlashStart = 0;

void initLEDs() {
  s_rgbLEDs.begin();
  s_rgbLEDs.setBrightness(kDefaultLedBrightness);
  s_rgbLEDs.show();
}

void setUSBDataOnlyMode(bool state) {
  s_usbDataOnly = state;
  digitalWrite(pins::kDisable5v, state);
}

void setLEDBrightness(uint8_t brightness) {
  s_rgbLEDs.setBrightness(brightness);
  s_rgbLEDs.show();
}

void setRGB(byte index, byte r, byte g, byte b) {
  setRGB(index, r, g, b, true);
}

void setRGB(byte index, byte r, byte g, byte b, bool push) {
  s_rgbLEDs.setPixelColor(index, s_rgbLEDs.Color(r, g, b));
  if (push) {
    s_rgbLEDs.show();
  }
}

void setRGB(byte r, byte g, byte b) {
  for (byte i = 0; i < kNumRgbLeds; i++) {
    setRGB(i, r, g, b);
  }
}

bool makeNextSDLog() {
  if (SD_MMC.cardType() == CARD_NONE) {
    return false;
  }

  SD_MMC.mkdir("/logs");

  uint16_t id = 0;
  while (id < 10000) {
    if (SD_MMC.exists("/logs/Kuhglocke_Log" + String(id) + ".txt")) {
      id++;
    } else {
      break;
    }
  }
  s_logFileNumber = id;
  Serial.println("Selected log file number");

  s_logFile = SD_MMC.open("/logs/Kuhglocke_Log" + String(id) + ".txt", FILE_WRITE);
  if (!s_logFile) {
    Serial.println("[WARN] Unable to open log file for writing");
    return false;
  }

  return true;
}

bool writeToSDLog(const String& txt) {
  if (!s_logFile) return false;

  s_logFile.print('[');
  s_logFile.print(millis());
  s_logFile.print("] ");
  bool r = s_logFile.println(txt);

  static uint32_t lastFlush = 0;
  if (millis() - lastFlush > 2000) {  // flush every 2s
    s_logFile.flush();
    lastFlush = millis();
  }
  return r;
}

void triggerRFFlash() {
  s_indRFFlashStart = millis();
}

void handleLEDs() {
  uint8_t chrgStatus = getChargingStatus();
  uint16_t battVolt = getBatteryVoltage();
  
  if (battVolt < kBattVoltLow) {
    if (chrgStatus == 1) {
      setRGB(1, 0, 255, 0, false);
    } else {
      setRGB(1, 255, 0, 0, false);
    }
  } else if (battVolt < kBattVoltMedium) {
    setRGB(1, 255, 80, 0, false);
  } else if (battVolt < kBattVoltHigh) {
    setRGB(1, 255, 220, 0, false);
  } else {
    setRGB(1, 0, 255, 0, false);
  }

  if (chrgStatus == 0) {
    if (s_indBattFlashState) {
      setRGB(1, 0, 0, 0, true);
    }
    if (millis() - s_indBattLastToggle > kIndBatteryFlashGap) {
      s_indBattLastToggle = millis();
      s_indBattFlashState = !s_indBattFlashState;
    }
  }

  if (isRFMConnected()) {
    if (millis() - s_indRFFlashStart < kIndRfFlashTime) {
      setRGB(2, 255, 255, 255); // Ping flash
    } else {
      setRGB(2, 0, 255, 0);     // Default green
    }
  } else {
    setRGB(2, 0, 0, 0, true);
  }

  if (isRFMConnected()) {
    if (getRocketStatus() == kRocketStatusAllNominal) {
      setRGB(3, 255, 255, 255);
    } else {
      setRGB(3, 0, 0, 0);
    }
  } else {
    setRGB(3, 0, 0, 0);
  }

  if (isRFMConnected()) {
    if (getRocketGPSSats() > 3) {
      setRGB(4, 0, 255, 0, true);
    } else {
      setRGB(4, 0, 0, 0, true);
    }
  } else {
    setRGB(4, 0, 0, 0, true);
  }
}


