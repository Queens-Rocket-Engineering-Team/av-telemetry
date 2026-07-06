#include "screen_control.h"
#include "pinouts.h"
#include "global.h"
#include "radio_control.h"
#include "power_sensors.h"
#include "gps.h"
#include "menu.h"
#include <SPI.h>
#include <GxEPD2_BW.h>
#include "GxEPD2_display_selection_new_style.h"
#include "GxEPD2_selection_check.h"

static SPIClass s_epdSPI(FSPI);
static GxEPD2_BW<GxEPD2_213_B74, GxEPD2_213_B74::HEIGHT>
s_display(GxEPD2_213_B74(pins::kEinkCs, pins::kEinkDc, pins::kEinkReset, pins::kEinkBusy));

static constexpr uint16_t kDisplayW  = 250;
static constexpr uint16_t kTitleBarH = 13;
static constexpr uint16_t kUpdateH   = 108;

static uint16_t centerX(uint16_t charCount, uint8_t textSize) {
  return (kDisplayW - charCount * 6U * textSize) / 2U;
}

static void drawTitleBar(const char* title) {
  s_display.fillRect(0, 0, kDisplayW, kTitleBarH, GxEPD_BLACK);
  s_display.setTextColor(GxEPD_WHITE);
  s_display.setTextSize(1);
  s_display.setCursor(centerX(strlen(title), 1), 3);
  s_display.print(title);
  s_display.setTextColor(GxEPD_BLACK);
}

// --- Screen: Telemetry ---
static void drawTelemetry() {
  drawTitleBar("TELEMETRY");

  s_display.setTextSize(2);

  s_display.setCursor(0, 16);
  s_display.print("LAT ");
  s_display.print(getRocketLatDeg(), 6);

  s_display.setCursor(0, 32);
  s_display.print("LON ");
  s_display.print(getRocketLonDeg(), 6);

  s_display.setCursor(0, 48);
  s_display.print("ALT ");
  s_display.print(getRocketAltitudeMeters() * 3.28084, 0);
  s_display.print("ft");

  s_display.setCursor(0, 64);
  s_display.print("VEL ");
  s_display.print(getRocketVelocity() * 3.28084f, 1);
  s_display.print("ft/s");

  s_display.drawLine(0, 82, kDisplayW, 82, GxEPD_BLACK);

  s_display.setTextSize(1);

  uint32_t linkAgeMs = millis() - getRfmLastRFReceived();
  s_display.setCursor(0, 86);
  s_display.print("AGE ");
  if (linkAgeMs < 100000U) {
    s_display.print(linkAgeMs / 1000.0, 1);
    s_display.print("s");
  } else {
    s_display.print("---");
  }

  s_display.setCursor(84, 86);
  s_display.print("DST ");
  int32_t dist = getDistanceToRocket();
  if (dist > 0) {
    s_display.print(dist);
    s_display.print("m");
  } else {
    s_display.print("---");
  }

  s_display.setCursor(168, 86);
  s_display.print("SAT ");
  s_display.print(getRocketGPSSats());

  s_display.setCursor(0, 97);
  s_display.print(getRfmLastRSSI());
  s_display.print("dBm");

  s_display.setCursor(60, 97);
  s_display.print(getRfmLastSNR(), 1);
  s_display.print("dB");

  s_display.setCursor(120, 97);
  s_display.print("#");
  s_display.print(getRfmPacketCount());

  s_display.setCursor(180, 97);
  s_display.print("BAT ");
  s_display.print(voltToPercent(getBatteryVoltage()));
  s_display.print("%");
}

// --- Screen: Node Health ---
static void drawNodeHealth() {
  drawTitleBar("NODES");

  const NodeStatus* nodes = getNodeStatusTable();
  uint32_t nowMs = millis();

  for (uint8_t i = 0; i < lora::kTrackedNodeCount; i++) {
    uint16_t rowY = 16 + i * 16;
    bool lost = nodes[i].everHeard &&
                (nowMs - nodes[i].lastHeardMs >= lora::kNodeAliveTimeoutMs);

    if (lost) {
      s_display.fillRect(0, rowY, kDisplayW, 16, GxEPD_BLACK);
      s_display.setTextColor(GxEPD_WHITE);
    }

    s_display.setTextSize(2);
    s_display.setCursor(2, rowY);
    s_display.print(nodes[i].name);

    if (!nodes[i].everHeard) {
      s_display.setCursor(60, rowY);
      s_display.print("---");
    } else {
      uint32_t ageMs = nowMs - nodes[i].lastHeardMs;
      s_display.setCursor(60, rowY);
      s_display.print(lost ? "LOST" : "OK");
      s_display.setCursor(132, rowY);
      if (ageMs < 10000U) {
        s_display.print(ageMs / 1000.0, 1);
      } else {
        s_display.print(ageMs / 1000U);
      }
      s_display.print("s");
    }

    s_display.setTextColor(GxEPD_BLACK);
  }

  s_display.drawLine(0, 97, kDisplayW, 97, GxEPD_BLACK);
  s_display.setTextSize(1);
  s_display.setCursor(0, 100);
  s_display.print(getRfmLastRSSI());
  s_display.print("dBm  ");
  s_display.print(getRfmLastSNR(), 1);
  s_display.print("dB  AGE ");
  uint32_t linkAge = millis() - getRfmLastRFReceived();
  if (linkAge < 100000U) {
    s_display.print(linkAge / 1000.0, 1);
    s_display.print("s");
  } else {
    s_display.print("---");
  }
}

// --- Screen: Radio ---
static void drawRadio() {
  bool editing = radioEditActive();
  uint8_t editIdx = radioEditParam();

  drawTitleBar(editing ? "RADIO  EDIT" : "RADIO");

  s_display.setTextSize(2);
  constexpr uint16_t kSignalX = 156;

  s_display.drawLine(kSignalX - 4, kTitleBarH, kSignalX - 4, 80, GxEPD_BLACK);

  double freqVal = editing ? radioEditFreqPreview() : getRadioFreq();
  s_display.setCursor(2, 16);
  s_display.print(editing && editIdx == 0 ? ">" : " ");
  s_display.print("F:");
  s_display.print(freqVal, 1);
  s_display.setCursor(kSignalX, 16);
  s_display.print(getRfmLastRSSI());
  s_display.print("dBm");

  double bwVal = editing ? radioEditBwPreview() : getRadioBandwidth();
  s_display.setCursor(2, 32);
  s_display.print(editing && editIdx == 1 ? ">" : " ");
  s_display.print("B:");
  s_display.print(bwVal, 1);
  s_display.setCursor(kSignalX, 32);
  s_display.print(getRfmLastSNR(), 1);
  s_display.print("dB");

  int32_t sfVal = editing ? radioEditSfPreview() : getRadioSF();
  s_display.setCursor(2, 48);
  s_display.print(editing && editIdx == 2 ? ">" : " ");
  s_display.print("SF:");
  s_display.print(sfVal);
  s_display.setCursor(kSignalX, 48);
  s_display.print(getRfmLastFreqErr());
  s_display.print("Hz");

  int32_t crVal = editing ? radioEditCrPreview() : getRadioCR();
  s_display.setCursor(2, 64);
  s_display.print(editing && editIdx == 3 ? ">" : " ");
  s_display.print("CR:");
  s_display.print(crVal);
  s_display.setCursor(kSignalX, 64);
  s_display.print(getRfmPacketCount());
  s_display.print("pk");

  s_display.drawLine(0, 84, kDisplayW, 84, GxEPD_BLACK);
  s_display.setTextSize(1);
  s_display.setCursor(0, 90);
  s_display.print("AGE ");
  uint32_t linkAge = millis() - getRfmLastRFReceived();
  if (linkAge < 100000U) {
    s_display.print(linkAge / 1000.0, 1);
    s_display.print("s");
  } else {
    s_display.print("---");
  }

  s_display.setCursor(84, 90);
  s_display.print("BAT ");
  s_display.print(voltToPercent(getBatteryVoltage()));
  s_display.print("%");

  if (!editing) {
    s_display.setCursor(168, 90);
    s_display.print("[ENTER=edit]");
  } else {
    s_display.setCursor(156, 90);
    s_display.print("[ENTER=apply]");
  }
}

// --- Public API ---

void screenInit() {
  s_epdSPI.begin(pins::kEinkSck, pins::kEinkMiso, pins::kEinkMosi, pins::kEinkCs);
  pinMode(s_epdSPI.pinSS(), OUTPUT);
  s_display.init(kEpdBaud, true, 2, false, s_epdSPI, SPISettings(kEpdSpiClock, MSBFIRST, SPI_MODE0));
  s_display.setRotation(1);
}

void drawLoadingScreen() {
  s_display.setTextColor(GxEPD_BLACK);
  s_display.firstPage();
  do {
    s_display.fillScreen(GxEPD_WHITE);

    // Inverted banner
    s_display.fillRect(0, 0, kDisplayW, 36, GxEPD_BLACK);
    s_display.setTextColor(GxEPD_WHITE);
    s_display.setTextSize(2);
    s_display.setCursor(centerX(4, 2), 4);
    s_display.print("GREG");
    s_display.setTextSize(1);
    s_display.setCursor(centerX(14, 1), 24);
    s_display.print("GROUND STATION");
    s_display.setTextColor(GxEPD_BLACK);

    s_display.drawLine(20, 44, 230, 44, GxEPD_BLACK);

    s_display.setTextSize(2);
    s_display.setCursor(centerX(4, 2), 52);
    s_display.print("QRET");

    s_display.drawLine(20, 72, 230, 72, GxEPD_BLACK);

    s_display.setTextSize(1);
    s_display.setCursor(centerX(strlen(kFirmwareVersion), 1), 82);
    s_display.print(kFirmwareVersion);

    // LED labels
    s_display.setCursor(0, 114);
    s_display.print("POWER  BATT    RADIO   SRADOK   GPS FIX");
  } while (s_display.nextPage());
}

void updateEPD() {
  s_display.setPartialWindow(0, 0, kDisplayW, kUpdateH);
  s_display.firstPage();
  do {
    s_display.fillScreen(GxEPD_WHITE);
    s_display.setTextColor(GxEPD_BLACK);

    switch (menuCurrentScreen()) {
      case MenuScreen::Telemetry:  drawTelemetry();  break;
      case MenuScreen::NodeHealth: drawNodeHealth();  break;
      case MenuScreen::Radio:      drawRadio();       break;
      default:                     drawTelemetry();   break;
    }
  } while (s_display.nextPage());
}
