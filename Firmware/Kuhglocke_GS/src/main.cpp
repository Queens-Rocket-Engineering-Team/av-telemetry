#include <Arduino.h>
#include "global.h"
#include "pinouts.h"
#include "power_sensors.h"
#include "radio_control.h"
#include "screen_control.h"
#include "user_interface.h"
#include "gps.h"
#include "menu.h"
#include "SD_MMC.h"
#include "WiFi.h"
#include "qlcp_uplink.h"
#include <Wire.h>
#include <MedianFilterLib.h>


// Forward declarations of Core0 entrypoint
static void loopAltCoreHandler(void* pvParameters);
static void loopAltCore();

// Static Task Handles and stats
static TaskHandle_t s_core0Task = nullptr;
static MedianFilter<uint16_t> s_core1LoopFilter(8);

static volatile uint32_t s_core0FreeStack = 0;
static volatile uint32_t s_core0LoopTime = 0;
static volatile uint32_t s_core0MaxLoopTime = 0;
static uint32_t s_core1MaxLoopTime = 0;
static uint32_t s_lastEPDUpdate = 0;

void setup() {
  // Configure pinmodes
  pinMode(pins::kDisable5v, OUTPUT);
  pinMode(pins::kDebugLed, OUTPUT);
  pinMode(pins::kGpsReset, OUTPUT);
  digitalWrite(pins::kGpsReset, HIGH);
  pinMode(pins::kMenuBtns, INPUT);
  pinMode(pins::kChrgStat, INPUT);
  analogReadResolution(12);

  // Ensure USB mode is PWR+DATA by default
  setUSBDataOnlyMode(false);

  // Initialize LEDs
  initLEDs();
  setRGB(0, 128, 32, 0); // Power LED to loading

  // Start USB Serial
  Serial.begin(kUsbBaud);

  // Initialize E-Paper Display
  screenInit();
  drawLoadingScreen();

  // Configure I2C Bus
  Wire.begin(pins::kI2cSda, pins::kI2cScl);
  Wire.setClock(kI2cSpeed);

  // Initialize MicroSD card (1-bit mode)
  SD_MMC.setPins(pins::kSdmmcClk, pins::kSdmmcCmd, pins::kSdmmcD0);
  if (!SD_MMC.begin("/sdcard", true)) {
    Serial.println("[WARN] MicroSD Card Mount Failed");
  } else {
    Serial.println("SD: Card Mount Success");
    makeNextSDLog();
    writeToSDLog("Kuhglocke SD card initialized");
  }

  // Initialize NAU7802 ADC
  initPowerSensors();

  // Initialize GPS
  gpsInit(); 

  // Initialize RFM95 Radio
  rfmInit();

  // Launch WiFi STA
  WiFi.mode(WIFI_STA);
  WiFi.begin(kSsid, kPassword);
  Serial.printf("Connecting to SSID: %s\n", kSsid);
  uint32_t wifiConnectStart = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - wifiConnectStart < 1000) {  // 1 second timeout
    delay(100);
    Serial.print(".");
  }
 
  if (WiFi.status() == WL_CONNECTED) {
    Serial.print("\nWiFi Connected! IP: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("\nWiFi connection pending, will connect in background");
  }

  // Initialize QLCP client
  qlcpUplinkInit();

  // Initialize menu system
  menuInit();

  // Launch background task for Core 0
  xTaskCreatePinnedToCore(
      loopAltCoreHandler,   // Task function
      "Core0Loop",          // Name of the task
      kAltCoreStacksSize,   // Stack size
      NULL,                 // Task input parameter
      1,                    // Priority of the task
      &s_core0Task,         // Task handle
      0                     // Core number
  );

  // Set power LED to "ready"
  setRGB(0, 0, 128, 0); // Turn on power LED
  Serial.println("Kuhglocke Initialization Complete!");
}

void loop() {
  uint32_t loopStart = millis();
  
  handleGPS();
  handleReadPowerSensors();
  handleLEDs();
  
  menuService(millis());

  // Check for incoming RFM95 packets
  if (getRfmReceivedFlag()) {
    clearRfmReceivedFlag();
    onRFMReceive();
  }

  // Service the QLCP uplink
  qlcpUplinkService();

  // Reconnect WiFi if disconnected
  static uint32_t s_lastWifiCheck = 0;
  if (millis() - s_lastWifiCheck > 5000) {
    s_lastWifiCheck = millis();
    if (WiFi.status() != WL_CONNECTED) {
      WiFi.begin(kSsid, kPassword);
    }
  }

  uint16_t core1LoopTime = (millis() - loopStart);
  s_core1LoopFilter.AddValue(core1LoopTime);
  if (core1LoopTime > s_core1MaxLoopTime) {
    s_core1MaxLoopTime = core1LoopTime;
  }
}

static void loopAltCoreHandler(void* pvParameters) {
  Serial.println("Starting background thread on Core0");
  while (true) {
    loopAltCore();
  }
}

static void loopAltCore() {
  uint32_t loopStart = millis();
  
  if (millis() - s_lastEPDUpdate > kEpdUpdateInt) {
    s_lastEPDUpdate = millis();
    updateEPD();
  }

  s_core0FreeStack = uxTaskGetStackHighWaterMark(NULL);
  s_core0LoopTime = (millis() - loopStart);
  if (s_core0LoopTime > s_core0MaxLoopTime) {
    s_core0MaxLoopTime = s_core0LoopTime;
  }
}

// Accessors for loop and core statistics
uint16_t getCore1LoopFiltered() {
  return s_core1LoopFilter.GetFiltered();
}

uint32_t getCore1MaxLoopTime() {
  return s_core1MaxLoopTime;
}

uint32_t getCore0FreeStack() {
  return s_core0FreeStack;
}

uint32_t getCore0LoopTime() {
  return s_core0LoopTime;
}

uint32_t getCore0MaxLoopTime() {
  return s_core0MaxLoopTime;
}
