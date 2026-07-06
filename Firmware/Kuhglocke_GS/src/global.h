#pragma once

#include <Arduino.h>
#include <WiFi.h>

inline constexpr double kEarthRadiusFeet = 20925524.9;

inline const char kFirmwareVersion[] = "Jun.21.2026, V2.0.0C";

// Pure constants => constexpr
inline constexpr uint8_t  kNumRgbLeds   = 5;
inline constexpr uint32_t kUsbBaud       = 115200;
inline constexpr uint32_t kGpsBaud       = 9600;
inline constexpr uint32_t kEpdSpiClock  = 1000000; // 1MHz
inline constexpr uint32_t kEpdBaud       = 115200;
inline constexpr uint32_t kRfmSpiClock  = 2000000; // 2MHz
inline constexpr uint32_t kI2cSpeed      = 100000;  // 100kHz

// Strings: use inline constexpr char[]
// Launch-control network credentials: set before flight; intentionally empty in VCS.
inline constexpr const char kSsid[] = "propnet";
inline constexpr const char kPassword[] = "propteambestteam";

inline constexpr double   kBattVdiv = 3.003579098067;
inline constexpr uint16_t kSensorSamplePeriod = 250;

inline constexpr uint8_t  kDefaultLedBrightness = 15; // 0-255
inline constexpr uint16_t kIndRfFlashTime = 130;     // ms
inline constexpr uint16_t kIndBatteryFlashGap = 750; // ms

inline constexpr uint16_t kRfmConnectedTimeout = 2000; // ms

inline constexpr uint32_t kLocalGpsLogRate = 1000;
inline constexpr uint32_t kAltCoreStacksSize = 10000;
inline constexpr uint16_t kEpdUpdateInt = 800;

// Named thresholds/status constants
inline constexpr uint16_t kBattVoltLow = 3500;
inline constexpr uint16_t kBattVoltMedium = 3700;
inline constexpr uint16_t kBattVoltHigh = 3900;
inline constexpr uint8_t  kRocketStatusAllNominal = 0b111;

