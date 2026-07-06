#pragma once
#include <cstdint>

// pinouts.h — Kuhglocke ground station, Rev 1.0 (ESP32-S3) pin map.
// Values are MCU-native ESP32 GPIO numbers.
namespace pins {

// --- GPS (UART) ---
constexpr uint8_t kGpsTx    = 20;
constexpr uint8_t kGpsRx    = 19;
constexpr uint8_t kGpsReset = 8;

// --- User interface ---
constexpr uint8_t kMenuBtns = 1;
constexpr uint8_t kRgbData  = 46;
constexpr uint8_t kDebugLed = kRgbData;   // same pin as the RGB LED

// --- Power management ---
constexpr uint8_t kChrgStat  = 2;
constexpr uint8_t kDisable5v = 3;

// --- LoRa radio (SPI + DIO) ---
constexpr uint8_t kRfDio2  = 4;
constexpr uint8_t kRfDio0  = 5;
constexpr uint8_t kRfDio1  = 6;
constexpr uint8_t kRfReset = 7;
constexpr uint8_t kRfMosi  = 17;
constexpr uint8_t kRfMiso  = 18;
constexpr uint8_t kRfCs    = 15;
constexpr uint8_t kRfSck   = 16;

// --- E-ink display (SPI + control) ---
constexpr uint8_t kEinkReset = 9;
constexpr uint8_t kEinkDc    = 10;
constexpr uint8_t kEinkCs    = 11;
constexpr uint8_t kEinkSck   = 12;
constexpr uint8_t kEinkMosi  = 13;
constexpr uint8_t kEinkMiso  = 42;
constexpr uint8_t kEinkBusy  = 40;

// --- Speaker (I2S) ---
constexpr uint8_t kSpkI2sLrclk = 14;
constexpr uint8_t kSpkI2sDin   = 21;
constexpr uint8_t kSpkI2sBclk  = 38;
constexpr uint8_t kSpkOn       = 41;

// --- SD card (SDMMC) ---
constexpr uint8_t kSdmmcCmd = 35;
constexpr uint8_t kSdmmcClk = 36;
constexpr uint8_t kSdmmcD0  = 37;

// --- Sensor I2C ---
constexpr uint8_t kI2cScl      = 47;
constexpr uint8_t kI2cSda      = 48;
constexpr uint8_t kNau7802Addr = 0x2A;   // ADC
constexpr uint8_t kP3t1755Addr = 0x48;   // temperature sensor

}  // namespace pins
