#ifndef PINOUTS_H
#define PINOUTS_H

#include <Arduino.h> // needed for PB/A# pin assignment
#include <cstdint>

// pinouts.h - WAGGLE Communications Module V2.0 2024/2025 (STM32F103CB) pin map.
namespace pins {

// --- Serial (USB-UART Bridge; TX/RX are swapped on the PCB) ---
//  !!! Fix in next board spin !!!
//  Use software serial
constexpr uint8_t kSerialTx = PB11; // USART_TX -> USB_RX
constexpr uint8_t kSerialRx = PB10; // USART_RX <- USB_TX

// --- CAN bus ---
constexpr uint8_t kCanRx = PB8;
constexpr uint8_t kCanTx = PB9;

// --- Flash (SPI) ---
constexpr uint8_t kFlashReset = PA8;
constexpr uint8_t kFlashCs    = PB12;
constexpr uint8_t kSpiSclk    = PB13;
constexpr uint8_t kSpiMiso    = PB14;
constexpr uint8_t kSpiMosi    = PB15;

// --- LoRa radio (SPI) ---
constexpr uint8_t kRfBusy  = PA0;
constexpr uint8_t kRfDio1  = PA1;
constexpr uint8_t kRfDio3  = PA2;
constexpr uint8_t kRfReset = PA3;
constexpr uint8_t kRfCs    = PA4;
constexpr uint8_t kRfSclk  = PA5;
constexpr uint8_t kRfMiso  = PA6;
constexpr uint8_t kRfMosi  = PA7;

// --- Buzzer (TIM1 complementary PWM pair) ---
constexpr uint8_t kBuzzerA = PA10;
constexpr uint8_t kBuzzerB = PB1;

// --- LEDs ---
constexpr uint8_t kRgbData  = PB3;
constexpr uint8_t kDebugLed = PA15;

// --- Analogue / power monitoring ---
constexpr uint8_t kBatterySense = PB0;

// --- Test points ---
constexpr uint8_t kTp4 = PA9;
constexpr uint8_t kTp6 = PB6;
constexpr uint8_t kTp7 = PB7;

}

#endif // PINOUTS_H

