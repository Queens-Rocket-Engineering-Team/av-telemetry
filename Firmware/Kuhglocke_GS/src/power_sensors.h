#pragma once

#include <Arduino.h>

// Initialization & periodic update
void initPowerSensors();
void handleReadPowerSensors();

// Sensor readings
uint16_t getPSUVoltage();
uint16_t getBatteryVoltage();
uint16_t getSystemCurrent();
int16_t getAmbTemperature();
uint8_t voltToPercent(uint16_t mv);
uint8_t getChargingStatus();
