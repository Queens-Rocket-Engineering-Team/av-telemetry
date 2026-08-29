#pragma once

#include <Arduino.h>

enum class MenuScreen : uint8_t {
  Telemetry = 0,
  NodeHealth,
  Radio,
  kCount
};

enum class MenuButton : uint8_t {
  None = 0,
  Enter,
  Back,
  Down,
  Up
};

void menuInit();
void menuService(uint32_t nowMs);

MenuScreen menuCurrentScreen();
MenuButton menuReadButton();
