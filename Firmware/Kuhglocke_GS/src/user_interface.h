#pragma once

#include <Arduino.h>

void initLEDs();
void setUSBDataOnlyMode(bool state);
void setLEDBrightness(uint8_t brightness);
void setRGB(byte index, byte r, byte g, byte b);
void setRGB(byte index, byte r, byte g, byte b, bool push);
void setRGB(byte r, byte g, byte b);
bool makeNextSDLog();
bool writeToSDLog(const String& txt);
void triggerRFFlash();
void handleLEDs();

