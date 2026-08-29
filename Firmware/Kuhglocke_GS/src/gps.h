#pragma once

#include <Arduino.h>

void gpsInit();
void handleGPS();
double degToRad(double degs);
int32_t getDistanceToRocket();


// Accessors for local GPS state
uint32_t getGPSAge();