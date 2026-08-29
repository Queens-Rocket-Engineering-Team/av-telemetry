#pragma once

#include <Arduino.h>
#include <lora_link.h>

void rfmInit();
bool isRFMConnected();
void onRFMReceive();
bool getRfmReceivedFlag();
void clearRfmReceivedFlag();

// Radio option accessors
double getRadioFreq();
double getRadioBandwidth();
int32_t getRadioSF();
int32_t getRadioCR();

// Incoming RF status
uint32_t getRfmLastRFReceived();
int16_t getRfmLastRSSI();
float getRfmLastSNR();
int32_t getRfmLastFreqErr();
uint32_t getRfmPacketCount();

// Decoded rocket packet data
int32_t getRocketGPSLat();
int32_t getRocketGPSLon();
uint8_t getRocketGPSSats();

// Engineering-unit accessors (catalog wire scaling undone here)
double getRocketLatDeg();
double getRocketLonDeg();
double getRocketAltitudeMeters();
float  getRocketAccelG();
float  getRocketBatteryVolts();
bool   getRocketGpsFix();
uint8_t getRocketFetStatus();
uint8_t getRocketFlightState();

uint8_t getRocketLivenessMask();

// Radio parameter editing (called by menu system)
void radioEditBegin();
void radioEditMove(int8_t dir);
void radioEditConfirm();
void radioEditCancel();
uint8_t radioEditParam();
bool radioEditActive();
void reapplyRadioConfig();

// In-edit preview values (show what will be applied)
double radioEditFreqPreview();
double radioEditBwPreview();
int32_t radioEditSfPreview();
int32_t radioEditCrPreview();
