#include "gps.h"
#include "pinouts.h"
#include "global.h"
#include "radio_control.h"
#include "user_interface.h"
#include <math.h>
#include <TinyGPSPlus.h>
#include <HardwareSerial.h>

static TinyGPSPlus s_gps;
static HardwareSerial s_gpsSerial(1);
static uint32_t s_lastLocalGPSLog = 0;

double degToRad(double degs) {
  return degs * (M_PI / 180.0);
}

int32_t getDistanceToRocket() {
  if (s_gps.location.lat() == 0 || s_gps.location.lng() == 0 || s_gps.location.age() > 5000) {
    return -1;
  }
  if (getRocketGPSLat() == 0 || getRocketGPSLon() == 0) {
    return -1;
  }

  double lat1Rad = degToRad(s_gps.location.lat());
  double lon1Rad = degToRad(s_gps.location.lng());
  double lat2Rad = degToRad(getRocketLatDeg());
  double lon2Rad = degToRad(getRocketLonDeg());
  
  double dLat = lat2Rad - lat1Rad;
  double dLon = lon2Rad - lon1Rad;
  
  double a = sin(dLat / 2) * sin(dLat / 2) +
             cos(lat1Rad) * cos(lat2Rad) *
             sin(dLon / 2) * sin(dLon / 2);
  double c = 2 * atan2(sqrt(a), sqrt(1 - a));
  double distance = kEarthRadiusFeet * c;
  
  return distance / 3.281;
}

void gpsInit() {
  s_gpsSerial.begin(kGpsBaud, SERIAL_8N1, pins::kGpsRx, pins::kGpsTx); 
}

void handleGPS() {
  while (s_gpsSerial.available() > 0) {
    s_gps.encode(s_gpsSerial.read());
  }

  if (millis() - s_lastLocalGPSLog > kLocalGpsLogRate) {
    s_lastLocalGPSLog = millis();
    String resp = String("LocalGPS: ");
    resp += String(s_gps.satellites.age());
    resp += "," + String(s_gps.satellites.value());
    resp += "," + String(s_gps.location.lat(), 6);
    resp += "," + String(s_gps.location.lng(), 6);
    writeToSDLog(resp);
  }

  if (millis() > 50000 && s_gps.charsProcessed() < 10) {
    Serial.println("[ERROR] No response from onboard GPS");
  }
}

void calculateRocketVelocity() {
  static double previousAltitudeM = 0;
  static uint32_t lastTime = 0;

  if (isRfmLastPacketValid() && millis() - getRfmLastRFReceived() < kRfmConnectedTimeout) {
    uint32_t currentTime = millis();
    double currentAltitudeM = getRocketAltitudeMeters();

    if (lastTime > 0 && previousAltitudeM > 0) {
      double timeDiff = (currentTime - lastTime) / 1000.0;  // seconds
      double altDiff = currentAltitudeM - previousAltitudeM; // meters
      setRocketVelocity(altDiff / timeDiff);                 // m/s
    }

    previousAltitudeM = currentAltitudeM;
    lastTime = currentTime;
  }
}

uint32_t getGPSAge() {
  return s_gps.location.age();
}


