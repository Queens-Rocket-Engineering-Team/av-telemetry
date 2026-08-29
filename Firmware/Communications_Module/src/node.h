#ifndef NODE_H
#define NODE_H

#include <Arduino.h>
#include <cstdint>

#include <aim_stm32_can_core.h>
#include <aim_network.h>
#include <aim_safety.h>

#include "pinouts.h"

// Node-level identity and interface configuration lives in this file.
namespace node {
constexpr char kName[] = "COMMS_MODULE";
constexpr aim::Source kSource = aim::Source::Comms;
constexpr uint32_t kCanBaud = 1000000U;
constexpr uint32_t kSerialBaud = 38400U;
const uint8_t kCallSign[6] = {'V','A','3','F','G','K'}; // VERY IMPORTANT; FILL OUT. MUST BE 6 CHARS
}  // namespace node

static constexpr uint8_t  kLogCols           = 3U;
static constexpr uint16_t kLogOriginRefresh  = 100U;
static constexpr uint32_t kLogMaxSize        = 0;
static const char* const  kLogHeaders[kLogCols] = {"time", "phase", "seq"};


void nodeInit();
void nodeUpdate(uint32_t nowMs);
void nodeServiceCanTx(uint32_t nowMs, AimNetwork& aim);
void nodeOnRx(const aim::Msg& m, uint32_t nowMs);

class AimFlightRecorder;
void nodeServiceLog(uint32_t nowMs, AimFlightRecorder& recorder);

aim::NodeState nodeCurrentState();
uint16_t nodeErrorBits();

#ifndef FLIGHT_BUILD
#include <aim_console.h>
const AimConsoleHook* nodeConsoleHooks(uint8_t& count);
#endif

#endif  // NODE_H
