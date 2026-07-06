#ifndef NODE_H
#define NODE_H

#include <Arduino.h>
#include <cstdint>

#include <aim_can_driver.h>
#include <aim_network.h>
#include <aim_safety.h>

#include "pinouts.h"

// Node-level identity and interface configuration lives in this file.
namespace node {
constexpr char kName[] = "COMMS_MODULE";
constexpr uint32_t kCanBaud = 500000U;
constexpr uint32_t kSerialBaud = 38400U;
const uint8_t kCallSign[6] = {'V','A','3','F','G','K'}; // VERY IMPORTANT; FILL OUT. MUST BE 6 CHARS
}  // namespace node

// --- Node liveness tracker ---
// v0.6.x dropped the library's built-in AimNodeHealth; Comms keeps a minimal
// local table so the console can report which nodes are on the bus. Any valid
// frame from a source proves its liveness (protocol invariant).
struct NodeLiveness {
  aim::Source source;
  uint32_t lastHeardMs;  // local millis() — never syncedMillis(), which steps
  bool everHeard;
};

void nodeInit();
void nodeUpdate(uint32_t nowMs);
void nodeServiceCanTx(uint32_t nowMs, AimNetwork& aim);
void nodeOnRx(const aim::Msg& m, uint32_t nowMs);

void nodeLivenessInit(uint32_t nowMs);
void nodeLivenessOnRx(aim::Source source, uint32_t nowMs);
const NodeLiveness* nodeLivenessTable(uint8_t* countOut);

aim::NodeState nodeCurrentState();
uint16_t nodeErrorBits();

#ifndef FLIGHT_BUILD
#include <aim_console.h>
const AimConsoleHook* nodeConsoleHooks(uint8_t& count);
#endif

#endif  // NODE_H
