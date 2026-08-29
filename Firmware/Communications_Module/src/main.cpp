#include "node.h"

#include <IWatchdog.h>
#include <logger.h>
#include <SoftwareSerial.h>
#include <SPI.h>

#include <aim_file_system.h>
#include <aim_flight_recorder.h>
#ifndef FLIGHT_BUILD
#include <aim_console.h>
#endif

static constexpr uint32_t kWatchdogTimeoutUs  = 10000000U; // 10 seconds timeout
static constexpr uint8_t  kMaxRxFramesPerLoop = 8U;
static constexpr uint32_t kLivenessTimeoutMs  = 10000U;  // node considered dead after 10 s silent

static AimStm32CanCore g_canHw(node::kCanBaud, CAN1);
static AimNetwork g_aim(&g_canHw, node::kSource);
static SoftwareSerial g_serial(pins::kSerialRx, pins::kSerialTx);
static Logger g_log(g_serial, static_cast<uint8_t>(node::kSource), LogLevel::INFO);

// Flash on SPI2: MOSI=PB15, MISO=PB14, SCLK=PB13, CS=PB12 (see pinouts.h).
static SPIClass g_flashSpi(pins::kSpiMosi, pins::kSpiMiso, pins::kSpiSclk);
static SpiNorFlashDriver g_flashDriver(pins::kFlashCs, g_flashSpi);
static AimFileSystem g_fs(&g_flashDriver);
static AimFlightRecorder g_recorder(g_fs, kLogCols, kLogOriginRefresh, kLogMaxSize, kLogHeaders);

static void serviceCanRx(void) {
  const uint32_t nowMs = millis();
  for (uint8_t i = 0U; i < kMaxRxFramesPerLoop; i++) {
    aim::Msg m = {};
    if (!g_aim.receive(m)) break;
    nodeOnRx(m, nowMs);
  }
}

#ifndef FLIGHT_BUILD
static void hookStatus(Stream& out) {
  out.print("name=");
  out.print(node::kName);
  out.print(" logMask=0x");
  out.print(static_cast<unsigned>(g_log.filterMask()), HEX);
  out.print(" syncedMs=");
  out.print(static_cast<unsigned long>(g_aim.syncedMillis()));
  out.print(" version=");
  out.print(aim::kNetworkVersionString);
  out.print(" schema=");
  out.print(static_cast<unsigned>(aim::kSchemaVersion));
  out.print(" build=");
  out.print(__DATE__);
  out.print(" ");
  out.println(__TIME__);
}
#endif  // FLIGHT_BUILD

void setup(void) {
  uint32_t start = millis();
  g_serial.begin(node::kSerialBaud);
  g_logger = &g_log;
  g_log.setFilterMask(static_cast<uint8_t>(LogLevel::INFO));  // INFO only

  LOG_INFO("Boot %s source=%u", node::kName, static_cast<unsigned>(aim::Source::Comms));

  // Comms is the downlink aggregator: accept everything worth forwarding, plus
  // Time (to discipline its clock from the master) and Heartbeat (liveness).
  if (!g_aim.begin(aim::classBit(aim::Class::Sensor) |
                   aim::classBit(aim::Class::State) |
                   aim::classBit(aim::Class::Event) |
                   aim::classBit(aim::Class::Heartbeat) |
                   aim::classBit(aim::Class::Time))) {
    LOG_ERROR("CAN init failed");
  }

  if (!g_fs.begin()) {
    LOG_WARN("Filesystem mount failed");
  } else if (!g_recorder.begin()) {
    LOG_WARN("Recorder init failed");
  } else {
    LOG_INFO("Flash ready");
  }

#ifndef FLIGHT_BUILD
  uint8_t nodeHookCount = 0U;
  const AimConsoleHook* nodeHooks = nodeConsoleHooks(nodeHookCount);

  static AimConsoleHook combinedHooks[8];
  uint8_t totalHooks = 0;
  combinedHooks[totalHooks++] = {'s', "status", hookStatus};
  for (uint8_t i = 0; i < nodeHookCount && totalHooks < 8; i++) {
    combinedHooks[totalHooks++] = nodeHooks[i];
  }
  aimConsoleInit(g_serial, g_fs, g_recorder, node::kName, combinedHooks, totalHooks);
  g_serial.println("Console ready. d=enter debug");
#endif

  nodeInit();
  IWatchdog.begin(kWatchdogTimeoutUs);
  LOG_INFO("Watchdog ready (%lus timeout)", static_cast<unsigned long>(kWatchdogTimeoutUs / 1000000U));

  const uint32_t end = millis();
  LOG_INFO("Setup complete (total=%lums)", static_cast<unsigned long>(end - start));
}

void loop(void) {
  const uint32_t nowMs = millis();

  serviceCanRx();
  nodeUpdate(nowMs);
  if (!aimConsoleIsActive()) {
    nodeServiceLog(nowMs, g_recorder);
  }
  nodeServiceCanTx(nowMs, g_aim);
  g_aim.service(nowMs, nodeCurrentState(), nodeErrorBits());   // heartbeat fills bus silence

#ifndef FLIGHT_BUILD
  aimConsoleService();                            // owns console + flash dump/erase
#endif

  IWatchdog.reload();
}
