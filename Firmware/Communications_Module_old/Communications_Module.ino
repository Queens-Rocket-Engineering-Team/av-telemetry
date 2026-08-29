/*
 * Authors: Kennan Bays, Ethan Toste
 * Hardware: QRET Comms Module Rev2 (WAGGLE)
 * Env: Arduino 1.8.10, STM32duino 2.7.1
 * Created: ~Jun.16.2024
 * Updated: Aug.04.2025
 * Purpose: SRAD firmware for communications module Rev2 (WAGGLE).
 * 
 * IMPORTANT: ADD YOUR CALLSIGN IN HERE
 */

#include "CANPackets.h"
#include "pinouts.h"
#include "STM32_CAN.h" //IMPORTANT: DO NOT DOWNLOAD THE LATEST VERSION (DOWNLOAD 1.1.2)
#include <Wire.h>
#include <RadioLib.h>
#include <SoftwareSerial.h>
#include <SerialFlash.h>
#include <SPI.h>
//#include "flashTable.h"

//TODO: Replace Defines
#define SERIAL_ENABLE true
#define SERIAL_BAUD 38400 // NOTE: While using software serial, dont push above 38400
#define CANBUS_BAUD 500000 //500kbps
#define BUZZER_PIN BUZZER_A_PIN

//Buzzer Settings
const uint32_t BEEP_DELAY = 3000;
const uint32_t BEEP_LENGTH = 1000;
const uint32_t BEEP_FREQ = 1000;

// LoRa Radio Settings
const uint8_t CALLSIGN[6] = {'V','A','3','F','G','K'}; // VERY IMPORTANT; FILL OUT. MUST BE 6 CHARS
uint8_t RADIO_TX_POWER = 20; //dBm
double FREQUENCY = 905.4;
double BANDWIDTH = 62.5;
int32_t SPREADING_RATE = 10;
uint8_t CODING_RATE = 6;

uint8_t powerOpts[] = {4, 7, 10, 13, 16, 19, 22};
double bandwidthOpts[] = {7.8, 10.4, 15.6, 20.8, 31.25, 41.7, 62.5, 125, 250, 500};
int32_t spreadOpts[] = {7, 8, 9, 10, 11, 12};
int32_t codingOpts[] = {5, 6, 7, 8};


#define NORMAL_TRANSMIT_GAP 100
#define BEACON_TRANSMIT_GAP 100 //15000 //15sec
uint32_t transmitGapTime = NORMAL_TRANSMIT_GAP;


#define POWER_DOWN_DELAY 6000 //600000
bool pendingPowerDown = false;
uint32_t powerDownInitTime = 0; //10min
bool inPowerDown = false;


// RADIO TX PACKET VALUES
uint32_t recvGPSLat = 0;
uint32_t recvGPSLon = 0;
uint8_t recvGPSSats = 0;
uint32_t recvAltitude = 0;
bool seenAltimeter = false;
bool seenSensors = false;
bool seenCamera = false;
bool seenGPS = false;
const uint8_t PACKET_SIZE = 14+6;
uint8_t packet[PACKET_SIZE] = {};


// Hardware serial object for USB comms
// NOTE: This had to be changed to software serial as the TX/RX pins are flipped on hardware.
SoftwareSerial usb(USB_RX_PIN, USB_TX_PIN);

// CANBus objects
STM32_CAN can( CAN1, ALT ); //CAN1 ALT is PB8+PB9
static CAN_message_t CAN_RX_msg;
static CAN_message_t CAN_TX_msg;

// Create FlashTable object
const uint8_t TABLE_NAME = 0;
const uint8_t TABLE_COLS = 3;
const uint32_t TABLE_SIZE = 204800; //4096000 [NOTE: DONT MAKE IT THE FULL SIZE OF FLASH]
//FlashTable table = FlashTable(TABLE_COLS, 16384, TABLE_SIZE, TABLE_NAME, 256);


// SX1262 (LoRa1262F30) objects
SPIClass newSPI(RF_MOSI_PIN, RF_MISO_PIN, RF_SCK_PIN);
Module* mod;
SX1262* radio;
int transmissionState = RADIOLIB_ERR_NONE;
volatile bool transmittedFlag = false; // flag to indicate that a packet was sent


void powerDown() {
  transmitGapTime = BEACON_TRANSMIT_GAP;
}//powerDown


void setFlag(void) {
  // we sent a packet, set the flag
  transmittedFlag = true;
  digitalWrite(STATUS_LED_PIN, LOW);
}//setFlag()

void radioInit() {
// initialize SX1262 with default settings
  usb.print(F("[SX1262] Initializing ... "));
  radio->reset();
  delay(100);
  int state = radio->begin();
  if (state == RADIOLIB_ERR_NONE) {
    usb.println(F("success!"));
  } else {
    usb.print(F("failed, code "));
    usb.println(state);
    while (true) {
      digitalWrite(STATUS_LED_PIN, HIGH);
      delay(500);
      digitalWrite(STATUS_LED_PIN, LOW);
      delay(500);
    }
  }


  //radio->forceLDRO(true);

  if (radio->setFrequency(FREQUENCY) == RADIOLIB_ERR_INVALID_FREQUENCY) {
    usb.println(F("Selected frequency is invalid for this module!"));
    while (true);
  }
  if (radio->setBandwidth(BANDWIDTH) == RADIOLIB_ERR_INVALID_BANDWIDTH) {
    usb.println(F("Selected bandwidth is invalid for this module!"));
    while (true);
  }

  // set spreading factor to 10
  if (radio->setSpreadingFactor(SPREADING_RATE) == RADIOLIB_ERR_INVALID_SPREADING_FACTOR) {
    usb.println(F("Selected spreading factor is invalid for this module!"));
    while (true);
  }

  // set coding rate to 6
  if (radio->setCodingRate(CODING_RATE) == RADIOLIB_ERR_INVALID_CODING_RATE) {
    usb.println(F("Selected coding rate is invalid for this module!"));
    while (true);
  }

  // set output power to 12 dBm (SX1262 max is +22, LoRa1262F30 has amp for up to +30)
  // TODO: Change this to a higher power. Check how the LoRa1262F30 is designed; does it use an external PA? Do we have to enable that?
  if (radio->setOutputPower(RADIO_TX_POWER) == RADIOLIB_ERR_INVALID_OUTPUT_POWER) {
    usb.println(F("Selected output power is invalid for this module!"));
    while (true);
  }
  //Enable LoRa CRC
  radio->setCRC(true);

}//radioInit()





// Empties all bytes from incoming serial buffer.
// Used by Debug mode
void emptySerialBuffer() {
  while (usb.available()) {usb.read();}
}//emptySerialBuffer()


void setup() {
  #if defined(SERIAL_ENABLE)
  usb.begin(SERIAL_BAUD);
  #endif
  
  // Set pinmodes
  pinMode(STATUS_LED_PIN, OUTPUT);
  pinMode(BUZZER_A_PIN, OUTPUT);
  pinMode(BUZZER_B_PIN, OUTPUT);
  digitalWrite(STATUS_LED_PIN, LOW);

  // Set up SPI
//  SPI.setSCLK(FLASH_SCK_PIN);
//  SPI.setMISO(FLASH_MISO_PIN);
//  SPI.setMOSI(FLASH_MOSI_PIN);

  // Init CANBUS
  can.begin(); //automatic retransmission can be done using arg "true"
  can.setBaudRate(CANBUS_BAUD);

  // Initialize Flash Chip
//  while (!SerialFlash.begin(FLASH_CS_PIN)) {
//    usb.println(F("Connecting to SPI Flash chip..."));
//    delay(250);
//    //toggleStatusLED();
//  }//while

  //digitalWrite(STATUS_LED_PIN, LOW);
  //delay(1000);

  // Initialize FlashTable object
  //table.init(&SerialFlash);

  //digitalWrite(STATUS_LED_PIN, HIGH);
  
  pinMode(STATUS_LED_PIN, OUTPUT);
  newSPI.begin();
  mod = new Module(RF_CS_PIN, RF_DIO1_PIN, RF_RESET_PIN, RF_BUSY_PIN, newSPI);
  radio = new SX1262(mod);
  radioInit();

  // set the function that will be called
  // when packet transmission is finished
  //radio->setDio0Action(setFlag, RISING);

  // STARTUP BEEP
  delay(BEEP_DELAY);
  initBuzzer();
  setBuzzerFreq(BEEP_FREQ);
  startBuzzer();
  delay(BEEP_LENGTH);
  stopBuzzer();

  // Startup delay - Check to enter debug mode
  uint32_t startTime = millis();
  while (!usb.available() and millis()-startTime < 5000) {}
  
  if (usb.available()) {
    byte d = usb.read();
    emptySerialBuffer();
    if (d == 'D') {
      usb.println(F("Entered Debug Mode"));
      //debugMode();
      while (true) {}
    }//if
  }//if
  usb.println(F("Running Normally"));

  packet[0] = CALLSIGN[0];
  packet[1] = CALLSIGN[1];
  packet[2] = CALLSIGN[2];
  packet[3] = CALLSIGN[3];
  packet[4] = CALLSIGN[4];
  packet[5] = CALLSIGN[5];
}//setup()



void loop() {
  // put your main code here, to run repeatedly:

  if (usb.available()) {
    uint8_t cmd = usb.read();
    uint8_t ind = usb.read() - '0';

    if (cmd=='P') {
      // {  4,    7,     10,       13,      16,    19,     22}; "dbm", 22=absolute max
      //  3mw   5mw     10mw      20mw     40mw    80mw    160mw
      //  ~19mw  ~39mw   ~78mw    ~156mw   ~312mw  ~625mw  ~1250mw
      //    P0    P1     P2        P3       P4     P5      P6
      RADIO_TX_POWER = powerOpts[ind];
      usb.print("RADIO_TX_POWER: ");
      usb.println(RADIO_TX_POWER);
      radioInit();
      usb.println("READY!");
    }
    if (cmd=='B') {
      // {7.8,  10.4,  15.6,  20.8,  31.25,  41.7,  62.5,  125,  250,  500};
      //  B0    B1     B2     B3     B4      B5     B6     B7    B8    B9
      BANDWIDTH = bandwidthOpts[ind];
      usb.print("BANDWIDTH: ");
      usb.println(BANDWIDTH);
      radioInit();
      usb.println("READY!");
    }
    if (cmd=='S') {
      // {7,  8,  9,  10, 11, 12};
      //  S0  S1  S2  S3  S4  S5
      SPREADING_RATE = spreadOpts[ind];
      usb.print("SPREADING_RATE: ");
      usb.println(SPREADING_RATE);
      radioInit();
      usb.println("READY!");
    }
    if (cmd=='C') {
      // {5,  6,  7,  8};
      //  C0  C1  C2  C3
      CODING_RATE = codingOpts[ind];
      usb.print("CODING_RATE: ");
      usb.println(CODING_RATE);
      radioInit();
      usb.println("READY!");
    }
  }//if (usb avail)
  
//  if (!inPowerDown && pendingPowerDown && millis()-powerDownInitTime > POWER_DOWN_DELAY) {
//    inPowerDown = true
//    powerDown();
//  }//if

  usb.println("Ping");

  digitalWrite(STATUS_LED_PIN, HIGH);
  uint32_t strtTr = millis();
  //int state = radio->transmit("QRET RF TEST; THIS IS 30 BYTES");
  int state = radio->transmit(packet, PACKET_SIZE);
  uint32_t endTr = millis();
  digitalWrite(STATUS_LED_PIN, LOW);
  delay(transmitGapTime);

  while (can.read(CAN_RX_msg)) {

    usb.print(F("CAN ID = "));
    usb.println(CAN_RX_msg.id, BIN);
    if (CAN_RX_msg.id == GPS_MOD_CANID+GPS_LAT_CANID) {
      //CAN_RX_msg.buf[i]
      packet[6] = CAN_RX_msg.buf[0];
      packet[7] = CAN_RX_msg.buf[1];
      packet[8] = CAN_RX_msg.buf[2];
      packet[9] = CAN_RX_msg.buf[3];
      seenGPS = true;
      usb.println(F("Recv LAT"));
    } else if (CAN_RX_msg.id == GPS_MOD_CANID+GPS_LON_CANID) {
      //CAN_RX_msg.buf[i]
      packet[10] = CAN_RX_msg.buf[0];
      packet[11] = CAN_RX_msg.buf[1];
      packet[12] = CAN_RX_msg.buf[2];
      packet[13] = CAN_RX_msg.buf[3];
      seenGPS = true;
      usb.println(F("Recv LON"));
    } else if (CAN_RX_msg.id == GPS_MOD_CANID+GPS_NUMSAT_CANID) {
      //CAN_RX_msg.buf[i]
      packet[14] = CAN_RX_msg.buf[0];
      usb.println(F("Recv NUM SAT"));
      seenGPS = true;
    } else if (CAN_RX_msg.id == ALTIMETER_MOD_CANID+ALTITUDE_CANID) {
      //CAN_RX_msg.buf[i]
      packet[15] = CAN_RX_msg.buf[0];
      packet[16] = CAN_RX_msg.buf[1];
      packet[17] = CAN_RX_msg.buf[2];
      packet[18] = CAN_RX_msg.buf[3];
      usb.println(F("Recv ALTITUDE"));
    } else if (CAN_RX_msg.id == ALTIMETER_MOD_CANID+FLIGHT_STAGE_CANID) {
      //check if landed
      usb.println(F("Recv FLGHT STAGE"));
      seenAltimeter = true;
//      if (CAN_RX_msg.buf[0] >= 5) {
//        if (!pendingPowerDown) {
//          pendingPowerDown = true;
//          powerDownInitTime = millis();
//        }//if
//      }//if(landed)
    } else if (CAN_RX_msg.id == SENSOR_MOD_CANID+STATUS_CANID) {
      seenSensors = true;
    } else if (CAN_RX_msg.id == CAMERA_MOD_CANID+STATUS_CANID) {
      seenCamera = true;
    }

    packet[19] = seenGPS*1 + seenAltimeter*2 + seenSensors*4 + seenCamera*8;

  }//while

  
}//loop()
