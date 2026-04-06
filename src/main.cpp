#include <Arduino.h>
#include <SPI.h>
#include <ACAN2515.h>
#include <ACAN2517FD.h>

// Pin defaults follow Battery-Emulator DevKit style.
static constexpr uint8_t MCP2515_SCK = 22;
static constexpr uint8_t MCP2515_MOSI = 21;
static constexpr uint8_t MCP2515_MISO = 19;
static constexpr uint8_t MCP2515_CS = 18;
static constexpr uint8_t MCP2515_INT = 23;

static constexpr uint8_t MCP2518_SCK = 33;
static constexpr uint8_t MCP2518_MOSI = 32; // SDI on MCP2518FD
static constexpr uint8_t MCP2518_MISO = 35; // SDO on MCP2518FD
static constexpr uint8_t MCP2518_CS = 25;
static constexpr uint8_t MCP2518_INT = 34;

static constexpr uint32_t CAN_BITRATE = 500000UL;

SPIClass spi2515(VSPI);
SPIClass spi2518(HSPI);

ACAN2515 can2515(MCP2515_CS, spi2515, MCP2515_INT);
ACAN2517FD can2518(MCP2518_CS, spi2518, MCP2518_INT);

uint32_t fwd2515to2518 = 0;
uint32_t fwd2518to2515 = 0;
uint32_t droppedFdTo2515 = 0;
uint32_t sendFail2515 = 0;
uint32_t sendFail2518 = 0;

static constexpr uint32_t OBD_FUNC_11BIT_ID = 0x7DF;
static constexpr uint32_t OBD_PHYS_REQ_11BIT_ID = 0x7E0;
static constexpr uint32_t OBD_RESP_11BIT_ID = 0x7E8;
static constexpr uint32_t OBD_FUNC_29BIT_ID = 0x18DB33F1;
static constexpr uint32_t OBD_PHYS_REQ_29BIT_ID = 0x18DA10F1;
static constexpr uint32_t OBD_RESP_29BIT_ID = 0x18DAF110;

static void printTracePrefix(const char* direction, uint32_t id, uint8_t len) {
  const uint32_t now = millis();
  const uint32_t seconds = now / 1000UL;
  const uint32_t milliseconds = now % 1000UL;

  Serial.print('(');
  Serial.print(seconds);
  Serial.print('.');
  if (milliseconds < 100) {
    Serial.print('0');
  }
  if (milliseconds < 10) {
    Serial.print('0');
  }
  Serial.print(milliseconds);
  Serial.print(") ");
  Serial.print(direction);
  Serial.print(' ');
  Serial.print(id, HEX);
  Serial.print(" [");
  Serial.print(len);
  Serial.print("] ");
}

static void printCan2515Message(const char* direction, const CANMessage& msg) {
  printTracePrefix(direction, msg.id, msg.len);
  for (uint8_t i = 0; i < msg.len; i++) {
    Serial.print(msg.data[i], HEX);
    Serial.print(' ');
  }
  Serial.println();
}

static void printCan2518Message(const char* direction, const CANFDMessage& msg) {
  printTracePrefix(direction, msg.id, msg.len);
  for (uint8_t i = 0; i < msg.len; i++) {
    Serial.print(msg.data[i], HEX);
    Serial.print(' ');
  }
  Serial.println();
}

static bool can2518MessageCanReachObdScanner(const CANFDMessage& msg) {
  return msg.len <= 8 
  // &&
      // msg.type != CANFDMessage::CANFD_NO_BIT_RATE_SWITCH &&
      // msg.type != CANFDMessage::CANFD_WITH_BIT_RATE_SWITCH
      ;
}

static bool sendCan2515Message(const CANMessage& msg) {
  if (!can2515.tryToSend(msg)) {
    sendFail2515++;
    return false;
  }
  return true;
}

static bool sendObdResponse11Bit(const uint8_t service, const uint8_t pid,
                                 const uint8_t a, const uint8_t b,
                                 const uint8_t c, const uint8_t d) {
  CANMessage reply;
  reply.id = OBD_RESP_11BIT_ID;
  reply.ext = false;
  reply.rtr = false;
  reply.len = 8;
  reply.data[0] = 0x06;
  reply.data[1] = service;
  reply.data[2] = pid;
  reply.data[3] = a;
  reply.data[4] = b;
  reply.data[5] = c;
  reply.data[6] = d;
  reply.data[7] = 0x00;
  return sendCan2515Message(reply);
}

static bool sendObdResponse29Bit(const uint8_t service, const uint8_t pid,
                                 const uint8_t a, const uint8_t b,
                                 const uint8_t c, const uint8_t d) {
  CANMessage reply;
  reply.id = OBD_RESP_29BIT_ID;
  reply.ext = true;
  reply.rtr = false;
  reply.len = 8;
  reply.data[0] = 0x06;
  reply.data[1] = service;
  reply.data[2] = pid;
  reply.data[3] = a;
  reply.data[4] = b;
  reply.data[5] = c;
  reply.data[6] = d;
  reply.data[7] = 0x00;
  return sendCan2515Message(reply);
}

static bool emulateObdService01(const CANMessage& request) {
  if (request.rtr || request.len < 3 || request.data[1] != 0x01) {
    return false;
  }

  const bool is11Bit = !request.ext &&
      (request.id == OBD_FUNC_11BIT_ID || request.id == OBD_PHYS_REQ_11BIT_ID);
  const bool is29Bit = request.ext &&
      (request.id == OBD_FUNC_29BIT_ID || request.id == OBD_PHYS_REQ_29BIT_ID);

  if (!is11Bit && !is29Bit) {
    return false;
  }

  const uint8_t pid = request.data[2];

  switch (pid) {
    case 0x00:
      // Supported PIDs bitmap for 0x01-0x20.
      if (is11Bit) {
        return sendObdResponse11Bit(0x41, pid, 0x18, 0x18, 0x80, 0x13);
      }
      return sendObdResponse29Bit(0x41, pid, 0x18, 0x18, 0x80, 0x13);

      // case 0x05:
      //   // Coolant temperature, value = A - 40.
      //   if (is11Bit) {
      //     return sendObdResponse11Bit(0x41, pid, 0x5A, 0x00, 0x00, 0x00);
      //   }
      //   return sendObdResponse29Bit(0x41, pid, 0x5A, 0x00, 0x00, 0x00);

      // case 0x0C:
      //   // Engine RPM, value = ((A * 256) + B) / 4 -> 750 rpm.
      //   if (is11Bit) {
      //     return sendObdResponse11Bit(0x41, pid, 0x0B, 0xB8, 0x00, 0x00);
      //   }
      //   return sendObdResponse29Bit(0x41, pid, 0x0B, 0xB8, 0x00, 0x00);

      // case 0x0D:
      //   // Vehicle speed in km/h.
      //   if (is11Bit) {
      //     return sendObdResponse11Bit(0x41, pid, 0x00, 0x00, 0x00, 0x00);
      //   }
      //   return sendObdResponse29Bit(0x41, pid, 0x00, 0x00, 0x00, 0x00);

      // case 0x11:
      //   // Throttle position.
      //   if (is11Bit) {
      //     return sendObdResponse11Bit(0x41, pid, 0x00, 0x00, 0x00, 0x00);
      //   }
      //   return sendObdResponse29Bit(0x41, pid, 0x00, 0x00, 0x00, 0x00);

    default:
      return false;
  }
}

static bool initCanControllers() {
  spi2515.begin(MCP2515_SCK, MCP2515_MISO, MCP2515_MOSI);
  ACAN2515Settings settings2515(MCP2515_XTAL_HZ, CAN_BITRATE);
  settings2515.mRequestedMode = ACAN2515Settings::NormalMode;
  const uint16_t errorCode2515 = can2515.begin(settings2515, [] { can2515.isr(); });

  spi2518.begin(MCP2518_SCK, MCP2518_MISO, MCP2518_MOSI);
  ACAN2517FDSettings settings2518(
      ACAN2517FDSettings::OSC_40MHz,
      CAN_BITRATE,
      DataBitRateFactor::x4);
  settings2518.mRequestedMode = ACAN2517FDSettings::NormalFD;
  const uint32_t errorCode2518 = can2518.begin(settings2518, [] { can2518.isr(); });
  can2518.poll();

  if (errorCode2515 != 0) {
    Serial.print("MCP2515 init error: 0x");
    Serial.println(errorCode2515, HEX);
    return false;
  }

  if (errorCode2518 != 0) {
    Serial.print("MCP2518 init error: 0x");
    Serial.println((uint16_t)errorCode2518, HEX);
    return false;
  }

  Serial.println("MCP2515 + MCP2518 initialized");
  return true;
}

static bool forward2515to2518(const CANMessage& inMsg) {
  CANFDMessage outMsg(inMsg);

  if (!can2518.tryToSend(outMsg)) {
    Serial.println("Sending to MCP2518FD failed");
    sendFail2518++;
    return false;
  }

  fwd2515to2518++;
  return true;
}

static bool forward2518to2515(const CANFDMessage& inMsg) {
  // MCP2515 can only transmit classic CAN payload (max 8 bytes, non-FD frame).
  if (
    // (inMsg.type == CANFDMessage::CANFD_NO_BIT_RATE_SWITCH ||
    //    inMsg.type == CANFDMessage::CANFD_WITH_BIT_RATE_SWITCH) ||
      inMsg.len > 8
    ) {
    droppedFdTo2515++;
    return false;
  }

  CANMessage outMsg;
  outMsg.id = inMsg.id;
  outMsg.ext = inMsg.ext;
  outMsg.rtr = (inMsg.type == CANFDMessage::CAN_REMOTE);
  outMsg.len = inMsg.len;
  for (uint8_t i = 0; i < outMsg.len; i++) {
    outMsg.data[i] = inMsg.data[i];
  }

  if (!can2515.tryToSend(outMsg)) {
    Serial.println("Sending to MCP2515 failed");
    sendFail2515++;
    return false;
  }

  fwd2518to2515++;
  return true;
}

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("Starting OBD bridge: MCP2515 <-> MCP2518");

  if (!initCanControllers()) {
    Serial.println("CAN init failed, rebooting in 3s");
    delay(3000);
    ESP.restart();
  }
}

void loop() {
  CANMessage msg2515;
  while (can2515.available()) {
    if (can2515.receive(msg2515)) {
      printCan2515Message("TX1", msg2515);
      if (emulateObdService01(msg2515)) {
        continue;
      }
      forward2515to2518(msg2515);
    }
  }

  CANFDMessage msg2518;
  while (can2518.available()) {
    if (can2518.receive(msg2518)) {
      if (can2518MessageCanReachObdScanner(msg2518)) {
        printCan2518Message("RX1", msg2518);
      }
      forward2518to2515(msg2518);
    }
  }

}
