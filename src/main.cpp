#include <Arduino.h>
#include <SPI.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <BLE2902.h>
#include <ACAN2517FD.h>

// todo: fix uds messages not reaching car scanner app.

static constexpr uint8_t MCP2518_SCK = 33;
static constexpr uint8_t MCP2518_MOSI = 32;
static constexpr uint8_t MCP2518_MISO = 35;
static constexpr uint8_t MCP2518_CS = 25;
static constexpr uint8_t MCP2518_INT = 34;

static constexpr uint32_t CAN_BITRATE = 500000UL;
static constexpr uint32_t OBD_REQUEST_ID = 0x7DF;
static constexpr uint32_t OBD_PHYS_REQ_11BIT_ID = 0x7E0;
static constexpr uint32_t OBD_RESP_11BIT_ID = 0x7E8;
static constexpr uint32_t OBD_FUNC_29BIT_ID = 0x18DB33F1;
static constexpr uint32_t OBD_PHYS_REQ_29BIT_ID = 0x18DA10F1;
static constexpr uint32_t OBD_RESP_29BIT_ID = 0x18DAF110;
static constexpr uint32_t RESPONSE_TIMEOUT_MS = 1200UL;
static constexpr bool CAN_TRANSPORT_ENABLED = false;
static constexpr uint16_t SIMULATED_CELL_COUNT = 192;  // Set to 180 for a 180-cell pack.
static_assert(SIMULATED_CELL_COUNT == 180 || SIMULATED_CELL_COUNT == 192,
              "The KIA E-GMP test profile supports 180 or 192 cells");

static constexpr char BLE_DEVICE_NAME[] = "ESP32-ELM327";
static constexpr char ELM_IDENTITY[] = "ELM327 v1.5";

// Nordic UART Service UUIDs are widely used by BLE serial apps.
static constexpr char BLE_SERVICE_UUID[] = "6E400001-B5A3-F393-E0A9-E50E24DCCA9E";
static constexpr char BLE_TX_CHAR_UUID[] = "6E400003-B5A3-F393-E0A9-E50E24DCCA9E";
static constexpr char BLE_RX_CHAR_UUID[] = "6E400002-B5A3-F393-E0A9-E50E24DCCA9E";

SPIClass spi2518(HSPI);
ACAN2517FD can2518(MCP2518_CS, spi2518, MCP2518_INT);

BLEServer* bleServer = nullptr;
BLECharacteristic* bleTxCharacteristic = nullptr;
BLEAdvertising* bleAdvertising = nullptr;
bool bleClientConnected = false;

struct ElmState {
  bool echo = false;
  bool lineFeeds = false;
  bool spaces = true;
  bool headers = false;
  bool canAutoFormatting = true;
  bool allowLongMessages = false;
  bool memoryEnabled = false;
  uint8_t adaptiveTiming = 1;
  uint8_t timeoutUnits = 0x32;
  uint32_t headerId = OBD_REQUEST_ID;
  bool useExtendedHeader = false;
} elm;

String commandBuffer;

static void resetElmState();
static void writeElmPrompt();
static void writeElmInlinePrompt();
static void handleElmCommand(const String& rawCommand);

static void logLine(const String& message) {
  Serial.println(message);
}

static String visualizeControlChars(const String& value) {
  String out;
  out.reserve(value.length() * 2);
  for (size_t i = 0; i < value.length(); i++) {
    const char c = value[i];
    if (c == '\r') {
      out += "<CR>";
    } else if (c == '\n') {
      out += "<LF>";
    } else {
      out += c;
    }
  }
  return out;
}

static void writeBlePayload(const String& value) {
  Serial.print("BLE TX [headers=");
  Serial.print(elm.headers ? "1" : "0");
  Serial.print(" spaces=");
  Serial.print(elm.spaces ? "1" : "0");
  Serial.print(" lineFeeds=");
  Serial.print(elm.lineFeeds ? "1" : "0");
  Serial.print("] : ");
  Serial.println(visualizeControlChars(value));

  if (!bleClientConnected || bleTxCharacteristic == nullptr || value.isEmpty()) {
    return;
  }

  const size_t maxChunkSize = 20;
  for (size_t offset = 0; offset < value.length(); offset += maxChunkSize) {
    const String chunk = value.substring(offset, offset + maxChunkSize);
    bleTxCharacteristic->setValue(
        reinterpret_cast<const uint8_t*>(chunk.c_str()),
        chunk.length());
    bleTxCharacteristic->notify();
    delay(50);
  }
}

static void writeElmRaw(const String& value) {
  writeBlePayload(value);
}

static uint16_t getElmPromptDelayMs() {
  switch (elm.adaptiveTiming) {
    case 0:
      return 35;
    case 2:
      return 10;
    case 1:
    default:
      return 20;
  }
}

static void writeElmPrompt() {
  if (elm.lineFeeds) {
    writeBlePayload("\r\n>");
  } else {
    writeBlePayload("\r>");
  }
}

static void writeElmInlinePrompt() {
  writeBlePayload(elm.lineFeeds ? "\r\n>" : "\r>");
}

static void writeElmResponse(const String& value) {
  String response = value;
  response += elm.lineFeeds ? "\r\n>" : "\r>";
  writeBlePayload(response);
}

static void writeElmOk() {
  writeElmResponse("OK");
}

static void writeElmNoData() {
  writeElmResponse("NO DATA");
}

static void writeElmError() {
  writeElmResponse("?");
}

static String trimAndUpper(const String& input) {
  String out = input;
  out.trim();
  out.replace(" ", "");
  out.toUpperCase();
  return out;
}

static bool parseHexNibble(char c, uint8_t& value) {
  if (c >= '0' && c <= '9') {
    value = static_cast<uint8_t>(c - '0');
    return true;
  }
  if (c >= 'A' && c <= 'F') {
    value = static_cast<uint8_t>(10 + c - 'A');
    return true;
  }
  return false;
}

static bool parseHexByte(const String& text, const uint16_t index, uint8_t& value) {
  if (index + 1 >= text.length()) {
    return false;
  }

  uint8_t high = 0;
  uint8_t low = 0;
  if (!parseHexNibble(text[index], high) || !parseHexNibble(text[index + 1], low)) {
    return false;
  }

  value = static_cast<uint8_t>((high << 4) | low);
  return true;
}

static bool parseHexUint32(const String& text, uint32_t& value) {
  value = 0;
  if (text.isEmpty() || text.length() > 8) {
    return false;
  }

  for (uint16_t i = 0; i < text.length(); i++) {
    uint8_t nibble = 0;
    if (!parseHexNibble(text[i], nibble)) {
      return false;
    }
    value = (value << 4) | nibble;
  }
  return true;
}

static String formatHexByte(const uint8_t value) {
  char buffer[3];
  snprintf(buffer, sizeof(buffer), "%02X", value);
  return String(buffer);
}

static String formatHexId(const uint32_t id, const bool extended) {
  char buffer[9];
  if (extended) {
    snprintf(buffer, sizeof(buffer), "%08lX", static_cast<unsigned long>(id));
  } else {
    snprintf(buffer, sizeof(buffer), "%03lX", static_cast<unsigned long>(id));
  }
  return String(buffer);
}

static String formatCanFrameForLog(const CANFDMessage& message) {
  String output = formatHexId(message.id, message.ext);
  output += " [";
  output += String(message.len);
  output += "] ";

  for (uint8_t i = 0; i < message.len; i++) {
    output += formatHexByte(message.data[i]);
    if (i + 1 != message.len) {
      output += ' ';
    }
  }

  return output;
}

static bool initCanController() {
  spi2518.begin(MCP2518_SCK, MCP2518_MISO, MCP2518_MOSI);

  ACAN2517FDSettings settings(
      ACAN2517FDSettings::OSC_40MHz,
      CAN_BITRATE,
      DataBitRateFactor::x4);
  settings.mRequestedMode = ACAN2517FDSettings::NormalFD;

  const uint32_t errorCode = can2518.begin(settings, [] { can2518.isr(); });
  can2518.poll();

  if (errorCode != 0) {
    Serial.print("MCP2518FD init error: 0x");
    Serial.println(static_cast<uint16_t>(errorCode), HEX);
    return false;
  }

  logLine("MCP2518FD initialized");
  return true;
}

static void resetElmState() {
  elm.echo = false;
  elm.lineFeeds = false;
  elm.spaces = true;
  elm.headers = false;
  elm.canAutoFormatting = true;
  elm.allowLongMessages = false;
  elm.memoryEnabled = false;
  elm.adaptiveTiming = 1;
  elm.timeoutUnits = 0x32;
  elm.headerId = OBD_REQUEST_ID;
  elm.useExtendedHeader = false; // Always present as ISO 15765-4 CAN 11/500.
}

static void flushCanRxQueue() {
  CANFDMessage discard;
  while (can2518.available()) {
    can2518.receive(discard);
  }
}

static bool isExpectedResponseId(const CANFDMessage& request, const CANFDMessage& response) {
  if (response.ext != request.ext) {
    return false;
  }

  if (!request.ext) {
    if (request.id == OBD_REQUEST_ID) {
      return response.id >= 0x7E8 && response.id <= 0x7EF;
    }

    // Physical 11-bit UDS/OBD addressing typically responds at request ID + 8.
    return response.id == (request.id + 0x08U);
  }

  // 29-bit physical reply usually swaps tester/ECU bytes, but as a fallback
  // accept any nearby extended ID if the app selected an extended header.
  return response.id != request.id;
}

static bool doesResponseMatchRequest(
    const CANFDMessage& request,
    const CANFDMessage& response) {
  if (response.len == 0) {
    return false;
  }

  const uint8_t pciType = response.data[0] >> 4;
  uint8_t payloadOffset = 1;
  uint8_t payloadLength = 0;

  if (pciType == 0x0U) {
    payloadLength = response.data[0] & 0x0F;
    payloadOffset = 1;
  } else if (pciType == 0x1U) {
    payloadLength = response.len >= 2 ? static_cast<uint8_t>(response.len - 2) : 0;
    payloadOffset = 2;
  } else {
    return false;
  }

  if (payloadLength == 0 || response.len < payloadOffset + 1) {
    return false;
  }

  const uint8_t requestService = request.data[1];
  const uint8_t responseService = response.data[payloadOffset];

  if (responseService == 0x7F) {
    return payloadLength >= 2 &&
           response.len >= payloadOffset + 2 &&
           response.data[payloadOffset + 1] == requestService;
  }

  if (responseService != static_cast<uint8_t>(requestService + 0x40U)) {
    return false;
  }

  if (request.data[0] >= 2) {
    if (payloadLength < 2 || response.len < payloadOffset + 2) {
      return false;
    }
    if (response.data[payloadOffset + 1] != request.data[2]) {
      return false;
    }
  }

  if (request.data[0] >= 3) {
    if (payloadLength < 3 || response.len < payloadOffset + 3) {
      return false;
    }
    if (response.data[payloadOffset + 2] != request.data[3]) {
      return false;
    }
  }

  return true;
}

static bool buildObdRequest(const String& command, CANFDMessage& request) {
  if ((command.length() % 2) != 0 || command.length() < 2 || command.length() > 14) {
    return false;
  }

  const uint8_t payloadLength = static_cast<uint8_t>(command.length() / 2);
  if (payloadLength > 7) {
    return false;
  }

  request.id = elm.headerId;
  request.ext = elm.useExtendedHeader;
  request.len = 8;
  request.idx = 0;
  request.type = CANFDMessage::CAN_DATA;
  request.data[0] = payloadLength;

  for (uint8_t i = 0; i < payloadLength; i++) {
    if (!parseHexByte(command, i * 2, request.data[i + 1])) {
      return false;
    }
  }

  for (uint8_t i = payloadLength + 1; i < 8; i++) {
    request.data[i] = 0x00;
  }

  return true;
}

static uint32_t getIsoTpFlowControlId(
    const CANFDMessage& request,
    const CANFDMessage& firstFrame) {
  if (request.ext != firstFrame.ext) {
    return request.id;
  }

  if (!request.ext) {
    if (request.id == OBD_REQUEST_ID &&
        firstFrame.id >= 0x7E8 && firstFrame.id <= 0x7EF) {
      return firstFrame.id - 0x08U;
    }
    return request.id;
  }

  return request.id;
}

static bool sendIsoTpFlowControl(
    const CANFDMessage& request,
    const CANFDMessage& firstFrame) {
  CANFDMessage flowControl;
  flowControl.id = getIsoTpFlowControlId(request, firstFrame);
  flowControl.ext = request.ext;
  flowControl.len = 8;
  flowControl.idx = 0;
  flowControl.type = CANFDMessage::CAN_DATA;
  flowControl.data[0] = 0x30;

  for (uint8_t i = 1; i < flowControl.len; i++) {
    flowControl.data[i] = 0x00;
  }

  logLine("TX MCP2518FD: " + formatCanFrameForLog(flowControl));
  return can2518.tryToSend(flowControl);
}

static String formatObdPayload(const CANFDMessage& response) {
  if (response.len == 0) {
    return "";
  }

  const uint8_t pciLength = response.data[0];
  if (pciLength == 0 || pciLength > 7 || response.len < pciLength + 1) {
    return "";
  }

  String output;
  if (elm.headers) {
    output += formatHexId(response.id, response.ext);
    output += ' ';

    // With CAN headers enabled, ELM-compatible adapters include the full
    // single-frame payload, including the leading PCI length byte.
    for (uint8_t i = 0; i <= pciLength; i++) {
      output += formatHexByte(response.data[i]);
      if (elm.spaces && i != pciLength) {
        output += ' ';
      }
    }
    return output;
  }

  for (uint8_t i = 1; i <= pciLength; i++) {
    output += formatHexByte(response.data[i]);
    if (elm.spaces && i != pciLength) {
      output += ' ';
    }
  }

  return output;
}

static String formatElmFrameWithHeader(const CANFDMessage& frame) {
  String output = formatHexId(frame.id, frame.ext);
  if (elm.spaces) {
    output += ' ';
  }

  for (uint8_t i = 0; i < frame.len; i++) {
    output += formatHexByte(frame.data[i]);
    if (elm.spaces && (i + 1) != frame.len) {
      output += ' ';
    }
  }

  return output;
}

static String formatIsoTpPayload(
    const uint32_t responseId,
    const bool extended,
    const uint8_t* payload,
    const size_t payloadLength) {
  if (payload == nullptr || payloadLength == 0) {
    return "";
  }

  String output;
  if (elm.headers) {
    output += formatHexId(responseId, extended);
    if (elm.spaces) {
      output += ' ';
    }
  }

  for (size_t i = 0; i < payloadLength; i++) {
    output += formatHexByte(payload[i]);
    if (elm.spaces && (i + 1) != payloadLength) {
      output += ' ';
    }
  }

  return output;
}

static void logReceivedCanFrame(const CANFDMessage& response) {
  if (!response.ext && (response.id < 0x7E0U || response.id > 0x7EFU)) {
    return;
  }

  logLine("RX MCP2518FD: " + formatCanFrameForLog(response));
}

static bool buildBuiltinObdResponse(const CANFDMessage& request, CANFDMessage& response) {
  if (request.len < 3 || request.data[1] != 0x01) {
    return false;
  }

  const bool is11Bit = !request.ext &&
      (request.id == OBD_REQUEST_ID || request.id == OBD_PHYS_REQ_11BIT_ID);
  const bool is29Bit = request.ext &&
      (request.id == OBD_FUNC_29BIT_ID || request.id == OBD_PHYS_REQ_29BIT_ID);

  if (!is11Bit && !is29Bit) {
    return false;
  }

  response.id = is11Bit ? OBD_RESP_11BIT_ID : OBD_RESP_29BIT_ID;
  response.ext = is29Bit;
  response.idx = 0;
  response.type = CANFDMessage::CAN_DATA;
  response.len = 8;
  for (uint8_t i = 0; i < 8; i++) {
    response.data[i] = 0x00;
  }

  const uint8_t pid = request.data[2];
  response.data[1] = 0x41;
  response.data[2] = pid;

  switch (pid) {
    case 0x00:
      response.data[0] = 0x06;
      response.data[3] = 0x18;
      response.data[4] = 0x18;
      response.data[5] = 0x80;
      response.data[6] = 0x13;
      return true;

    case 0x20:
    case 0x40:
    case 0x60:
    case 0x80:
    case 0xA0:
    case 0xC0:
      response.data[0] = 0x06;
      response.data[2] = pid;
      response.data[3] = 0x00;
      response.data[4] = 0x00;
      response.data[5] = 0x00;
      response.data[6] = 0x00;
      return true;

    case 0x05:
      response.data[0] = 0x03;
      response.data[3] = 0x5A; // 50 C coolant temp
      return true;

    case 0x0C:
      response.data[0] = 0x04;
      response.data[3] = 0x0B;
      response.data[4] = 0xB8; // 750 rpm
      return true;

    case 0x0D:
      response.data[0] = 0x03;
      response.data[3] = 0x00; // 0 km/h
      return true;

    case 0x11:
      response.data[0] = 0x03;
      response.data[3] = 0x00; // 0% throttle
      return true;

    default:
      return false;
  }
}

static bool buildSyntheticResponse(const CANFDMessage& request, CANFDMessage& response) {
  if (request.ext || request.id != 0x7E4 || request.len < 3) {
    return false;
  }

  response.id = 0x7EC;
  response.ext = false;
  response.idx = 0;
  response.type = CANFDMessage::CAN_DATA;
  response.len = 8;
  for (uint8_t i = 0; i < 8; i++) {
    response.data[i] = 0x00;
  }

  const uint8_t service = request.data[1];

  if (service == 0x10 && request.data[2] == 0x90) {
    response.data[0] = 0x06;
    response.data[1] = 0x50;
    response.data[2] = 0x90;
    response.data[3] = 0x00;
    response.data[4] = 0x32;
    response.data[5] = 0x01;
    response.data[6] = 0xF4;
    return true;
  }

  if (service == 0x10 && request.data[2] == 0x03) {
    response.data[0] = 0x06;
    response.data[1] = 0x50;
    response.data[2] = 0x03;
    response.data[3] = 0x00;
    response.data[4] = 0x32;
    response.data[5] = 0x01;
    response.data[6] = 0xF4;
    return true;
  }

  if (service == 0x18 && request.data[2] == 0x00 && request.data[3] == 0xFF &&
      request.data[4] == 0x00) {
    response.data[0] = 0x04;
    response.data[1] = 0x58;
    response.data[2] = 0x00;
    response.data[3] = 0xFF;
    response.data[4] = 0x00;
    return true;
  }

  if (service == 0x1A && request.data[2] == 0x80) {
    response.data[0] = 0x06;
    response.data[1] = 0x5A;
    response.data[2] = 0x80;
    response.data[3] = 'E';
    response.data[4] = 'S';
    response.data[5] = 'P';
    response.data[6] = '3';
    return true;
  }

  if (service == 0x1A && request.data[2] == 0x8D) {
    response.data[0] = 0x06;
    response.data[1] = 0x5A;
    response.data[2] = 0x8D;
    response.data[3] = 'D';
    response.data[4] = 'O';
    response.data[5] = 'N';
    response.data[6] = 'G';
    return true;
  }

  if (service == 0x1A && request.data[2] == 0x8E) {
    response.data[0] = 0x06;
    response.data[1] = 0x5A;
    response.data[2] = 0x8E;
    response.data[3] = 'L';
    response.data[4] = 'E';
    response.data[5] = ' ';
    response.data[6] = '1';
    return true;
  }

  if (service == 0x1A && request.data[2] == 0x91) {
    response.data[0] = 0x06;
    response.data[1] = 0x5A;
    response.data[2] = 0x91;
    response.data[3] = 'V';
    response.data[4] = '1';
    response.data[5] = '.';
    response.data[6] = '0';
    return true;
  }

  if (service == 0x19 && request.data[2] == 0x02 && request.data[3] == 0xAC) {
    response.data[0] = 0x04;
    response.data[1] = 0x59;
    response.data[2] = 0x02;
    response.data[3] = 0xFF;
    response.data[4] = 0x00;
    return true;
  }

  return false;
}

static bool buildSyntheticIsoTpResponse(const CANFDMessage& request, String& payload) {
  if (request.ext || request.len < 4) {
    return false;
  }

  const bool supportedRequestId = request.id == OBD_REQUEST_ID ||
      (request.id >= 0x7E0U && request.id <= 0x7E7U);
  if (!supportedRequestId) {
    return false;
  }

  if (request.data[1] != 0x22 || request.data[2] != 0x01) {
    return false;
  }

  const uint8_t did = request.data[3];
  const bool isCellVoltageDid = did == 0x02 || did == 0x03 || did == 0x04 ||
      did == 0x0A || did == 0x0B || did == 0x0C;
  if (isCellVoltageDid) {
    // Match the battery definition's intended UDS value layout: four metadata
    // bytes followed by 32 cell bytes, each scaled by 20 mV.
    static constexpr size_t CELL_COUNT_PER_DID = 32;
    static constexpr size_t UDS_HEADER_LENGTH = 3;
    static constexpr size_t UDS_METADATA_LENGTH = 4;
    static constexpr size_t UDS_PAYLOAD_LENGTH =
        UDS_HEADER_LENGTH + UDS_METADATA_LENGTH + CELL_COUNT_PER_DID;
    uint8_t udsPayload[UDS_PAYLOAD_LENGTH] = {0x62, 0x01, did, 0xFF, 0xFF, 0xFF, 0xFF};

    const uint16_t groupOffset = did == 0x02 ? 0 : did == 0x03 ? 32 : did == 0x04 ? 64 :
        did == 0x0A ? 96 : did == 0x0B ? 128 : 160;
    const uint8_t groupIndex = static_cast<uint8_t>(groupOffset / CELL_COUNT_PER_DID);
    const size_t simulatedInGroup = groupOffset >= SIMULATED_CELL_COUNT
        ? 0
        : min(CELL_COUNT_PER_DID, static_cast<size_t>(SIMULATED_CELL_COUNT - groupOffset));
    logLine("Simulated DID 01" + formatHexByte(did) + " cells " +
            String(groupOffset + 1) + "-" + String(groupOffset + CELL_COUNT_PER_DID) +
            " (" + String(simulatedInGroup) + " present)");

    for (size_t i = 0; i < CELL_COUNT_PER_DID; ++i) {
      const uint16_t cellNumber = groupOffset + i;
      if (cellNumber >= SIMULATED_CELL_COUNT) {
        udsPayload[UDS_HEADER_LENGTH + UDS_METADATA_LENGTH + i] = 0;
        continue;
      }

      // Group-specific bands reveal missing DID responses; the small repeating
      // variation distinguishes cells within each 32-cell range.
      const uint8_t variation = static_cast<uint8_t>((i * 3U + groupIndex) % 5U);
      udsPayload[UDS_HEADER_LENGTH + UDS_METADATA_LENGTH + i] =
          static_cast<uint8_t>(183U + groupIndex * 5U + variation);
    }

    CANFDMessage responseFrames[6] = {};
    size_t responseFrameCount = 1;
    for (size_t i = 0; i < 6; ++i) {
      responseFrames[0].data[i + 2] = udsPayload[i];
    }
    responseFrames[0].data[0] = static_cast<uint8_t>(0x10U | ((UDS_PAYLOAD_LENGTH >> 8) & 0x0FU));
    responseFrames[0].data[1] = static_cast<uint8_t>(UDS_PAYLOAD_LENGTH & 0xFFU);
    responseFrames[0].id = 0x7EC;
    responseFrames[0].ext = false;
    responseFrames[0].len = 8;
    responseFrames[0].idx = 0;
    responseFrames[0].type = CANFDMessage::CAN_DATA;

    size_t copied = 6;
    uint8_t sequence = 1;
    while (copied < UDS_PAYLOAD_LENGTH) {
      CANFDMessage& frame = responseFrames[responseFrameCount++];
      frame.id = 0x7EC;
      frame.ext = false;
      frame.len = 8;
      frame.idx = 0;
      frame.type = CANFDMessage::CAN_DATA;
      frame.data[0] = static_cast<uint8_t>(0x20U | sequence);
      const size_t count = min(UDS_PAYLOAD_LENGTH - copied, static_cast<size_t>(7));
      memcpy(&frame.data[1], &udsPayload[copied], count);
      copied += count;
      sequence = static_cast<uint8_t>((sequence + 1U) & 0x0FU);
    }

    for (size_t i = 0; i < responseFrameCount; ++i) {
      CANFDMessage& frame = responseFrames[i];
      logLine("Simulated UDS cell response: " + formatCanFrameForLog(frame));
      if (elm.headers) {
        if (!payload.isEmpty()) {
          payload += elm.lineFeeds ? "\r\n" : "\r";
        }
        payload += formatElmFrameWithHeader(frame);
      }
    }
    if (!elm.headers) {
      payload = formatIsoTpPayload(0x7EC, false, udsPayload, UDS_PAYLOAD_LENGTH);
    }
    return !payload.isEmpty();
  }

  const uint8_t (*frames)[8] = nullptr;
  size_t frameCount = 0;

  static const uint8_t frames_220101[][8] = {
      {0x10, 0x3E, 0x62, 0x01, 0x01, 0xEF, 0xFB, 0xE7},
      {0x21, 0xEF, 0x8E, 0x00, 0x00, 0x00, 0x00, 0x00},
      {0x22, 0x00, 0x01, 0xBA, 0xF0, 0xE, 0x0B, 0x0C},
      {0x23, 0x0D, 0x0D, 0x0D, 0x0D, 0x00, 0x7F, 0xC6},
      {0x24, 0xAC, 0xC3, 0xA2, 0x00, 0x00, 0x78, 0x00},
      {0x25, 0x01, 0x11, 0xE9, 0x00, 0x01, 0x0D, 0x11},
      {0x26, 0x00, 0x00, 0xBF, 0xF0, 0x00, 0x00, 0xB5},
      {0x27, 0x33, 0x00, 0x5F, 0xC7, 0x77, 0x00, 0x19},
      {0x28, 0x99, 0x7F, 0xFF, 0x00, 0x00, 0x0B, 0xB8}
  };

  static const uint8_t frames_220102[][8] = {
      {0x10, 0x22, 0x62, 0x01, 0x02, 0xFF, 0xFF, 0xFF},
      {0x21, 0xFF, 0xC4, 0xC4, 0xC4, 0xC4, 0xC4, 0xC4},
      {0x22, 0xC4, 0xC4, 0xC4, 0xC4, 0xC4, 0xC4, 0xC4},
      {0x23, 0xC4, 0xC4, 0xC4, 0xC4, 0xC4, 0xC4, 0xC4},
      {0x24, 0xC4, 0xC4, 0xC4, 0xC4, 0xC4, 0xC4, 0xC4},
      {0x25, 0xC4, 0xC4, 0xC4, 0xC4, 0xC4, 0xAA, 0xAA}
  };

  static const uint8_t frames_220103[][8] = {
      {0x10, 0x27, 0x62, 0x01, 0x03, 0xFF, 0xFF, 0xFF},
      {0x21, 0xFF, 0xC4, 0xC4, 0xC4, 0xC4, 0xC4, 0xC4},
      {0x22, 0xC4, 0xC4, 0xC4, 0xC4, 0xC4, 0xC4, 0xC4},
      {0x23, 0xC4, 0xC4, 0xC4, 0xC4, 0xC4, 0xC4, 0xC4},
      {0x24, 0xC4, 0xC4, 0xC4, 0xC4, 0xC4, 0xC4, 0xC4},
      {0x25, 0xC4, 0xC4, 0xC4, 0xC4, 0xC4, 0xAA, 0xAA}
  };

  static const uint8_t frames_220104[][8] = {
      {0x10, 0x27, 0x62, 0x01, 0x04, 0xFF, 0xFF, 0xFF},
      {0x21, 0xFF, 0xC4, 0xC4, 0xC4, 0xC4, 0xC4, 0xC4},
      {0x22, 0xC4, 0xC4, 0xC4, 0xC4, 0xC4, 0xC4, 0xC4},
      {0x23, 0xC4, 0xC4, 0xC4, 0xC4, 0xC4, 0xC4, 0xC4},
      {0x24, 0xC4, 0xC4, 0xC4, 0xC4, 0xC4, 0xC4, 0xC4},
      {0x25, 0xC4, 0xC4, 0xC4, 0xC4, 0xC4, 0xAA, 0xAA}
  };

  static const uint8_t frames_220105[][8] = {
      {0x10, 0x2E, 0x62, 0x01, 0x05, 0xFF, 0xFB, 0x74},
      {0x21, 0x0F, 0x01, 0x2C, 0x01, 0x01, 0x2C, 0x0D},
      {0x22, 0x0C, 0x0C, 0x0B, 0x0C, 0x0C, 0x0D, 0x57},
      {0x23, 0xED, 0x62, 0xD4, 0x02, 0x00, 0x64, 0xEF},
      {0x24, 0x00, 0x03, 0xE6, 0x85, 0x59, 0xA6, 0x00},
      {0x25, 0x91, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
      {0x26, 0x00, 0xC0, 0xE0, 0xD0, 0xDA, 0xAA, 0xAA}
  };

  static const uint8_t frames_22010A[][8] = {
      {0x10, 0x22, 0x62, 0x01, 0x0A, 0xFF, 0xFF, 0xFF},
      {0x21, 0xFF, 0xC4, 0xC4, 0xC4, 0xC4, 0xC4, 0xC4},
      {0x22, 0xC4, 0xC4, 0xC4, 0xC4, 0xC4, 0xC4, 0xC4},
      {0x23, 0xC4, 0xC4, 0xC4, 0xC4, 0xC5, 0xC5, 0xC4},
      {0x24, 0xC5, 0xC4, 0xC5, 0xC4, 0xC4, 0xC4, 0xAA}
  };

  static const uint8_t frames_22010B[][8] = {
      {0x10, 0x22, 0x62, 0x01, 0x0B, 0xFF, 0xFF, 0xFF},
      {0x21, 0xFF, 0xC5, 0xC4, 0xC4, 0xC4, 0xC4, 0xC4},
      {0x22, 0xC4, 0xC4, 0xC4, 0xC4, 0xC4, 0xC4, 0xC4},
      {0x23, 0xC4, 0xC4, 0xC4, 0xC4, 0xC4, 0xC4, 0xC4},
      {0x24, 0xC4, 0xC4, 0xC4, 0xC4, 0xC4, 0xC4, 0xC4},
      {0x25, 0xC4, 0xC3, 0xC3, 0xC3, 0xC3, 0xAA, 0xAA}
  };

  static const uint8_t frames_22010C[][8] = {
      {0x10, 0x27, 0x62, 0x01, 0x0C, 0xFF, 0xFF, 0xF0},
      {0x21, 0x00, 0xC3, 0xC3, 0xC3, 0xC3, 0xC3, 0xC3},
      {0x22, 0xC3, 0xC3, 0xC5, 0xC5, 0xC6, 0xC6, 0xC6},
      {0x23, 0xC5, 0xC6, 0xC6, 0xC6, 0xC6, 0xC5, 0xC6},
      {0x24, 0x03, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00},
      {0x25, 0x01, 0x00, 0x0E, 0x0E, 0x01, 0xAA, 0xAA}
  };

  switch (did) {
    case 0x01:
      frames = frames_220101;
      frameCount = sizeof(frames_220101) / sizeof(frames_220101[0]);
      break;
    case 0x02:
      frames = frames_220102;
      frameCount = sizeof(frames_220102) / sizeof(frames_220102[0]);
      break;
    case 0x03:
      frames = frames_220103;
      frameCount = sizeof(frames_220103) / sizeof(frames_220103[0]);
      break;
    case 0x04:
      frames = frames_220104;
      frameCount = sizeof(frames_220104) / sizeof(frames_220104[0]);
      break;
    case 0x05:
      frames = frames_220105;
      frameCount = sizeof(frames_220105) / sizeof(frames_220105[0]);
      break;
    case 0x0A:
      frames = frames_22010A;
      frameCount = sizeof(frames_22010A) / sizeof(frames_22010A[0]);
      break;
    case 0x0B:
      frames = frames_22010B;
      frameCount = sizeof(frames_22010B) / sizeof(frames_22010B[0]);
      break;
    case 0x0C:
      frames = frames_22010C;
      frameCount = sizeof(frames_22010C) / sizeof(frames_22010C[0]);
      break;
    default:
      return false;
  }

  for (size_t index = 0; index < frameCount; ++index) {
    CANFDMessage response;
    response.id = 0x7EC;
    response.ext = false;
    response.idx = 0;
    response.type = CANFDMessage::CAN_DATA;
    response.len = 8;
    memcpy(response.data, frames[index], 8);

    logLine("RX MCP2518FD: " + formatCanFrameForLog(response));

    if (index > 0) {
      payload += elm.lineFeeds ? "\r\n" : "\r";
    }

    if (elm.headers) {
      payload += formatElmFrameWithHeader(response);
    } else {
      if (index == 0) {
        payload += formatIsoTpPayload(response.id, response.ext, &response.data[2], response.len - 2);
      } else {
        payload += formatIsoTpPayload(response.id, response.ext, &response.data[1], response.len - 1);
      }
    }
  }

  return !payload.isEmpty();
}

static bool transactObdRequest(const String& command) {
  CANFDMessage request;
  if (!buildObdRequest(command, request)) {
    return false;
  }

  CANFDMessage syntheticResponse;
  if (buildBuiltinObdResponse(request, syntheticResponse)) {
    logLine("Built-in OBD reply: " + formatCanFrameForLog(syntheticResponse));
    const String payload = formatObdPayload(syntheticResponse);
    writeElmResponse(payload);
    return true;
  }

  if (!CAN_TRANSPORT_ENABLED) {
    String replayPayload;
    if (buildSyntheticIsoTpResponse(request, replayPayload)) {
      logLine("Replayed ECU response for " + command + ": " + replayPayload);
      writeElmResponse(replayPayload);
      return true;
    }

    if (buildSyntheticResponse(request, syntheticResponse)) {
      logLine("Replayed ECU response: " + formatCanFrameForLog(syntheticResponse));
      writeElmResponse(formatObdPayload(syntheticResponse));
      return true;
    }

    logLine("No replay data for " + command);
    writeElmNoData();
    return true;
  }

  flushCanRxQueue();
  logLine("TX MCP2518FD: " + formatCanFrameForLog(request));

  if (!can2518.tryToSend(request)) {
    String syntheticPayload;
    if (buildSyntheticIsoTpResponse(request, syntheticPayload)) {
      writeElmResponse(syntheticPayload);
      return true;
    }

    if (buildSyntheticResponse(request, syntheticResponse)) {
      logLine("Synthetic ECU reply: " + formatCanFrameForLog(syntheticResponse));
      const String payload = formatObdPayload(syntheticResponse);
      writeElmResponse(payload);
      return true;
    }

    logLine("Failed to send OBD request to MCP2518FD");
    writeElmResponse("BUFFER FULL");
    return true;
  }

  const uint32_t timeoutMs = static_cast<uint32_t>(elm.timeoutUnits) * 4U;
  uint32_t responseDeadline = millis() + timeoutMs;
  CANFDMessage response;
  uint8_t isoTpPayload[512];
  String isoTpHeaderFrames;
  String isoTpFrameData;

  while (static_cast<int32_t>(millis() - responseDeadline) < 0) {
    can2518.poll();

    if (!can2518.available()) {
      delay(2);
      continue;
    }

    if (!can2518.receive(response)) {
      continue;
    }

    logReceivedCanFrame(response);

    if (!isExpectedResponseId(request, response)) {
      continue;
    }

    const uint8_t pciType = response.data[0] >> 4;
    if ((pciType == 0x0U || pciType == 0x1U) &&
        !doesResponseMatchRequest(request, response)) {
      continue;
    }

    responseDeadline = millis() + timeoutMs;

    if (pciType == 0x1U) {
      if (response.len < 3) {
        writeElmNoData();
        return true;
      }

      size_t totalPayloadLength =
          (static_cast<size_t>(response.data[0] & 0x0F) << 8) |
          static_cast<size_t>(response.data[1]);
      if (totalPayloadLength == 0 || totalPayloadLength > sizeof(isoTpPayload)) {
        writeElmNoData();
        return true;
      }

      size_t receivedPayloadLength = min(
          totalPayloadLength,
          static_cast<size_t>(response.len - 2));
      memcpy(isoTpPayload, &response.data[2], receivedPayloadLength);
      bool strippedUdsHeader = false;
      if (!elm.headers && receivedPayloadLength >= 3 && isoTpPayload[0] == 0x62) {
        // Strip UDS response header: 0x62 + 2-byte DID
        totalPayloadLength -= 3;
        receivedPayloadLength -= 3;
        memmove(isoTpPayload, &isoTpPayload[3], receivedPayloadLength);
        strippedUdsHeader = true;
      }
      if (elm.headers) {
        isoTpHeaderFrames = formatElmFrameWithHeader(response);
      } else {
        if (strippedUdsHeader) {
          isoTpFrameData = formatIsoTpPayload(
              response.id,
              response.ext,
              &response.data[5],
              response.len - 5);
        } else {
          isoTpFrameData = formatIsoTpPayload(
              response.id,
              response.ext,
              &response.data[2],
              response.len - 2);
        }
      }

      if (!sendIsoTpFlowControl(request, response)) {
        writeElmResponse("BUFFER FULL");
        return true;
      }

      uint8_t expectedSequence = 1;
      while (receivedPayloadLength < totalPayloadLength &&
             static_cast<int32_t>(millis() - responseDeadline) < 0) {
        can2518.poll();

        if (!can2518.available()) {
          delay(2);
          continue;
        }

        CANFDMessage consecutiveFrame;
        if (!can2518.receive(consecutiveFrame)) {
          continue;
        }

        logReceivedCanFrame(consecutiveFrame);

        if (!isExpectedResponseId(request, consecutiveFrame)) {
          continue;
        }

        if (consecutiveFrame.len < 2) {
          continue;
        }

        const uint8_t consecutivePciType = consecutiveFrame.data[0] >> 4;
        if (consecutivePciType != 0x2U) {
          continue;
        }

        const uint8_t sequenceNumber = consecutiveFrame.data[0] & 0x0F;
        if (sequenceNumber != expectedSequence) {
          writeElmNoData();
          return true;
        }

        const size_t bytesRemaining = totalPayloadLength - receivedPayloadLength;
        const size_t bytesAvailable = consecutiveFrame.len - 1;
        const size_t bytesToCopy = min(bytesRemaining, bytesAvailable);
        memcpy(
            &isoTpPayload[receivedPayloadLength],
            &consecutiveFrame.data[1],
            bytesToCopy);
        receivedPayloadLength += bytesToCopy;
        if (elm.headers) {
          isoTpHeaderFrames += elm.lineFeeds ? "\r\n" : "\r";
          isoTpHeaderFrames += formatElmFrameWithHeader(consecutiveFrame);
        } else {
          isoTpFrameData += elm.lineFeeds ? "\r\n" : "\r";
          isoTpFrameData += formatIsoTpPayload(
              response.id,
              response.ext,
              &consecutiveFrame.data[1],
              consecutiveFrame.len - 1);
        }
        responseDeadline = millis() + timeoutMs;
        expectedSequence = static_cast<uint8_t>((expectedSequence + 1U) & 0x0F);
      }

      if (receivedPayloadLength != totalPayloadLength) {
        writeElmNoData();
        return true;
      }

      const String payload = elm.headers
          ? isoTpHeaderFrames
          : isoTpFrameData;
      if (payload.isEmpty()) {
        writeElmNoData();
        return true;
      }

      logLine("OBD " + command + " -> " + payload);
      writeElmResponse(payload);
      return true;
    }

    const String payload = formatObdPayload(response);
    if (payload.isEmpty()) {
      writeElmNoData();
      return true;
    }

    logLine("OBD " + command + " -> " + payload);
    writeElmResponse(payload);
    return true;
  }

  String syntheticPayload;
  if (buildSyntheticIsoTpResponse(request, syntheticPayload)) {
    writeElmResponse(syntheticPayload);
    return true;
  }

  if (buildSyntheticResponse(request, syntheticResponse)) {
    logLine("Synthetic ECU reply: " + formatCanFrameForLog(syntheticResponse));
    const String payload = formatObdPayload(syntheticResponse);
    writeElmResponse(payload);
    return true;
  }

  writeElmNoData();
  return true;
}

static bool handleAtCommand(const String& command) {
  if (command == "ATZ") {
    resetElmState();
    writeElmResponse(ELM_IDENTITY);
    return true;
  }

  if (command == "ATI") {
    writeElmResponse(ELM_IDENTITY);
    return true;
  }

  if (command == "ATD") {
    resetElmState();
    writeElmOk();
    return true;
  }

  if (command == "ATWS") {
    resetElmState();
    writeElmResponse(ELM_IDENTITY);
    return true;
  }

  if (command == "ATE0") {
    elm.echo = false;
    writeElmOk();
    return true;
  }

  if (command == "ATE1") {
    elm.echo = true;
    writeElmOk();
    return true;
  }

  if (command == "ATL0") {
    elm.lineFeeds = false;
    writeElmOk();
    return true;
  }

  if (command == "ATL1") {
    elm.lineFeeds = true;
    writeElmOk();
    return true;
  }

  if (command == "ATS0") {
    elm.spaces = false;
    writeElmOk();
    return true;
  }

  if (command == "ATS1") {
    elm.spaces = true;
    writeElmOk();
    return true;
  }

  if (command == "ATD0" || command == "ATD1") {
    writeElmOk();
    return true;
  }

  if (command == "ATH0") {
    elm.headers = false;
    elm.spaces = false;
    writeElmOk();
    return true;
  }

  if (command == "ATH1") {
    elm.headers = true;
    elm.spaces = true;
    writeElmOk();
    return true;
  }

  if (command == "ATCAF0") {
    elm.canAutoFormatting = false;
    writeElmOk();
    return true;
  }

  if (command == "ATCAF1") {
    elm.canAutoFormatting = true;
    writeElmOk();
    return true;
  }

  if (command == "ATSP0") {
    elm.useExtendedHeader = false;
    elm.headerId = OBD_REQUEST_ID;
    writeElmOk();
    return true;
  }

  if (command == "ATSP6" || command == "ATTP6") {
    elm.useExtendedHeader = false;
    elm.headerId = OBD_REQUEST_ID;
    writeElmOk();
    return true;
  }

  if (command == "ATSP7" || command == "ATTP7") {
    // Some apps try protocol 7 during probing; keep reporting/using 11-bit 500k.
    elm.useExtendedHeader = false;
    elm.headerId = OBD_REQUEST_ID;
    writeElmOk();
    return true;
  }

  if (command == "ATDP") {
    writeElmResponse("ISO 15765-4 CAN (11 bit, 500 kbaud)");
    return true;
  }

  if (command == "ATDPN") {
    writeElmResponse("A6");
    return true;
  }

  if (command.startsWith("ATDPN")) {
    writeElmResponse("A6");
    return true;
  }

  if (command.startsWith("ATSH")) {
    const String idText = command.substring(4);
    uint32_t parsedId = 0;
    if (!parseHexUint32(idText, parsedId)) {
      writeElmError();
      return true;
    }

    elm.headerId = parsedId;
    elm.useExtendedHeader = idText.length() > 3;
    writeElmOk();
    return true;
  }

  if (command == "ATPC") {
    flushCanRxQueue();
    writeElmOk();
    return true;
  }

  if (command == "ATAT0" || command == "ATAT1" || command == "ATAT2") {
    elm.adaptiveTiming = static_cast<uint8_t>(command[4] - '0');
    writeElmOk();
    return true;
  }

  if (command == "ATKW" || command == "ATAR" || command == "ATV0" ||
      command.startsWith("ATSW")) {
    writeElmOk();
    return true;
  }

  if (command == "ATM0") {
    elm.memoryEnabled = false;
    writeElmOk();
    return true;
  }

  if (command == "ATM1") {
    elm.memoryEnabled = true;
    writeElmOk();
    return true;
  }

  if (command == "ATAL") {
    elm.allowLongMessages = true;
    writeElmOk();
    return true;
  }

  if (command == "ATNL") {
    elm.allowLongMessages = false;
    writeElmOk();
    return true;
  }

  if (command == "ATBI") {
    writeElmOk();
    return true;
  }

  if (command == "ATFI") {
    writeElmOk();
    return true;
  }

  if (command.startsWith("ATCM") || command.startsWith("ATCF") ||
      command.startsWith("ATCEA") || command.startsWith("ATCFC") ||
      command.startsWith("ATFCSD") || command.startsWith("ATFCSH") ||
      command.startsWith("ATFCSM")) {
    writeElmOk();
    return true;
  }

  if (command.startsWith("ATST")) {
    const String valueText = command.substring(4);
    uint8_t value = 0;
    if (valueText.length() != 2 || !parseHexByte(valueText, 0, value)) {
      writeElmError();
      return true;
    }

    elm.timeoutUnits = value;
    writeElmOk();
    return true;
  }

  if (command == "AT@1") {
    writeElmResponse("ESP32 ELM327");
    return true;
  }

  if (command == "AT@2") {
    writeElmResponse("ESP32-ELM327");
    return true;
  }

  if (command == "ATRV") {
    writeElmResponse("12.4V");
    return true;
  }

  if (command == "AT-VER") {
    writeElmResponse("ELM327");
    return true;
  }

  if (command == "STDI") {
    writeElmResponse(BLE_DEVICE_NAME);
    return true;
  }

  if (command == "STI") {
    writeElmResponse("STN1170 v4.3.0");
    return true;
  }

  if (command == "VTI") {
    writeElmResponse("V1.0");
    return true;
  }

  return false;
}

static void handleElmCommand(const String& rawCommand) {
  const String command = trimAndUpper(rawCommand);
  if (command.isEmpty()) {
    writeElmPrompt();
    return;
  }

  logLine("ELM command: " + command);

  if (handleAtCommand(command)) {
    return;
  }

  if (!transactObdRequest(command)) {
    writeElmError();
  }
}

static void consumeElmInput(const String& inputChunk) {
  for (size_t i = 0; i < inputChunk.length(); i++) {
    const char c = inputChunk[i];

    if (c == '\r' || c == '\n') {
      if (commandBuffer.isEmpty()) {
        continue;
      }

      if (elm.echo) {
        writeElmRaw(commandBuffer);
      }

      handleElmCommand(commandBuffer);
      commandBuffer = "";
      continue;
    }

    if (commandBuffer.length() < 64) {
      commandBuffer += c;
    }
  }
}

class ElmBleServerCallbacks : public BLEServerCallbacks {
 public:
  void onConnect(BLEServer* server) override {
    bleClientConnected = true;
    resetElmState();
    commandBuffer = "";
    logLine("BLE client connected");
    writeElmPrompt();
  }

  void onDisconnect(BLEServer* server) override {
    bleClientConnected = false;
    commandBuffer = "";
    logLine("BLE client disconnected");
    delay(100);
    if (bleAdvertising != nullptr) {
      bleAdvertising->start();
      logLine("BLE advertising restarted");
    }
  }
};

class ElmBleRxCallbacks : public BLECharacteristicCallbacks {
 public:
  void onWrite(BLECharacteristic* characteristic) override {
    const String value = characteristic->getValue();
    if (value.isEmpty()) {
      return;
    }

    Serial.print("BLE RX: ");
    Serial.println(value);
    consumeElmInput(value);
  }
};

static void initBleElmAdapter() {
  BLEDevice::init(BLE_DEVICE_NAME);

  bleServer = BLEDevice::createServer();
  bleServer->setCallbacks(new ElmBleServerCallbacks());

  BLEService* service = bleServer->createService(BLE_SERVICE_UUID);

  bleTxCharacteristic = service->createCharacteristic(
      BLE_TX_CHAR_UUID,
      BLECharacteristic::PROPERTY_NOTIFY | BLECharacteristic::PROPERTY_READ);
  bleTxCharacteristic->addDescriptor(new BLE2902());

  BLECharacteristic* rxCharacteristic = service->createCharacteristic(
      BLE_RX_CHAR_UUID,
      BLECharacteristic::PROPERTY_WRITE |
          BLECharacteristic::PROPERTY_WRITE_NR);
  rxCharacteristic->setCallbacks(new ElmBleRxCallbacks());

  bleTxCharacteristic->setValue(">");
  service->start();

  bleAdvertising = BLEDevice::getAdvertising();
  bleAdvertising->addServiceUUID(BLE_SERVICE_UUID);
  bleAdvertising->setScanResponse(true);
  bleAdvertising->setMinPreferred(0x06);
  bleAdvertising->setMinPreferred(0x12);
  BLEDevice::startAdvertising();

  Serial.print("BLE ready as ");
  Serial.println(BLE_DEVICE_NAME);
}

void setup() {
  Serial.begin(115200);
  delay(300);

  resetElmState();
  initBleElmAdapter();

  if (CAN_TRANSPORT_ENABLED && !initCanController()) {
    Serial.println("CAN init failed, rebooting in 3s");
    delay(3000);
    ESP.restart();
  }
}

void loop() {
  if (CAN_TRANSPORT_ENABLED) {
    can2518.poll();
  }
  delay(1);
}
