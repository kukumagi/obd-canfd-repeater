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
  Serial.print("BLE TX: ");
  Serial.println(visualizeControlChars(value));

  if (!bleClientConnected || bleTxCharacteristic == nullptr || value.isEmpty()) {
    return;
  }

  const size_t maxChunkSize = 180;
  for (size_t offset = 0; offset < value.length(); offset += maxChunkSize) {
    const String chunk = value.substring(offset, offset + maxChunkSize);
    bleTxCharacteristic->setValue(
        reinterpret_cast<const uint8_t*>(chunk.c_str()),
        chunk.length());
    bleTxCharacteristic->notify();
    delay(10);
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
  writeBlePayload(">");
}

static void writeElmResponse(const String& value) {
  if (elm.lineFeeds) {
    if (value.indexOf('\r') < 0) {
      writeBlePayload(value + "\r\n");
    } else {
      size_t start = 0;
      while (start < value.length()) {
        const int end = value.indexOf('\r', start);
        const String line = (end < 0)
            ? value.substring(start)
            : value.substring(start, static_cast<size_t>(end));

        if (!line.isEmpty()) {
          writeBlePayload(line + "\r\n");
        }

        if (end < 0) {
          break;
        }
        start = static_cast<size_t>(end) + 1;
      }
    }
  } else {
    if (value.indexOf('\r') < 0) {
      writeBlePayload(value + "\r");
    } else {
      size_t start = 0;
      while (start < value.length()) {
        const int end = value.indexOf('\r', start);
        const String line = (end < 0)
            ? value.substring(start)
            : value.substring(start, static_cast<size_t>(end));

        if (!line.isEmpty()) {
          writeBlePayload(line + "\r");
        }

        if (end < 0) {
          break;
        }
        start = static_cast<size_t>(end) + 1;
      }
    }
  }

  delay(getElmPromptDelayMs());
  writeElmInlinePrompt();
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
  elm.spaces = false;
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

  flushCanRxQueue();
  logLine("TX MCP2518FD: " + formatCanFrameForLog(request));

  if (!can2518.tryToSend(request)) {
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
          isoTpHeaderFrames += '\r';
          isoTpHeaderFrames += formatElmFrameWithHeader(consecutiveFrame);
        } else {
          isoTpFrameData += '\r';
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

  if (!initCanController()) {
    Serial.println("CAN init failed, rebooting in 3s");
    delay(3000);
    ESP.restart();
  }
}

void loop() {
  can2518.poll();
  delay(1);
}
