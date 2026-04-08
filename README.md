# ESP32 CANFD OBD BLE Scanner

> Work in progress. CANbus data currently does not reach the app correctly; BLE transport and app integration remain under development.

## What this does
- Exposes the ESP32 as a BLE ELM327-style dongle named `ESP32-ELM327`
- Sends OBD-II CAN requests to the vehicle through `MCP2518FD`
- Returns ECU replies back to the BLE client in ELM327-style text format
- Removes the old `MCP2515 <-> MCP2518FD` bridge design

## Current scope
- `MCP2518FD` is the only vehicle CAN interface
- Uses a BLE UART-style service for app communication
- Supports a minimal set of ELM327 `AT` commands:
  - `ATZ`, `ATI`, `ATD`
  - `ATE0/1`, `ATL0/1`, `ATS0/1`, `ATH0/1`
  - `ATCAF0/1`, `ATSP0`, `ATSP6`, `ATSP7`, `ATDP`, `ATDPN`, `ATSH`, `ATPC`
- Supports single-frame OBD hex requests such as:
  - `0100`
  - `010C`
  - `010D`
- Waits for ECU replies on standard OBD response IDs `0x7E8` to `0x7EF`
- Reassembles ISO-TP multi-frame ECU responses before returning them to the BLE client

## Current limitations
- No automatic protocol detection beyond simple 11-bit / 29-bit mode switching
- No ISO-TP multi-frame request transmit support yet
- Header filtering is still minimal and tuned for standard OBD polling

## Default pins (ESP32 DevKit)
- MCP2518FD: `SCK=33`, `MOSI(SDI)=32`, `MISO(SDO)=35`, `CS=25`, `INT=34`

## Build / upload
```bash
pio run
pio run -t upload
pio device monitor -b 115200
```

## Notes
- CAN arbitration bitrate defaults to `500 kbps`
- BLE device name defaults to `ESP32-ELM327`
- The firmware currently targets common OBD polling apps that talk to ELM327-style serial adapters
- In the OBD scanner app, choose a connection mode that scans for `BLE` devices; do not expect the dongle to appear as classic Bluetooth
- BLE transport currently uses Nordic UART Service UUIDs
- The next major improvement should be ISO-TP multi-frame request transmit support for longer tester commands
