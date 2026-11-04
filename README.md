# ESP32 CANFD OBD BLE Scanner

> Replay mode is enabled by default to serve saved battery responses without an MCP2518FD connected.

## What this does
- Exposes the ESP32 as a BLE ELM327-style dongle named `ESP32-ELM327`
- Sends OBD-II CAN requests to the vehicle through `MCP2518FD` when CAN transport is enabled
- Returns ECU replies back to the BLE client in ELM327-style text format
- Removes the old `MCP2515 <-> MCP2518FD` bridge design

## Current scope
- BLE replay mode simulates 180 or 192 cell values for UDS DIDs `0x0102`, `0x0103`, `0x0104`, and `0x010A`-`0x010C`, using the battery definition's four-byte prefix plus 32 one-byte values at 20 mV/count. Each DID group has a distinct voltage band to reveal missing responses. Set `SIMULATED_CELL_COUNT` in `src/main.cpp` to `180` or `192`; unused positions in a 180-cell pack return zero. These are test values, not live battery measurements.
- Set `CAN_TRANSPORT_ENABLED` in `src/main.cpp` to `true` to use the MCP2518FD
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
