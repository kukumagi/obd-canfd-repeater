# ESP32 OBD Bridge (MCP2515 <-> MCP2518FD)

## What this does
- Receives CAN frames from OBD scanner side on `MCP2515`
- Forwards those frames to `MCP2518FD`
- Receives frames on `MCP2518FD`
- Sends them back to OBD side through `MCP2515`

Driver usage style follows Battery-Emulator project:
- `ACAN2515` with `ACAN2515Settings`
- `ACAN2517FD` with `ACAN2517FDSettings`
- ISR lambda style: `[] { canX.isr(); }`

## Default pins (ESP32 DevKit)
- MCP2515: `SCK=22`, `MOSI=21`, `MISO=19`, `CS=18`, `INT=23`
- MCP2518FD: `SCK=33`, `MOSI(SDI)=32`, `MISO(SDO)=35`, `CS=25`, `INT=34`

## Build / upload
```bash
pio run
pio run -t upload
pio device monitor -b 115200
```

## Notes
- Bitrate defaults to `500 kbps` on both controllers.
- MCP2515 is classic CAN only; incoming CAN-FD frames from MCP2518FD are dropped.
