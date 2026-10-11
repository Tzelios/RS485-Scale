# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this is

Arduino Uno firmware (PlatformIO) that turns an HX711 load-cell amplifier into a Modbus RTU slave over RS485. `src/main.cpp` holds the hardware wiring (HX711, Modbus, EEPROM). Hardware-free logic (register-write handling, config validation, calibration math, weight encoding) lives in `lib/scale_logic/` so it can be unit tested on the host.

The untracked `skills/` directory is a copy of a third-party Claude skills collection, not part of the firmware.

## Commands

```bash
pio run                      # build firmware (default_envs = uno)
pio test -e native           # host unit tests for lib/scale_logic (needs g++)
pio test -e native -f test_scale_logic   # run one test folder
pio run -t upload            # flash the board
pio device monitor           # serial monitor at 9600 (monitor_speed)
pio run -t clean
```

Dependencies (`cmb27/ModbusRTUSlave`, `bogde/HX711`) are fetched automatically from `lib_deps` in `platformio.ini`. The `native` env has no Arduino framework, so anything under `lib/scale_logic/` must not include Arduino or library headers. Tests are Unity, in `test/test_scale_logic/test_main.cpp`; add new cases to its `main()` with `RUN_TEST`.

## Architecture

**Hardware pins:** HX711 DOUT=3, SCK=2; RS485 transceiver DE/RE=4. The hardware `Serial` port is the Modbus bus, so any `Serial.print` debugging corrupts Modbus traffic. Don't add it to the running firmware.

**Holding registers** (`regs[30]`, exposed through `modbus.configureHoldingRegisters`):

| Reg | Meaning |
|-----|---------|
| 0  | Weight × 10, as `int16_t` (read), clamped to the int16 range. Holds the last good value while a load-cell fault is set |
| 1  | Status bits (read): bit 0 `STATUS_LOADCELL_FAULT` = no HX711 sample for `HX_TIMEOUT_MS`; bit 1 `STATUS_CAL_REJECTED` = last calibration rejected; bit 2 `STATUS_OVERRANGE` = weight clamped |
| 12 | Command: `1` = tare, `2` = calibrate with known weight. Reads non-zero while pending and 0 when done. Unknown values, and calibrate with known weight 0, are cleared at once |
| 13 | Known calibration weight (same units as the weight output) |
| 20 | Slave address (1–247). Writing it restarts Modbus; invalid writes revert |
| 21 | Baud code: 1=2400, 2=4800, 3=9600 (`baudTable`). Writing it restarts Modbus; invalid writes revert |

**Control flow:** `loop()` never blocks. Each pass calls `modbus.poll()`, compares the registers with the `Config` copy in RAM (a master's write is applied and persisted on the next pass; there are no write callbacks), and calls `Sampler::poll()`, which takes at most one HX711 sample and only if one is ready. When `averageCount` samples have been collected (about 3 s at 30 samples and the HX711's default 10 SPS), the weight register updates and any pending command runs on that block. A newly accepted command restarts the block, so it only uses samples taken after it arrived. A load-cell fault drops a pending command.

**Load cell reads:** go through `Sampler` and `Hx711LoadCell` only. Don't call HX711's `get_units`/`tare`/`read_average`/`wait_ready`: they block (forever if the chip stops answering). Weight is computed by `rawToUnits()` from `cfg.offset`/`cfg.scaleFactor`; the HX711 object's own OFFSET/SCALE are unused.

**Persistence:** the whole `Config` struct (defined in `scale_logic.h`) is stored at EEPROM address 0 with `EEPROM.get/put`. `loadConfig()` falls back to defaults (addr 3, baud code 3, scale 1.0, known weight 4, averageCount 30, offset 0) when any field fails `isConfigValid()`. Changing the field order or types in `Config` invalidates the data already on deployed boards. If the new bytes still pass validation, the firmware loads garbage instead of the defaults.

**Calibration math:** tare saves the raw block average as the offset; tare before calibrating. Calibrate computes `scaleFactor = (raw - offset) / knownWeight` and is rejected when `|raw - offset| < MIN_CAL_COUNTS`.

**EEPROM reset utility:** the commented-out `setup()`/`loop()` at the bottom of `main.cpp` is a separate sketch that wipes EEPROM to 0xFF and dumps it. To use it, swap it in for the main firmware temporarily.
