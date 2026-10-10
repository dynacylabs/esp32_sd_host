// Pin/board configuration for the ESP32 SD-card test harness.
//
// ROOT CAUSE FOUND 2026-10-10 (see blog/journal.md): the original (non-S3) ESP32's SDMMC driver
// slot config (sd_session.cpp's fill_slot_pins(), slot.cmd/d0-d3) is a NO-OP on this chip in this
// SDK generation - SOC_SDMMC_USE_GPIO_MATRIX is undefined for it, so the peripheral permanently
// drives its hardcoded native pins regardless of what the slot config says. Proven directly on
// hardware via dual-core GPIO self-sampling (SCOPEPIN/SCOPEIDLE in cmd_sys.cpp): the "configured"
// pins (old CMD=17, D0=25) showed zero difference between idle and a real CARDINIT attempt, while
// the chip's true native pins (CMD=15, D0=2) showed real transitions only during the real attempt.
// Every earlier "collision-avoidance" pin move this session (JTAG/RTC/I2C-default avoidance) was
// therefore moving the wires away from the only pins the peripheral actually drives - that's the
// entire reason every real command has decoded on the Tang Nano as a flat/CMD0 line all along.
//
// Fix: wire to the chip's real native pins below. CLK already happened to be right. 1-bit mode
// (SD_USE_4BIT_MODE false) only needs CLK+CMD+D0, so the physical fix is just 2 wires (CMD, D0)
// instead of 5 - D1-D3 are listed for when 4-bit mode is wired up later.
#pragma once

#define SD_PIN_CLK 14   // chip's native/hardcoded SDMMC CLK - unchanged, was already correct.
#define SD_PIN_CMD 15   // chip's native/hardcoded SDMMC CMD (ALSO JTAG TDO - fine with no debug
                         // probe attached; proven by direct GPIO self-sampling to be the pin the
                         // peripheral actually drives, unlike any "custom" pin tried before it).
#define SD_PIN_D0  2    // chip's native/hardcoded SDMMC D0 - proven the same way.
#define SD_PIN_D1  4    // native D1 - only needed once 4-bit mode is wired up.
#define SD_PIN_D2  12   // native D2 - only needed once 4-bit mode is wired up.
#define SD_PIN_D3  13   // native D3 - only needed once 4-bit mode is wired up.

#define SD_MOUNT_POINT "/sdcard"
#define SD_USE_4BIT_MODE false   // 1-bit mode: only needs CLK+CMD+D0 wired (2 wires to move from
                                  // the old, non-functional pin set), enough for full enumeration/
                                  // read/write - 4-bit can be wired up later as a separate step.

#define UART_BAUD 1000000   // matches the Tang Nano side's own UART convention for this project.

#define FW_UPDATE_MAGIC_TIMEOUT_MS 20000   // how long FWUPDATE waits for the promised byte count
                                             // to actually arrive before giving up.
