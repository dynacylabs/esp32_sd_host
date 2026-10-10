// Pin/board configuration for the ESP32 SD-card test harness.
//
// REWIRED 2026-10-10 (final): CLK=14/CMD=17/D0=25/D1=26/D2=27/D3=19. The real, settled finding
// from a long night of pin experiments (see blog/journal.md's 2026-10-10 entries for the full
// trail): CLK only ever produced a working clock output on GPIO14 (tried 14, 32, 18 - only 14
// worked, for reasons not fully understood but consistently reproduced), so it stays there. Every
// *other* pin tried for CMD happened to collide with something: GPIO15 is JTAG TDO, GPIO33 sits
// in the RTC-capable block (32-39), GPIO21 is Arduino-ESP32's well-known default I2C SDA pin -
// each showed a real, reproducible "stuck" output in a pure self-loopback test with no SDMMC
// peripheral or Tang Nano involved at all. GPIO17 is the first pin tried with no such collision,
// and it passes that same self-loopback test cleanly.
//
// Important: picking a collision-free CMD pin did NOT fix the actual enumeration mystery (every
// real command still decodes on the Tang Nano as a clean CMD0, identically, even on GPIO17) - it
// only fixed a real, separate problem (bad pin choices colliding with other peripherals). The
// self-loopback test proves GPIO17 itself is electrically fine, which proves the real mystery is
// independent of ESP32-side pin/GPIO factors entirely and lives somewhere in the SDMMC
// peripheral's own internal command path - not something more pin changes can reach.
#pragma once

#define SD_PIN_CLK 14   // ALSO JTAG TMS - an input to the chip, never driven by it. The only pin
                         // tried (of 14/32/18) where this chip's SDMMC peripheral actually clocks.
#define SD_PIN_CMD 17   // The first CMD pin tried with no collision with JTAG/RTC-block/I2C
                         // defaults/flash/strapping/UART0 - passes self-loopback cleanly.
#define SD_PIN_D0  25
#define SD_PIN_D1  26
#define SD_PIN_D2  27
#define SD_PIN_D3  19

#define SD_MOUNT_POINT "/sdcard"
#define SD_USE_4BIT_MODE true   // full 4-bit mode by default - exercises more of the spec
                                 // (bus-width switch, all 4 DAT lines) than 1-bit would.

#define UART_BAUD 1000000   // matches the Tang Nano side's own UART convention for this project.

#define FW_UPDATE_MAGIC_TIMEOUT_MS 20000   // how long FWUPDATE waits for the promised byte count
                                             // to actually arrive before giving up.
