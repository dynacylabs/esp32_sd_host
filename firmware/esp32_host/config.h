// Pin/board configuration for the ESP32 SD-card test harness.
//
// CLK=14/CMD=15/D0=2/D1=4/D2=12/D3=13 is AI-Thinker's reference design's own chosen wiring (and
// virtually every cheap clone copying it uses the same pins) - NOT the original ESP32's own
// default SDMMC IOMUX routing as an earlier version of this comment claimed (checked directly
// against sdmmc_default_configs.h 2026-10-10: the original ESP32's real default is
// clk=6/cmd=11/d0=7/d1=8/d2=9/d3=10; 14/15/2/4/12/13 is actually the *ESP32-S3's* default). This
// firmware explicitly sets these six pins itself (fill_slot_pins() in sd_session.cpp) regardless
// of chip variant, so the mismatch doesn't change behavior - just corrects why these pins are
// used: it's this specific board's wiring choice, not an unavoidable default. Also worth knowing:
// all four are shared with the ESP32's own JTAG interface - CLK=TMS, CMD=TDO, D2=TDI, D3=TCK (see
// blog/journal.md's 2026-10-10 entries for why this mattered once real testing got underway). If
// a specific board turns out to differ, override the six defines below - everything else in this
// firmware reads pins from here only.
#pragma once

#define SD_PIN_CLK 14   // ALSO JTAG TMS (an input to the chip, never driven by it).
#define SD_PIN_CMD 15   // ALSO JTAG TDO - the one JTAG *output*; see the file header comment.
#define SD_PIN_D0  2
#define SD_PIN_D1  4   // shared with the AI-Thinker board's onboard flash LED - expect it to
                        // flicker during 4-bit-mode SD traffic, harmless.
#define SD_PIN_D2  12   // ALSO JTAG TDI and the flash-voltage strapping pin (MTDI) - a card/tap
                         // inserted at boot can misdetect flash voltage. Not an issue once
                         // running, only matters for the initial serial flash (see repo README).
#define SD_PIN_D3  13   // ALSO JTAG TCK.

#define SD_MOUNT_POINT "/sdcard"
#define SD_USE_4BIT_MODE true   // full 4-bit mode by default - exercises more of the spec
                                 // (bus-width switch, all 4 DAT lines) than 1-bit would.

#define UART_BAUD 1000000   // matches the Tang Nano side's own UART convention for this project.

#define FW_UPDATE_MAGIC_TIMEOUT_MS 20000   // how long FWUPDATE waits for the promised byte count
                                             // to actually arrive before giving up.
