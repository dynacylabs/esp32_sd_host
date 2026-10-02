// Pin/board configuration for the ESP32 SD-card test harness.
//
// CLK=14/CMD=15/D0=2/D1=4/D2=12/D3=13 is the ESP32's own DEFAULT SDMMC peripheral IOMUX routing,
// not a custom remap - this is why AI-Thinker's reference design (and virtually every cheap clone
// copying it) wires the SD slot to exactly these pins. If a specific board turns out to differ,
// override the six defines below - everything else in this firmware reads pins from here only.
#pragma once

#define SD_PIN_CLK 14
#define SD_PIN_CMD 15
#define SD_PIN_D0  2
#define SD_PIN_D1  4   // shared with the AI-Thinker board's onboard flash LED - expect it to
                        // flicker during 4-bit-mode SD traffic, harmless.
#define SD_PIN_D2  12   // ALSO the flash-voltage strapping pin (MTDI) - a card/tap inserted at
                         // boot can misdetect flash voltage. Not an issue once running, only
                         // matters for the initial serial flash (see repo README).
#define SD_PIN_D3  13

#define SD_MOUNT_POINT "/sdcard"
#define SD_USE_4BIT_MODE true   // full 4-bit mode by default - exercises more of the spec
                                 // (bus-width switch, all 4 DAT lines) than 1-bit would.

#define UART_BAUD 1000000   // matches the Tang Nano side's own UART convention for this project.

#define FW_UPDATE_MAGIC_TIMEOUT_MS 20000   // how long FWUPDATE waits for the promised byte count
                                             // to actually arrive before giving up.
