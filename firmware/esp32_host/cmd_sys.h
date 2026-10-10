// System-level commands: PING, INFO, REBOOT, HOSTINIT, MOUNT/UNMOUNT, FORMAT, CARDINIT, WIDTH,
// FREQ, and FWUPDATE - the button-free, UART-only firmware update path (no OTA/WiFi at all in
// this firmware by design). FWUPDATE uses a per-chunk-acknowledged protocol, not a single bulk
// transfer - see the comment above do_fwupdate() in cmd_sys.cpp for why a naive "ready, then
// blast everything" approach silently lost bytes during real flash-write stalls.
// WIDTH [1|4]: with no argument, reports the current runtime SD bus width. With an argument,
// switches it (tearing down any mounted filesystem/host state first) for the NEXT HOSTINIT/MOUNT
// call - added to test a board's 1-bit vs 4-bit wiring without a reflash each time.
// FREQ [khz]: with no argument, reports the current runtime SDMMC clock-rate ceiling (kHz). With
// an argument, switches it for the NEXT HOSTINIT/MOUNT call - added to test whether a
// non-compliant target needs the real SD spec's slower identification-phase rate (default is
// ESP-IDF's own SDMMC_FREQ_DEFAULT, 20MHz, normally only meant for AFTER identification).
// FORMAT: like MOUNT, but ERASES the card and reformats it as FAT32 if no recognized filesystem
// is found, then mounts it. Unlike MOUNT, this is destructive by design - only run it against a
// card you don't need the existing contents of.
// CARDINIT: calls ESP-IDF's sdmmc_card_init() directly, bypassing the FATFS/VFS mount step
// entirely - useful to isolate whether a MOUNT/FORMAT failure is in card negotiation itself or
// in the filesystem-specific step on top of it, and to get a real esp_err_t instead of MOUNT's
// generic failure code.
//
// Raw-GPIO diagnostics, none of which touch the SDMMC peripheral:
//   PINSET <gpio> <0|1>       drives a pin to a fixed level (plain digitalWrite).
//   PINGET <gpio>             reads a pin's current level (plain digitalRead).
//   PINTOG <count> [low]      toggles SD_PIN_CMD/SD_PIN_CLK <count> times (a manual clock/data
//                             square wave), then leaves CMD high unless "low" is given.
//   SCOPEPIN <gpio> <samples> self-samples a pin on core 0 (up to SCOPE_MAX_SAMPLES times) while
//                             a real CARDINIT runs on core 1 - reports ones/transitions, to check
//                             whether a candidate pin is actually driven by the SDMMC peripheral.
//   SCOPEIDLE <gpio> <samples> same self-sampling, but with no SD activity triggered - an idle
//                             baseline to compare SCOPEPIN's result against.
#pragma once

#include <Arduino.h>

// Returns true if `cmd` was one this module handles (whether or not it succeeded - uart_ok/
// uart_err is always sent by the handler itself before returning).
bool cmd_sys_dispatch(const String &cmd, String *parts, int nparts);
