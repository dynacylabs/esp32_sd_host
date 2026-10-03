// Shared SD card session.
//
// Deliberately split into two independent layers rather than one all-in-one mount call:
//
//   1. "Host" layer (sd_session_begin_host): brings up the SDMMC peripheral and GPIO slot only -
//      no card negotiation at all. This is always safe to call and never depends on a real card
//      responding correctly. Raw/protocol-level commands (RAWCMD and friends) work against this
//      layer alone, via a minimally-populated sdmmc_card_t whose .host field is real but whose
//      rca/csd/cid start zeroed - exactly matching the fact that this project's FPGA target is
//      built up one command at a time (CMD0/CMD8/CMD55+ACMD41 exist so far, nothing further) -
//      raw command testing must not be gated behind a full init sequence the FPGA can't complete
//      yet.
//   2. "Filesystem" layer (sd_session_try_mount_fs): the full esp_vfs_fat_sdmmc_mount() automatic
//      negotiation + FATFS mount, for the filesystem-level commands (PUT/GET/MKDIR/...). This is
//      expected to fail until the FPGA under test implements enough of the real init sequence
//      (CMD2/CMD3/CMD9/CMD7/SET_BLOCKLEN and real block I/O) - failure here is normal, not a bug,
//      and filesystem commands just report "not mounted" until it succeeds.
#pragma once

#include "sdmmc_cmd.h"

bool sd_session_begin_host();
bool sd_session_host_ready();

// Runtime bus-width override (1 or 4), applied on the NEXT sd_session_begin_host()/
// sd_session_try_mount_fs() call - added to test a real board's 4-bit wiring without needing a
// reflash each time, since config.h's SD_USE_4BIT_MODE is compile-time only. Forces a fresh host
// teardown/rebuild if the host is already up.
void sd_session_set_bus_width(int width);
int sd_session_get_bus_width();

// Runtime SDMMC clock-rate override (kHz), applied on the NEXT sd_session_begin_host() call -
// defaults to SDMMC_FREQ_DEFAULT (20MHz), which ESP-IDF normally only uses AFTER identification
// completes against a real card. Added to test whether a non-compliant target (e.g. an FPGA fake
// card under development, which may not be built to handle a 20MHz external clock) needs the
// real SD spec's slower identification-phase rate (<=400kHz) to be recognized at all. Forces a
// fresh host teardown/rebuild if the host is already up, same as sd_session_set_bus_width().
void sd_session_set_max_freq_khz(int khz);
int sd_session_get_max_freq_khz();

// Always valid once sd_session_begin_host() has succeeded - rca/csd/cid start zeroed and get
// filled in as command handlers process real responses (e.g. after a real CMD3, call
// sd_session_set_rca() so later per-card commands use the right address).
sdmmc_card_t *sd_session_card();

void sd_session_set_rca(uint16_t rca);

// Attempts the full automatic mount (card_init + FATFS). Returns true and leaves the filesystem
// usable at SD_MOUNT_POINT on success. Safe to call again (unmounts first) after the Tang Nano
// side has been reflashed/reset, without rebooting the ESP32.
bool sd_session_try_mount_fs();
void sd_session_unmount_fs();
bool sd_session_fs_mounted();

// Calls ESP-IDF's sdmmc_card_init() directly against the host-only layer, bypassing the
// FATFS/VFS mount step entirely - added to isolate whether a MOUNT failure is coming from card
// negotiation itself (this call) or from the filesystem-specific step on top of it. On success,
// fully populates sd_session_card()'s cid/csd/scr/ocr (unlike the manual RAWCMD-driven step
// commands, which only dump raw response words). Returns the real esp_err_t so the caller can
// report more than MOUNT's generic 0xffffffff.
int sd_session_card_init();

// Same as sd_session_try_mount_fs(), but passes format_if_mount_failed=true: if the filesystem
// isn't recognized (e.g. a factory exFAT card against a FATFS build without exFAT support), this
// ERASES the card and reformats it as FAT32, then mounts it. Deliberately a separate, explicitly-
// named call rather than a flag on sd_session_try_mount_fs()/MOUNT - MOUNT must never silently
// destroy data on a card under test.
bool sd_session_format_and_mount_fs();
