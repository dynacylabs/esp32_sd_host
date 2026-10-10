// Manual, bit-banged native-mode SD command transmitter/receiver - bypasses the ESP-IDF SDMMC
// hardware peripheral entirely, driving CLK/CMD directly via plain GPIO on whatever pins the Tang
// Nano's wires actually reach right now. Exists for two independent reasons documented in
// blog/journal.md and hdl/step14_top.v: (1) this chip's SDMMC peripheral is hardwired to its
// native pins and cannot be remapped to this board's wiring without a physical rewire; (2) even
// correctly wired, the hardware peripheral's own response-detection interrupt
// (SDMMC_INTMASK_RESP_ERR) was found (2026-10-09) to reject every real response because of how
// the Tang Nano releases CMD to idle - a bit-banged receiver reads raw levels instead of relying
// on that circuitry, sidestepping both problems. Only CLK+CMD are needed for the full
// identification sequence (CMD0/CMD8/ACMD41/CMD2/CMD3/CMD9/CMD7) - no DAT lines, so this works
// over the current physical wiring with no rewire.
#pragma once

#include <Arduino.h>

bool cmd_bb_dispatch(const String &cmd, String *parts, int nparts);
