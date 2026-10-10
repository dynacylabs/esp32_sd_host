// ESP32 SD-card test harness for tang_nano_usd_mitm.
//
// Purpose: a real, comprehensive SD-protocol host to drive the Tang Nano FPGA card emulator
// against, since this laptop's own RTS5208-based reader turned out to gate card enumeration on
// something (likely a mechanical card-detect switch) that pure electrical signal testing can't
// satisfy. The ESP32's SD controller runs entirely open-source (ESP-IDF), so its own init
// sequence can be read directly instead of reverse-engineered from a black box, and it doesn't
// need to see any card-detect signal before attempting real commands at all.
//
// No WiFi, no OTA - deliberately. Firmware updates happen entirely over this same UART via
// FWUPDATE (see cmd_sys.cpp), which needs no network and no physical button press, since it's
// driven by application code already listening on UART rather than the ROM serial bootloader.
//
// Command reference: see cmd_raw.h (raw SD protocol commands - the actual point of this
// firmware), cmd_fs.h (filesystem-level commands, useful once the FPGA implements enough for
// FATFS to mount), cmd_sys.h (PING/INFO/REBOOT/HOSTINIT/MOUNT/UNMOUNT/FWUPDATE).
#include "config.h"
#include "sd_session.h"
#include "uart_util.h"
#include "cmd_sys.h"
#include "cmd_raw.h"
#include "cmd_fs.h"
#include "cmd_bb.h"
#include "esp_log.h"

void setup() {
  // Verbose so MOUNT's internal sdmmc_card_init()/SCR-read failures show their real step instead
  // of just the top-level "mount_to_vfs failed" summary - diagnostic only, not load-bearing.
  esp_log_level_set("*", ESP_LOG_VERBOSE);
  // Must be set before begin(). The default RX ring buffer (256 bytes) is nowhere near enough
  // for bulk transfers (PUT/GET/FWUPDATE/block writes) at UART_BAUD while handlers are busy doing
  // real flash writes (Update.write(), fwrite()) between reads. Confirmed via a real FWUPDATE
  // transfer that silently desynced (bytes lost mid-transfer); an isolated pure-receive test with
  // the identical read loop but no Update.write() calls at all received the same 385KB cleanly in
  // 4.3s with an 8KB buffer, proving the UART path itself is reliable and the loss happens
  // specifically during Update.write()'s occasional long blocking stalls (flash sector erase can
  // take well over the ~80ms an 8KB buffer covers at this baud) - sized up further here for
  // headroom against exactly that.
  Serial.setRxBufferSize(16384);
  Serial.begin(UART_BAUD);
  Serial.setTimeout(30000);
  delay(200);   // let the USB-serial bridge settle before the first print
  Serial.print("\nBOOT esp32_host " __DATE__ " " __TIME__ "\n");
  // Bring up the SDMMC host/slot immediately so raw commands work right away, without waiting
  // for a HOSTINIT - matches this project's established "zero dependency on cooperation from
  // the other side" testing philosophy.
  bool host_ok = sd_session_begin_host();
  Serial.print(host_ok ? "HOSTINIT ok\n" : "HOSTINIT failed\n");
  Serial.print("READY\n");
}

void loop() {
  String line = uart_read_line();
  if (line.length() == 0) {
    return;
  }

  String parts[8];
  int nparts = uart_split(line, parts, 8);
  if (nparts == 0) {
    return;
  }
  String cmd = parts[0];
  cmd.toUpperCase();

  if (cmd_sys_dispatch(cmd, parts, nparts)) return;
  if (cmd_raw_dispatch(cmd, parts, nparts)) return;
  if (cmd_fs_dispatch(cmd, parts, nparts)) return;
  if (cmd_bb_dispatch(cmd, parts, nparts)) return;

  uart_err("unknown_command:" + cmd);
}
