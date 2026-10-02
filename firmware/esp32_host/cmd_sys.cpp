#include "cmd_sys.h"
#include "config.h"
#include "sd_session.h"
#include "uart_util.h"

#include <Update.h>
#include <esp_chip_info.h>
#include <esp_flash.h>

static void do_ping() {
  Serial.print("P");
}

static void do_info() {
  esp_chip_info_t chip;
  esp_chip_info(&chip);
  uint32_t flash_size = 0;
  esp_flash_get_size(NULL, &flash_size);

  String out;
  out += "fw_build=" __DATE__ " " __TIME__ "\n";
  out += "chip_model=" + String((int)chip.model) + " revision=" + String(chip.revision) + " cores=" + String(chip.cores) + "\n";
  out += "flash_bytes=" + String(flash_size) + "\n";
  out += "free_heap=" + String(ESP.getFreeHeap()) + "\n";
  out += "sd_host_ready=" + String(sd_session_host_ready() ? 1 : 0) + "\n";
  out += "sd_bus_width=" + String(sd_session_get_bus_width()) + "\n";
  out += "sd_fs_mounted=" + String(sd_session_fs_mounted() ? 1 : 0) + "\n";
  if (sd_session_fs_mounted()) {
    sdmmc_card_t *card = sd_session_card();
    out += "card_name=" + String(card->cid.name) + "\n";
    out += "card_rca=" + String(card->rca) + "\n";
    out += "card_is_sdhc_or_higher=" + String((card->ocr & (1 << 30)) ? 1 : 0) + "\n";
    out += "card_sectors=" + String((uint32_t)(card->csd.capacity)) + "\n";
    out += "card_sector_size=" + String(card->csd.sector_size) + "\n";
  }
  Serial.print(out);
  uart_ok();
}

// Per-chunk-acknowledged protocol, not a single bulk "ready then blast everything" transfer -
// found necessary the hard way: Update.write() occasionally blocks for well over 100ms (real
// flash sector-erase latency), and at 1Mbaud that's enough incoming bytes to overflow even a
// 16KB RX ring buffer, silently desyncing the transfer. A plain receive loop with the identical
// read code but no Update.write() calls handled the same 385KB cleanly, isolating the real cause.
// Explicit per-chunk ACKs make the sender wait for however long any single flash operation
// actually takes, instead of racing a fixed buffer against it - this is correct regardless of
// how slow a given flash chip's erase happens to be, not just tuned to one observed stall.
//
// Protocol: FWUPDATE <len>\n -> "OK ready <chunk_size>\n". Sender then sends exactly
// min(chunk_size, remaining) bytes and WAITS for "OK <total_written>\n" before sending the next
// chunk. After the last chunk: "OK flashed_rebooting\n", then the device reboots.
#define FWUPDATE_CHUNK_SIZE 4096   // matches typical SPI flash erase-sector size

static void do_fwupdate(const String &len_str) {
  size_t len = (size_t)len_str.toInt();
  if (len == 0) {
    uart_err("bad_length");
    return;
  }

  if (!Update.begin(len)) {
    uart_err("update_begin_failed:" + String(Update.errorString()));
    return;
  }

  uart_ok("ready " + String(FWUPDATE_CHUNK_SIZE));

  static uint8_t buf[FWUPDATE_CHUNK_SIZE];
  size_t remaining = len;
  size_t written = 0;
  while (remaining > 0) {
    size_t n = min((size_t)FWUPDATE_CHUNK_SIZE, remaining);
    if (!uart_read_exact(buf, n, FW_UPDATE_MAGIC_TIMEOUT_MS)) {
      Update.abort();
      uart_err("timeout_during_transfer");
      return;
    }
    if (Update.write(buf, n) != n) {
      Update.abort();
      uart_err("flash_write_failed:" + String(Update.errorString()));
      return;
    }
    remaining -= n;
    written += n;
    uart_ok(String(written));   // sender must wait for this before sending the next chunk
  }

  if (!Update.end(true)) {
    uart_err("update_end_failed:" + String(Update.errorString()));
    return;
  }

  uart_ok("flashed_rebooting");
  Serial.flush();
  delay(100);
  ESP.restart();
}

bool cmd_sys_dispatch(const String &cmd, String *parts, int nparts) {
  if (cmd == "PING") {
    do_ping();
    return true;
  }
  if (cmd == "INFO") {
    do_info();
    return true;
  }
  if (cmd == "REBOOT") {
    uart_ok("rebooting");
    Serial.flush();
    delay(100);
    ESP.restart();
    return true;
  }
  if (cmd == "FWUPDATE") {
    if (nparts < 2) {
      uart_err("usage: FWUPDATE <length>");
      return true;
    }
    do_fwupdate(parts[1]);
    return true;
  }
  if (cmd == "HOSTINIT") {
    if (sd_session_begin_host()) {
      uart_ok();
    } else {
      uart_err("host_init_failed");
    }
    return true;
  }
  if (cmd == "MOUNT") {
    if (sd_session_try_mount_fs()) {
      uart_ok("mounted");
    } else {
      uart_err("mount_failed_raw_commands_still_available");
    }
    return true;
  }
  if (cmd == "UNMOUNT") {
    sd_session_unmount_fs();
    uart_ok();
    return true;
  }
  if (cmd == "FORMAT") {
    if (sd_session_format_and_mount_fs()) {
      uart_ok("formatted_and_mounted");
    } else {
      uart_err("format_failed");
    }
    return true;
  }
  if (cmd == "CARDINIT") {
    int err = sd_session_card_init();
    if (err == 0) {
      uart_ok("card_init_ok");
    } else {
      uart_err("card_init_failed:0x" + String(err, 16));
    }
    return true;
  }
  if (cmd == "WIDTH") {
    if (nparts < 2) {
      uart_ok(String(sd_session_get_bus_width()));
      return true;
    }
    int w = parts[1].toInt();
    if (w != 1 && w != 4) {
      uart_err("usage: WIDTH <1|4>");
      return true;
    }
    sd_session_unmount_fs();
    sd_session_set_bus_width(w);
    uart_ok(String(w));
    return true;
  }
  return false;
}
