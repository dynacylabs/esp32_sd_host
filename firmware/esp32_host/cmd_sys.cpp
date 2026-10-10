#include "cmd_sys.h"
#include "config.h"
#include "sd_session.h"
#include "uart_util.h"

#include <Update.h>
#include <esp_chip_info.h>
#include <esp_flash.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

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
  out += "sd_max_freq_khz=" + String(sd_session_get_max_freq_khz()) + "\n";
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

// Original (non-S3) ESP32's SDMMC peripheral has no GPIO-matrix remap support
// (SOC_SDMMC_USE_GPIO_MATRIX is undefined for this chip) in the older driver generation - open
// question is whether the newer driver's slot.cmd/d0-d3 "custom pin" config actually routes
// anything on this chip, or silently no-ops while the peripheral keeps driving its native pins
// (CLK=14/CMD=15/D0=2/D1=4/D2=12/D3=13) regardless. This self-samples an arbitrary GPIO on core 0
// while CARDINIT's real negotiation runs on core 1, so a currently-unused native pin (e.g. 15, 2,
// 4, 12, 13) can be checked for real signal activity during a real attempt without touching wiring.
#define SCOPE_MAX_SAMPLES 16000
static uint8_t s_scope_buf[SCOPE_MAX_SAMPLES];
static volatile int s_scope_gpio = -1;
static volatile size_t s_scope_n = 0;
static volatile size_t s_scope_count = 0;
static volatile bool s_scope_stop = false;

static void scope_task_fn(void *arg) {
  int gpio = s_scope_gpio;
  size_t n = s_scope_n;
  size_t i = 0;
  while (i < n && !s_scope_stop) {
    s_scope_buf[i++] = (uint8_t)digitalRead(gpio);
  }
  s_scope_count = i;
  vTaskDelete(NULL);
}

static void do_scopepin(const String &gpio_str, const String &n_str, bool trigger_init) {
  int gpio = gpio_str.toInt();
  size_t n = (size_t)n_str.toInt();
  if (n == 0 || n > SCOPE_MAX_SAMPLES) {
    uart_err("usage: <gpio> <samples 1-" + String(SCOPE_MAX_SAMPLES) + ">");
    return;
  }
  pinMode(gpio, INPUT);

  s_scope_gpio = gpio;
  s_scope_n = n;
  s_scope_count = 0;
  s_scope_stop = false;

  TaskHandle_t task = nullptr;
  xTaskCreatePinnedToCore(scope_task_fn, "scope", 4096, NULL, 1, &task, 0);
  delay(5);   // let the sampling task spin up and start reading before triggering the real init

  int err = 0;
  if (trigger_init) {
    err = sd_session_card_init();
  } else {
    delay(30);   // idle baseline - no bus activity triggered, same approximate window length
  }

  delay(200);   // give the task a chance to fill the buffer naturally if it hasn't already
  s_scope_stop = true;
  delay(20);    // let the task observe the stop flag and self-delete

  size_t transitions = 0;
  for (size_t i = 1; i < s_scope_count; i++) {
    if (s_scope_buf[i] != s_scope_buf[i - 1]) transitions++;
  }
  size_t ones = 0;
  for (size_t i = 0; i < s_scope_count; i++) ones += s_scope_buf[i];

  String out = "card_init_err=0x" + String(err, 16) + " gpio=" + String(gpio) +
               " samples=" + String((uint32_t)s_scope_count) +
               " ones=" + String((uint32_t)ones) +
               " transitions=" + String((uint32_t)transitions);
  Serial.println(out);
  uart_ok();
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
  if (cmd == "PINSET") {
    if (nparts < 3) {
      uart_err("usage: PINSET <gpio> <0|1>");
      return true;
    }
    int gpio = parts[1].toInt();
    int level = parts[2].toInt() ? HIGH : LOW;
    pinMode(gpio, OUTPUT);
    digitalWrite(gpio, level);
    uart_ok(String(gpio));
    return true;
  }
  if (cmd == "PINGET") {
    if (nparts < 2) {
      uart_err("usage: PINGET <gpio>");
      return true;
    }
    int gpio = parts[1].toInt();
    pinMode(gpio, INPUT);
    uart_ok(String(digitalRead(gpio)));
    return true;
  }
  if (cmd == "PINTOG") {
    int n = nparts >= 2 ? parts[1].toInt() : 0;
    if (n <= 0 || n > 10000) {
      uart_err("usage: PINTOG <count 1-10000>");
      return true;
    }
    pinMode(SD_PIN_CLK, OUTPUT);
    pinMode(SD_PIN_CMD, OUTPUT);
    for (int i = 0; i < n; i++) {
      digitalWrite(SD_PIN_CMD, (i % 2) ? HIGH : LOW);
      digitalWrite(SD_PIN_CLK, HIGH);
      delayMicroseconds(20);
      digitalWrite(SD_PIN_CLK, LOW);
      delayMicroseconds(20);
    }
    bool hold_low = nparts >= 3 && parts[2].equalsIgnoreCase("low");
    digitalWrite(SD_PIN_CMD, hold_low ? LOW : HIGH);
    uart_ok(String(n));
    return true;
  }
  if (cmd == "SCOPEPIN") {
    if (nparts < 3) {
      uart_err("usage: SCOPEPIN <gpio> <samples>");
      return true;
    }
    do_scopepin(parts[1], parts[2], true);
    return true;
  }
  if (cmd == "SCOPEIDLE") {
    if (nparts < 3) {
      uart_err("usage: SCOPEIDLE <gpio> <samples>");
      return true;
    }
    do_scopepin(parts[1], parts[2], false);
    return true;
  }
  if (cmd == "FREQ") {
    if (nparts < 2) {
      uart_ok(String(sd_session_get_max_freq_khz()));
      return true;
    }
    int khz = parts[1].toInt();
    if (khz <= 0) {
      uart_err("usage: FREQ <khz>");
      return true;
    }
    sd_session_unmount_fs();
    sd_session_set_max_freq_khz(khz);
    uart_ok(String(khz));
    return true;
  }
  return false;
}
