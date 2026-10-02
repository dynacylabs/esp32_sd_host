#include "cmd_raw.h"
#include "sd_session.h"
#include "uart_util.h"

#include <string.h>
#include "sd_protocol_defs.h"
#include "esp_private/sdmmc_common.h"   // sdmmc_send_cmd() - not in the public sdmmc_cmd.h header,
                                          // but this project deliberately wants raw command access
                                          // beyond the public convenience API, see cmd_raw.h.

static uint32_t hex_to_u32(const String &s) {
  return (uint32_t)strtoul(s.c_str(), nullptr, 16);
}

static String hex_of(uint32_t v) {
  char buf[9];
  snprintf(buf, sizeof(buf), "%08lx", (unsigned long)v);
  return String(buf);
}

static void dump_response(const sdmmc_command_t &cmd) {
  String out = "resp=" + hex_of(cmd.response[0]) + " " + hex_of(cmd.response[1]) + " " + hex_of(cmd.response[2]) + " " + hex_of(cmd.response[3]);
  uart_ok(out);
}

// Sends one command with no data phase. Reports OK/ERR itself; returns true on success so
// higher-level sequences (ACMD, the manual init steps) can chain without double-reporting.
static bool send_simple(uint32_t opcode, uint32_t arg, int flags, sdmmc_command_t *out = nullptr, uint32_t timeout_ms = 1000) {
  if (!sd_session_host_ready()) {
    uart_err("host_not_ready_run_HOSTINIT_first");
    return false;
  }
  sdmmc_command_t cmd = {};
  cmd.opcode = opcode;
  cmd.arg = arg;
  cmd.flags = flags;
  cmd.timeout_ms = timeout_ms;
  esp_err_t err = sdmmc_send_cmd(sd_session_card(), &cmd);
  if (out) {
    *out = cmd;
  }
  if (err != ESP_OK) {
    uart_err("cmd_failed:0x" + hex_of((uint32_t)err));
    return false;
  }
  dump_response(cmd);
  return true;
}

// CMD55 (APP_CMD) using the session's current RCA, then the target ACMD - every real SD ACMD is
// always preceded by this, by spec; building it in once here rather than making every ACMD
// command handler repeat the same two-step sequence.
static bool send_acmd(uint32_t acmd_opcode, uint32_t arg, int flags, sdmmc_command_t *out = nullptr) {
  if (!sd_session_host_ready()) {
    uart_err("host_not_ready_run_HOSTINIT_first");
    return false;
  }
  sdmmc_command_t cmd55 = {};
  cmd55.opcode = MMC_APP_CMD;
  cmd55.arg = ((uint32_t)sd_session_card()->rca) << 16;
  cmd55.flags = SCF_RSP_R1;
  cmd55.timeout_ms = 1000;
  esp_err_t err = sdmmc_send_cmd(sd_session_card(), &cmd55);
  if (err != ESP_OK) {
    uart_err("cmd55_failed:0x" + hex_of((uint32_t)err));
    return false;
  }
  return send_simple(acmd_opcode, arg, flags, out);
}

static bool parse_resp_type(const String &s, int *flags_out) {
  struct { const char *name; int flags; } table[] = {
    {"R0", SCF_RSP_R0}, {"R1", SCF_RSP_R1}, {"R1B", SCF_RSP_R1B}, {"R2", SCF_RSP_R2},
    {"R3", SCF_RSP_R3}, {"R4", SCF_RSP_R4}, {"R5", SCF_RSP_R5}, {"R5B", SCF_RSP_R5B},
    {"R6", SCF_RSP_R6}, {"R7", SCF_RSP_R7},
  };
  for (auto &e : table) {
    if (s.equalsIgnoreCase(e.name)) {
      *flags_out = e.flags;
      return true;
    }
  }
  return false;
}

static void do_rdblock(const String &lba_str) {
  uint32_t lba = (uint32_t)lba_str.toInt();
  static uint8_t buf[512];
  esp_err_t err = sdmmc_read_sectors(sd_session_card(), buf, lba, 1);
  if (err != ESP_OK) {
    uart_err("read_failed:0x" + hex_of((uint32_t)err));
    return;
  }
  uart_ok("512");
  Serial.write(buf, 512);
}

static void do_wrblock(const String &lba_str) {
  uint32_t lba = (uint32_t)lba_str.toInt();
  static uint8_t buf[512];
  if (!uart_read_exact(buf, 512, 10000)) {
    uart_err("timeout_waiting_for_512_bytes");
    return;
  }
  esp_err_t err = sdmmc_write_sectors(sd_session_card(), buf, lba, 1);
  if (err != ESP_OK) {
    uart_err("write_failed:0x" + hex_of((uint32_t)err));
    return;
  }
  uart_ok();
}

static void do_rdmulti(const String &lba_str, const String &count_str) {
  uint32_t lba = (uint32_t)lba_str.toInt();
  uint32_t count = (uint32_t)count_str.toInt();
  if (count == 0 || count > 64) {
    uart_err("count_out_of_range_1_64");
    return;
  }
  static uint8_t buf[64 * 512];
  esp_err_t err = sdmmc_read_sectors(sd_session_card(), buf, lba, count);
  if (err != ESP_OK) {
    uart_err("read_failed:0x" + hex_of((uint32_t)err));
    return;
  }
  uart_ok(String(count * 512));
  Serial.write(buf, count * 512);
}

static void do_wrmulti(const String &lba_str, const String &count_str) {
  uint32_t lba = (uint32_t)lba_str.toInt();
  uint32_t count = (uint32_t)count_str.toInt();
  if (count == 0 || count > 64) {
    uart_err("count_out_of_range_1_64");
    return;
  }
  static uint8_t buf[64 * 512];
  if (!uart_read_exact(buf, count * 512, 30000)) {
    uart_err("timeout_waiting_for_data");
    return;
  }
  esp_err_t err = sdmmc_write_sectors(sd_session_card(), buf, lba, count);
  if (err != ESP_OK) {
    uart_err("write_failed:0x" + hex_of((uint32_t)err));
    return;
  }
  uart_ok();
}

static void do_erase(const String &start_str, const String &end_str) {
  uint32_t start = (uint32_t)start_str.toInt();
  uint32_t end = (uint32_t)end_str.toInt();
  esp_err_t err = sdmmc_erase_sectors(sd_session_card(), start, end - start + 1, SDMMC_ERASE_ARG);
  if (err != ESP_OK) {
    uart_err("erase_failed:0x" + hex_of((uint32_t)err));
    return;
  }
  uart_ok();
}

static void do_cardinfo() {
  sdmmc_card_t *card = sd_session_card();
  String out;
  out += "ocr=" + hex_of(card->ocr) + "\n";
  out += "rca=" + String(card->rca) + "\n";
  out += "is_mem=" + String(card->is_mem) + " is_sdio=" + String(card->is_sdio) + " is_mmc=" + String(card->is_mmc) + "\n";
  out += "log_bus_width=" + String(card->log_bus_width) + " is_ddr=" + String(card->is_ddr) + " is_uhs1=" + String(card->is_uhs1) + "\n";
  out += "max_freq_khz=" + String(card->max_freq_khz) + " real_freq_khz=" + String(card->real_freq_khz) + "\n";
  out += "cid.mfg_id=" + hex_of(card->cid.mfg_id) + " cid.oem_id=" + hex_of(card->cid.oem_id) + " cid.name=" + String(card->cid.name) + "\n";
  out += "cid.revision=" + String(card->cid.revision) + " cid.serial=" + hex_of(card->cid.serial) + " cid.date=" + String(card->cid.date) + "\n";
  out += "csd.capacity_sectors=" + String(card->csd.capacity) + " csd.sector_size=" + String(card->csd.sector_size) + "\n";
  out += "csd.read_block_len=" + String(card->csd.read_block_len) + " csd.card_command_class=" + hex_of(card->csd.card_command_class) + "\n";
  out += "csd.tr_speed=" + String(card->csd.tr_speed) + "\n";
  out += "scr.sd_spec=" + String(card->scr.sd_spec) + " scr.bus_width=" + hex_of(card->scr.bus_width) + "\n";
  Serial.print(out);
  uart_ok();
}

static void do_switchfunc(const String &mode_str, const String &group_str, const String &value_str) {
  uint32_t mode = (uint32_t)mode_str.toInt();
  uint32_t group = (uint32_t)group_str.toInt();
  uint32_t value = (uint32_t)value_str.toInt();
  // CMD6 argument layout per the SD spec: bit31=mode (0=check,1=switch), function values packed
  // into 4-bit groups, group `group` set to `value`, all other groups set to 0xF (no change).
  uint32_t arg = (mode ? (1UL << 31) : 0) | 0x00FFFFFFUL;
  arg &= ~(0xFUL << (group * 4));
  arg |= (value & 0xF) << (group * 4);

  static uint8_t buf[64];
  sdmmc_command_t cmd = {};
  cmd.opcode = SD_SEND_SWITCH_FUNC;
  cmd.arg = arg;
  cmd.flags = SCF_RSP_R1 | SCF_CMD_ADTC | SCF_CMD_READ;
  cmd.data = buf;
  cmd.datalen = sizeof(buf);
  cmd.blklen = sizeof(buf);
  cmd.timeout_ms = 1000;
  if (!sd_session_host_ready()) {
    uart_err("host_not_ready_run_HOSTINIT_first");
    return;
  }
  esp_err_t err = sdmmc_send_cmd(sd_session_card(), &cmd);
  if (err != ESP_OK) {
    uart_err("cmd_failed:0x" + hex_of((uint32_t)err));
    return;
  }
  uart_ok("64");
  Serial.write(buf, sizeof(buf));
}

// CMD42 (LOCK_UNLOCK) has no named macro in this IDF version's sd_protocol_defs.h - using the
// raw spec-defined index directly, same as CMD28/29/30/56 below.
#define CMD_LOCK_UNLOCK 42
#define CMD_SET_WRITE_PROT 28
#define CMD_CLR_WRITE_PROT 29
#define CMD_SEND_WRITE_PROT 30
#define CMD_GEN_CMD 56

static void do_lock_family(uint32_t mode_bits, const String &pwd_hex) {
  // CMD42 data phase: 1 byte of mode bits (ERASE/LOCK_UNLOCK/CLR_PWD/SET_PWD) followed by a
  // 1-byte password length and the password bytes themselves (0 bytes for FORCEERASE).
  size_t pwd_bytes = pwd_hex.length() / 2;
  static uint8_t buf[64];
  size_t i = 0;
  buf[i++] = (uint8_t)mode_bits;
  buf[i++] = (uint8_t)pwd_bytes;
  for (size_t b = 0; b < pwd_bytes && i < sizeof(buf); b++) {
    buf[i++] = (uint8_t)strtoul(pwd_hex.substring(b * 2, b * 2 + 2).c_str(), nullptr, 16);
  }

  if (!sd_session_host_ready()) {
    uart_err("host_not_ready_run_HOSTINIT_first");
    return;
  }

  // CMD42's data block length must match the card's actual SET_BLOCKLEN register, not just the
  // length ESP-IDF is told to clock out for this one transfer - sdmmc_card_init() already set it
  // to 512 for normal block I/O, and without first re-pointing it at our small payload (`i` bytes,
  // 3-4 here) via a real CMD16, the card keeps expecting a 512-byte block and stalls mid-transfer
  // waiting for the remaining bytes, timing out every time. Found the hard way: every LOCK/UNLOCK/
  // CLRPWD attempt failed with ESP_ERR_TIMEOUT and left the card transiently stuck in the "rcv"
  // state (self-recovered on the next fresh CARDINIT, but never completed the actual operation).
  if (!send_simple(MMC_SET_BLOCKLEN, (uint32_t)i, SCF_RSP_R1)) {
    return;
  }

  sdmmc_command_t cmd = {};
  cmd.opcode = CMD_LOCK_UNLOCK;
  cmd.arg = 0;
  cmd.flags = SCF_RSP_R1B | SCF_CMD_ADTC;
  cmd.data = buf;
  cmd.datalen = i;
  cmd.blklen = i;
  cmd.timeout_ms = 5000;   // card may need real time to actually erase/relock
  esp_err_t err = sdmmc_send_cmd(sd_session_card(), &cmd);

  // Restore the normal 512-byte block length regardless of how the lock/unlock itself went, so
  // RDBLOCK/WRBLOCK/MOUNT and friends keep working afterward without needing a fresh CARDINIT.
  // The card needs a brief moment to settle right after CMD42's data+busy phase before it accepts
  // another command - confirmed on real hardware: an immediate SET_BLOCKLEN right after a
  // successful CMD42 reliably timed out, while retrying after a short delay succeeds.
  bool restore_ok = false;
  for (int attempt = 0; attempt < 5 && !restore_ok; attempt++) {
    if (attempt > 0) {
      delay(100);
    }
    restore_ok = send_simple(MMC_SET_BLOCKLEN, 512, SCF_RSP_R1);
  }
  if (!restore_ok) {
    uart_err("blocklen_restore_failed_after_cmd42_rdblock_wrblock_may_be_broken_until_next_CARDINIT");
    return;
  }

  // ESP-IDF's sdmmc_send_cmd() reliably reports ESP_ERR_TIMEOUT on CMD42's post-data R1B busy
  // wait even when the card actually completed the operation correctly - confirmed on real
  // hardware: LOCK/UNLOCK both visibly flip the card's CARD_IS_LOCKED status bit despite this
  // call "failing" every time. Rather than report a misleading failure, fall back to asking the
  // card directly (CMD13/SEND_STATUS) whether CARD_IS_LOCKED actually matches what was requested,
  // since that bit is ground truth regardless of what the host-side busy-wait timed out on.
  if (err != ESP_OK) {
    sdmmc_command_t status_cmd = {};
    status_cmd.opcode = MMC_SEND_STATUS;
    status_cmd.arg = ((uint32_t)sd_session_card()->rca) << 16;
    status_cmd.flags = SCF_RSP_R1 | SCF_CMD_AC;
    status_cmd.timeout_ms = 1000;
    bool locked = false;
    bool got_status = (sdmmc_send_cmd(sd_session_card(), &status_cmd) == ESP_OK);
    if (got_status) {
      locked = (status_cmd.response[0] & (1UL << 25)) != 0;   // CARD_IS_LOCKED
    }
    bool requested_lock = (mode_bits & 0x01) != 0;   // LOCK_UNLOCK bit: 1=lock, 0=unlock
    bool is_lock_or_unlock = (mode_bits & 0x01) || mode_bits == 0x00;
    if (got_status && is_lock_or_unlock && locked == requested_lock) {
      uart_ok("driver_reported_timeout_but_card_status_confirms_" + String(locked ? "locked" : "unlocked"));
      return;
    }
    uart_err("cmd_failed:0x" + hex_of((uint32_t)err) + (got_status ? (locked ? "_card_now_locked" : "_card_now_unlocked") : "_and_status_check_also_failed"));
    return;
  }
  dump_response(cmd);
}

static void do_wrprot(uint32_t opcode, const String &lba_str) {
  uint32_t lba = (uint32_t)lba_str.toInt();
  if (opcode == CMD_SEND_WRITE_PROT) {
    static uint8_t buf[4];
    sdmmc_command_t cmd = {};
    cmd.opcode = opcode;
    cmd.arg = lba;
    cmd.flags = SCF_RSP_R1 | SCF_CMD_ADTC | SCF_CMD_READ;
    cmd.data = buf;
    cmd.datalen = sizeof(buf);
    cmd.blklen = sizeof(buf);
    cmd.timeout_ms = 1000;
    if (!sd_session_host_ready()) {
      uart_err("host_not_ready_run_HOSTINIT_first");
      return;
    }
    esp_err_t err = sdmmc_send_cmd(sd_session_card(), &cmd);
    if (err != ESP_OK) {
      uart_err("cmd_failed:0x" + hex_of((uint32_t)err));
      return;
    }
    uart_ok("bitmap=" + hex_of(buf[0]) + hex_of(buf[1]) + hex_of(buf[2]) + hex_of(buf[3]));
    return;
  }
  send_simple(opcode, lba, SCF_RSP_R1B);
}

static void do_gencmd(const String &arg_hex, const String &dir) {
  uint32_t arg = hex_to_u32(arg_hex);
  bool is_read = dir.equalsIgnoreCase("read");
  arg = (arg & ~1UL) | (is_read ? 1UL : 0UL);   // LSB: 0=write to card, 1=read from card, per spec
  static uint8_t buf[512];
  sdmmc_command_t cmd = {};
  cmd.opcode = CMD_GEN_CMD;
  cmd.arg = arg;
  cmd.flags = SCF_RSP_R1 | SCF_CMD_ADTC | (is_read ? SCF_CMD_READ : 0);
  cmd.data = buf;
  cmd.datalen = sizeof(buf);
  cmd.blklen = sizeof(buf);
  cmd.timeout_ms = 1000;
  if (!is_read) {
    if (!uart_read_exact(buf, sizeof(buf), 10000)) {
      uart_err("timeout_waiting_for_512_bytes");
      return;
    }
  }
  if (!sd_session_host_ready()) {
    uart_err("host_not_ready_run_HOSTINIT_first");
    return;
  }
  esp_err_t err = sdmmc_send_cmd(sd_session_card(), &cmd);
  if (err != ESP_OK) {
    uart_err("cmd_failed:0x" + hex_of((uint32_t)err));
    return;
  }
  if (is_read) {
    uart_ok("512");
    Serial.write(buf, sizeof(buf));
  } else {
    dump_response(cmd);
  }
}

static void do_tuning() {
  // CMD19 (SEND_TUNING_BLOCK) is SD's real UHS-I tuning command (SDR50/SDR104). CMD20 is NOT a
  // tuning command for SD at all - it's the legacy WRITE_DAT_UNTIL_STOP (MMC_WRITE_DAT_UNTIL_STOP
  // in sd_protocol_defs.h) - an earlier draft of this file incorrectly treated both as "tuning",
  // corrected here rather than left in.
  uint32_t opcode = MMC_SEND_TUNING_BLOCK;
  static uint8_t buf[64];
  sdmmc_command_t cmd = {};
  cmd.opcode = opcode;
  cmd.arg = 0;
  cmd.flags = SCF_RSP_R1 | SCF_CMD_ADTC | SCF_CMD_READ;
  cmd.data = buf;
  cmd.datalen = sizeof(buf);
  cmd.blklen = sizeof(buf);
  cmd.timeout_ms = 1000;
  if (!sd_session_host_ready()) {
    uart_err("host_not_ready_run_HOSTINIT_first");
    return;
  }
  esp_err_t err = sdmmc_send_cmd(sd_session_card(), &cmd);
  if (err != ESP_OK) {
    uart_err("cmd_failed:0x" + hex_of((uint32_t)err));
    return;
  }
  uart_ok("64");
  Serial.write(buf, sizeof(buf));
}

static void do_rawcmd(String *parts, int nparts) {
  if (nparts < 4) {
    uart_err("usage: RAWCMD <index> <arg_hex> <resp_type> [datalen] [read|write]");
    return;
  }
  uint32_t opcode = (uint32_t)parts[1].toInt();
  uint32_t arg = hex_to_u32(parts[2]);
  int resp_flags;
  if (!parse_resp_type(parts[3], &resp_flags)) {
    uart_err("bad_resp_type_want_R0_R1_R1B_R2_R3_R4_R5_R5B_R6_R7");
    return;
  }
  size_t datalen = (nparts >= 5) ? (size_t)parts[4].toInt() : 0;
  bool is_read = (nparts >= 6) && parts[5].equalsIgnoreCase("read");

  static uint8_t buf[512];
  if (datalen > sizeof(buf)) {
    uart_err("datalen_too_large_max_512");
    return;
  }

  sdmmc_command_t cmd = {};
  cmd.opcode = opcode;
  cmd.arg = arg;
  cmd.flags = resp_flags;
  cmd.timeout_ms = 2000;
  if (datalen > 0) {
    cmd.flags |= SCF_CMD_ADTC | (is_read ? SCF_CMD_READ : 0);
    cmd.data = buf;
    cmd.datalen = datalen;
    cmd.blklen = datalen;
    if (!is_read) {
      if (!uart_read_exact(buf, datalen, 10000)) {
        uart_err("timeout_waiting_for_data");
        return;
      }
    }
  }

  if (!sd_session_host_ready()) {
    uart_err("host_not_ready_run_HOSTINIT_first");
    return;
  }
  esp_err_t err = sdmmc_send_cmd(sd_session_card(), &cmd);
  if (err != ESP_OK) {
    uart_err("cmd_failed:0x" + hex_of((uint32_t)err));
    return;
  }
  dump_response(cmd);
  if (datalen > 0 && is_read) {
    Serial.write(buf, datalen);
  }
}

bool cmd_raw_dispatch(const String &cmd, String *parts, int nparts) {
  if (cmd == "CMD0") {
    // CMD0 has no response by spec - send_simple's own dump_response() would print a meaningless
    // all-zero "resp=..." line since there's nothing real to show, so this bypasses it rather
    // than reporting OK twice.
    if (!sd_session_host_ready()) {
      uart_err("host_not_ready_run_HOSTINIT_first");
      return true;
    }
    sdmmc_command_t go_idle = {};
    go_idle.opcode = MMC_GO_IDLE_STATE;
    go_idle.flags = SCF_RSP_R0 | SCF_CMD_BC;
    go_idle.timeout_ms = 1000;
    sdmmc_send_cmd(sd_session_card(), &go_idle);   // no response to check - sending it is the point
    uart_ok();
    return true;
  }
  if (cmd == "CMD8") {
    uint32_t arg = (nparts >= 2) ? hex_to_u32(parts[1]) : 0x1AAUL;
    send_simple(SD_SEND_IF_COND, arg, SCF_RSP_R7);
    return true;
  }
  if (cmd == "CMD55") {
    send_simple(MMC_APP_CMD, ((uint32_t)sd_session_card()->rca) << 16, SCF_RSP_R1);
    return true;
  }
  if (cmd == "ACMD41") {
    uint32_t arg = (nparts >= 2) ? hex_to_u32(parts[1]) : 0x40FF8000UL;
    send_acmd(SD_APP_OP_COND, arg, SCF_RSP_R3);
    return true;
  }
  if (cmd == "CMD2") {
    send_simple(MMC_ALL_SEND_CID, 0, SCF_RSP_R2 | SCF_CMD_BCR);
    return true;
  }
  if (cmd == "CMD3") {
    sdmmc_command_t out;
    if (send_simple(SD_SEND_RELATIVE_ADDR, 0, SCF_RSP_R6 | SCF_CMD_BCR, &out)) {
      uint16_t rca = (uint16_t)(out.response[0] >> 16);
      sd_session_set_rca(rca);
      Serial.print("rca_stored=" + String(rca) + "\n");
    }
    return true;
  }
  if (cmd == "CMD9") {
    send_simple(MMC_SEND_CSD, ((uint32_t)sd_session_card()->rca) << 16, SCF_RSP_R2 | SCF_CMD_AC);
    return true;
  }
  if (cmd == "CMD7") {
    send_simple(MMC_SELECT_CARD, ((uint32_t)sd_session_card()->rca) << 16, SCF_RSP_R1B | SCF_CMD_AC);
    return true;
  }
  if (cmd == "CMD13" || cmd == "CARDSTATUS") {
    send_simple(MMC_SEND_STATUS, ((uint32_t)sd_session_card()->rca) << 16, SCF_RSP_R1 | SCF_CMD_AC);
    return true;
  }

  if (cmd == "RDBLOCK") {
    if (nparts < 2) { uart_err("usage: RDBLOCK <lba>"); return true; }
    do_rdblock(parts[1]);
    return true;
  }
  if (cmd == "WRBLOCK") {
    if (nparts < 2) { uart_err("usage: WRBLOCK <lba>"); return true; }
    do_wrblock(parts[1]);
    return true;
  }
  if (cmd == "RDMULTI") {
    if (nparts < 3) { uart_err("usage: RDMULTI <lba> <count>"); return true; }
    do_rdmulti(parts[1], parts[2]);
    return true;
  }
  if (cmd == "WRMULTI") {
    if (nparts < 3) { uart_err("usage: WRMULTI <lba> <count>"); return true; }
    do_wrmulti(parts[1], parts[2]);
    return true;
  }
  if (cmd == "ERASE") {
    if (nparts < 3) { uart_err("usage: ERASE <start_lba> <end_lba>"); return true; }
    do_erase(parts[1], parts[2]);
    return true;
  }

  if (cmd == "CARDINFO") {
    do_cardinfo();
    return true;
  }

  if (cmd == "SWITCHFUNC") {
    if (nparts < 4) { uart_err("usage: SWITCHFUNC <mode:0|1> <group:0-5> <value:0-15>"); return true; }
    do_switchfunc(parts[1], parts[2], parts[3]);
    return true;
  }

  if (cmd == "LOCK") {
    if (nparts < 2) { uart_err("usage: LOCK <pwd_hex>"); return true; }
    do_lock_family(0x05 /* SET_PWD|LOCK_UNLOCK */, parts[1]);
    return true;
  }
  if (cmd == "UNLOCK") {
    if (nparts < 2) { uart_err("usage: UNLOCK <pwd_hex>"); return true; }
    do_lock_family(0x00 /* neither bit set = unlock with the given password */, parts[1]);
    return true;
  }
  if (cmd == "CLRPWD") {
    if (nparts < 2) { uart_err("usage: CLRPWD <old_pwd_hex>"); return true; }
    do_lock_family(0x02 /* CLR_PWD */, parts[1]);
    return true;
  }
  if (cmd == "FORCEERASE") {
    do_lock_family(0x08 /* ERASE */, "");
    return true;
  }

  if (cmd == "SETWRPROT") {
    if (nparts < 2) { uart_err("usage: SETWRPROT <lba>"); return true; }
    do_wrprot(CMD_SET_WRITE_PROT, parts[1]);
    return true;
  }
  if (cmd == "CLRWRPROT") {
    if (nparts < 2) { uart_err("usage: CLRWRPROT <lba>"); return true; }
    do_wrprot(CMD_CLR_WRITE_PROT, parts[1]);
    return true;
  }
  if (cmd == "SENDWRPROT") {
    if (nparts < 2) { uart_err("usage: SENDWRPROT <lba>"); return true; }
    do_wrprot(CMD_SEND_WRITE_PROT, parts[1]);
    return true;
  }

  if (cmd == "VOLTAGESWITCH") {
    send_simple(SD_SWITCH_VOLTAGE, 0, SCF_RSP_R1);
    return true;
  }
  if (cmd == "TUNING") {
    do_tuning();
    return true;
  }
  if (cmd == "GENCMD") {
    if (nparts < 3) { uart_err("usage: GENCMD <arg_hex> <read|write>"); return true; }
    do_gencmd(parts[1], parts[2]);
    return true;
  }
  if (cmd == "PREERASE") {
    // ACMD23 (SET_WR_BLK_ERASE_COUNT for SD) shares its index with the legacy
    // MMC_SET_BLOCK_COUNT macro (23) - same opcode number, different meaning depending on
    // whether it's sent plain (MMC) or via the CMD55 prefix (SD ACMD), exactly like CARDDETECT
    // below shares its index with CMD42 LOCK_UNLOCK.
    if (nparts < 2) { uart_err("usage: PREERASE <count>"); return true; }
    send_acmd(MMC_SET_BLOCK_COUNT, (uint32_t)parts[1].toInt(), SCF_RSP_R1);
    return true;
  }
  if (cmd == "CARDDETECT") {
    // ACMD42 (SET_CLR_CARD_DETECT) - no named macro in this IDF version; raw index 42, sent via
    // the CMD55 prefix so it's unambiguous against CMD42 (LOCK_UNLOCK), which is a different
    // command despite the shared opcode number.
    if (nparts < 2) { uart_err("usage: CARDDETECT <0|1>"); return true; }
    send_acmd(42, (uint32_t)parts[1].toInt(), SCF_RSP_R1);
    return true;
  }

  if (cmd == "RAWCMD") {
    do_rawcmd(parts, nparts);
    return true;
  }

  return false;
}
