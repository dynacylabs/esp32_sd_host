#include "cmd_bb.h"
#include "sd_session.h"
#include "uart_util.h"

#include <string.h>
#include <driver/gpio.h>

// Bit-bangs on whichever physical pins the Tang Nano's wires actually reach right now - NOT
// config.h's SD_PIN_CLK/SD_PIN_CMD, since those were just corrected to point at this chip's real
// native SDMMC pins (14/15), which this board's CMD wire is no longer physically connected to.
// The real CMD wire is still on GPIO17 (unchanged all session); CLK happens to already be right
// either way, since 14 is both the native pin and where the wire has always been.
#define BB_PIN_CLK 14
#define BB_PIN_CMD 17

#define BB_RESP_MAX_BYTES 24   // fits R2's 136 bits (17 bytes) with room to spare

static uint8_t crc7_update(uint8_t crc, uint8_t bit) {
  uint8_t crc_in = bit ^ ((crc >> 6) & 1);
  crc = (crc << 1) & 0x7F;
  if (crc_in) crc ^= 0x09;
  return crc;
}

static uint8_t crc7_of_bits(const uint8_t *buf, int nbits) {
  uint8_t crc = 0;
  for (int i = 0; i < nbits; i++) {
    uint8_t bit = (buf[i / 8] >> (7 - (i % 8))) & 1;
    crc = crc7_update(crc, bit);
  }
  return crc;
}

static void build_frame(uint8_t index, uint32_t arg, uint8_t *buf6) {
  buf6[0] = 0x40 | (index & 0x3F);   // start=0, transmission=1 (host), 6-bit command index
  buf6[1] = (arg >> 24) & 0xFF;
  buf6[2] = (arg >> 16) & 0xFF;
  buf6[3] = (arg >> 8) & 0xFF;
  buf6[4] = arg & 0xFF;
  uint8_t crc = crc7_of_bits(buf6, 40);
  buf6[5] = (uint8_t)((crc << 1) | 1);   // 7-bit CRC + end bit (always 1)
}

// Releases the SDMMC peripheral's claim on these pins (it owns GPIO14 as its native CLK the
// moment sd_session_begin_host() has ever run, which setup() does at boot) and takes them back as
// plain GPIO - same pattern proven out during tonight's self-loopback pin testing.
static void bb_claim_pins() {
  sd_session_set_bus_width(sd_session_get_bus_width());   // tears down the host/slot if it's up
  gpio_reset_pin((gpio_num_t)BB_PIN_CLK);
  gpio_reset_pin((gpio_num_t)BB_PIN_CMD);
  pinMode(BB_PIN_CLK, OUTPUT);
  digitalWrite(BB_PIN_CLK, LOW);
  pinMode(BB_PIN_CMD, OUTPUT);
  digitalWrite(BB_PIN_CMD, HIGH);   // idle-high, matching real SD bus idle convention
}

// Card (or, here, the Tang Nano) samples CMD on the rising edge of CLK - confirmed against this
// project's own HDL (sd_cmd_pad_in is registered on posedge sd_clk_up in hdl/step14_top.v). CMD
// must already be stable before this call.
static inline void bb_clk_pulse() {
  digitalWrite(BB_PIN_CLK, HIGH);
  delayMicroseconds(3);
  digitalWrite(BB_PIN_CLK, LOW);
  delayMicroseconds(3);
}

static bool s_bb_trace = false;

static void bb_send_frame(const uint8_t *buf6) {
  pinMode(BB_PIN_CMD, OUTPUT);
  for (int i = 0; i < 48; i++) {
    uint8_t bit = (buf6[i / 8] >> (7 - (i % 8))) & 1;
    digitalWrite(BB_PIN_CMD, bit);
    delayMicroseconds(2);
    if (s_bb_trace) {
      Serial.print(digitalRead(BB_PIN_CMD));
    }
    bb_clk_pulse();
  }
  if (s_bb_trace) Serial.println();
  digitalWrite(BB_PIN_CMD, HIGH);   // release to idle immediately after the end bit
}

// Waits up to max_wait_clocks clock cycles for a start bit (CMD pulled low), then captures
// resp_bits total (including that start bit) into out_buf. Returns the number of wait cycles
// consumed before the start bit appeared, or -1 on timeout. out_buf must be >= (resp_bits+7)/8
// bytes and <= BB_RESP_MAX_BYTES.
static int bb_recv_response(uint8_t *out_buf, int resp_bits, int max_wait_clocks) {
  // INPUT_PULLUP, not plain INPUT: with nobody actively driving the shared CMD line (ESP32
  // released, Tang Nano not yet responding), a truly floating net has no defined level at all -
  // confirmed on real hardware to let the Tang Nano's receiver (which re-triggers a new "frame"
  // on any 0 bit with no debounce, see sd_cmd_receiver.v) see a stable low baseline and
  // continuously re-decode bogus all-zero frames every 48 cycles, even while the ESP32's own
  // read of the same net happened to read high - a classic floating-net symptom where the two
  // ends of an undriven wire can disagree. A real pull-up establishes the spec-required idle-high
  // level both ends agree on.
  pinMode(BB_PIN_CMD, INPUT_PULLUP);
  int waited = 0;
  int level = HIGH;
  for (; waited < max_wait_clocks; waited++) {
    digitalWrite(BB_PIN_CLK, HIGH);
    delayMicroseconds(3);
    level = digitalRead(BB_PIN_CMD);
    digitalWrite(BB_PIN_CLK, LOW);
    delayMicroseconds(3);
    if (level == LOW) break;
  }
  if (waited >= max_wait_clocks) {
    return -1;
  }
  memset(out_buf, 0, (resp_bits + 7) / 8);
  for (int i = 1; i < resp_bits; i++) {   // bit 0 is the start bit already known to be 0
    digitalWrite(BB_PIN_CLK, HIGH);
    delayMicroseconds(3);
    level = digitalRead(BB_PIN_CMD);
    digitalWrite(BB_PIN_CLK, LOW);
    delayMicroseconds(3);
    if (level) {
      out_buf[i / 8] |= (1 << (7 - (i % 8)));
    }
  }
  pinMode(BB_PIN_CMD, OUTPUT);
  digitalWrite(BB_PIN_CMD, HIGH);   // back to driven idle-high
  return waited;
}

static String bytes_to_hex(const uint8_t *buf, int nbytes) {
  String out;
  char b[3];
  for (int i = 0; i < nbytes; i++) {
    snprintf(b, sizeof(b), "%02x", buf[i]);
    out += b;
  }
  return out;
}

// Core primitive: send one command frame, optionally wait for and capture a response. resp_bits
// <= 0 means no response expected (e.g. CMD0). Reports via uart_ok/uart_err itself.
static bool do_bbcmd(uint8_t index, uint32_t arg, int resp_bits, String *resp_hex_out, bool *crc_ok_out) {
  bb_claim_pins();
  uint8_t frame[6];
  build_frame(index, arg, frame);
  bb_send_frame(frame);

  if (resp_bits <= 0) {
    return true;
  }

  uint8_t resp[BB_RESP_MAX_BYTES] = {0};
  int waited = bb_recv_response(resp, resp_bits, 256);
  if (waited < 0) {
    uart_err("no_response_timeout");
    return false;
  }
  bool crc_ok = false;
  if (resp_bits >= 8) {
    uint8_t got_crc = resp[(resp_bits / 8) - 1] >> 1;
    uint8_t want_crc = crc7_of_bits(resp, resp_bits - 8);
    crc_ok = (got_crc == want_crc);
  }
  if (resp_hex_out) *resp_hex_out = bytes_to_hex(resp, (resp_bits + 7) / 8);
  if (crc_ok_out) *crc_ok_out = crc_ok;
  return true;
}

static void cmd_bbcmd(String *parts, int nparts) {
  if (nparts < 3) {
    uart_err("usage: BBCMD <index> <arg_hex> [resp_bits]");
    return;
  }
  uint8_t index = (uint8_t)parts[1].toInt();
  uint32_t arg = (uint32_t)strtoul(parts[2].c_str(), nullptr, 16);
  int resp_bits = nparts >= 4 ? parts[3].toInt() : 0;

  String resp_hex;
  bool crc_ok = false;
  if (!do_bbcmd(index, arg, resp_bits, &resp_hex, &crc_ok)) {
    return;   // do_bbcmd already reported the error
  }
  if (resp_bits <= 0) {
    uart_ok("sent_no_response_expected");
    return;
  }
  uart_ok("resp=" + resp_hex + " crc_ok=" + String(crc_ok ? 1 : 0));
}

// Runs the real identification sequence this project's Tang Nano target implements
// (CMD0/CMD8/CMD55+ACMD41/CMD2/CMD3/CMD9/CMD7), entirely bit-banged, and reports a clear pass/
// fail summary for each step - directly exercises "enumeration" without needing the ESP32's own
// SDMMC hardware peripheral (and its documented RESP_ERR false-rejection issue) at all.
static void cmd_bbinit() {
  String out;
  bool any_fail = false;

  do_bbcmd(0, 0x00000000, 0, nullptr, nullptr);   // CMD0 (GO_IDLE_STATE) - no response
  out += "CMD0=sent ";

  String resp;
  bool crc_ok;
  if (do_bbcmd(8, 0x000001AA, 48, &resp, &crc_ok)) {
    out += "CMD8=" + resp + (crc_ok ? "(crc_ok) " : "(CRC_BAD) ");
    if (!crc_ok) any_fail = true;
  } else {
    out += "CMD8=TIMEOUT ";
    any_fail = true;
  }

  bool acmd41_ready = false;
  for (int i = 0; i < 10 && !acmd41_ready; i++) {
    if (!do_bbcmd(55, 0x00000000, 48, &resp, &crc_ok)) {
      out += "CMD55=TIMEOUT ";
      any_fail = true;
      break;
    }
    if (!do_bbcmd(41, 0x40FF8000, 48, &resp, &crc_ok)) {
      out += "ACMD41=TIMEOUT ";
      any_fail = true;
      break;
    }
    // R3's own CRC field is spec-defined as don't-care/all-ones - not meaningful to validate.
    uint32_t ocr = strtoul(resp.substring(2, 10).c_str(), nullptr, 16);
    if (ocr & 0x80000000UL) {   // bit 31 = card power-up status (busy=0 means ready)
      acmd41_ready = true;
      out += "ACMD41=" + resp + "(ready,try" + String(i + 1) + ") ";
    }
  }
  if (!acmd41_ready) {
    out += "ACMD41=NEVER_READY ";
    any_fail = true;
  }

  if (do_bbcmd(2, 0x00000000, 136, &resp, &crc_ok)) {
    out += "CMD2=" + resp + (crc_ok ? "(crc_ok) " : "(CRC_BAD) ");
    if (!crc_ok) any_fail = true;
  } else {
    out += "CMD2=TIMEOUT ";
    any_fail = true;
  }

  uint16_t rca = 0;
  if (do_bbcmd(3, 0x00000000, 48, &resp, &crc_ok)) {
    out += "CMD3=" + resp + (crc_ok ? "(crc_ok) " : "(CRC_BAD) ");
    if (!crc_ok) any_fail = true;
    rca = (uint16_t)strtoul(resp.substring(2, 6).c_str(), nullptr, 16);
  } else {
    out += "CMD3=TIMEOUT ";
    any_fail = true;
  }

  if (do_bbcmd(9, ((uint32_t)rca) << 16, 136, &resp, &crc_ok)) {
    out += "CMD9=" + resp + (crc_ok ? "(crc_ok) " : "(CRC_BAD) ");
    if (!crc_ok) any_fail = true;
  } else {
    out += "CMD9=TIMEOUT ";
    any_fail = true;
  }

  if (do_bbcmd(7, ((uint32_t)rca) << 16, 48, &resp, &crc_ok)) {
    out += "CMD7=" + resp + (crc_ok ? "(crc_ok) " : "(CRC_BAD) ");
    if (!crc_ok) any_fail = true;
  } else {
    out += "CMD7=TIMEOUT ";
    any_fail = true;
  }

  Serial.println(out);
  if (any_fail) {
    uart_err("bbinit_incomplete");
  } else {
    uart_ok("bbinit_complete");
  }
}

bool cmd_bb_dispatch(const String &cmd, String *parts, int nparts) {
  if (cmd == "BBCMD") {
    cmd_bbcmd(parts, nparts);
    return true;
  }
  if (cmd == "BBINIT") {
    cmd_bbinit();
    return true;
  }
  if (cmd == "BBTRACE") {
    s_bb_trace = (nparts >= 2) ? (parts[1].toInt() != 0) : !s_bb_trace;
    uart_ok(String(s_bb_trace ? 1 : 0));
    return true;
  }
  return false;
}
