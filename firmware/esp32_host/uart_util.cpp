#include "uart_util.h"

String uart_read_line() {
  String line = Serial.readStringUntil('\n');
  if (line.endsWith("\r")) {
    line.remove(line.length() - 1);
  }
  return line;
}

bool uart_read_exact(uint8_t *buf, size_t len, uint32_t timeout_ms) {
  size_t got = 0;
  uint32_t start = millis();
  while (got < len) {
    if (millis() - start > timeout_ms) {
      return false;
    }
    int avail = Serial.available();
    if (avail <= 0) {
      continue;
    }
    int n = Serial.read(buf + got, min((size_t)avail, len - got));
    if (n > 0) {
      got += n;
      start = millis();   // reset timeout on forward progress, not just on first byte
    }
  }
  return true;
}

void uart_ok() {
  Serial.print("OK\n");
}

void uart_ok(const String &rest) {
  Serial.print("OK ");
  Serial.print(rest);
  Serial.print("\n");
}

void uart_err(const String &reason) {
  Serial.print("ERR ");
  Serial.print(reason);
  Serial.print("\n");
}

int uart_split(const String &line, String *parts, int max_parts) {
  int count = 0;
  int start = 0;
  int len = line.length();
  while (start < len && count < max_parts) {
    while (start < len && line[start] == ' ') {
      start++;
    }
    if (start >= len) {
      break;
    }
    int end = line.indexOf(' ', start);
    if (end < 0 || end >= len) {
      end = len;
    }
    parts[count++] = line.substring(start, end);
    start = end + 1;
  }
  return count;
}
