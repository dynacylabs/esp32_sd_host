// Shared UART helpers: line-based command reading, length-prefixed binary transfer, and small
// response conventions used consistently across every command handler, mirroring the
// line-command-plus-raw-binary-blob convention already established on the Tang Nano FPGA side of
// this project (SDTEST/APPCMD/PUT-style framing).
#pragma once

#include <Arduino.h>

// Reads one '\n'-terminated line from Serial, blocking. Trailing '\r' (if present) is stripped.
// Returns the line without the terminator.
String uart_read_line();

// Reads exactly `len` raw bytes from Serial into buf, blocking, with an overall timeout. Returns
// false (and leaves buf short) on timeout - callers must check before trusting the buffer.
bool uart_read_exact(uint8_t *buf, size_t len, uint32_t timeout_ms);

// Response conventions: every command replies with exactly one of these as its first line
// (further data, if any, follows after "OK" on the same or subsequent lines per command).
void uart_ok();
void uart_ok(const String &rest);          // "OK <rest>\n"
void uart_err(const String &reason);       // "ERR <reason>\n"

// Splits a command line on spaces into up to `max_parts` tokens. Returns the actual count.
int uart_split(const String &line, String *parts, int max_parts);
