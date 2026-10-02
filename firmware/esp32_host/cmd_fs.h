// Filesystem-level commands - standard POSIX calls (opendir/mkdir/unlink/rename/stat/fopen) via
// ESP-IDF's VFS once sd_session_try_mount_fs() has succeeded. All of these report "not mounted"
// until the FPGA under test implements enough of the real init sequence for FATFS to come up -
// that's expected, not a bug in this firmware.
//
// Commands:
//   LIST <path>              newline-separated "type size name" entries, blank line + OK to end
//   MKDIR <path>
//   RMDIR <path>
//   RM <path>
//   RENAME <old> <new>
//   STAT <path>              "size=<n> type=<file|dir>"
//   PUT <path> <length>      caller then sends exactly <length> raw bytes
//   GET <path>               replies "OK <length>" then exactly <length> raw bytes, or ERR
#pragma once

#include <Arduino.h>

bool cmd_fs_dispatch(const String &cmd, String *parts, int nparts);
