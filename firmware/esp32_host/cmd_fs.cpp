#include "cmd_fs.h"
#include "config.h"
#include "sd_session.h"
#include "uart_util.h"

#include <dirent.h>
#include <sys/stat.h>
#include <stdio.h>

static String full_path(const String &p) {
  if (p.startsWith("/")) {
    return String(SD_MOUNT_POINT) + p;
  }
  return String(SD_MOUNT_POINT) + "/" + p;
}

static bool require_mounted() {
  if (!sd_session_fs_mounted()) {
    uart_err("not_mounted_run_MOUNT_first");
    return false;
  }
  return true;
}

static void do_list(const String &path) {
  if (!require_mounted()) return;
  String fp = full_path(path);
  DIR *d = opendir(fp.c_str());
  if (!d) {
    uart_err("opendir_failed");
    return;
  }
  struct dirent *entry;
  String out;
  while ((entry = readdir(d)) != nullptr) {
    String entry_path = fp + "/" + entry->d_name;
    struct stat st;
    long size = 0;
    bool is_dir = false;
    if (stat(entry_path.c_str(), &st) == 0) {
      size = (long)st.st_size;
      is_dir = S_ISDIR(st.st_mode);
    }
    out += String(is_dir ? "dir " : "file ") + String(size) + " " + entry->d_name + "\n";
  }
  closedir(d);
  Serial.print(out);
  uart_ok();
}

static void do_mkdir(const String &path) {
  if (!require_mounted()) return;
  if (mkdir(full_path(path).c_str(), 0755) != 0) {
    uart_err("mkdir_failed:" + String(strerror(errno)));
    return;
  }
  uart_ok();
}

static void do_rmdir(const String &path) {
  if (!require_mounted()) return;
  if (rmdir(full_path(path).c_str()) != 0) {
    uart_err("rmdir_failed:" + String(strerror(errno)));
    return;
  }
  uart_ok();
}

static void do_rm(const String &path) {
  if (!require_mounted()) return;
  if (unlink(full_path(path).c_str()) != 0) {
    uart_err("rm_failed:" + String(strerror(errno)));
    return;
  }
  uart_ok();
}

static void do_rename(const String &oldp, const String &newp) {
  if (!require_mounted()) return;
  if (rename(full_path(oldp).c_str(), full_path(newp).c_str()) != 0) {
    uart_err("rename_failed:" + String(strerror(errno)));
    return;
  }
  uart_ok();
}

static void do_stat(const String &path) {
  if (!require_mounted()) return;
  struct stat st;
  if (stat(full_path(path).c_str(), &st) != 0) {
    uart_err("stat_failed:" + String(strerror(errno)));
    return;
  }
  uart_ok("size=" + String((long)st.st_size) + " type=" + (S_ISDIR(st.st_mode) ? "dir" : "file"));
}

static void do_put(const String &path, const String &len_str) {
  if (!require_mounted()) return;
  size_t len = (size_t)len_str.toInt();
  FILE *f = fopen(full_path(path).c_str(), "wb");
  if (!f) {
    uart_err("open_for_write_failed:" + String(strerror(errno)));
    return;
  }
  uart_ok("ready");

  const size_t CHUNK = 1024;
  uint8_t buf[CHUNK];
  size_t remaining = len;
  bool ok = true;
  while (remaining > 0) {
    size_t n = min(CHUNK, remaining);
    if (!uart_read_exact(buf, n, 10000)) {
      ok = false;
      break;
    }
    if (fwrite(buf, 1, n, f) != n) {
      ok = false;
      break;
    }
    remaining -= n;
  }
  fclose(f);
  if (!ok) {
    uart_err("transfer_failed_partial_file_left_on_card");
    return;
  }
  uart_ok("written");
}

static void do_get(const String &path) {
  if (!require_mounted()) return;
  FILE *f = fopen(full_path(path).c_str(), "rb");
  if (!f) {
    uart_err("open_for_read_failed:" + String(strerror(errno)));
    return;
  }
  fseek(f, 0, SEEK_END);
  long size = ftell(f);
  fseek(f, 0, SEEK_SET);

  uart_ok(String(size));

  const size_t CHUNK = 1024;
  uint8_t buf[CHUNK];
  long remaining = size;
  while (remaining > 0) {
    size_t n = min((long)CHUNK, remaining);
    size_t got = fread(buf, 1, n, f);
    if (got != n) {
      break;   // truncated transfer - receiver will time out waiting for the rest, which is a
                // correct (if blunt) signal that something went wrong mid-read.
    }
    Serial.write(buf, n);
    remaining -= n;
  }
  fclose(f);
}

bool cmd_fs_dispatch(const String &cmd, String *parts, int nparts) {
  if (cmd == "LIST") {
    do_list(nparts >= 2 ? parts[1] : "/");
    return true;
  }
  if (cmd == "MKDIR") {
    if (nparts < 2) { uart_err("usage: MKDIR <path>"); return true; }
    do_mkdir(parts[1]);
    return true;
  }
  if (cmd == "RMDIR") {
    if (nparts < 2) { uart_err("usage: RMDIR <path>"); return true; }
    do_rmdir(parts[1]);
    return true;
  }
  if (cmd == "RM") {
    if (nparts < 2) { uart_err("usage: RM <path>"); return true; }
    do_rm(parts[1]);
    return true;
  }
  if (cmd == "RENAME") {
    if (nparts < 3) { uart_err("usage: RENAME <old> <new>"); return true; }
    do_rename(parts[1], parts[2]);
    return true;
  }
  if (cmd == "STAT") {
    if (nparts < 2) { uart_err("usage: STAT <path>"); return true; }
    do_stat(parts[1]);
    return true;
  }
  if (cmd == "PUT") {
    if (nparts < 3) { uart_err("usage: PUT <path> <length>"); return true; }
    do_put(parts[1], parts[2]);
    return true;
  }
  if (cmd == "GET") {
    if (nparts < 2) { uart_err("usage: GET <path>"); return true; }
    do_get(parts[1]);
    return true;
  }
  return false;
}
