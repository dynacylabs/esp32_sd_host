# esp32_host

An ESP32 firmware acting as a real, comprehensive SD-protocol host to drive the Tang Nano FPGA
card emulator against. Built because this laptop's own SD reader (Realtek RTS5208-based) turned
out to gate card enumeration on something beyond pure electrical signaling (likely a mechanical
card-detect switch) - see `blog/journal.md` in the parent `blink-re` repo for that investigation.
The ESP32's own SD controller runs entirely open-source (ESP-IDF), so its init sequence can be
read directly instead of reverse-engineered from a black box, and - more importantly - it doesn't
gate anything on card-detect at all: it tries real commands unconditionally whenever asked to.

## Hardware

Tested against a cheap AliExpress ESP32-CAM clone (ESP32-D0WDQ6 rev v1.1, 4MB flash) with a
separate CH340-based USB-serial programmer board, built and flashed from `re-pi`. Should work on
any ESP32 board whose SD slot follows the same reference pinout (CLK=14, CMD=15, D0=2, D1=4,
D2=12, D3=13 - the chip's own default SDMMC GPIO-matrix routing, not a board-specific remap, so
most clones copying the AI-Thinker design use it unchanged). Override the six pins in `config.h`
if a specific board differs.

**Known hardware limitation**: GPIO12 (SD DAT2) doubles as the ESP32's flash-voltage strapping
pin. A real card/breakout board inserted into the slot *during any reset* can cause the chip to
misdetect flash voltage and fail to boot - recoverable by removing the card and rebooting, not
damage, but it means **a card physically present in the slot across an unplanned reset can leave
the board needing a manual card-pull to recover**. Confirmed on real hardware this session. Not
yet worked around in software (can't be - it's sampled by the SoC before any firmware runs); the
practical fix would be physically preventing the DAT2 contact from reaching GPIO12 (e.g. a small
piece of tape over that one pad on the breakout board) and running the firmware in 1-bit mode
(`SD_USE_4BIT_MODE false` in `config.h`, which only needs CLK/CMD/D0).

## Build and flash

```bash
arduino-cli compile --fqbn 'esp32:esp32:esp32cam:PartitionScheme=min_spiffs' \
    --output-dir /path/to/out firmware/esp32_host
```

**`PartitionScheme=min_spiffs` is required, not optional** - the board's default scheme ("Huge
APP") has no OTA partitions at all, which `FWUPDATE` depends on. Omitting this flag compiles fine
but `FWUPDATE` fails immediately with "Partition Could Not be Found".

First flash needs `esptool` directly (no button press needed - the auto-reset circuit works over
plain DTR/RTS toggling on this hardware):

```bash
esptool --port /dev/ttyUSB0 --baud 115200 --before default-reset --after hard-reset \
    write-flash 0x0 out/esp32_host.ino.merged.bin
```

Use `--baud 115200` explicitly rather than letting esptool auto-upgrade to 460800 - this specific
board's crystal runs measurably off nominal (41.01MHz vs the expected 40MHz) and the higher baud
rate's reduced per-bit timing margin caused real upload failures during development. 115200 has
been completely reliable.

Every update after the first can go entirely over UART via `FWUPDATE` (see below) - no `esptool`,
no button, no network needed.

### A note on testing this firmware over serial

**pyserial (and likely other serial libraries) can hold the ESP32 in reset for as long as the
connection is open**, depending on how DTR/RTS default on open - this cost real debugging time
this session (looked like a dead board/bad crystal for a while; it wasn't). Always do this right
after opening a port intended to talk to the *running* application, not the bootloader:

```python
ser = serial.Serial(port, baud, dsrdtr=False, rtscts=False)
ser.dtr = False
ser.rts = False
time.sleep(1.5)   # let any reset this triggers finish booting before sending anything
```

## Command reference

No WiFi, no OTA over a network - deliberately. All firmware updates happen over the same UART
command channel used for everything else (`FWUPDATE`), which needs no network setup and no
physical button press, since it's handled by application code already listening on UART rather
than the ROM serial bootloader.

See the header comments in each module for the authoritative, detailed list:

- [`cmd_sys.h`](firmware/esp32_host/cmd_sys.h) - `PING`, `INFO`, `REBOOT`, `HOSTINIT`,
  `MOUNT`/`UNMOUNT`, `FORMAT` (explicit opt-in destructive reformat-as-FAT32), `CARDINIT` (calls
  ESP-IDF's `sdmmc_card_init()` directly, bypassing FATFS - useful to isolate a MOUNT failure to
  card negotiation vs. the filesystem step), `WIDTH [1|4]` (runtime SD bus-width override, no
  reflash needed), `FWUPDATE`.
- [`cmd_raw.h`](firmware/esp32_host/cmd_raw.h) - the actual point of this firmware: real,
  protocol-level SD commands sent directly via ESP-IDF's `sdmmc_send_cmd()`, bypassing any
  filesystem concept. Manual step-by-step init commands (`CMD0`/`CMD8`/`CMD55`/`ACMD41`/`CMD2`/
  `CMD3`/`CMD9`/`CMD7`/`CMD13`), block I/O (`RDBLOCK`/`WRBLOCK`/`RDMULTI`/`WRMULTI`/`ERASE`),
  card registers (`CARDINFO`/`CARDSTATUS`), and the less-common commands this project actually
  exists to exercise (`SWITCHFUNC`/`LOCK`/`UNLOCK`/`CLRPWD`/`FORCEERASE`/`SETWRPROT`/`CLRWRPROT`/
  `SENDWRPROT`/`VOLTAGESWITCH`/`TUNING`/`GENCMD`/`PREERASE`/`CARDDETECT`), plus a fully generic
  `RAWCMD <index> <arg_hex> <resp_type> [datalen] [read|write]` escape hatch for anything not
  named above.
- [`cmd_fs.h`](firmware/esp32_host/cmd_fs.h) - filesystem-level commands (`LIST`/`MKDIR`/`RMDIR`/
  `RM`/`RENAME`/`STAT`/`PUT`/`GET`) via standard POSIX calls once `MOUNT` succeeds. Expected to
  report "not mounted" until the FPGA under test implements enough of the real init sequence for
  FATFS to actually come up - that's normal, not a bug in this firmware.

### Verified so far (2026-10-01)

Everything below was exercised against a real physical microSD card (a ~58GB SDXC card, not the
Tang Nano FPGA target) seated in the ESP32-CAM's own slot, in addition to the empty-slot clean
error-path testing done earlier:

- Boot/flash (button-free, both via `esptool` and via `FWUPDATE`), `PING`/`INFO`/`REBOOT`.
- Manual step-by-step init: `CMD0`/`CMD8`/`CMD55`/`ACMD41`/`CMD2`/`CMD3`/`CMD9`/`CMD7`/`CMD13` -
  all confirmed against a real card (ACMD41 needs to be retried in a loop until the OCR busy bit
  clears, exactly like the automatic path does internally).
- Automatic negotiation: `CARDINIT` (bypasses FATFS, returns a real `esp_err_t`) and `MOUNT`/
  `FORMAT` (full FATFS path) - real card fully negotiates, CID/CSD/OCR all decode correctly.
- Block I/O: `RDBLOCK`/`WRBLOCK`/`RDMULTI`/`WRMULTI`/`ERASE` - all byte-exact verified
  (write-then-readback matches, `ERASE` correctly fills with `0xFF`).
- Filesystem: `MKDIR`/`RMDIR`/`RM`/`RENAME`/`STAT`/`PUT`/`GET`/`LIST` (including nested paths) -
  all confirmed, including round-tripping real file content.
- The less-common command family: `SWITCHFUNC`, `LOCK`/`UNLOCK`/`CLRPWD`/`FORCEERASE`,
  `SETWRPROT`/`CLRWRPROT`/`SENDWRPROT`, `VOLTAGESWITCH`, `TUNING`, `GENCMD`, `PREERASE`,
  `CARDDETECT` - all confirmed to send correctly and get a real response. `TUNING`,
  `SETWRPROT`/`CLRWRPROT`/`SENDWRPROT`, and `VOLTAGESWITCH` correctly time out/no-op on this
  card, which is the expected outcome (it isn't in a UHS signaling mode and is an SDXC card, which
  doesn't implement the legacy write-protect-group feature) - not a bug. `FORCEERASE` sends and
  completes cleanly but this particular (cheap clone) card doesn't appear to actually erase data
  for it, unlike the regular `ERASE` command which does - a card/vendor quirk, not a firmware bug.
- `RAWCMD` against a real responder (previously only tested against an empty slot).

### Real bugs found and fixed while building this (not just flagged gaps)

- **`FWUPDATE`'s partition scheme**: the AI-Thinker board's default ("Huge APP") has no OTA
  partitions at all - `Update.begin()` failed immediately with "Partition Could Not be Found"
  until the build was switched to `PartitionScheme=min_spiffs`.
- **`FWUPDATE`'s transfer protocol silently desynced**: a naive "reply ready, then the sender
  blasts the whole file" design lost bytes mid-transfer. Root-caused by isolating the exact same
  receive loop into a trivial echo-only sketch with no `Update.write()` calls at all - it received
  385KB cleanly in 4.3 seconds with an 8KB buffer, proving the UART path itself was never the
  problem. `Update.write()` occasionally blocks for well over 100ms during real flash sector
  erases, long enough to overflow even a 16KB RX ring buffer at this baud rate. Fixed properly
  with a per-chunk-acknowledged protocol (device replies "OK ready `<chunk_size>`", sender must
  wait for an explicit "OK" after each chunk before sending the next) rather than just raising the
  buffer size further, since no fixed buffer size is correct in general against an unbounded flash
  operation - an explicit handshake is.
- **GPIO12/flash-voltage strapping conflict**: see "Known hardware limitation" above - found while
  debugging an apparently dead board, which turned out to be entirely explained by a real SD
  card/breakout board sitting in the slot during a reset.
- **`MOUNT` failing against a real card (`mount_to_vfs failed (0xffffffff)`)**: turned out to have
  nothing to do with wiring or bus width (ruled both out with the new `WIDTH` command) - the real
  cause was that the test card held leftover Rockchip firmware-image data, not a FAT/exFAT
  filesystem at all. `sdmmc_card_init()` called directly (see `CARDINIT`) negotiated the card
  perfectly the whole time; only the FATFS-specific step had nothing to recognize. Added `FORMAT`
  (explicit opt-in `format_if_mount_failed=true`) to reformat a card as FAT32 from the firmware
  itself, no separate reader needed.
- **`LOCK`/`UNLOCK`/`CLRPWD`/`FORCEERASE` (CMD42) always timed out and briefly stalled the card**:
  root cause was a block-length mismatch - `sdmmc_card_init()` leaves the card's SET_BLOCKLEN
  register at 512 for normal block I/O, but CMD42's data phase is only 3-4 bytes. Without an
  explicit `CMD16 (SET_BLOCKLEN)` to the small payload size immediately before CMD42 (and back to
  512 immediately after), the card kept waiting for the rest of a 512-byte block that was never
  coming, stalling in the "rcv" state until it eventually gave up. Fixed in `do_lock_family()`
  (`cmd_raw.cpp`). The restore-to-512 call also needs a short retry/delay loop immediately after
  CMD42 - the card needs a brief moment to settle before it accepts another command.
- **A second `CARDINIT` in the same session corrupted the SDMMC peripheral's state**
  (`handle_idle_state_events unhandled`, everything timing out afterward) - `sd_session_card_init()`
  only initialized the host/slot if one wasn't already up, so a second call renegotiated the card
  against a peripheral still carrying state from the first negotiation (and any raw command/CMD42
  activity in between) instead of a clean one. Fixed by always tearing down and rebuilding the
  host first, matching what `sd_session_try_mount_fs()` already did for the same reason.
