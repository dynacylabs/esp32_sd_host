#include "sd_session.h"
#include "config.h"

#include <string.h>
#include "driver/sdmmc_host.h"
#include "driver/sdmmc_default_configs.h"
#include "esp_vfs_fat.h"

static sdmmc_host_t s_host_config;
static sdmmc_card_t s_card;
static bool s_host_ready = false;
static bool s_fs_mounted = false;
static sdmmc_card_t *s_fs_card = nullptr;   // populated by esp_vfs_fat_sdmmc_mount on success
static int s_bus_width = SD_USE_4BIT_MODE ? 4 : 1;   // runtime-overridable, see sd_session.h
static int s_max_freq_khz = SDMMC_FREQ_DEFAULT;      // runtime-overridable, see sd_session.h

static void fill_slot_pins(sdmmc_slot_config_t *slot) {
  slot->clk = (gpio_num_t)SD_PIN_CLK;
  slot->cmd = (gpio_num_t)SD_PIN_CMD;
  slot->d0  = (gpio_num_t)SD_PIN_D0;
  if (s_bus_width == 4) {
    slot->d1 = (gpio_num_t)SD_PIN_D1;
    slot->d2 = (gpio_num_t)SD_PIN_D2;
    slot->d3 = (gpio_num_t)SD_PIN_D3;
  }
  slot->width = s_bus_width;
}

void sd_session_set_bus_width(int width) {
  s_bus_width = (width == 4) ? 4 : 1;
  if (s_host_ready) {
    sdmmc_host_deinit_slot(SDMMC_HOST_SLOT_1);
    sdmmc_host_deinit();
    s_host_ready = false;
  }
}

int sd_session_get_bus_width() {
  return s_bus_width;
}

void sd_session_set_max_freq_khz(int khz) {
  s_max_freq_khz = khz;
  if (s_host_ready) {
    sdmmc_host_deinit_slot(SDMMC_HOST_SLOT_1);
    sdmmc_host_deinit();
    s_host_ready = false;
  }
}

int sd_session_get_max_freq_khz() {
  return s_max_freq_khz;
}

bool sd_session_begin_host() {
  if (s_host_ready) {
    return true;
  }

  memset(&s_card, 0, sizeof(s_card));

  sdmmc_host_t host = SDMMC_HOST_DEFAULT();
  host.slot = SDMMC_HOST_SLOT_1;
  host.max_freq_khz = s_max_freq_khz;
  s_host_config = host;

  sdmmc_slot_config_t slot = SDMMC_SLOT_CONFIG_DEFAULT();
  // Override the macro's own per-chip default pins (unusable on most real boards - see
  // config.h's header comment) with this board's actual wiring.
  fill_slot_pins(&slot);

  if (sdmmc_host_init() != ESP_OK) {
    return false;
  }
  if (sdmmc_host_init_slot(SDMMC_HOST_SLOT_1, &slot) != ESP_OK) {
    return false;
  }

  s_card.host = s_host_config;
  s_host_ready = true;
  return true;
}

bool sd_session_host_ready() {
  return s_host_ready;
}

sdmmc_card_t *sd_session_card() {
  // Once the full automatic mount has succeeded, prefer that fully-populated card struct (real
  // RCA/CSD/CID/OCR) over the minimal one - same underlying host/slot either way.
  return s_fs_mounted && s_fs_card ? s_fs_card : &s_card;
}

void sd_session_set_rca(uint16_t rca) {
  s_card.rca = rca;
}

static bool do_mount(bool format_if_mount_failed) {
  if (s_fs_mounted) {
    sd_session_unmount_fs();
  }
  if (!sd_session_begin_host()) {
    return false;
  }

  sdmmc_slot_config_t slot = SDMMC_SLOT_CONFIG_DEFAULT();
  fill_slot_pins(&slot);

  esp_vfs_fat_mount_config_t mount_config = VFS_FAT_MOUNT_DEFAULT_CONFIG();
  mount_config.format_if_mount_failed = format_if_mount_failed;
  mount_config.max_files = 8;

  // esp_vfs_fat_sdmmc_mount() does its OWN host/slot init internally - it will conflict with the
  // host init already done by sd_session_begin_host() above (ESP_ERR_INVALID_STATE on double
  // host init is otherwise likely), so tear down the raw-host-only state first and let this call
  // own the whole sequence from scratch.
  sdmmc_host_deinit_slot(SDMMC_HOST_SLOT_1);
  sdmmc_host_deinit();
  s_host_ready = false;

  esp_err_t err = esp_vfs_fat_sdmmc_mount(SD_MOUNT_POINT, &s_host_config, &slot, &mount_config, &s_fs_card);
  if (err != ESP_OK) {
    s_fs_mounted = false;
    s_fs_card = nullptr;
    // Fall back to raw-host-only mode so RAWCMD testing still works even though the automatic
    // full negotiation failed - this is the expected, normal state until the FPGA under test
    // implements enough of the real init sequence.
    sd_session_begin_host();
    return false;
  }

  s_fs_mounted = true;
  return true;
}

bool sd_session_try_mount_fs() {
  // false: never silently reformat a card we're testing.
  return do_mount(false);
}

bool sd_session_format_and_mount_fs() {
  return do_mount(true);
}

void sd_session_unmount_fs() {
  if (s_fs_mounted) {
    esp_vfs_fat_sdcard_unmount(SD_MOUNT_POINT, s_fs_card);
    s_fs_mounted = false;
    s_fs_card = nullptr;
  }
}

bool sd_session_fs_mounted() {
  return s_fs_mounted;
}

int sd_session_card_init() {
  // Always tear down and rebuild the host/slot first, even if one is already up - confirmed on
  // real hardware that calling sdmmc_card_init() a second time against an already-negotiated host
  // (e.g. after prior RAWCMD/CMD42 lock-family activity changed block length or bus state)
  // corrupts the renegotiation: the card's own state is fine, but the SDMMC peripheral/driver
  // gets confused ("handle_idle_state_events unhandled") and every subsequent command times out.
  // sd_session_try_mount_fs() already does this same deinit-then-rebuild dance before its own
  // negotiation for the same reason.
  if (s_fs_mounted) {
    sd_session_unmount_fs();
  }
  if (s_host_ready) {
    sdmmc_host_deinit_slot(SDMMC_HOST_SLOT_1);
    sdmmc_host_deinit();
    s_host_ready = false;
  }
  if (!sd_session_begin_host()) {
    return ESP_FAIL;
  }
  memset(&s_card, 0, sizeof(s_card));
  s_card.host = s_host_config;
  esp_err_t err = sdmmc_card_init(&s_host_config, &s_card);
  return (int)err;
}
