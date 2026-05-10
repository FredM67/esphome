#include "emontx_updater.h"

#include <cstdio>
#include <cstring>

#include "esphome/core/application.h"
#include "esphome/core/log.h"
#include "esp_http_client.h"
#include "esp_crt_bundle.h"

namespace esphome::emontx_updater {

static const char *const TAG = "emontx_updater";

// ─── WordCopy applet binary for ARM Cortex-M0+ ───────────────────────────
// Source: BOSSA project, src/WordCopyArm.cpp (ShumaTech, BSD license).
// This 52-byte Thumb-2 program runs on the SAMD21; it copies `words` 32-bit
// words from src_addr to dst_addr. Parameters are embedded in the data region
// at the end of the code blob (offsets 0x20…0x33) and are written via SAM-BA
// 'W' commands before each invocation.
static const uint8_t WORD_COPY_APPLET[APPLET_CODE_SIZE] = {
    // Executable Thumb-2 instructions (bytes 0x00–0x1F, 32 bytes / 16 words)
    0x09, 0x48,  // ldr  r0, [pc, #36]  ; = dst_addr
    0x0a, 0x49,  // ldr  r1, [pc, #40]  ; = src_addr
    0x0a, 0x4a,  // ldr  r2, [pc, #40]  ; = words
    0x02, 0xe0,  // b    +8             ; branch over loop body (first iteration pre-check)
    0x08, 0xc9,  // ldmia r1!, {r3}    ; load 1 word from src (advances r1 by 4)
    0x08, 0xc0,  // stmia r0!, {r3}    ; store 1 word to dst  (advances r0 by 4)
    0x01, 0x3a,  // subs  r2, r2, #1
    0x00, 0x2a,  // cmp   r2, #0
    0xfa, 0xd1,  // bne   -12           ; loop
    0x04, 0x48,  // ldr   r0, [pc, #16] ; = stack (used as "completion signal" check)
    0x00, 0x28,  // cmp   r0, #0
    0x01, 0xd1,  // bne   +4
    0x01, 0x48,  // ldr   r0, [pc, #4]
    0x85, 0x46,  // mov   sp, r0        ; restore SP from parameter area
    0x70, 0x47,  // bx    lr            ; return to SAM-BA monitor
    0xc0, 0x46,  // nop                 ; padding to reach 4-byte alignment
    // Parameter region (bytes 0x20–0x33), initialised to zero; filled by host:
    0x00, 0x00, 0x00, 0x00,  // [+0x20] initial stack pointer  (= SAMD21_STACK_ADDR)
    0x00, 0x00, 0x00, 0x00,  // [+0x24] reset vector / PC+1    (= SAMD21_APPLET_ADDR+1)
    0x00, 0x00, 0x00, 0x00,  // [+0x28] dst_addr
    0x00, 0x00, 0x00, 0x00,  // [+0x2C] src_addr
    0x00, 0x00, 0x00, 0x00,  // [+0x30] words count            (= PAGE_SIZE/4 = 16)
};

// ─── CRC-16/CCITT lookup table (from BOSSA Samba.cpp, ShumaTech, BSD) ────
static const uint16_t CRC16_TABLE[256] = {
    0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50a5, 0x60c6, 0x70e7,
    0x8108, 0x9129, 0xa14a, 0xb16b, 0xc18c, 0xd1ad, 0xe1ce, 0xf1ef,
    0x1231, 0x0210, 0x3273, 0x2252, 0x52b5, 0x4294, 0x72f7, 0x62d6,
    0x9339, 0x8318, 0xb37b, 0xa35a, 0xd3bd, 0xc39c, 0xf3ff, 0xe3de,
    0x2462, 0x3443, 0x0420, 0x1401, 0x64e6, 0x74c7, 0x44a4, 0x5485,
    0xa56a, 0xb54b, 0x8528, 0x9509, 0xe5ee, 0xf5cf, 0xc5ac, 0xd58d,
    0x3653, 0x2672, 0x1611, 0x0630, 0x76d7, 0x66f6, 0x5695, 0x46b4,
    0xb75b, 0xa77a, 0x9719, 0x8738, 0xf7df, 0xe7fe, 0xd79d, 0xc7bc,
    0x48c4, 0x58e5, 0x6886, 0x78a7, 0x0840, 0x1861, 0x2802, 0x3823,
    0xc9cc, 0xd9ed, 0xe98e, 0xf9af, 0x8948, 0x9969, 0xa90a, 0xb92b,
    0x5af5, 0x4ad4, 0x7ab7, 0x6a96, 0x1a71, 0x0a50, 0x3a33, 0x2a12,
    0xdbfd, 0xcbdc, 0xfbbf, 0xeb9e, 0x9b79, 0x8b58, 0xbb3b, 0xab1a,
    0x6ca6, 0x7c87, 0x4ce4, 0x5cc5, 0x2c22, 0x3c03, 0x0c60, 0x1c41,
    0xedae, 0xfd8f, 0xcdec, 0xddcd, 0xad2a, 0xbd0b, 0x8d68, 0x9d49,
    0x7e97, 0x6eb6, 0x5ed5, 0x4ef4, 0x3e13, 0x2e32, 0x1e51, 0x0e70,
    0xff9f, 0xefbe, 0xdfdd, 0xcffc, 0xbf1b, 0xaf3a, 0x9f59, 0x8f78,
    0x9188, 0x81a9, 0xb1ca, 0xa1eb, 0xd10c, 0xc12d, 0xf14e, 0xe16f,
    0x1080, 0x00a1, 0x30c2, 0x20e3, 0x5004, 0x4025, 0x7046, 0x6067,
    0x83b9, 0x9398, 0xa3fb, 0xb3da, 0xc33d, 0xd31c, 0xe37f, 0xf35e,
    0x02b1, 0x1290, 0x22f3, 0x32d2, 0x4235, 0x5214, 0x6277, 0x7256,
    0xb5ea, 0xa5cb, 0x95a8, 0x8589, 0xf56e, 0xe54f, 0xd52c, 0xc50d,
    0x34e2, 0x24c3, 0x14a0, 0x0481, 0x7466, 0x6447, 0x5424, 0x4405,
    0xa7db, 0xb7fa, 0x8799, 0x97b8, 0xe75f, 0xf77e, 0xc71d, 0xd73c,
    0x26d3, 0x36f2, 0x0691, 0x16b0, 0x6657, 0x7676, 0x4615, 0x5634,
    0xd94c, 0xc96d, 0xf90e, 0xe92f, 0x99c8, 0x89e9, 0xb98a, 0xa9ab,
    0x5844, 0x4865, 0x7806, 0x6827, 0x18c0, 0x08e1, 0x3882, 0x28a3,
    0xcb7d, 0xdb5c, 0xeb3f, 0xfb1e, 0x8bf9, 0x9bd8, 0xabbb, 0xbb9a,
    0x4a75, 0x5a54, 0x6a37, 0x7a16, 0x0af1, 0x1ad0, 0x2ab3, 0x3a92,
    0xfd2e, 0xed0f, 0xdd6c, 0xcd4d, 0xbdaa, 0xad8b, 0x9de8, 0x8dc9,
    0x7c26, 0x6c07, 0x5c64, 0x4c45, 0x3ca2, 0x2c83, 0x1ce0, 0x0cc1,
    0xef1f, 0xff3e, 0xcf5d, 0xdf7c, 0xaf9b, 0xbfba, 0x8fd9, 0x9ff8,
    0x6e17, 0x7e36, 0x4e55, 0x5e74, 0x2e93, 0x3eb2, 0x0ed1, 0x1ef0,
};

// ═════════════════════════════════════════════════════════════════════════════
// Component lifecycle
// ═════════════════════════════════════════════════════════════════════════════

void EmonTxUpdater::setup() {
  register_service(&EmonTxUpdater::on_flash_firmware_, "flash_emontx6", {"url"});
}

void EmonTxUpdater::dump_config() {
  ESP_LOGCONFIG(TAG, "EmonTx Updater:");
  ESP_LOGCONFIG(TAG, "  Bootloader timeout: %u ms", this->bootloader_timeout_ms_);
  if (this->dry_run_)
    ESP_LOGW(TAG, "  DRY RUN enabled — bootloader will NOT be entered");
  ESP_LOGCONFIG(TAG, "  IMPORTANT: OTA flash requires the UART SAM-BA bootloader on the emonTx6.");
  ESP_LOGCONFIG(TAG, "  If this is the first OTA flash, install it once via USB:");
  ESP_LOGCONFIG(TAG, "  copy change-bootloader-uart.uf2 from the emon32-fw release onto the EMONBOOT drive.");
}

// ═════════════════════════════════════════════════════════════════════════════
// HA service handler
// ═════════════════════════════════════════════════════════════════════════════

void EmonTxUpdater::on_flash_firmware_(std::string url) {
  ESP_LOGI(TAG, "Flash request received for URL: %s", url.c_str());
  this->fire_flash_status_("started", 0, "Downloading firmware");

  // 1. Download firmware into ESP32 RAM (~60 KB).
  std::vector<uint8_t> firmware;
  if (!this->download_firmware_(url, firmware)) {
    ESP_LOGE(TAG, "Firmware download failed — aborting");
    this->fire_flash_status_("failed", 0, "Firmware download failed");
    return;
  }
  ESP_LOGI(TAG, "Downloaded %zu bytes", firmware.size());
  this->fire_flash_status_("flashing", 10, "Firmware downloaded, validating");

  // 2. Validate the binary before touching the SAMD21.
  //    This is the last safe abort point — after this we enter bootloader mode.
  if (!this->validate_firmware_(firmware)) {
    ESP_LOGE(TAG, "Firmware validation failed — aborting before bootloader entry");
    this->fire_flash_status_("failed", 0, "Firmware validation failed — device untouched");
    return;
  }
  this->fire_flash_status_("flashing", 15, "Firmware valid, entering bootloader");

  // DRY RUN: stop here without touching the SAMD21 (remove dry_run: true from YAML when done).
  if (this->dry_run_) {
    ESP_LOGW(TAG, "DRY RUN — skipping bootloader entry and flash (dry_run: true in YAML)");
    this->fire_flash_status_("complete", 100, "Dry run complete — device was NOT touched");
    return;
  }

  // 2. Pause the emontx parser so it does not consume SAM-BA response bytes.
  this->emontx_->set_paused(true);

  // 3. Enter SAM-BA bootloader mode via the emon32 firmware two-step command.
  //    Source: emon32-fw src/configuration.c lines 853–862, 1195–1212.
  //    Step 1: send "e\r\n" — firmware prints a confirmation prompt.
  ESP_LOGI(TAG, "Sending bootloader entry step 1 ('e')");
  {
    const uint8_t cmd_e[] = {'e', '\r', '\n'};
    this->uart_write_(cmd_e, sizeof(cmd_e));
  }
  // Wait ~200 ms for the firmware to print the confirmation prompt.
  delay(200);
  // Step 2: confirm with "y\r\n" — firmware writes the magic key
  //         (0xF01669EF → 0x20003FFC) and calls NVIC_SystemReset().
  ESP_LOGI(TAG, "Sending bootloader entry step 2 ('y')");
  {
    const uint8_t cmd_y[] = {'y', '\r', '\n'};
    this->uart_write_(cmd_y, sizeof(cmd_y));
  }
  ESP_LOGI(TAG, "Bootloader entry sent — waiting %u ms for SAM-BA UART monitor to start",
           this->bootloader_timeout_ms_);
  // Feed the watchdog every 100 ms so the TWDT (default 3 s) does not fire
  // during this multi-second wait.
  for (uint32_t elapsed = 0; elapsed < this->bootloader_timeout_ms_; elapsed += 100) {
    delay(100);
    App.feed_wdt();
  }

  // 4. Flash.
  bool ok = this->do_flash_(firmware);

  // 5. Resume normal emonTx serial parsing regardless of flash outcome.
  this->emontx_->set_paused(false);

  if (ok) {
    ESP_LOGI(TAG, "Firmware flash complete — emonTx6 rebooting into new firmware");
    this->fire_flash_status_("complete", 100, "Firmware flash complete");
  } else {
    ESP_LOGE(TAG, "Firmware flash FAILED. Power-cycle the emonTx6 to recover.");
    this->fire_flash_status_("failed", 0, "Firmware flash failed — check ESPHome logs");
  }
}

// ═════════════════════════════════════════════════════════════════════════════
// Download phase
// ═════════════════════════════════════════════════════════════════════════════

std::string EmonTxUpdater::resolve_redirect_(const std::string &url) {
  // GitHub release URLs may chain through multiple redirects, each returning large
  // security headers (CSP, cookies, etc.) that exceed http_request's buffer.
  // Follow every hop using esp_http_client directly with a 32 KB buffer, fetching
  // only headers (not the body) at each step, until we reach a non-3xx response.
  // http_request then downloads from the final URL which has minimal headers.
  struct Ctx {
    std::string location;
  } ctx;

  auto location_handler = [](esp_http_client_event_t *evt) -> esp_err_t {
    if (evt->event_id == HTTP_EVENT_ON_HEADER && strcasecmp(evt->header_key, "Location") == 0)
      static_cast<Ctx *>(evt->user_data)->location = evt->header_value;
    return ESP_OK;
  };

  std::string current = url;

  for (int hop = 0; hop < 5; hop++) {
    ctx.location.clear();

    esp_http_client_config_t cfg = {};
    cfg.url = current.c_str();
    cfg.disable_auto_redirect = true;
    cfg.buffer_size = 32768;
    cfg.buffer_size_tx = 4096;
    cfg.timeout_ms = 5000;
    cfg.crt_bundle_attach = esp_crt_bundle_attach;
    cfg.user_data = &ctx;
    cfg.event_handler = location_handler;

    esp_http_client_handle_t client = esp_http_client_init(&cfg);
    if (!client) {
      ESP_LOGW(TAG, "resolve_redirect_: init failed at hop %d", hop);
      break;
    }

    bool redirect = false;
    esp_err_t err = esp_http_client_open(client, 0);
    if (err == ESP_OK) {
      esp_http_client_fetch_headers(client);
      int status = esp_http_client_get_status_code(client);
      ESP_LOGD(TAG, "resolve_redirect_ hop %d: HTTP %d", hop, status);
      if (status >= 300 && status < 400 && !ctx.location.empty()) {
        current = ctx.location;
        redirect = true;
        ESP_LOGI(TAG, "  → %s", current.c_str());
      }
    } else {
      ESP_LOGW(TAG, "resolve_redirect_ hop %d open failed: %s", hop, esp_err_to_name(err));
    }

    esp_http_client_close(client);
    esp_http_client_cleanup(client);

    if (!redirect)
      break;
  }

  if (current != url)
    ESP_LOGI(TAG, "Final download URL: %s", current.c_str());

  return current;
}

bool EmonTxUpdater::download_firmware_(const std::string &url, std::vector<uint8_t> &out) {
  const std::string download_url = this->resolve_redirect_(url);
  ESP_LOGI(TAG, "Downloading firmware from %s", download_url.c_str());

  auto container = this->http_->get(download_url);
  if (!container || container->status_code != 200) {
    ESP_LOGE(TAG, "HTTP GET failed (status %d)", container ? container->status_code : -1);
    if (container)
      container->end();
    return false;
  }

  size_t total = container->content_length;
  if (total == 0) {
    ESP_LOGE(TAG, "Content-Length is 0 or missing");
    container->end();
    return false;
  }
  // Guard against absurdly large files (>256 KB) — SAMD21J17 has 128 KB flash.
  if (total > 256 * 1024u) {
    ESP_LOGE(TAG, "Firmware too large (%zu bytes)", total);
    container->end();
    return false;
  }

  out.resize(total);
  auto result = http_request::http_read_fully(container.get(), out.data(), total, 256, 30000);
  container->end();

  if (result.status != http_request::HttpReadStatus::OK) {
    ESP_LOGE(TAG, "HTTP read failed (error %d)", result.error_code);
    out.clear();
    return false;
  }
  return true;
}

// ═════════════════════════════════════════════════════════════════════════════
// Firmware validation
// ═════════════════════════════════════════════════════════════════════════════

bool EmonTxUpdater::validate_firmware_(const std::vector<uint8_t> &firmware) {
  const size_t fw_size = firmware.size();

  // ── Size bounds ───────────────────────────────────────────────────────────
  // Minimum: must be at least two vector table entries (8 bytes).
  // Maximum: SAMD21J17 application area = 128 KB total - 8 KB bootloader = 120 KB.
  static constexpr size_t FW_MIN_SIZE = 8u;
  static constexpr size_t FW_MAX_SIZE = 120u * 1024u;

  if (fw_size < FW_MIN_SIZE) {
    ESP_LOGE(TAG, "Firmware too small (%zu bytes)", fw_size);
    return false;
  }
  if (fw_size > FW_MAX_SIZE) {
    ESP_LOGE(TAG, "Firmware too large (%zu bytes, max %zu)", fw_size, FW_MAX_SIZE);
    return false;
  }

  // ── ARM Cortex-M0+ vector table (first 8 bytes of the .bin) ──────────────
  // [0..3] Initial stack pointer — must point into SAMD21J17 SRAM
  //        (0x20000000 … 0x20004000, 16 KB).
  // [4..7] Reset vector — must be an odd address (Thumb bit set) within the
  //        application flash region (0x00002001 … 0x0001FFFF).
  const uint32_t sp = (static_cast<uint32_t>(firmware[3]) << 24) |
                      (static_cast<uint32_t>(firmware[2]) << 16) |
                      (static_cast<uint32_t>(firmware[1]) << 8)  |
                      static_cast<uint32_t>(firmware[0]);

  const uint32_t reset_vec = (static_cast<uint32_t>(firmware[7]) << 24) |
                              (static_cast<uint32_t>(firmware[6]) << 16) |
                              (static_cast<uint32_t>(firmware[5]) << 8)  |
                              static_cast<uint32_t>(firmware[4]);

  static constexpr uint32_t SRAM_BASE = 0x20000000u;
  static constexpr uint32_t SRAM_TOP  = 0x20004000u;  // 16 KB
  static constexpr uint32_t APP_END   = 0x00020000u;  // 128 KB flash top

  if (sp < SRAM_BASE || sp > SRAM_TOP) {
    ESP_LOGE(TAG, "Invalid vector table: SP=0x%08X is outside SRAM (0x%08X–0x%08X)",
             sp, SRAM_BASE, SRAM_TOP);
    return false;
  }
  if ((reset_vec & 0x1u) == 0u) {
    ESP_LOGE(TAG, "Invalid vector table: reset vector 0x%08X has no Thumb bit", reset_vec);
    return false;
  }
  if ((reset_vec & ~0x1u) < SAMD21_APP_ADDR || (reset_vec & ~0x1u) >= APP_END) {
    ESP_LOGE(TAG, "Invalid vector table: reset vector 0x%08X outside app flash (0x%08X–0x%08X)",
             reset_vec, SAMD21_APP_ADDR, APP_END);
    return false;
  }

  ESP_LOGI(TAG, "Firmware validated: %zu bytes, SP=0x%08X, PC=0x%08X", fw_size, sp, reset_vec);
  return true;
}

// ═════════════════════════════════════════════════════════════════════════════
// Top-level flash orchestration
// ═════════════════════════════════════════════════════════════════════════════

bool EmonTxUpdater::do_flash_(const std::vector<uint8_t> &firmware) {
  const size_t fw_size = firmware.size();

  // Compute number of complete + partial pages needed.
  const uint32_t pages_needed = (fw_size + SAMD21_PAGE_SIZE - 1) / SAMD21_PAGE_SIZE;
  ESP_LOGI(TAG, "Flashing %zu bytes → %u pages (%u rows)", fw_size, pages_needed,
           (pages_needed + SAMD21_PAGES_PER_ROW - 1) / SAMD21_PAGES_PER_ROW);

  // ── SAM-BA handshake ──────────────────────────────────────────────────────
  if (!this->samba_init_()) {
    ESP_LOGE(TAG, "SAM-BA init failed");
    return false;
  }

  // ── Verify device responds with expected SAMD21 version string ────────────
  std::string ver;
  if (this->samba_version_(ver)) {
    ESP_LOGI(TAG, "SAM-BA version: %s", ver.c_str());
    this->fire_flash_status_("flashing", 20, "SAM-BA ready: " + ver);
  } else {
    this->fire_flash_status_("flashing", 20, "SAM-BA ready");
  }

  // ── Disable NVM cache and enable manual page write (once for all pages) ───
  uint32_t ctrlb_orig = 0;
  if (!this->samba_read_word_(NVM_BASE + NVM_REG_CTRLB, ctrlb_orig)) {
    ESP_LOGE(TAG, "Failed to read NVM CTRLB");
    return false;
  }
  if (!this->samba_write_word_(NVM_BASE + NVM_REG_CTRLB,
                               ctrlb_orig | NVM_CTRLB_CACHE_BIT | NVM_CTRLB_MANW_BIT)) {
    return false;
  }

  // ── Upload WordCopy applet to SAMD21 SRAM ─────────────────────────────────
  if (!this->flash_upload_applet_()) {
    ESP_LOGE(TAG, "Applet upload failed");
    return false;
  }
  this->fire_flash_status_("flashing", 25, "Writing flash pages");

  // ── Page-by-page write ────────────────────────────────────────────────────
  // Ping-pong between the two SRAM page buffers (A/B) to slightly improve
  // throughput: while SAM-BA processes NVM_CMD_WP for one page, the host can
  // begin uploading the next page's data into the other buffer.  In practice
  // both operations are fast, so this is mostly a correctness measure.
  bool use_buf_a = true;
  uint8_t page_data[SAMD21_PAGE_SIZE];

  for (uint32_t page = 0; page < pages_needed; page++) {
    App.feed_wdt();

    // Erase the row at the start of every 4-page group.
    if (page % SAMD21_PAGES_PER_ROW == 0) {
      uint32_t row_byte_addr = SAMD21_APP_ADDR + page * SAMD21_PAGE_SIZE;
      ESP_LOGD(TAG, "Erasing row %u (addr 0x%08X)", page / SAMD21_PAGES_PER_ROW, row_byte_addr);
      if (!this->nvm_erase_row_(row_byte_addr)) {
        ESP_LOGE(TAG, "Row erase failed at page %u", page);
        return false;
      }
    }

    // Copy page data; pad the last (possibly partial) page with 0xFF.
    size_t offset = (size_t) page * SAMD21_PAGE_SIZE;
    size_t avail = (fw_size > offset) ? (fw_size - offset) : 0u;
    size_t copy_len = (avail < SAMD21_PAGE_SIZE) ? avail : SAMD21_PAGE_SIZE;
    memcpy(page_data, firmware.data() + offset, copy_len);
    if (copy_len < SAMD21_PAGE_SIZE)
      memset(page_data + copy_len, 0xFF, SAMD21_PAGE_SIZE - copy_len);

    if (!this->nvm_write_page_(page, page_data, use_buf_a)) {
      ESP_LOGE(TAG, "Page write failed at page %u", page);
      return false;
    }
    use_buf_a = !use_buf_a;

    if (page % 64 == 0) {
      int pct = 25 + static_cast<int>(page * 70u / pages_needed);
      char msg[32];
      snprintf(msg, sizeof(msg), "Page %u / %u", page, pages_needed);
      this->fire_flash_status_("flashing", pct, msg);
      ESP_LOGI(TAG, "Progress: %u/%u pages", page, pages_needed);
    }
  }

  ESP_LOGI(TAG, "All pages written — resetting SAMD21");
  this->fire_flash_status_("flashing", 95, "Resetting device");

  // ── Software reset via ARM AIRCR ──────────────────────────────────────────
  // writeWord will likely cause a SAM-BA timeout since the device resets
  // immediately; this is expected.
  this->samba_write_word_(ARM_AIRCR_ADDR, ARM_AIRCR_RESET);

  return true;
}

// ═════════════════════════════════════════════════════════════════════════════
// SAM-BA protocol layer
// ═════════════════════════════════════════════════════════════════════════════

bool EmonTxUpdater::samba_init_() {
  this->uart_flush_rx_();

  // RS-232 auto-baud sequence: two 0x80 bytes so the SAM-BA monitor can lock
  // onto the baud rate, followed by '#' which triggers a "\n\r" response.
  uint8_t resp;
  this->emontx_->write_byte(0x80);
  this->uart_read_byte_(resp, 200);  // discard echo / response

  this->emontx_->write_byte(0x80);
  this->uart_read_byte_(resp, 200);

  this->emontx_->write_byte('#');
  // SAM-BA responds with "\n\r" (2 bytes); read and discard up to 4 bytes.
  uint8_t discard[4];
  this->uart_read_bytes_(discard, 3, 300);

  // Switch to non-terminal (binary) mode: "N#" → response "\n\r".
  const uint8_t n_cmd[] = {'N', '#'};
  this->uart_write_(n_cmd, sizeof(n_cmd));
  this->uart_read_bytes_(discard, 2, 300);

  // Verify via version command.
  std::string ver;
  if (!this->samba_version_(ver)) {
    ESP_LOGE(TAG, "SAM-BA version query failed — device not in bootloader mode?");
    return false;
  }
  ESP_LOGD(TAG, "SAM-BA handshake OK, version: %s", ver.c_str());
  return true;
}

bool EmonTxUpdater::samba_version_(std::string &ver_out) {
  const uint8_t cmd[] = {'V', '#'};
  this->uart_write_(cmd, sizeof(cmd));

  // Read printable characters until a non-printable byte or timeout (100 ms).
  ver_out.clear();
  uint8_t b;
  uint32_t deadline = millis() + 300;
  while (millis() < deadline) {
    if (!this->uart_read_byte_(b, 50))
      break;
    if (b < 0x20 || b > 0x7E)
      break;
    ver_out += static_cast<char>(b);
    if (ver_out.size() > 200)
      break;
  }
  return !ver_out.empty();
}

bool EmonTxUpdater::samba_write_word_(uint32_t addr, uint32_t value) {
  // "W%08X,%08X#" — 19 chars; no response over UART.
  char cmd[20];
  snprintf(cmd, sizeof(cmd), "W%08" PRIX32 ",%08" PRIX32 "#", addr, value);
  this->uart_write_(reinterpret_cast<const uint8_t *>(cmd), 19);
  return true;
}

bool EmonTxUpdater::samba_read_word_(uint32_t addr, uint32_t &value_out) {
  // "w%08X,4#" — 12 chars; response is 4 raw bytes (little-endian uint32).
  char cmd[13];
  snprintf(cmd, sizeof(cmd), "w%08" PRIX32 ",4#", addr);
  this->uart_write_(reinterpret_cast<const uint8_t *>(cmd), 12);

  uint8_t buf[4];
  if (!this->uart_read_bytes_(buf, 4, 500)) {
    ESP_LOGE(TAG, "samba_read_word_ timeout (addr=0x%08X)", addr);
    return false;
  }
  value_out = (static_cast<uint32_t>(buf[3]) << 24) |
              (static_cast<uint32_t>(buf[2]) << 16) |
              (static_cast<uint32_t>(buf[1]) << 8) |
              static_cast<uint32_t>(buf[0]);
  return true;
}

bool EmonTxUpdater::samba_write_(uint32_t addr, const uint8_t *data, size_t size) {
  // "S%08X,%08X#" — 19 chars, then XModem-CRC transfer.
  char cmd[20];
  snprintf(cmd, sizeof(cmd), "S%08" PRIX32 ",%08" PRIX32 "#", addr, (uint32_t) size);
  this->uart_write_(reinterpret_cast<const uint8_t *>(cmd), 19);
  return this->xmodem_send_(data, size);
}

bool EmonTxUpdater::samba_go_(uint32_t addr) {
  // "G%08X#" — 10 chars; no response expected over UART.
  char cmd[11];
  snprintf(cmd, sizeof(cmd), "G%08" PRIX32 "#", addr);
  this->uart_write_(reinterpret_cast<const uint8_t *>(cmd), 10);
  return true;
}

// ═════════════════════════════════════════════════════════════════════════════
// XModem-CRC sender
// ═════════════════════════════════════════════════════════════════════════════

uint16_t EmonTxUpdater::crc16_(const uint8_t *data, size_t len) {
  uint16_t crc = 0;
  while (len-- > 0)
    crc = (crc << 8) ^ CRC16_TABLE[((crc >> 8) ^ *data++) & 0xFF];
  return crc;
}

bool EmonTxUpdater::xmodem_send_(const uint8_t *data, size_t size) {
  // Wait for receiver to send 'C' (CRC mode request).
  uint8_t start_byte;
  bool got_start = false;
  for (int i = 0; i < XMODEM_MAX_RETRIES; i++) {
    if (this->uart_read_byte_(start_byte, 3000) && start_byte == XMODEM_CRC_START) {
      got_start = true;
      break;
    }
  }
  if (!got_start) {
    ESP_LOGE(TAG, "XModem: no CRC start ('C') received from SAM-BA");
    return false;
  }

  uint8_t frame[XMODEM_FRAME_SZ];
  uint8_t blk_num = 1;
  size_t  sent = 0;

  while (sent < size || (sent == 0 && size == 0)) {
    // Build the 128-byte payload: real data + zero padding.
    size_t chunk = (size - sent > XMODEM_BLK_SIZE) ? XMODEM_BLK_SIZE : (size - sent);
    frame[0] = XMODEM_SOH;
    frame[1] = blk_num;
    frame[2] = static_cast<uint8_t>(~blk_num);
    memcpy(&frame[3], data + sent, chunk);
    if (chunk < XMODEM_BLK_SIZE)
      memset(&frame[3] + chunk, 0x00, XMODEM_BLK_SIZE - chunk);

    uint16_t crc = crc16_(&frame[3], XMODEM_BLK_SIZE);
    frame[3 + XMODEM_BLK_SIZE]     = static_cast<uint8_t>(crc >> 8);
    frame[3 + XMODEM_BLK_SIZE + 1] = static_cast<uint8_t>(crc & 0xFF);

    // Transmit block with retries.
    bool block_ok = false;
    for (int retry = 0; retry < XMODEM_MAX_RETRIES; retry++) {
      this->uart_write_(frame, XMODEM_FRAME_SZ);
      uint8_t ack;
      if (this->uart_read_byte_(ack, 2000)) {
        if (ack == XMODEM_ACK) {
          block_ok = true;
          break;
        } else if (ack == XMODEM_NAK) {
          ESP_LOGW(TAG, "XModem: NAK on block %u, retrying", blk_num);
        }
      } else {
        ESP_LOGW(TAG, "XModem: timeout waiting for ACK on block %u", blk_num);
      }
    }
    if (!block_ok) {
      ESP_LOGE(TAG, "XModem: block %u failed after %d retries", blk_num, XMODEM_MAX_RETRIES);
      return false;
    }

    sent += chunk;
    blk_num++;

    // If we've sent all the data and the last chunk was exactly XMODEM_BLK_SIZE,
    // we're done — exit the loop.
    if (chunk == XMODEM_BLK_SIZE && sent >= size)
      break;
  }

  // EOT handshake.
  for (int retry = 0; retry < XMODEM_MAX_RETRIES; retry++) {
    this->emontx_->write_byte(XMODEM_EOT);
    uint8_t ack;
    if (this->uart_read_byte_(ack, 2000) && ack == XMODEM_ACK)
      return true;
  }
  ESP_LOGE(TAG, "XModem: EOT not acknowledged");
  return false;
}

// ═════════════════════════════════════════════════════════════════════════════
// D2x NVM flash layer
// ═════════════════════════════════════════════════════════════════════════════

bool EmonTxUpdater::flash_upload_applet_() {
  ESP_LOGD(TAG, "Uploading WordCopy applet to 0x%08X", SAMD21_APPLET_ADDR);

  // Upload the 52-byte applet code to SAMD21 SRAM via XModem.
  if (!this->samba_write_(SAMD21_APPLET_ADDR, WORD_COPY_APPLET, APPLET_CODE_SIZE)) {
    ESP_LOGE(TAG, "Applet XModem transfer failed");
    return false;
  }

  // Set the initial stack pointer (= top of SAMD21 SRAM).
  if (!this->samba_write_word_(SAMD21_APPLET_ADDR + APPLET_STACK_OFF, SAMD21_STACK_ADDR))
    return false;

  // Set the words-to-copy count (one page = 16 32-bit words).
  if (!this->samba_write_word_(SAMD21_APPLET_ADDR + APPLET_WORDS_OFF,
                               SAMD21_PAGE_SIZE / sizeof(uint32_t)))
    return false;

  ESP_LOGD(TAG, "WordCopy applet ready (page buffers A=0x%08X B=0x%08X)",
           PAGE_BUFFER_A, PAGE_BUFFER_B);
  return true;
}

bool EmonTxUpdater::nvm_wait_ready_(uint32_t timeout_ms) {
  uint32_t deadline = millis() + timeout_ms;
  while (millis() < deadline) {
    uint32_t intflag = 0;
    if (!this->samba_read_word_(NVM_BASE + NVM_REG_INTFLAG, intflag))
      return false;
    if (intflag & 0x1u)  // READY bit
      return true;
    App.feed_wdt();
  }
  ESP_LOGE(TAG, "NVM not ready (timeout %u ms)", timeout_ms);
  return false;
}

bool EmonTxUpdater::nvm_command_(uint8_t cmd) {
  if (!this->nvm_wait_ready_())
    return false;
  if (!this->samba_write_word_(NVM_BASE + NVM_REG_CTRLA,
                               static_cast<uint32_t>(NVM_CMDEX_KEY | cmd)))
    return false;
  if (!this->nvm_wait_ready_())
    return false;

  // Check for errors in INTFLAG bit 1.
  uint32_t intflag = 0;
  if (!this->samba_read_word_(NVM_BASE + NVM_REG_INTFLAG, intflag))
    return false;
  if (intflag & 0x2u) {
    ESP_LOGE(TAG, "NVM command 0x%02X produced an error (INTFLAG=0x%08X)", cmd, intflag);
    // Clear the error bit.
    this->samba_write_word_(NVM_BASE + NVM_REG_INTFLAG, 0x2u);
    return false;
  }
  return true;
}

bool EmonTxUpdater::nvm_erase_row_(uint32_t row_byte_addr) {
  if (!this->nvm_wait_ready_())
    return false;

  // Clear any pending error bits in STATUS.
  uint32_t status = 0;
  if (!this->samba_read_word_(NVM_BASE + NVM_REG_STATUS, status))
    return false;
  if (!this->samba_write_word_(NVM_BASE + NVM_REG_STATUS, status | NVM_STATUS_ERR_MASK))
    return false;

  // Write the word address (byte_addr / 2) to ADDR, then issue ERASE ROW.
  if (!this->samba_write_word_(NVM_BASE + NVM_REG_ADDR, row_byte_addr / 2u))
    return false;
  return this->nvm_command_(NVM_CMD_ER);
}

bool EmonTxUpdater::nvm_write_page_(uint32_t page_idx, const uint8_t *data, bool use_buf_a) {
  const uint32_t page_byte_addr = SAMD21_APP_ADDR + page_idx * SAMD21_PAGE_SIZE;
  const uint32_t sram_buf_addr  = use_buf_a ? PAGE_BUFFER_A : PAGE_BUFFER_B;

  // 1. Issue PBC (Page Buffer Clear) to clear the NVM's internal page buffer.
  if (!this->nvm_command_(NVM_CMD_PBC))
    return false;

  // 2. Upload this page's data to the SRAM page buffer via SAM-BA XModem.
  if (!this->samba_write_(sram_buf_addr, data, SAMD21_PAGE_SIZE))
    return false;

  // 3. Invoke the WordCopy applet: copy SRAM buffer → NVM page buffer.
  //    a. Set destination and source addresses in the applet parameter area.
  if (!this->samba_write_word_(SAMD21_APPLET_ADDR + APPLET_DST_ADDR_OFF, page_byte_addr))
    return false;
  if (!this->samba_write_word_(SAMD21_APPLET_ADDR + APPLET_SRC_ADDR_OFF, sram_buf_addr))
    return false;

  //    b. runv(): write the Thumb-mode start address into the reset-vector slot,
  //       then jump to the vector-table entry (stack slot) via SAM-BA 'G'.
  //       The Cortex-M0+ loads SP from [stack_slot] and PC from [reset_slot].
  if (!this->samba_write_word_(SAMD21_APPLET_ADDR + APPLET_RESET_OFF,
                               SAMD21_APPLET_ADDR + 1u /* Thumb bit */))
    return false;
  if (!this->samba_go_(SAMD21_APPLET_ADDR + APPLET_STACK_OFF))
    return false;

  // 4. Wait for NVM to be ready (applet completes in microseconds; NVM is
  //    driven by SAM-BA 'W' commands issued next, which are already buffered
  //    in the SAMD21's UART hardware FIFO while the applet ran).
  if (!this->nvm_wait_ready_())
    return false;

  // 5. Program the NVM page buffer to flash: set ADDR, issue WRITE PAGE.
  if (!this->samba_write_word_(NVM_BASE + NVM_REG_ADDR, page_byte_addr / 2u))
    return false;
  return this->nvm_command_(NVM_CMD_WP);
}

// ═════════════════════════════════════════════════════════════════════════════
// UART helpers
// ═════════════════════════════════════════════════════════════════════════════

void EmonTxUpdater::uart_flush_rx_() {
  uint8_t b;
  while (this->emontx_->available() > 0)
    this->emontx_->read_byte(&b);
}

void EmonTxUpdater::uart_write_(const uint8_t *data, size_t len) {
  this->emontx_->write_array(data, len);
}

bool EmonTxUpdater::uart_read_byte_(uint8_t &byte, uint32_t timeout_ms) {
  uint32_t deadline = millis() + timeout_ms;
  uint32_t wdt_tick = millis();
  while (millis() < deadline) {
    if (this->emontx_->available() > 0 && this->emontx_->read_byte(&byte))
      return true;
    delay(1);
    if (millis() - wdt_tick >= 100) {
      App.feed_wdt();
      wdt_tick = millis();
    }
  }
  return false;
}

bool EmonTxUpdater::uart_read_bytes_(uint8_t *buf, size_t len, uint32_t timeout_ms) {
  uint32_t deadline = millis() + timeout_ms;
  uint32_t wdt_tick = millis();
  size_t   read = 0;
  while (read < len && millis() < deadline) {
    if (this->emontx_->available() > 0 && this->emontx_->read_byte(&buf[read]))
      read++;
    else
      delay(1);
    if (millis() - wdt_tick >= 100) {
      App.feed_wdt();
      wdt_tick = millis();
    }
  }
  return read == len;
}

// ═════════════════════════════════════════════════════════════════════════════
// HA status event helper
// ═════════════════════════════════════════════════════════════════════════════

void EmonTxUpdater::fire_flash_status_(const std::string &status, int progress,
                                       const std::string &message) {
  if (!this->is_connected())
    return;
  this->fire_homeassistant_event("esphome.emontx_flash_status", {
      {"device_id", App.get_name().str()},
      {"status", status},
      {"progress", std::to_string(progress)},
      {"message", message},
  });
}

}  // namespace esphome::emontx_updater
