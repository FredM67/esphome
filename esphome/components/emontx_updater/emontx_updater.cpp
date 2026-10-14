#include "emontx_updater.h"

#include <algorithm>
#include <cinttypes>
#include <cstdio>
#include <cstring>

#include "freertos/task.h"

#include "esphome/core/application.h"
#include "esphome/core/hal.h"
#include "esphome/core/helpers.h"
#include "esphome/core/log.h"
#include "esp_crt_bundle.h"
#include "esp_http_client.h"
#include "esp_image_format.h"
#include "esp_ota_ops.h"

namespace esphome::emontx_updater {

ESPHOME_LOG_TAG(TAG, "emontx_updater");

static constexpr int MAX_REDIRECTS = 5;

static void close_client(esp_http_client_handle_t client) {
  esp_http_client_close(client);
  esp_http_client_cleanup(client);
}

static uint32_t read_le32(const uint8_t *buf) { return encode_uint32(buf[3], buf[2], buf[1], buf[0]); }

void EmonTxUpdater::setup() {
  this->status_queue_ = xQueueCreate(32, sizeof(FlashStatusPayload));
  if (this->status_queue_ == nullptr) {
    ESP_LOGE(TAG, "Failed to create flash status queue — status events will be lost");
  }
  this->register_service(&EmonTxUpdater::on_flash_firmware_, "flash_emontx6", {"url"});
}

void EmonTxUpdater::dump_config() {
  ESP_LOGCONFIG(TAG,
                "EmonTx Updater:\n"
                "  Bootloader timeout: %" PRIu32 " ms\n"
                "  Verify SSL: %s",
                this->bootloader_timeout_ms_, YESNO(this->verify_ssl_));
  if (this->dry_run_) {
    ESP_LOGW(TAG, "  DRY RUN enabled — bootloader will NOT be entered");
  }
  ESP_LOGCONFIG(TAG, "  IMPORTANT: OTA flash requires the UART SAM-BA bootloader on the emonTx6.\n"
                     "  If this is the first OTA flash, install it once via USB:\n"
                     "  copy change-bootloader-uart.uf2 from the emon32-fw release onto the EMONBOOT drive.");
}

// Sends the events queued by fire_flash_status_() to Home Assistant, on the main loop where the API is safe
void EmonTxUpdater::loop() {
  FlashStatusPayload payload;
  while (this->status_queue_ != nullptr && xQueueReceive(this->status_queue_, &payload, 0) == pdTRUE) {
    if (!this->is_connected())
      continue;
    this->fire_homeassistant_event("esphome.emontx_flash_status",
                                   {
                                       {"device_id", App.get_name().str()},
                                       {"status", payload.status},
                                       {"progress", std::to_string(payload.progress)},  // NOLINT
                                       {"message", payload.message},
                                   });
  }
  this->disable_loop();
}

void EmonTxUpdater::on_flash_firmware_(std::string url) {  // NOLINT(performance-unnecessary-value-param)
  if (this->flash_task_running_) {
    ESP_LOGW(TAG, "Flash already in progress — ignoring new request");
    return;
  }

  ESP_LOGI(TAG, "Flash request received for URL: %s", url.c_str());
#ifdef USE_OTA
  if (!this->update_lock_.try_acquire()) {
    ESP_LOGW(TAG, "An ESPHome OTA update is in progress — ignoring flash request");
    this->fire_flash_status_("failed", 0, "ESPHome OTA update in progress");
    return;
  }
#endif
  this->fire_flash_status_("started", 0, "Downloading firmware");

  if (!this->download_firmware_(url)) {
    this->release_update_lock_();
    ESP_LOGE(TAG, "Firmware download failed — aborting");
    this->fire_flash_status_("failed", 0, "Firmware download failed");
    return;
  }
  ESP_LOGI(TAG, "Downloaded %zu bytes", this->staged_size_);
  this->fire_flash_status_("flashing", 10, "Firmware downloaded, validating");

  // Last point where aborting leaves the emonTx untouched
  if (!this->validate_firmware_()) {
    this->release_update_lock_();
    ESP_LOGE(TAG, "Firmware validation failed — aborting before bootloader entry");
    this->fire_flash_status_("failed", 0, "Firmware validation failed — device untouched");
    return;
  }
  this->fire_flash_status_("flashing", 15, "Firmware valid, entering bootloader");

  if (this->dry_run_) {
    this->release_update_lock_();
    ESP_LOGW(TAG, "DRY RUN — skipping bootloader entry and flash (dry_run: true in YAML)");
    this->fire_flash_status_("complete", 100, "Dry run complete — device was NOT touched");
    return;
  }

  // Stop the emontx parser from eating the SAM-BA replies
  this->emontx_->set_paused(true);

  // emon32-fw asks to confirm 'e' with 'y', then writes the bootloader key and resets (src/configuration.c)
  ESP_LOGI(TAG, "Sending bootloader entry step 1 ('e')");
  this->uart_write_("e\r\n", 3);
  delay(500);  // NOLINT
  ESP_LOGI(TAG, "Sending bootloader entry step 2 ('y')");
  this->uart_write_("y\r\n", 3);
  ESP_LOGI(TAG, "Bootloader entry sent — waiting %" PRIu32 " ms for SAM-BA UART monitor to start",
           this->bootloader_timeout_ms_);
  for (uint32_t elapsed = 0; elapsed < this->bootloader_timeout_ms_; elapsed += 100) {
    delay(100);  // NOLINT
    App.feed_wdt();
  }

  // The flash takes about 60 s; a task keeps the main loop and the API running meanwhile
  this->flash_task_running_ = true;
  if (xTaskCreate(flash_task_fn, "emontx_flash", 32768, this, 5, nullptr) != pdPASS) {
    ESP_LOGE(TAG, "Failed to create flash task — aborting");
    this->flash_task_running_ = false;
    this->release_update_lock_();
    this->emontx_->set_paused(false);
    this->fire_flash_status_("failed", 0, "Failed to start flash task");
  }
}

void EmonTxUpdater::flash_task_fn(void *param) {
  auto *self = static_cast<EmonTxUpdater *>(param);

  bool ok = self->do_flash_();
  self->release_update_lock_();
  self->emontx_->set_paused(false);

  if (ok) {
    ESP_LOGI(TAG, "Firmware flash complete — emonTx6 rebooting into new firmware");
    self->fire_flash_status_("complete", 100, "Firmware flash complete");
  } else {
    ESP_LOGE(TAG, "Firmware flash FAILED. Power-cycle the emonTx6 to recover.");
    self->fire_flash_status_("failed", 0, "Firmware flash failed — check ESPHome logs");
  }

  self->flash_task_running_ = false;
  vTaskDelete(nullptr);
}

void EmonTxUpdater::release_update_lock_() {
#ifdef USE_OTA
  this->update_lock_.release();
#endif
}

bool EmonTxUpdater::download_firmware_(const std::string &url) {
  ESP_LOGI(TAG, "Download request for URL: %s", url.c_str());

  // Redirects are followed by hand: auto-redirect only works with esp_http_client_perform(), not with
  // the open/read streaming used here. Keeping the 200 connection open limits the TLS handshakes to two,
  // which the main task stack can afford. Buffers stay at 4 KB: 8 KB or more crash the ESP32-C3 Wi-Fi driver.
  struct Ctx {
    std::string location;
  } ctx;
  auto hdr_handler = [](esp_http_client_event_t *e) -> esp_err_t {
    if (e->event_id == HTTP_EVENT_ON_HEADER && strcasecmp(e->header_key, "Location") == 0)
      static_cast<Ctx *>(e->user_data)->location = e->header_value;
    return ESP_OK;
  };

  std::string current = url;
  esp_http_client_handle_t client = nullptr;
  int64_t content_length = 0;

  for (int hop = 0; hop < MAX_REDIRECTS && client == nullptr; hop++) {
    ctx.location.clear();

    esp_http_client_config_t cfg = {};
    cfg.url = current.c_str();
    cfg.disable_auto_redirect = true;
    cfg.buffer_size = 4096;
    cfg.buffer_size_tx = 4096;
    cfg.timeout_ms = 30000;
    cfg.crt_bundle_attach = this->verify_ssl_ ? esp_crt_bundle_attach : nullptr;
    cfg.skip_cert_common_name_check = !this->verify_ssl_;
    cfg.user_data = &ctx;
    cfg.event_handler = hdr_handler;

    esp_http_client_handle_t hop_client = esp_http_client_init(&cfg);
    if (hop_client == nullptr) {
      ESP_LOGE(TAG, "HTTP client init failed at hop %d", hop);
      return false;
    }
    esp_err_t err = esp_http_client_open(hop_client, 0);
    if (err != ESP_OK) {
      ESP_LOGE(TAG, "HTTP open failed at hop %d: %s", hop, esp_err_to_name(err));
      esp_http_client_cleanup(hop_client);
      return false;
    }

    content_length = esp_http_client_fetch_headers(hop_client);
    int status = esp_http_client_get_status_code(hop_client);
    ESP_LOGD(TAG, "download hop %d: HTTP %d", hop, status);

    if (status >= 300 && status < 400 && !ctx.location.empty()) {
      ESP_LOGI(TAG, "  → %s", ctx.location.c_str());
      current = ctx.location;
      close_client(hop_client);
      continue;
    }
    if (status != 200) {
      ESP_LOGE(TAG, "HTTP GET returned status %d", status);
      close_client(hop_client);
      return false;
    }
    client = hop_client;
  }

  if (client == nullptr) {
    ESP_LOGE(TAG, "Too many redirects");
    return false;
  }
  if (content_length <= 0) {
    ESP_LOGE(TAG, "Content-Length is 0 or missing");
    close_client(client);
    return false;
  }
  bool ok = this->stage_firmware_(client, static_cast<size_t>(content_length));
  close_client(client);
  return ok;
}

bool EmonTxUpdater::stage_firmware_(esp_http_client_handle_t client, size_t total) {
  if (total > FW_MAX_SIZE) {
    ESP_LOGE(TAG, "Firmware too large (%zu bytes, max %zu)", total, FW_MAX_SIZE);
    return false;
  }

  // Stage the image at the end of the spare OTA slot rather than in RAM: a 120 KB contiguous
  // buffer is often unavailable on a fragmented heap. The app image at the start stays for rollback.
  const esp_partition_t *part = esp_ota_get_next_update_partition(nullptr);
  if (part == nullptr) {
    ESP_LOGE(TAG, "No spare OTA partition to stage the firmware");
    return false;
  }
  const size_t sector = part->erase_size;
  size_t image_end = 0;
  esp_image_metadata_t meta = {};
  const esp_partition_pos_t pos = {.offset = part->address, .size = part->size};
  App.feed_wdt();
  if (esp_image_verify(ESP_IMAGE_VERIFY_SILENT, &pos, &meta) == ESP_OK)
    image_end = meta.image_len;
  const size_t offset = total <= part->size ? (part->size - total) / sector * sector : 0;
  if (total > part->size || offset < image_end) {
    ESP_LOGE(TAG, "Not enough room after the app image in partition '%s' (image %zu, slot %" PRIu32 " bytes)",
             part->label, image_end, part->size);
    return false;
  }
  ESP_LOGI(TAG, "Staging firmware in partition '%s' at 0x%zx", part->label, offset);

  for (size_t off = offset; off < offset + total; off += sector) {
    esp_err_t err = esp_partition_erase_range(part, off, sector);
    if (err != ESP_OK) {
      ESP_LOGE(TAG, "Partition erase failed at 0x%zx: %s", off, esp_err_to_name(err));
      return false;
    }
    App.feed_wdt();
  }

  uint8_t buf[512];
  size_t received = 0;
  while (received < total) {
    const size_t want = std::min(sizeof(buf), total - received);
    int len = esp_http_client_read(client, reinterpret_cast<char *>(buf), want);
    if (len < 0) {
      ESP_LOGE(TAG, "HTTP read error at byte %zu", received);
      return false;
    }
    if (len == 0)
      break;
    esp_err_t err = esp_partition_write(part, offset + received, buf, len);
    if (err != ESP_OK) {
      ESP_LOGE(TAG, "Partition write failed at byte %zu: %s", received, esp_err_to_name(err));
      return false;
    }
    received += len;
    App.feed_wdt();
  }
  if (received != total) {
    ESP_LOGE(TAG, "HTTP read incomplete: %zu / %zu bytes", received, total);
    return false;
  }

  this->staging_partition_ = part;
  this->staging_offset_ = offset;
  this->staged_size_ = total;
  return true;
}

bool EmonTxUpdater::validate_firmware_() {
  const size_t fw_size = this->staged_size_;
  // At least the stack pointer and reset vector of the vector table
  static constexpr size_t FW_MIN_SIZE = 8u;
  static constexpr uint32_t SRAM_BASE = 0x20000000u;
  static constexpr uint32_t SRAM_TOP = 0x20004000u;  // 16 KB
  static constexpr uint32_t APP_END = 0x00020000u;   // 128 KB flash

  if (fw_size < FW_MIN_SIZE) {
    ESP_LOGE(TAG, "Firmware too small (%zu bytes)", fw_size);
    return false;
  }
  if (fw_size > FW_MAX_SIZE) {
    ESP_LOGE(TAG, "Firmware too large (%zu bytes, max %zu)", fw_size, FW_MAX_SIZE);
    return false;
  }

  uint8_t vectors[FW_MIN_SIZE];
  esp_err_t err = esp_partition_read(this->staging_partition_, this->staging_offset_, vectors, sizeof(vectors));
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Failed to read staged firmware: %s", esp_err_to_name(err));
    return false;
  }
  const uint32_t sp = read_le32(vectors);
  const uint32_t reset_vec = read_le32(vectors + 4);
  const uint32_t reset_addr = reset_vec & ~0x1u;

  if (sp < SRAM_BASE || sp > SRAM_TOP) {
    ESP_LOGE(TAG, "Invalid vector table: SP=0x%08" PRIX32 " is outside SRAM (0x%08" PRIX32 "–0x%08" PRIX32 ")", sp,
             SRAM_BASE, SRAM_TOP);
    return false;
  }
  if ((reset_vec & 0x1u) == 0u) {
    ESP_LOGE(TAG, "Invalid vector table: reset vector 0x%08" PRIX32 " has no Thumb bit", reset_vec);
    return false;
  }
  if (reset_addr < SAMD21_APP_ADDR || reset_addr >= APP_END) {
    ESP_LOGE(TAG,
             "Invalid vector table: reset vector 0x%08" PRIX32 " outside app flash (0x%08" PRIX32 "–0x%08" PRIX32 ")",
             reset_vec, SAMD21_APP_ADDR, APP_END);
    return false;
  }

  ESP_LOGI(TAG, "Firmware validated: %zu bytes, SP=0x%08" PRIX32 ", PC=0x%08" PRIX32, fw_size, sp, reset_vec);
  return true;
}

bool EmonTxUpdater::do_flash_() {
  const size_t fw_size = this->staged_size_;
  const uint32_t pages_needed = (fw_size + SAMD21_PAGE_SIZE - 1) / SAMD21_PAGE_SIZE;
  ESP_LOGI(TAG, "Flashing %zu bytes → %" PRIu32 " pages (%" PRIu32 " rows)", fw_size, pages_needed,
           (pages_needed + SAMD21_PAGES_PER_ROW - 1) / SAMD21_PAGES_PER_ROW);

  char msg[sizeof(FlashStatusPayload::message)];
  char ver[64];
  if (!this->samba_init_(ver, sizeof(ver))) {
    ESP_LOGE(TAG, "SAM-BA init failed");
    return false;
  }
  if (this->samba_version_(ver, sizeof(ver))) {
    ESP_LOGI(TAG, "SAM-BA version: %s", ver);
    snprintf(msg, sizeof(msg), "SAM-BA ready: %s", ver);
    this->fire_flash_status_("flashing", 20, msg);
  } else {
    this->fire_flash_status_("flashing", 20, "SAM-BA ready");
  }

  // Disable the NVM cache and enable manual page write, once for all pages
  uint32_t ctrlb = 0;
  if (!this->samba_read_word_(NVM_BASE + NVM_REG_CTRLB, ctrlb)) {
    ESP_LOGE(TAG, "Failed to read NVM CTRLB");
    return false;
  }
  this->samba_write_word_(NVM_BASE + NVM_REG_CTRLB, ctrlb | NVM_CTRLB_CACHE_BIT | NVM_CTRLB_MANW_BIT);

  this->fire_flash_status_("flashing", 25, "Writing flash pages");
  uint8_t page_data[SAMD21_PAGE_SIZE];
  for (uint32_t page = 0; page < pages_needed; page++) {
    App.feed_wdt();

    if (page % SAMD21_PAGES_PER_ROW == 0) {
      uint32_t row_byte_addr = SAMD21_APP_ADDR + page * SAMD21_PAGE_SIZE;
      ESP_LOGD(TAG, "Erasing row %" PRIu32 " (addr 0x%08" PRIX32 ")", page / SAMD21_PAGES_PER_ROW, row_byte_addr);
      if (!this->nvm_erase_row_(row_byte_addr)) {
        ESP_LOGE(TAG, "Row erase failed at page %" PRIu32, page);
        return false;
      }
    }

    // Pad the last, partial page with 0xFF
    const size_t offset = static_cast<size_t>(page) * SAMD21_PAGE_SIZE;
    const size_t copy_len = std::min<size_t>(fw_size - offset, SAMD21_PAGE_SIZE);
    esp_err_t err = esp_partition_read(this->staging_partition_, this->staging_offset_ + offset, page_data, copy_len);
    if (err != ESP_OK) {
      ESP_LOGE(TAG, "Failed to read staged firmware at byte %zu: %s", offset, esp_err_to_name(err));
      return false;
    }
    if (copy_len < SAMD21_PAGE_SIZE)
      memset(page_data + copy_len, 0xFF, SAMD21_PAGE_SIZE - copy_len);

    ESP_LOGD(TAG, "Writing page %" PRIu32 " / %" PRIu32 " (row %" PRIu32 ")", page, pages_needed - 1,
             page / SAMD21_PAGES_PER_ROW);
    if (!this->nvm_write_page_(page, page_data)) {
      ESP_LOGE(TAG, "Page write failed at page %" PRIu32, page);
      return false;
    }

    if (page % 64 == 0) {
      snprintf(msg, sizeof(msg), "Page %" PRIu32 " / %" PRIu32, page, pages_needed);
      this->fire_flash_status_("flashing", 25 + static_cast<int>(page * 70u / pages_needed), msg);
      ESP_LOGI(TAG, "Progress: %" PRIu32 "/%" PRIu32 " pages", page, pages_needed);
    }
  }

  ESP_LOGI(TAG, "All pages written — resetting SAMD21");
  this->fire_flash_status_("flashing", 95, "Resetting device");
  this->samba_write_word_(ARM_AIRCR_ADDR, ARM_AIRCR_RESET);
  return true;
}

bool EmonTxUpdater::samba_init_(char *ver_out, size_t ver_size) {
  this->uart_flush_rx_();

  // Auto-baud: two 0x80 bytes let the monitor lock onto the baud rate, then '#' gets "\n\r" back.
  // The replies are read only to drop them.
  uint8_t discard[3];
  this->emontx_->write_byte(0x80);
  this->uart_read_bytes_(discard, 1, 200);
  this->emontx_->write_byte(0x80);
  this->uart_read_bytes_(discard, 1, 200);
  this->emontx_->write_byte('#');
  this->uart_read_bytes_(discard, 3, 300);

  // Binary mode
  this->uart_write_("N#", 2);
  this->uart_read_bytes_(discard, 2, 300);

  if (!this->samba_version_(ver_out, ver_size)) {
    ESP_LOGE(TAG, "SAM-BA version query failed — device not in bootloader mode?");
    return false;
  }
  return true;
}

bool EmonTxUpdater::samba_version_(char *ver_out, size_t ver_size) {
  this->uart_write_("V#", 2);

  // Read until a 50 ms gap. Non-printable bytes are read too, so they can't confuse later replies.
  size_t len = 0;
  uint8_t b;
  while (this->uart_read_bytes_(&b, 1, 50)) {
    if (b >= 0x20 && b <= 0x7E && len + 1 < ver_size)
      ver_out[len++] = static_cast<char>(b);
  }
  ver_out[len] = '\0';
  return len > 0;
}

void EmonTxUpdater::samba_write_word_(uint32_t addr, uint32_t value) {
  char cmd[20];
  int len = snprintf(cmd, sizeof(cmd), "W%08" PRIX32 ",%08" PRIX32 "#", addr, value);
  this->uart_write_(cmd, len);
}

bool EmonTxUpdater::samba_read_word_(uint32_t addr, uint32_t &value_out) {
  char cmd[13];
  int len = snprintf(cmd, sizeof(cmd), "w%08" PRIX32 ",4#", addr);
  this->uart_write_(cmd, len);

  uint8_t buf[4];
  if (!this->uart_read_bytes_(buf, sizeof(buf), 500)) {
    ESP_LOGE(TAG, "samba_read_word_ timeout (addr=0x%08" PRIX32 ")", addr);
    return false;
  }
  value_out = read_le32(buf);
  return true;
}

bool EmonTxUpdater::nvm_wait_ready_(uint32_t timeout_ms) {
  const uint32_t start = millis();
  while (millis() - start < timeout_ms) {
    uint32_t intflag = 0;
    if (!this->samba_read_word_(NVM_BASE + NVM_REG_INTFLAG, intflag))
      return false;
    if (intflag & NVM_INTFLAG_READY)
      return true;
    App.feed_wdt();
  }
  ESP_LOGE(TAG, "NVM not ready (timeout %" PRIu32 " ms)", timeout_ms);
  return false;
}

bool EmonTxUpdater::nvm_command_(uint8_t cmd) {
  if (!this->nvm_wait_ready_())
    return false;
  this->samba_write_word_(NVM_BASE + NVM_REG_CTRLA, NVM_CMDEX_KEY | cmd);
  if (!this->nvm_wait_ready_())
    return false;

  uint32_t intflag = 0;
  if (!this->samba_read_word_(NVM_BASE + NVM_REG_INTFLAG, intflag))
    return false;
  if (intflag & NVM_INTFLAG_ERROR) {
    ESP_LOGE(TAG, "NVM command 0x%02X produced an error (INTFLAG=0x%08" PRIX32 ")", cmd, intflag);
    this->samba_write_word_(NVM_BASE + NVM_REG_INTFLAG, NVM_INTFLAG_ERROR);
    return false;
  }
  return true;
}

bool EmonTxUpdater::nvm_erase_row_(uint32_t row_byte_addr) {
  if (!this->nvm_wait_ready_())
    return false;
  this->samba_write_word_(NVM_BASE + NVM_REG_STATUS, NVM_STATUS_ERR_MASK);
  // ADDR takes a 16-bit word address
  this->samba_write_word_(NVM_BASE + NVM_REG_ADDR, row_byte_addr / 2u);
  if (!this->nvm_command_(NVM_CMD_ER)) {
    ESP_LOGE(TAG, "nvm_erase_row_ failed at 0x%08" PRIX32, row_byte_addr);
    return false;
  }
  return true;
}

bool EmonTxUpdater::nvm_write_page_(uint32_t page_idx, const uint8_t *data) {
  const uint32_t page_byte_addr = SAMD21_APP_ADDR + page_idx * SAMD21_PAGE_SIZE;

  if (!this->nvm_command_(NVM_CMD_PBC)) {
    ESP_LOGE(TAG, "nvm_write_page_ PBC failed at page %" PRIu32, page_idx);
    return false;
  }

  // With MANW=1 these writes only fill the page buffer; WP below programs it (datasheet §22.8.7)
  ESP_LOGV(TAG, "  page %" PRIu32 " addr=0x%08" PRIX32 ": filling page buffer", page_idx, page_byte_addr);
  for (size_t i = 0; i < SAMD21_PAGE_SIZE; i += 4)
    this->samba_write_word_(page_byte_addr + i, read_le32(data + i));

  if (!this->nvm_wait_ready_()) {
    ESP_LOGE(TAG, "nvm_write_page_ wait-ready (pre-WP) timeout at page %" PRIu32, page_idx);
    return false;
  }
  // The last word write already set ADDR; set it again as BOSSA does
  this->samba_write_word_(NVM_BASE + NVM_REG_ADDR, page_byte_addr / 2u);
  if (!this->nvm_command_(NVM_CMD_WP)) {
    ESP_LOGE(TAG, "nvm_write_page_ WP failed at page %" PRIu32 " (addr 0x%08" PRIX32 ")", page_idx, page_byte_addr);
    return false;
  }
  ESP_LOGV(TAG, "  page %" PRIu32 " written OK", page_idx);
  return true;
}

void EmonTxUpdater::uart_flush_rx_() {
  uint8_t b;
  while (this->emontx_->available() > 0)
    this->emontx_->read_byte(&b);
}

void EmonTxUpdater::uart_write_(const char *data, size_t len) {
  this->emontx_->write_array(reinterpret_cast<const uint8_t *>(data), len);
}

// Runs on the flash task only, so it can block
bool EmonTxUpdater::uart_read_bytes_(uint8_t *buf, size_t len, uint32_t timeout_ms) {
  const uint32_t start = millis();
  uint32_t wdt_tick = start;
  size_t read = 0;
  while (read < len && millis() - start < timeout_ms) {
    if (this->emontx_->available() > 0 && this->emontx_->read_byte(&buf[read])) {
      read++;
    } else {
      delay(1);  // NOLINT
    }
    if (millis() - wdt_tick >= 100) {
      App.feed_wdt();
      wdt_tick = millis();
    }
  }
  return read == len;
}

void EmonTxUpdater::fire_flash_status_(const char *status, int progress, const char *message) {
  if (this->status_queue_ == nullptr)
    return;
  FlashStatusPayload payload{status, progress, {}};
  strncpy(payload.message, message, sizeof(payload.message) - 1);
  // Drop the event rather than block when the queue is full
  if (xQueueSend(this->status_queue_, &payload, 0) == pdTRUE)
    this->enable_loop_soon_any_context();
}

}  // namespace esphome::emontx_updater
