#pragma once

#include <atomic>
#include <string>

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

#include "esphome/core/component.h"
#include "esphome/components/api/custom_api_device.h"
#include "esphome/components/emontx/emontx.h"
#ifdef USE_OTA
#include "esphome/components/ota/ota_backend.h"
#endif
#include "esp_http_client.h"
#include "esp_partition.h"

namespace esphome::emontx_updater {

// ATSAMD21J17 flash layout (BOSSA Device.cpp: ATSAMD21x17)
static constexpr uint32_t SAMD21_APP_ADDR = 0x00002000u;  // after the 8 KB bootloader
static constexpr uint32_t SAMD21_PAGE_SIZE = 64u;
static constexpr uint32_t SAMD21_PAGES_PER_ROW = 4u;  // pages per erase row
static constexpr size_t FW_MAX_SIZE = 120u * 1024u;   // 128 KB flash minus the bootloader

// NVM controller registers, offsets from NVM_BASE (SAMD21 datasheet §22)
static constexpr uint32_t NVM_BASE = 0x41004000u;
static constexpr uint8_t NVM_REG_CTRLA = 0x00u;
static constexpr uint8_t NVM_REG_CTRLB = 0x04u;
static constexpr uint8_t NVM_REG_INTFLAG = 0x14u;
static constexpr uint8_t NVM_REG_STATUS = 0x18u;
static constexpr uint8_t NVM_REG_ADDR = 0x1Cu;  // word address for the next command

static constexpr uint32_t NVM_CMDEX_KEY = 0xA500u;
static constexpr uint8_t NVM_CMD_ER = 0x02u;   // Erase Row
static constexpr uint8_t NVM_CMD_WP = 0x04u;   // Write Page
static constexpr uint8_t NVM_CMD_PBC = 0x44u;  // Page Buffer Clear
static constexpr uint32_t NVM_INTFLAG_READY = 1u << 0;
static constexpr uint32_t NVM_INTFLAG_ERROR = 1u << 1;
static constexpr uint32_t NVM_STATUS_ERR_MASK = 0x001Cu;   // PROGE | LOCKE | NVME, write 1 to clear
static constexpr uint32_t NVM_CTRLB_MANW_BIT = 1u << 7;    // manual page write
static constexpr uint32_t NVM_CTRLB_CACHE_BIT = 1u << 18;  // cache disable

// ARM Cortex-M software reset
static constexpr uint32_t ARM_AIRCR_ADDR = 0xE000ED0Cu;
static constexpr uint32_t ARM_AIRCR_RESET = 0x05FA0004u;

/// Posted by the flash task, sent to Home Assistant in loop(). Copied by value through the queue.
struct FlashStatusPayload {
  const char *status;  // string literal
  int progress;
  char message[80];
};

/**
 * Flashes new firmware into the ATSAMD21J17 of an emonTx6 over UART, with the SAM-BA
 * bootloader protocol (as BOSSA does).
 *
 * The Home Assistant service flash_emontx6(url) downloads the image into the spare OTA
 * partition and validates it. A FreeRTOS task then puts the emonTx into its bootloader
 * ('e' then 'y'), and for each row erases it and writes its pages (PBC, 16 word writes
 * with MANW=1, WP), then resets the SAMD21.
 *
 * References: BOSSA Samba.cpp, D2xNvmFlash.cpp (ShumaTech); SAMD21 datasheet §22.
 */
class EmonTxUpdater : public Component, public api::CustomAPIDevice {
 public:
  explicit EmonTxUpdater(emontx::EmonTx *emontx) : emontx_(emontx) {}

  void set_bootloader_timeout(uint32_t ms) { this->bootloader_timeout_ms_ = ms; }
  void set_dry_run(bool dry_run) { this->dry_run_ = dry_run; }
  void set_verify_ssl(bool verify) { this->verify_ssl_ = verify; }

  void setup() override;
  void loop() override;
  void dump_config() override;
  float get_setup_priority() const override { return setup_priority::LATE; }

 protected:
  void on_flash_firmware_(std::string url);  // NOLINT
  void release_update_lock_();
  static void flash_task_fn(void *param);

  bool download_firmware_(const std::string &url);
  /// Streams the body of an open HTTP response into the spare OTA partition.
  bool stage_firmware_(esp_http_client_handle_t client, size_t total);
  /// Checks the size and the ARM vector table of the staged image, without touching the SAMD21.
  bool validate_firmware_();
  bool do_flash_();

  /// Auto-baud, binary mode and version query. Fills ver_out with the version string.
  bool samba_init_(char *ver_out, size_t ver_size);
  bool samba_version_(char *ver_out, size_t ver_size);
  /// SAM-BA 'W': write a 32-bit word. There is no reply over UART.
  void samba_write_word_(uint32_t addr, uint32_t value);
  /// SAM-BA 'w': read a 32-bit word.
  bool samba_read_word_(uint32_t addr, uint32_t &value_out);

  bool nvm_wait_ready_(uint32_t timeout_ms = 3000);
  bool nvm_command_(uint8_t cmd);
  bool nvm_erase_row_(uint32_t row_byte_addr);
  /// Writes SAMD21_PAGE_SIZE bytes to the page with absolute index page_idx.
  bool nvm_write_page_(uint32_t page_idx, const uint8_t *data);

  void uart_flush_rx_();
  void uart_write_(const char *data, size_t len);
  bool uart_read_bytes_(uint8_t *buf, size_t len, uint32_t timeout_ms);

  /// Safe from any task: queues the event and wakes loop() to send it.
  void fire_flash_status_(const char *status, int progress, const char *message = "");

  emontx::EmonTx *emontx_;
  uint32_t bootloader_timeout_ms_{5000};
  bool dry_run_{false};
  bool verify_ssl_{true};

  QueueHandle_t status_queue_{nullptr};
  std::atomic<bool> flash_task_running_{false};
  /// Spare OTA partition holding the downloaded image at staging_offset_.
  const esp_partition_t *staging_partition_{nullptr};
  size_t staging_offset_{0};
  size_t staged_size_{0};
#ifdef USE_OTA
  /// Held from download until the flash ends, so an ESPHome OTA can't overwrite the staged image.
  ota::UpdateLock update_lock_;
#endif
};

}  // namespace esphome::emontx_updater
