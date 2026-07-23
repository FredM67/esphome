#pragma once

#include <string>
#include <vector>

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

#include "esphome/core/component.h"
#include "esphome/components/api/custom_api_device.h"
#include "esphome/components/emontx/emontx.h"
#include "esphome/components/http_request/http_request.h"
#include "esp_http_client.h"

namespace esphome::emontx_updater {

// ─── ATSAMD21J17 device parameters (BOSSA Device.cpp: ATSAMD21x17) ────────
static constexpr uint32_t SAMD21_APP_ADDR      = 0x00002000u; // first writable byte for application (after 8 KB bootloader)
static constexpr uint32_t SAMD21_PAGES         = 2048u;
static constexpr uint32_t SAMD21_PAGE_SIZE     = 64u;        // bytes per page
static constexpr uint32_t SAMD21_PAGES_PER_ROW = 4u;         // pages per erase row
static constexpr uint32_t SAMD21_ROW_SIZE      = 256u;       // bytes per erase row

// ─── NVM controller register offsets from NVM_BASE (SAMD21 datasheet §22) ─
static constexpr uint32_t NVM_BASE        = 0x41004000u;
static constexpr uint8_t  NVM_REG_CTRLA  = 0x00u; // 16-bit command register
static constexpr uint8_t  NVM_REG_CTRLB  = 0x04u; // 32-bit config (MANW, CACHEDIS)
static constexpr uint8_t  NVM_REG_INTFLAG = 0x14u; // READY flag (bit 0)
static constexpr uint8_t  NVM_REG_STATUS  = 0x18u; // 16-bit status / error bits
static constexpr uint8_t  NVM_REG_ADDR    = 0x1Cu; // word address for next command

static constexpr uint32_t NVM_CMDEX_KEY      = 0xA500u; // key for all CTRLA commands
static constexpr uint8_t  NVM_CMD_ER         = 0x02u;   // Erase Row
static constexpr uint8_t  NVM_CMD_WP         = 0x04u;   // Write Page
static constexpr uint8_t  NVM_CMD_PBC        = 0x44u;   // Page Buffer Clear
// W1C bits in STATUS: bit2=PROGE, bit3=LOCKE, bit4=NVME (writing 1 clears them).
static constexpr uint32_t NVM_STATUS_ERR_MASK = 0x001Cu; // clear PROGE|LOCKE|NVME
static constexpr uint32_t NVM_CTRLB_MANW_BIT  = (1u << 7);  // manual write enable
static constexpr uint32_t NVM_CTRLB_CACHE_BIT = (1u << 18); // cache disable

// ─── ARM Cortex-M reset register (used for software reset after flashing) ─
static constexpr uint32_t ARM_AIRCR_ADDR  = 0xE000ED0Cu;
static constexpr uint32_t ARM_AIRCR_RESET = 0x05FA0004u;

// ─── SAM-BA / XModem constants ────────────────────────────────────────────
static constexpr uint8_t XMODEM_SOH       = 0x01u;
static constexpr uint8_t XMODEM_EOT       = 0x04u;
static constexpr uint8_t XMODEM_ACK       = 0x06u;
static constexpr uint8_t XMODEM_NAK       = 0x15u;
static constexpr uint8_t XMODEM_CRC_START = 'C';
static constexpr size_t  XMODEM_BLK_SIZE  = 128u;
static constexpr size_t  XMODEM_FRAME_SZ  = 133u; // SOH(1)+blk(1)+~blk(1)+data(128)+CRC16(2)
static constexpr int     XMODEM_MAX_RETRIES = 5;

// ─────────────────────────────────────────────────────────────────────────

// ─── Flash status event (posted from background task, dispatched in loop()) ──
// Fixed-size struct so xQueueSend/Receive copies by value without heap ops.
struct FlashStatusPayload {
  char status[16];
  int  progress;
  char message[80];
};

/**
 * @class EmonTxUpdater
 * @brief Flashes new firmware into the ATSAMD21J17 on an emonTx6 board over
 *        UART, using the SAM-BA bootloader protocol (as used by BOSSA).
 *
 * Flow:
 *   1. HA calls service  flash_emontx6(url=<firmware URL>)
 *   2. Component downloads the binary via http_request into ESP32 SRAM (~60 KB)
 *   3. emonTx firmware is commanded into SAM-BA bootloader mode via 'e'/'y' sequence
 *   4. SAM-BA handshake (auto-baud, N#, V#)
 *   5. NVM CTRLB: disable cache, set MANW=1 (manual page write)
 *   6. For each 256-byte row: ER (erase row)
 *      For each 64-byte page: PBC + 16× W (fill page buffer) + WP (write page)
 *   7. ARM AIRCR software reset → SAMD21 boots new firmware
 *
 * Page buffer fill uses 16× SAM-BA 'W' (32-bit word write) commands directly
 * to flash addresses.  With MANW=1 the NVM controller captures these writes
 * into its page buffer without auto-programming; WP then commits the buffer.
 *
 * do_flash_() runs in a dedicated FreeRTOS task so the ESPHome main loop
 * remains responsive (feeds TWDT, serves API) during the ~60 s flash.
 * Status events are posted to status_queue_ from the task and dispatched to
 * Home Assistant in loop() on the main-loop thread.
 *
 * References: BOSSA Samba.cpp, D2xNvmFlash.cpp (ShumaTech); SAMD21 datasheet §22
 */
class EmonTxUpdater : public Component, public api::CustomAPIDevice {
 public:
  void set_emontx(emontx::EmonTx *emontx) { this->emontx_ = emontx; }
  void set_http_request(http_request::HttpRequestComponent *http) { this->http_ = http; }
  void set_bootloader_timeout(uint32_t ms) { this->bootloader_timeout_ms_ = ms; }
  void set_dry_run(bool dry_run) { this->dry_run_ = dry_run; }

  void setup() override;
  void loop() override;   ///< Drains status_queue_ and fires HA events (main-loop thread).
  void dump_config() override;
  float get_setup_priority() const override { return setup_priority::LATE; }

 protected:
  emontx::EmonTx *emontx_{nullptr};
  http_request::HttpRequestComponent *http_{nullptr};
  uint32_t bootloader_timeout_ms_{500};
  bool dry_run_{false};

  // ── Background flash task ─────────────────────────────────────────────────
  /// Status events enqueued by the flash task; loop() dispatches them to HA.
  QueueHandle_t  status_queue_{nullptr};
  TaskHandle_t   flash_task_handle_{nullptr};
  volatile bool  flash_task_running_{false};
  /// Firmware binary handed off to the background task.
  std::vector<uint8_t> flash_pending_firmware_;
  /// Background task entry point (static, calls do_flash_ on this).
  static void flash_task_fn_(void *param);

  // ── HA service entry point ───────────────────────────────────────────────
  void on_flash_firmware_(std::string url);  // NOLINT

  // ── Download phase ───────────────────────────────────────────────────────
  bool download_firmware_(const std::string &url, std::vector<uint8_t> &out);
  /// Follow redirects using a large buffer (needed for GitHub's long CSP+Location headers).
  /// Returns the final URL (after all 3xx hops) or an empty string on error.
  /// A separate small-buffer client is then opened for the body download to avoid
  /// the ESP32-C3 pbuf/esf_buf_recycle recursive-mutex crash seen with large buffers.
  std::string resolve_final_url_(const std::string &url);

  // ── Firmware validation (runs before bootloader entry) ───────────────────
  /// Sanity-check the downloaded binary without touching the SAMD21.
  /// Verifies size bounds and ARM Cortex-M vector table (SP + reset vector).
  bool validate_firmware_(const std::vector<uint8_t> &firmware);

  // ── Top-level flash orchestration (blocking) ─────────────────────────────
  bool do_flash_(const std::vector<uint8_t> &firmware);

  // ── SAM-BA protocol layer (maps 1:1 to BOSSA Samba.cpp) ─────────────────
  /// Full SAM-BA handshake: flush, auto-baud (0x80×2+'#'), N#, V#.
  bool samba_init_();
  /// SAM-BA 'W' command: write 32-bit word (no response expected over UART).
  bool samba_write_word_(uint32_t addr, uint32_t value);
  /// SAM-BA 'w' command: read 32-bit word (returns 4 binary bytes LE).
  bool samba_read_word_(uint32_t addr, uint32_t &value_out);
  /// SAM-BA 'S' command: write block to SAMD21 SRAM via XModem-CRC.
  bool samba_write_(uint32_t addr, const uint8_t *data, size_t size);
  /// SAM-BA 'G' command: jump to address (no response expected over UART).
  bool samba_go_(uint32_t addr);
  /// SAM-BA 'V' command: read version string (printable chars until non-print).
  bool samba_version_(std::string &ver_out);

  // ── XModem-CRC sender ────────────────────────────────────────────────────
  bool xmodem_send_(const uint8_t *data, size_t size);
  static uint16_t crc16_(const uint8_t *data, size_t len);

  // ── D2x NVM flash layer (maps to BOSSA D2xNvmFlash.cpp) ─────────────────
  /// Poll NVM INTFLAG.READY until set (or timeout).
  bool nvm_wait_ready_(uint32_t timeout_ms = 3000);
  /// Execute one NVM command: waitReady + write CTRLA + waitReady + check error.
  bool nvm_command_(uint8_t cmd);
  /// Erase the 256-byte row starting at row_byte_addr.
  bool nvm_erase_row_(uint32_t row_byte_addr);
  /// Write one 64-byte page to flash: PBC + 16× W (fill page buffer) + WP.
  /// @param page_idx  Absolute page index (0 … SAMD21_PAGES-1).
  /// @param data      Exactly SAMD21_PAGE_SIZE bytes to write.
  bool nvm_write_page_(uint32_t page_idx, const uint8_t *data);

  // ── UART helpers (all go through emontx_'s UARTDevice interface) ─────────
  void uart_flush_rx_();
  void uart_write_(const uint8_t *data, size_t len);
  bool uart_read_byte_(uint8_t &byte, uint32_t timeout_ms);
  bool uart_read_bytes_(uint8_t *buf, size_t len, uint32_t timeout_ms);

  // ── HA status event helper ────────────────────────────────────────────────
  /// Fires esphome.emontx_flash_status with device_id, status, progress (0-100), message.
  void fire_flash_status_(const std::string &status, int progress, const std::string &message = "");
};

}  // namespace esphome::emontx_updater
