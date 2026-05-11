#pragma once

#include <string>
#include <vector>

#include "esphome/core/component.h"
#include "esphome/components/api/custom_api_device.h"
#include "esphome/components/emontx/emontx.h"
#include "esphome/components/http_request/http_request.h"

namespace esphome::emontx_updater {

// ─── ATSAMD21J17 device parameters (BOSSA Device.cpp: ATSAMD21x17) ────────
static constexpr uint32_t SAMD21_APP_ADDR      = 0x00002000u; // first writable byte for application (after 8 KB bootloader)
static constexpr uint32_t SAMD21_PAGES         = 2048u;
static constexpr uint32_t SAMD21_PAGE_SIZE     = 64u;        // bytes per page
static constexpr uint32_t SAMD21_PAGES_PER_ROW = 4u;         // pages per erase row
static constexpr uint32_t SAMD21_ROW_SIZE      = 256u;       // bytes per erase row
static constexpr uint32_t SAMD21_APPLET_ADDR   = 0x20002000u; // SRAM base for applet
static constexpr uint32_t SAMD21_STACK_ADDR    = 0x20004000u; // top of 16 KB SRAM

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
static constexpr uint32_t NVM_STATUS_ERR_MASK = 0xFFEBu; // writable error-clear bits
static constexpr uint32_t NVM_CTRLB_MANW_BIT  = (1u << 7);  // manual write enable
static constexpr uint32_t NVM_CTRLB_CACHE_BIT = (1u << 18); // cache disable

// ─── ARM Cortex-M reset register (used for software reset after flashing) ─
static constexpr uint32_t ARM_AIRCR_ADDR  = 0xE000ED0Cu;
static constexpr uint32_t ARM_AIRCR_RESET = 0x05FA0004u;

// ─── WordCopy applet layout in SAMD21 SRAM ────────────────────────────────
// The applet binary occupies APPLET_CODE_SIZE bytes starting at SAMD21_APPLET_ADDR.
// The last 24 bytes of that block are a parameter region the applet reads via LDR:
//   +0x20: initial stack pointer  (filled once during applet upload)
//   +0x24: reset vector (PC+1)    (filled per runv() call)
//   +0x28: dst_addr               (flash page byte address, filled per page)
//   +0x2C: src_addr               (SRAM page buffer address, filled per page)
//   +0x30: words to copy          (filled once: PAGE_SIZE/4 = 16)
static constexpr size_t   APPLET_CODE_SIZE      = 52u;
static constexpr uint32_t APPLET_STACK_OFF      = 0x20u;
static constexpr uint32_t APPLET_RESET_OFF      = 0x24u;
static constexpr uint32_t APPLET_DST_ADDR_OFF   = 0x28u;
static constexpr uint32_t APPLET_SRC_ADDR_OFF   = 0x2Cu;
static constexpr uint32_t APPLET_WORDS_OFF      = 0x30u;
// APPLET_CODE_SIZE (52) is already word-aligned, so page buffers start right after.
static constexpr uint32_t PAGE_BUFFER_A = SAMD21_APPLET_ADDR + APPLET_CODE_SIZE; // 0x20002034
static constexpr uint32_t PAGE_BUFFER_B = PAGE_BUFFER_A + SAMD21_PAGE_SIZE;      // 0x20002074

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
 *   5. WordCopy applet uploaded to SAMD21 SRAM
 *   6. For each 256-byte row: erase row, write 4×64-byte pages via applet
 *   7. ARM AIRCR software reset → SAMD21 boots new firmware
 *
 * References: BOSSA Samba.cpp, D2xNvmFlash.cpp, WordCopyArm.cpp (ShumaTech)
 */
class EmonTxUpdater : public Component, public api::CustomAPIDevice {
 public:
  void set_emontx(emontx::EmonTx *emontx) { this->emontx_ = emontx; }
  void set_http_request(http_request::HttpRequestComponent *http) { this->http_ = http; }
  void set_bootloader_timeout(uint32_t ms) { this->bootloader_timeout_ms_ = ms; }
  void set_dry_run(bool dry_run) { this->dry_run_ = dry_run; }

  void setup() override;
  void dump_config() override;
  float get_setup_priority() const override { return setup_priority::LATE; }

 protected:
  emontx::EmonTx *emontx_{nullptr};
  http_request::HttpRequestComponent *http_{nullptr};
  uint32_t bootloader_timeout_ms_{5000};
  bool dry_run_{false};

  // ── HA service entry point ───────────────────────────────────────────────
  void on_flash_firmware_(std::string url);  // NOLINT

  // ── Download phase ───────────────────────────────────────────────────────
  bool download_firmware_(const std::string &url, std::vector<uint8_t> &out);
  /// Follow a single HTTP redirect (e.g. GitHub release → CDN) using a dedicated
  /// esp_http_client with a large buffer, bypassing http_request's fixed buffer.
  /// Returns the Location URL on a 3xx response, or the original URL otherwise.
  std::string resolve_redirect_(const std::string &url);

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
  /// Upload the WordCopy applet to SRAM and set stack/words parameters.
  bool flash_upload_applet_();
  /// Poll NVM INTFLAG.READY until set (or timeout).
  bool nvm_wait_ready_(uint32_t timeout_ms = 3000);
  /// Execute one NVM command: waitReady + write CTRLA + waitReady + check error.
  bool nvm_command_(uint8_t cmd);
  /// Erase the 256-byte row starting at row_byte_addr.
  bool nvm_erase_row_(uint32_t row_byte_addr);
  /**
   * Write one 64-byte page to flash.
   * @param page_idx  Absolute page index (0 … SAMD21_PAGES-1).
   * @param data      Exactly SAMD21_PAGE_SIZE bytes to write.
   * @param use_buf_a Use PAGE_BUFFER_A (true) or PAGE_BUFFER_B (false).
   */
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
